#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <optional>

#include <zmq.hpp>
#include <nlohmann/json.hpp>

#include "drivers/I2C_Bus.h"
#include "drivers/PCA9685.h"
#include "drivers/VL53L1X.h"
#include "drivers/MAX30102.h"
#include "drivers/MLX90614.h"
#include "drivers/INA219.h"
#include "modules/FaceController.h"
#include "modules/VitalsMonitor.h"
#include "modules/PowerSystem.h"
#include "ipc/SharedData.h"

using SteadyClock = std::chrono::steady_clock;
using TimePoint   = SteadyClock::time_point;
using Duration    = std::chrono::duration<double>;
using json        = nlohmann::json;

namespace Config {
    constexpr double   TICK_RATE_HZ        = 50.0;
    constexpr int64_t  TICK_PERIOD_US      = static_cast<int64_t>(1'000'000.0 / TICK_RATE_HZ);
    constexpr float    TICK_DT             = static_cast<float>(1.0 / TICK_RATE_HZ);
    constexpr double   BRAIN_TIMEOUT_SEC   = 2.0;
    constexpr int      I2C_BUS_ID          = 1;
    constexpr uint8_t  PCA9685_ADDR        = 0x40;
    constexpr uint8_t  VL53L1X_ADDR        = 0x29;
    constexpr uint8_t  MAX30102_ADDR       = 0x57;
    constexpr uint8_t  MLX90614_ADDR       = 0x5A;
    constexpr uint8_t  INA219_ADDR         = 0x41;
    constexpr int      ZMQ_IO_THREADS      = 1;
    constexpr char     ZMQ_ENDPOINT[]      = "tcp://127.0.0.1:5555";
    constexpr char     ZMQ_PUB_ENDPOINT[]  = "tcp://127.0.0.1:5556";
    constexpr char     ZMQ_TOPIC_CMD[]     = "CMD";
    constexpr char     ZMQ_TOPIC_TELEM[]   = "TELEM";
    constexpr double   BLINK_INTERVAL_SEC  = 4.0;
    constexpr double   BLINK_DURATION_SEC  = 0.15;
    constexpr double   BREATH_CYCLE_SEC    = 5.0;
}

struct BrainCommand {
    std::string type;
    json        data;
    bool        valid = false;
};

enum class SystemState : uint8_t {
    BOOT,
    CONNECTED,
    AUTONOMOUS,
    SHUTDOWN
};

static std::atomic<bool> g_running{true};

static void signalHandler(int sig) {
    (void)sig;
    g_running.store(false, std::memory_order_release);
}

static uint64_t microsNow() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            SteadyClock::now().time_since_epoch()
        ).count()
    );
}

static face::Expression stringToExpression(const std::string& s) {
    if (s == "happy")     return face::Expression::HAPPY;
    if (s == "sad")       return face::Expression::SAD;
    if (s == "surprised") return face::Expression::SURPRISED;
    if (s == "angry")     return face::Expression::ANGRY;
    if (s == "sleepy")    return face::Expression::SLEEPY;
    if (s == "concerned") return face::Expression::CONCERNED;
    if (s == "curious")   return face::Expression::CURIOUS;
    if (s == "love")      return face::Expression::LOVE;
    if (s == "thinking")  return face::Expression::THINKING;
    return face::Expression::NEUTRAL;
}

class BrainLink {
public:
    BrainLink()
        : m_ctx(Config::ZMQ_IO_THREADS)
        , m_sub(m_ctx, zmq::socket_type::sub)
        , m_pub(m_ctx, zmq::socket_type::pub)
    {}

    bool init() {
        try {
            m_sub.set(zmq::sockopt::subscribe, Config::ZMQ_TOPIC_CMD);
            m_sub.set(zmq::sockopt::conflate, 1);
            m_sub.set(zmq::sockopt::rcvtimeo, 0);
            m_sub.connect(Config::ZMQ_ENDPOINT);

            m_pub.set(zmq::sockopt::sndhwm, 1);
            m_pub.set(zmq::sockopt::conflate, 1);
            m_pub.bind(Config::ZMQ_PUB_ENDPOINT);

            m_alive = true;
            return true;
        } catch (const zmq::error_t&) {
            m_alive = false;
            return false;
        }
    }

    std::optional<BrainCommand> receive() {
        if (!m_alive) return std::nullopt;

        try {
            zmq::message_t topic;
            zmq::message_t payload;

            auto res_topic = m_sub.recv(topic, zmq::recv_flags::dontwait);
            if (!res_topic.has_value()) return std::nullopt;

            auto res_payload = m_sub.recv(payload, zmq::recv_flags::dontwait);
            if (!res_payload.has_value()) return std::nullopt;

            std::string raw(static_cast<char*>(payload.data()), payload.size());
            json j = json::parse(raw, nullptr, false);
            if (j.is_discarded()) return std::nullopt;

            BrainCommand cmd;
            cmd.type  = j.value("cmd", "");
            cmd.data  = j.value("data", json::object());
            cmd.valid = !cmd.type.empty();
            return cmd;

        } catch (const zmq::error_t&) {
            return std::nullopt;
        } catch (const json::exception&) {
            return std::nullopt;
        }
    }

    void publishTelemetry(const ipc::TelemetryFrame& frame) {
        if (!m_alive) return;

        try {
            zmq::message_t topic(Config::ZMQ_TOPIC_TELEM, std::strlen(Config::ZMQ_TOPIC_TELEM));
            zmq::message_t payload(&frame, sizeof(frame));
            m_pub.send(topic, zmq::send_flags::sndmore | zmq::send_flags::dontwait);
            m_pub.send(payload, zmq::send_flags::dontwait);
        } catch (const zmq::error_t&) {}
    }

    void shutdown() {
        m_alive = false;
        try {
            m_sub.close();
            m_pub.close();
            m_ctx.close();
        } catch (...) {}
    }

    bool alive() const { return m_alive; }

private:
    zmq::context_t m_ctx;
    zmq::socket_t  m_sub;
    zmq::socket_t  m_pub;
    bool           m_alive = false;
};

class AutonomousController {
public:
    void reset() {
        m_elapsed     = 0.0;
        m_blinkTimer  = 0.0;
        m_blinking    = false;
    }

    void update(double dt, FaceController& face) {
        m_elapsed    += dt;
        m_blinkTimer += dt;

        if (!m_blinking && m_blinkTimer >= Config::BLINK_INTERVAL_SEC) {
            m_blinking   = true;
            m_blinkTimer = 0.0;
            face.triggerBlink();
        }

        if (m_blinking && m_blinkTimer >= Config::BLINK_DURATION_SEC) {
            m_blinking = false;
        }

        double breathPhase = std::fmod(m_elapsed, Config::BREATH_CYCLE_SEC) / Config::BREATH_CYCLE_SEC;
        float breathValue = static_cast<float>(0.5 + 0.5 * std::sin(breathPhase * 2.0 * M_PI));
        face.setBreathIntensity(breathValue);
    }

private:
    double m_elapsed     = 0.0;
    double m_blinkTimer  = 0.0;
    bool   m_blinking    = false;
};

class Orchestrator {
public:
    Orchestrator()
        : m_bus(Config::I2C_BUS_ID)
        , m_pca9685(m_bus, Config::PCA9685_ADDR)
        , m_tof(m_bus, Config::VL53L1X_ADDR)
        , m_pulseOx(m_bus, Config::MAX30102_ADDR)
        , m_thermo(m_bus, Config::MLX90614_ADDR)
        , m_ina219(m_bus, Config::INA219_ADDR)
        , m_face(m_pca9685)
        , m_vitals(m_pulseOx, m_thermo)
        , m_powerSys(m_ina219)
    {}

    int run() {
        if (!bootstrap()) return 1;

        m_state = SystemState::BOOT;
        TimePoint nextTick = SteadyClock::now();

        while (g_running.load(std::memory_order_acquire)) {
            nextTick += std::chrono::microseconds(Config::TICK_PERIOD_US);

            tick();

            std::this_thread::sleep_until(nextTick);

            TimePoint now = SteadyClock::now();
            if (now > nextTick) {
                int64_t behind_us = std::chrono::duration_cast<std::chrono::microseconds>(now - nextTick).count();
                if (behind_us > Config::TICK_PERIOD_US * 3) {
                    nextTick = now;
                    ++m_status.overrun_count;
                }
            }

            ++m_status.tick_count;
        }

        teardown();
        return 0;
    }

private:
    bool bootstrap() {
        if (!m_bus.open()) return false;

        bool sensorsOk = true;
        sensorsOk &= m_pca9685.init();
        sensorsOk &= m_tof.init();
        sensorsOk &= m_pulseOx.init();
        sensorsOk &= m_thermo.init();
        sensorsOk &= m_ina219.init();

        if (!sensorsOk) {
            std::fprintf(stderr, "baymax: sensor init partial failure\n");
        }

        m_tof.startRanging();

        m_ina219.calibrate(0.1f, 3.2f);

        m_face.init();

        m_vitals.init();
        m_vitals.startMeasurement();

        m_powerSys.init();

        power::BatteryConfig battCfg;
        battCfg.nominalVoltage = 11.1f;
        battCfg.fullVoltage    = 12.6f;
        battCfg.emptyVoltage   = 9.0f;
        battCfg.capacityMAh    = 2200.0f;
        battCfg.cellCount      = 3;
        m_powerSys.setBatteryConfig(battCfg);

        power::ProtectionConfig protCfg;
        protCfg.overcurrentTripMA  = 3000.0f;
        protCfg.overcurrentClearMA = 2500.0f;
        protCfg.servoStallTripMA   = 2000.0f;
        protCfg.servoStallClearMA  = 1200.0f;
        m_powerSys.setProtectionConfig(protCfg);

        m_powerSys.setCallbacks({
            nullptr,
            nullptr,
            [this](float) { m_face.enterAlertMode(); },
            [this]()      { g_running.store(false, std::memory_order_release); },
            [this](float) { m_face.enterAlertMode(); }
        });

        bool zmqOk = m_brain.init();
        m_state = zmqOk ? SystemState::CONNECTED : SystemState::AUTONOMOUS;

        m_lastBrainContact = SteadyClock::now();
        m_autonomous.reset();
        return true;
    }

    void tick() {
        TimePoint tickStart = SteadyClock::now();

        readSensors();
        processCommands();
        updateLogic();
        writeOutputs();

        float tickMs = static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                SteadyClock::now() - tickStart
            ).count() / 1000.0
        );

        m_status.avg_tick_ms += (tickMs - m_status.avg_tick_ms) * 0.02f;
        if (tickMs > m_status.max_tick_ms) m_status.max_tick_ms = tickMs;
    }

    void readSensors() {
        m_telem.timestamp_us = microsNow();

        uint16_t distMM = 0;
        bool ready = false;
        if (m_tof.dataReady(ready) && ready) {
            m_telem.proximity_valid = m_tof.readDistance(distMM) ? 1 : 0;
            m_telem.distance_mm = static_cast<float>(distMM);
            m_tof.clearInterrupt();
        }

        double hr = 0.0, spo2 = 0.0;
        if (m_pulseOx.readFIFO(hr, spo2)) {
            m_telem.heart_rate_bpm = static_cast<float>(hr);
            m_telem.spo2_percent   = static_cast<float>(spo2);
        }

        double bodyT = 0.0, ambT = 0.0;
        if (m_thermo.readTemperatures(bodyT, ambT)) {
            m_telem.skin_temp_c    = static_cast<float>(bodyT);
            m_telem.ambient_temp_c = static_cast<float>(ambT);
        }

        float busV = 0.0f, curMA = 0.0f, powMW = 0.0f;
        if (m_ina219.readPower(busV, curMA, powMW)) {
            m_telem.bus_voltage_v = busV;
            m_telem.current_ma    = curMA;
            m_telem.power_mw      = powMW;
            m_telem.power_valid   = 1;
        }

        m_telem.battery_pct = m_ina219.batteryPercent(12.6f, 9.0f);

        m_vitals.update(Config::TICK_DT);
        m_powerSys.update(Config::TICK_DT);

        auto vitData = m_vitals.currentVitals();
        m_telem.vitals_valid = vitData.readingValid ? 1 : 0;
    }

    void processCommands() {
        auto cmd = m_brain.receive();

        if (cmd.has_value() && cmd->valid) {
            m_lastBrainContact = SteadyClock::now();

            if (m_state == SystemState::AUTONOMOUS || m_state == SystemState::BOOT) {
                m_state = SystemState::CONNECTED;
            }

            dispatchCommand(cmd.value());
        }

        double elapsed = Duration(SteadyClock::now() - m_lastBrainContact).count();
        if (m_state == SystemState::CONNECTED && elapsed > Config::BRAIN_TIMEOUT_SEC) {
            m_state = SystemState::AUTONOMOUS;
            m_autonomous.reset();
        }
    }

    void dispatchCommand(const BrainCommand& cmd) {
        if (cmd.type == "set_expression") {
            std::string expr = cmd.data.value("type", "neutral");
            float transition = cmd.data.value("transition", 0.3f);
            m_face.setExpression(stringToExpression(expr), transition);
        }
        else if (cmd.type == "set_eyelid") {
            float openness = cmd.data.value("openness", 1.0f);
            float duration = cmd.data.value("duration", 0.2f);
            m_face.setEyelidOpenness(openness, duration);
        }
        else if (cmd.type == "trigger_blink") {
            m_face.triggerBlink();
        }
        else if (cmd.type == "set_gaze") {
            float x = cmd.data.value("x", 0.0f);
            float y = cmd.data.value("y", 0.0f);
            float speed = cmd.data.value("speed", 1.0f);
            m_face.setGaze(x, y, speed);
        }
        else if (cmd.type == "set_brow") {
            float left  = cmd.data.value("left", 0.5f);
            float right = cmd.data.value("right", 0.5f);
            float dur   = cmd.data.value("duration", 0.3f);
            m_face.setBrowPosition(left, right, dur);
        }
        else if (cmd.type == "sleep") {
            m_face.enterSleepMode(cmd.data.value("transition", 2.0f));
        }
        else if (cmd.type == "wake") {
            m_face.exitSleepMode(cmd.data.value("transition", 1.0f));
        }
        else if (cmd.type == "start_vitals") {
            m_vitals.startMeasurement();
        }
        else if (cmd.type == "stop_vitals") {
            m_vitals.stopMeasurement();
        }
        else if (cmd.type == "shutdown") {
            g_running.store(false, std::memory_order_release);
        }
    }

    void updateLogic() {
        switch (m_state) {
            case SystemState::CONNECTED:
                m_face.update(Config::TICK_DT);
                break;

            case SystemState::AUTONOMOUS:
                m_autonomous.update(Config::TICK_DT, m_face);
                m_face.update(Config::TICK_DT);
                break;

            case SystemState::BOOT:
                m_face.update(Config::TICK_DT);
                if (m_brain.alive()) m_state = SystemState::CONNECTED;
                break;

            case SystemState::SHUTDOWN:
                break;
        }

        if (m_powerSys.isCritical()) {
            m_face.enterAlertMode();
        }
    }

    void writeOutputs() {
        auto snap = m_powerSys.snapshot();
        m_telem.battery_pct = static_cast<float>(snap.batteryPercent);

        m_telem.face_valid = 1;
        m_telem.eyelid_openness = m_face.leftEyePose().upperLid;
        m_telem.gaze_x = m_face.leftEyePose().gazeX;
        m_telem.gaze_y = m_face.leftEyePose().gazeY;

        ipc::ExpressionType ipcExpr = static_cast<ipc::ExpressionType>(
            static_cast<uint8_t>(m_face.currentExpression())
        );
        m_telem.expression = ipcExpr;

        m_telem.state = static_cast<ipc::SystemStateId>(
            static_cast<uint8_t>(m_state)
        );

        if (m_powerSys.currentState() != power::State::NOMINAL) {
            m_telem.alert = ipc::AlertLevel::WARNING;
        } else {
            m_telem.alert = ipc::AlertLevel::NONE;
        }

        m_status.uptime_us = microsNow();
        m_status.i2c_errors = static_cast<uint8_t>(
            std::min(m_bus.errorCount(), static_cast<uint32_t>(255))
        );

        uint32_t seq = m_telem.sequence + 1;
        m_telem.stamp(seq, microsNow());

        m_brain.publishTelemetry(m_telem);
    }

    void teardown() {
        m_state = SystemState::SHUTDOWN;
        m_face.setEyelidOpenness(0.0f, 0.5f);
        m_face.update(0.5f);

        m_vitals.stopMeasurement();
        m_tof.stopRanging();

        m_tof.shutdown();
        m_pulseOx.shutdown();
        m_pca9685.shutdown();
        m_ina219.shutdown();

        m_brain.shutdown();
        m_bus.close();
    }

    I2CBus   m_bus;
    PCA9685  m_pca9685;
    VL53L1X  m_tof;
    MAX30102 m_pulseOx;
    MLX90614 m_thermo;
    INA219   m_ina219;

    FaceController       m_face;
    VitalsMonitor        m_vitals;
    PowerSystem          m_powerSys;

    BrainLink            m_brain;
    AutonomousController m_autonomous;

    ipc::TelemetryFrame  m_telem{};
    ipc::SystemStatus    m_status{};

    SystemState          m_state = SystemState::BOOT;
    TimePoint            m_lastBrainContact;
};

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    Orchestrator orch;
    return orch.run();
}