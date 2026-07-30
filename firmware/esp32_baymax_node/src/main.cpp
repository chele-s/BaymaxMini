#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Config.h"
#include "SerialProtocol.h"
#include "CamServer.h"

#include "I2CBus.h"
#include "SharedData.h"

#include "PCA9685.h"
#include "VL53L1X.h"
#include "MAX30102.h"
#include "MLX90614.h"
#include "INA219.h"

#include "FaceController.h"
#include "VitalsMonitor.h"
#include "PowerSystem.h"

static I2CBus       g_i2c(cfg::I2C_SDA_PIN, cfg::I2C_SCL_PIN, cfg::I2C_FREQ_HZ);

static PCA9685      g_pca(g_i2c, cfg::PCA9685_ADDR);
static VL53L1X      g_tof(g_i2c, cfg::VL53L1X_ADDR);
static MAX30102     g_pulseOx(g_i2c, cfg::MAX30102_ADDR);
static MLX90614     g_tempSensor(g_i2c, cfg::MLX90614_ADDR);
static INA219       g_ina(g_i2c, cfg::INA219_ADDR);

static FaceController  g_face(g_pca);
static VitalsMonitor   g_vitals(g_pulseOx, g_tempSensor);
static PowerSystem     g_power(g_ina);

static SerialProtocol  g_serial(Serial);
static CamServer       g_cam;

static ipc::SystemStateId  g_systemState = ipc::SystemStateId::BOOT;
static ipc::AlertLevel     g_alertLevel  = ipc::AlertLevel::NONE;
static uint32_t            g_tickCount   = 0;
static float               g_maxTickMs   = 0.0f;
static float               g_avgTickMs   = 0.0f;
static uint32_t            g_overruns    = 0;

static bool g_brainConnected = false;
static bool g_autonomousMode = false;

static void processCommand(const ipc::CommandFrame& cmd) {
    switch (cmd.type) {
        case ipc::CommandType::SET_EXPRESSION: {
            auto expr = static_cast<face::Expression>(cmd.payload.expression.type);
            g_face.setExpression(expr, cmd.payload.expression.transition);
            break;
        }
        case ipc::CommandType::SET_EYELID: {
            g_face.setEyelidOpenness(cmd.payload.eyelid.openness, cmd.payload.eyelid.duration);
            break;
        }
        case ipc::CommandType::TRIGGER_BLINK: {
            g_face.triggerBlink();
            break;
        }
        case ipc::CommandType::SET_GAZE: {
            g_face.setGaze(cmd.payload.gaze.x, cmd.payload.gaze.y, cmd.payload.gaze.speed);
            break;
        }
        case ipc::CommandType::SET_BREATH: {
            g_face.setBreathIntensity(cmd.payload.breath.intensity);
            break;
        }
        case ipc::CommandType::EMERGENCY_STOP: {
            g_pca.allOff();
            g_systemState = ipc::SystemStateId::ERROR;
            g_alertLevel = ipc::AlertLevel::CRITICAL;
            break;
        }
        case ipc::CommandType::SHUTDOWN: {
            g_face.enterSleepMode(1.0f);
            g_pca.shutdown();
            g_systemState = ipc::SystemStateId::SHUTDOWN;
            break;
        }
        default:
            break;
    }
}

static void buildTelemetry(ipc::TelemetryFrame& frame) {
    uint16_t distMM = 0;
    bool distOk = false;
    bool tofReady = false;
    if (g_tof.dataReady(tofReady) && tofReady) {
        distOk = g_tof.readDistance(distMM);
        g_tof.clearInterrupt();
    }

    frame.distance_mm     = distOk ? static_cast<float>(distMM) : -1.0f;
    frame.proximity_valid = distOk ? 1 : 0;

    auto vitData = g_vitals.currentVitals();
    frame.heart_rate_bpm  = static_cast<float>(vitData.heartRateBPM);
    frame.spo2_percent    = static_cast<float>(vitData.spo2Percent);
    frame.skin_temp_c     = static_cast<float>(vitData.bodyTempC);
    frame.ambient_temp_c  = static_cast<float>(vitData.ambientTempC);
    frame.vitals_valid    = (vitData.heartRateBPM > 0.0) ? 1 : 0;

    frame.bus_voltage_v   = g_power.busVoltage();
    frame.current_ma      = g_power.currentMA();
    frame.power_mw        = g_power.powerMW();
    frame.battery_pct     = g_power.batteryPercent();
    frame.power_valid     = 1;

    frame.eyelid_openness = g_face.leftEyePose().upperLid;
    frame.gaze_x          = g_face.leftEyePose().gazeX;
    frame.gaze_y          = g_face.leftEyePose().gazeY;
    frame.breath_level    = g_face.breathLevel();
    frame.face_valid      = 1;

    frame.state           = g_systemState;
    frame.expression      = static_cast<ipc::ExpressionType>(g_face.currentExpression());
    frame.alert           = g_alertLevel;
}

static void updateAutonomousBehavior(float dt) {
    if (!g_autonomousMode) return;

    g_face.setAutoBlinkEnabled(true);
    g_face.setMicroMovementsEnabled(true);

    if (g_power.isLowBattery()) {
        g_face.setExpression(face::Expression::SAD, 1.0f);
    }

    if (g_power.isCritical()) {
        g_face.enterSleepMode(2.0f);
    }

    (void)dt;
}

static void realtimeLoopTask(void* param) {
    (void)param;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(static_cast<uint32_t>(1000.0 / cfg::TICK_RATE_HZ));

    while (true) {
        uint32_t tickStart = micros();

        g_power.update(cfg::TICK_DT);

        double ambientT = 0.0;
        g_tempSensor.readAmbientTemp(ambientT);
        g_power.setTemperature(static_cast<float>(ambientT));

        g_vitals.update(cfg::TICK_DT);
        g_face.update(static_cast<double>(cfg::TICK_DT));

        if (g_power.isOvercurrent() || g_power.isServoStalled()) {
            g_pca.allOff();
            g_alertLevel = ipc::AlertLevel::CRITICAL;
        }

        updateAutonomousBehavior(cfg::TICK_DT);

        ipc::TelemetryFrame telemetry;
        buildTelemetry(telemetry);
        g_serial.sendTelemetry(telemetry);

        ++g_tickCount;
        uint32_t tickEnd = micros();
        float tickMs = static_cast<float>(tickEnd - tickStart) / 1000.0f;
        if (tickMs > g_maxTickMs) g_maxTickMs = tickMs;
        g_avgTickMs = g_avgTickMs * 0.95f + tickMs * 0.05f;

        if (tickMs > (1000.0f / static_cast<float>(cfg::TICK_RATE_HZ))) {
            ++g_overruns;
        }

        vTaskDelayUntil(&lastWake, periodTicks);
    }
}

static void commTask(void* param) {
    (void)param;

    while (true) {
        ipc::CommandFrame cmd;
        while (g_serial.receiveCommand(cmd)) {
            processCommand(cmd);
        }

        g_serial.updateHeartbeat(g_systemState);
        g_serial.checkTimeout();

        bool wasConnected = g_brainConnected;
        g_brainConnected = g_serial.isConnected();

        if (wasConnected && !g_brainConnected) {
            g_systemState = ipc::SystemStateId::AUTONOMOUS;
            g_autonomousMode = true;
        } else if (!wasConnected && g_brainConnected) {
            g_systemState = ipc::SystemStateId::CONNECTED;
            g_autonomousMode = false;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void setup() {
    g_serial.begin();

    if (!g_i2c.open()) {
        g_systemState = ipc::SystemStateId::ERROR;
        g_alertLevel = ipc::AlertLevel::CRITICAL;
    }

    g_ina.init();
    g_ina.calibrate(cfg::ina::SHUNT_RESISTANCE_OHM, cfg::ina::MAX_EXPECTED_CURRENT);
    g_power.init();

    power::BatteryConfig battCfg;
    battCfg.nominalVoltage    = cfg::batt::NOMINAL_VOLTAGE;
    battCfg.fullVoltage       = cfg::batt::FULL_VOLTAGE;
    battCfg.emptyVoltage      = cfg::batt::EMPTY_VOLTAGE;
    battCfg.capacityMAh       = cfg::batt::CAPACITY_MAH;
    battCfg.cellCount         = cfg::batt::CELL_COUNT;
    g_power.setBatteryConfig(battCfg);

    power::ProtectionConfig protCfg;
    protCfg.overcurrentTripMA  = cfg::prot::OVERCURRENT_TRIP_MA;
    protCfg.overcurrentClearMA = cfg::prot::OVERCURRENT_CLEAR_MA;
    protCfg.servoStallTripMA   = cfg::prot::SERVO_STALL_TRIP_MA;
    protCfg.servoStallClearMA  = cfg::prot::SERVO_STALL_CLEAR_MA;
    g_power.setProtectionConfig(protCfg);

    g_pca.init();
    g_face.init();

    g_pulseOx.init();
    g_tempSensor.init();
    g_vitals.init();

    g_tof.init();
    g_tof.setDistanceMode(VL53L1X::DistanceMode::LONG);
    g_tof.setTimingBudgetMs(50);
    g_tof.setInterMeasurementMs(0);
    g_tof.startRanging();

    if (g_cam.initCamera()) {
        g_cam.startAP("BaymaxMini_CAM");
    }

    g_systemState = ipc::SystemStateId::AUTONOMOUS;
    g_autonomousMode = true;

    xTaskCreatePinnedToCore(
        realtimeLoopTask,
        "rt_loop",
        8192,
        nullptr,
        configMAX_PRIORITIES - 1,
        nullptr,
        1
    );

    xTaskCreatePinnedToCore(
        commTask,
        "comm",
        4096,
        nullptr,
        2,
        nullptr,
        0
    );
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
