#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <array>
#include <functional>

class INA219;

namespace power {

enum class State : uint8_t {
    NOMINAL,
    LOW_BATTERY,
    CRITICAL_BATTERY,
    OVERCURRENT,
    SERVO_STALL,
    OVERTEMPERATURE,
    SHUTDOWN_IMMINENT
};

enum class ChargeState : uint8_t {
    UNKNOWN,
    DISCHARGING,
    CHARGING,
    FULL,
    EMPTY
};

struct BatteryConfig {
    float nominalVoltage    = 11.1f;
    float fullVoltage       = 12.6f;
    float emptyVoltage      = 9.0f;
    float lowThreshold      = 10.2f;
    float criticalThreshold = 9.6f;
    float capacityMAh       = 2200.0f;
    uint8_t cellCount       = 3;
};

struct ProtectionConfig {
    float overcurrentTripMA     = 3000.0f;
    float overcurrentClearMA    = 2500.0f;
    float servoStallTripMA      = 2000.0f;
    float servoStallClearMA     = 1200.0f;
    float maxPowerMW            = 40000.0f;
    float shutdownVoltage       = 8.5f;
    float overTempCelsius       = 60.0f;
    float overTempClearCelsius  = 50.0f;
    float currentSpikeWindowMs  = 500.0f;
    uint8_t maxConsecutiveFaults = 5;
};

struct PowerSnapshot {
    float busVoltageV      = 0.0f;
    float currentMA        = 0.0f;
    float powerMW          = 0.0f;
    float batteryPercent   = 0.0f;
    float runtimeHours     = 0.0f;
    float avgCurrentMA     = 0.0f;
    float peakCurrentMA    = 0.0f;
    float cellVoltage      = 0.0f;
    float energyConsumedMWh = 0.0f;
    State state            = State::NOMINAL;
    ChargeState charge     = ChargeState::UNKNOWN;
};

struct PowerCallbacks {
    std::function<void(State)>     onStateChange;
    std::function<void(float)>     onLowBattery;
    std::function<void(float)>     onOvercurrent;
    std::function<void()>          onShutdownImminent;
    std::function<void(float)>     onServoStall;
};

}

class PowerSystem {
public:
    PowerSystem(INA219& sensor);

    bool init();
    void update(float dt);

    void setBatteryConfig(const power::BatteryConfig& config);
    void setProtectionConfig(const power::ProtectionConfig& config);
    void setCallbacks(const power::PowerCallbacks& cb);

    void setTemperature(float celsius);

    power::PowerSnapshot snapshot() const;
    power::State currentState() const;

    float busVoltage()     const;
    float currentMA()      const;
    float powerMW()        const;
    float batteryPercent() const;
    float runtimeHours()   const;
    float cellVoltage()    const;
    float avgCurrentMA()   const;
    float peakCurrentMA()  const;
    float energyConsumedMWh() const;

    power::ChargeState chargeState() const;

    bool isLowBattery()     const;
    bool isCritical()       const;
    bool isOvercurrent()    const;
    bool isServoStalled()   const;
    bool isOverTemp()       const;

    void clearFaults();
    void resetEnergyCounter();
    void resetPeakCurrent();

    uint32_t faultCount() const;
    uint32_t readErrorCount() const;

private:
    struct HysteresisState {
        bool  active = false;
        float high   = 0.0f;
        float low    = 0.0f;

        bool process(float value) {
            if (active) {
                if (value < low) active = false;
            } else {
                if (value > high) active = true;
            }
            return active;
        }

        void reset() { active = false; }
    };

    struct VoltageHysteresis {
        bool  active = false;
        float high   = 0.0f;
        float low    = 0.0f;

        bool process(float value) {
            if (active) {
                if (value > high) active = false;
            } else {
                if (value < low) active = true;
            }
            return active;
        }

        void reset() { active = false; }
    };

    struct EMAFilter {
        float value = 0.0f;
        float alpha = 0.1f;
        bool  init  = false;

        float process(float input) {
            if (!init) {
                value = input;
                init = true;
                return value;
            }
            value += alpha * (input - value);
            return value;
        }

        void reset() {
            value = 0.0f;
            init  = false;
        }
    };

    void readSensor();
    void updateFilters(float dt);
    void updateProtection(float dt);
    void updateBatteryEstimation();
    void updateChargeState(float dt);
    void updateEnergyAccumulator(float dt);
    void checkShutdown();
    void setState(power::State newState);

    float lookupBatteryPercent(float cellV) const;

    INA219& m_sensor;

    power::BatteryConfig     m_battConfig;
    power::ProtectionConfig  m_protConfig;
    power::PowerCallbacks    m_callbacks;

    power::State       m_state;
    power::ChargeState m_chargeState;

    float m_busVoltage;
    float m_currentMA;
    float m_powerMW;
    float m_batteryPct;
    float m_runtimeHours;
    float m_cellVoltage;
    float m_peakCurrentMA;
    float m_energyMWh;
    float m_temperature;

    EMAFilter m_voltageFilter;
    EMAFilter m_currentFilter;
    EMAFilter m_powerFilter;

    HysteresisState m_overcurrentHyst;
    HysteresisState m_servoStallHyst;
    HysteresisState m_overTempHyst;
    VoltageHysteresis m_lowBattHyst;
    VoltageHysteresis m_critBattHyst;

    float    m_currentSpikeAccum;
    uint8_t  m_consecutiveFaults;
    uint32_t m_totalFaults;
    uint32_t m_readErrors;

    float m_prevCurrentMA;
    float m_chargeDebounce;

    bool m_initialized;

    static constexpr std::size_t LUT_SIZE = 11;
    static constexpr float s_battLutVoltage[LUT_SIZE] = {
        3.00f, 3.30f, 3.50f, 3.60f, 3.70f, 3.75f,
        3.80f, 3.85f, 3.95f, 4.10f, 4.20f
    };
    static constexpr float s_battLutPercent[LUT_SIZE] = {
        0.0f, 5.0f, 10.0f, 20.0f, 30.0f, 40.0f,
        50.0f, 60.0f, 70.0f, 90.0f, 100.0f
    };
};
