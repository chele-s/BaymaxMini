#include "PowerSystem.h"
#include "../drivers/INA219.h"

#include <cmath>
#include <algorithm>

PowerSystem::PowerSystem(INA219& sensor)
    : m_sensor(sensor)
    , m_state(power::State::NOMINAL)
    , m_chargeState(power::ChargeState::UNKNOWN)
    , m_busVoltage(0.0f)
    , m_currentMA(0.0f)
    , m_powerMW(0.0f)
    , m_batteryPct(0.0f)
    , m_runtimeHours(0.0f)
    , m_cellVoltage(0.0f)
    , m_peakCurrentMA(0.0f)
    , m_energyMWh(0.0f)
    , m_temperature(25.0f)
    , m_currentSpikeAccum(0.0f)
    , m_consecutiveFaults(0)
    , m_totalFaults(0)
    , m_readErrors(0)
    , m_prevCurrentMA(0.0f)
    , m_chargeDebounce(0.0f)
    , m_initialized(false)
{
    m_voltageFilter.alpha = 0.05f;
    m_currentFilter.alpha = 0.15f;
    m_powerFilter.alpha   = 0.10f;
}

bool PowerSystem::init() {
    m_overcurrentHyst.high = m_protConfig.overcurrentTripMA;
    m_overcurrentHyst.low  = m_protConfig.overcurrentClearMA;

    m_servoStallHyst.high = m_protConfig.servoStallTripMA;
    m_servoStallHyst.low  = m_protConfig.servoStallClearMA;

    m_overTempHyst.high = m_protConfig.overTempCelsius;
    m_overTempHyst.low  = m_protConfig.overTempClearCelsius;

    m_lowBattHyst.low  = m_battConfig.lowThreshold;
    m_lowBattHyst.high = m_battConfig.lowThreshold + 0.3f;

    m_critBattHyst.low  = m_battConfig.criticalThreshold;
    m_critBattHyst.high = m_battConfig.criticalThreshold + 0.3f;

    readSensor();
    if (m_busVoltage > 0.1f) {
        m_voltageFilter.value = m_busVoltage;
        m_voltageFilter.init  = true;
        m_currentFilter.value = m_currentMA;
        m_currentFilter.init  = true;
        m_powerFilter.value   = m_powerMW;
        m_powerFilter.init    = true;
    }

    updateBatteryEstimation();

    m_initialized = true;
    return true;
}

void PowerSystem::update(float dt) {
    if (!m_initialized) return;

    readSensor();
    updateFilters(dt);
    updateProtection(dt);
    updateBatteryEstimation();
    updateChargeState(dt);
    updateEnergyAccumulator(dt);
    checkShutdown();
}

void PowerSystem::readSensor() {
    float v, i, p;
    if (m_sensor.readPower(v, i, p)) {
        m_busVoltage = v;
        m_currentMA  = std::abs(i);
        m_powerMW    = p;
        m_consecutiveFaults = 0;
    } else {
        ++m_readErrors;
        ++m_consecutiveFaults;
    }
}

void PowerSystem::updateFilters(float dt) {
    float filteredV = m_voltageFilter.process(m_busVoltage);
    float filteredI = m_currentFilter.process(m_currentMA);
    float filteredP = m_powerFilter.process(m_powerMW);

    m_busVoltage = filteredV;
    m_currentMA  = filteredI;
    m_powerMW    = filteredP;

    if (m_currentMA > m_peakCurrentMA) {
        m_peakCurrentMA = m_currentMA;
    }

    (void)dt;
}

void PowerSystem::updateProtection(float dt) {
    power::State prevState = m_state;

    bool overcurrent = m_overcurrentHyst.process(m_currentMA);
    if (overcurrent) {
        m_currentSpikeAccum += dt * 1000.0f;
    } else {
        m_currentSpikeAccum = std::max(0.0f, m_currentSpikeAccum - dt * 500.0f);
    }

    bool sustainedOvercurrent = m_currentSpikeAccum >= m_protConfig.currentSpikeWindowMs;

    bool servoStall = m_servoStallHyst.process(m_currentMA);

    bool overTemp = m_overTempHyst.process(m_temperature);

    bool lowBatt = m_lowBattHyst.process(m_busVoltage);
    bool critBatt = m_critBattHyst.process(m_busVoltage);

    if (m_busVoltage < m_protConfig.shutdownVoltage && m_busVoltage > 0.5f) {
        setState(power::State::SHUTDOWN_IMMINENT);
    } else if (critBatt) {
        setState(power::State::CRITICAL_BATTERY);
    } else if (sustainedOvercurrent) {
        setState(power::State::OVERCURRENT);
    } else if (overTemp) {
        setState(power::State::OVERTEMPERATURE);
    } else if (servoStall && m_currentMA > m_protConfig.servoStallTripMA) {
        setState(power::State::SERVO_STALL);
    } else if (lowBatt) {
        setState(power::State::LOW_BATTERY);
    } else {
        setState(power::State::NOMINAL);
    }

    if (m_state != prevState && m_callbacks.onStateChange) {
        m_callbacks.onStateChange(m_state);
    }

    if (m_state == power::State::LOW_BATTERY && prevState != power::State::LOW_BATTERY) {
        if (m_callbacks.onLowBattery) m_callbacks.onLowBattery(m_batteryPct);
    }

    if (m_state == power::State::OVERCURRENT) {
        ++m_totalFaults;
        if (m_callbacks.onOvercurrent) m_callbacks.onOvercurrent(m_currentMA);
    }

    if (m_state == power::State::SERVO_STALL && prevState != power::State::SERVO_STALL) {
        ++m_totalFaults;
        if (m_callbacks.onServoStall) m_callbacks.onServoStall(m_currentMA);
    }

    if (m_state == power::State::SHUTDOWN_IMMINENT && prevState != power::State::SHUTDOWN_IMMINENT) {
        if (m_callbacks.onShutdownImminent) m_callbacks.onShutdownImminent();
    }
}

void PowerSystem::updateBatteryEstimation() {
    if (m_battConfig.cellCount > 0) {
        m_cellVoltage = m_busVoltage / static_cast<float>(m_battConfig.cellCount);
    } else {
        m_cellVoltage = m_busVoltage;
    }

    m_batteryPct = lookupBatteryPercent(m_cellVoltage);

    float avgI = m_currentFilter.value;
    if (avgI > 1.0f) {
        float remainingCapacity = m_battConfig.capacityMAh * (m_batteryPct / 100.0f);
        m_runtimeHours = remainingCapacity / avgI;
    } else {
        m_runtimeHours = 999.0f;
    }
}

void PowerSystem::updateChargeState(float dt) {
    float dI = m_currentMA - m_prevCurrentMA;
    m_prevCurrentMA = m_currentMA;

    float dV = m_busVoltage - m_voltageFilter.value;

    power::ChargeState candidate = power::ChargeState::UNKNOWN;

    if (m_batteryPct >= 99.0f && m_currentMA < 50.0f) {
        candidate = power::ChargeState::FULL;
    } else if (m_batteryPct <= 1.0f) {
        candidate = power::ChargeState::EMPTY;
    } else if (dV > 0.01f && m_currentMA < 100.0f) {
        candidate = power::ChargeState::CHARGING;
    } else {
        candidate = power::ChargeState::DISCHARGING;
    }

    if (candidate != m_chargeState) {
        m_chargeDebounce += dt;
        if (m_chargeDebounce > 2.0f) {
            m_chargeState = candidate;
            m_chargeDebounce = 0.0f;
        }
    } else {
        m_chargeDebounce = 0.0f;
    }

    (void)dI;
}

void PowerSystem::updateEnergyAccumulator(float dt) {
    float hoursElapsed = dt / 3600.0f;
    m_energyMWh += m_powerMW * hoursElapsed;
}

void PowerSystem::checkShutdown() {
    if (m_consecutiveFaults >= m_protConfig.maxConsecutiveFaults) {
        setState(power::State::SHUTDOWN_IMMINENT);
        if (m_callbacks.onShutdownImminent) {
            m_callbacks.onShutdownImminent();
        }
    }
}

void PowerSystem::setState(power::State newState) {
    m_state = newState;
}

float PowerSystem::lookupBatteryPercent(float cellV) const {
    if (cellV <= s_battLutVoltage[0]) return s_battLutPercent[0];
    if (cellV >= s_battLutVoltage[LUT_SIZE - 1]) return s_battLutPercent[LUT_SIZE - 1];

    for (std::size_t i = 0; i < LUT_SIZE - 1; ++i) {
        if (cellV >= s_battLutVoltage[i] && cellV < s_battLutVoltage[i + 1]) {
            float span = s_battLutVoltage[i + 1] - s_battLutVoltage[i];
            if (span < 0.001f) return s_battLutPercent[i];
            float t = (cellV - s_battLutVoltage[i]) / span;
            return s_battLutPercent[i] + t * (s_battLutPercent[i + 1] - s_battLutPercent[i]);
        }
    }

    return 0.0f;
}

void PowerSystem::setBatteryConfig(const power::BatteryConfig& config) {
    m_battConfig = config;
    m_lowBattHyst.low  = config.lowThreshold;
    m_lowBattHyst.high = config.lowThreshold + 0.3f;
    m_critBattHyst.low  = config.criticalThreshold;
    m_critBattHyst.high = config.criticalThreshold + 0.3f;
}

void PowerSystem::setProtectionConfig(const power::ProtectionConfig& config) {
    m_protConfig = config;
    m_overcurrentHyst.high = config.overcurrentTripMA;
    m_overcurrentHyst.low  = config.overcurrentClearMA;
    m_servoStallHyst.high  = config.servoStallTripMA;
    m_servoStallHyst.low   = config.servoStallClearMA;
    m_overTempHyst.high    = config.overTempCelsius;
    m_overTempHyst.low     = config.overTempClearCelsius;
}

void PowerSystem::setCallbacks(const power::PowerCallbacks& cb) {
    m_callbacks = cb;
}

void PowerSystem::setTemperature(float celsius) {
    m_temperature = celsius;
}

power::PowerSnapshot PowerSystem::snapshot() const {
    power::PowerSnapshot snap;
    snap.busVoltageV       = m_busVoltage;
    snap.currentMA         = m_currentMA;
    snap.powerMW           = m_powerMW;
    snap.batteryPercent    = m_batteryPct;
    snap.runtimeHours      = m_runtimeHours;
    snap.avgCurrentMA      = m_currentFilter.value;
    snap.peakCurrentMA     = m_peakCurrentMA;
    snap.cellVoltage       = m_cellVoltage;
    snap.energyConsumedMWh = m_energyMWh;
    snap.state             = m_state;
    snap.charge            = m_chargeState;
    return snap;
}

power::State       PowerSystem::currentState()      const { return m_state; }
float              PowerSystem::busVoltage()         const { return m_busVoltage; }
float              PowerSystem::currentMA()          const { return m_currentMA; }
float              PowerSystem::powerMW()            const { return m_powerMW; }
float              PowerSystem::batteryPercent()     const { return m_batteryPct; }
float              PowerSystem::runtimeHours()       const { return m_runtimeHours; }
float              PowerSystem::cellVoltage()        const { return m_cellVoltage; }
float              PowerSystem::avgCurrentMA()       const { return m_currentFilter.value; }
float              PowerSystem::peakCurrentMA()      const { return m_peakCurrentMA; }
float              PowerSystem::energyConsumedMWh()  const { return m_energyMWh; }
power::ChargeState PowerSystem::chargeState()        const { return m_chargeState; }

bool PowerSystem::isLowBattery()     const { return m_state == power::State::LOW_BATTERY || m_state == power::State::CRITICAL_BATTERY; }
bool PowerSystem::isCritical()       const { return m_state == power::State::CRITICAL_BATTERY || m_state == power::State::SHUTDOWN_IMMINENT; }
bool PowerSystem::isOvercurrent()    const { return m_state == power::State::OVERCURRENT; }
bool PowerSystem::isServoStalled()   const { return m_state == power::State::SERVO_STALL; }
bool PowerSystem::isOverTemp()       const { return m_state == power::State::OVERTEMPERATURE; }

void PowerSystem::clearFaults() {
    m_overcurrentHyst.reset();
    m_servoStallHyst.reset();
    m_overTempHyst.reset();
    m_currentSpikeAccum = 0.0f;
    m_consecutiveFaults = 0;
    m_state = power::State::NOMINAL;
}

void PowerSystem::resetEnergyCounter() {
    m_energyMWh = 0.0f;
}

void PowerSystem::resetPeakCurrent() {
    m_peakCurrentMA = m_currentMA;
}

uint32_t PowerSystem::faultCount()     const { return m_totalFaults; }
uint32_t PowerSystem::readErrorCount() const { return m_readErrors; }
