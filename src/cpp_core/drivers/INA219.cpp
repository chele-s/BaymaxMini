#include "INA219.h"
#include "I2C_Bus.h"

#include <cmath>
#include <algorithm>

namespace Reg {
    constexpr uint8_t CONFIG       = 0x00;
    constexpr uint8_t SHUNT_VOLT   = 0x01;
    constexpr uint8_t BUS_VOLT     = 0x02;
    constexpr uint8_t POWER        = 0x03;
    constexpr uint8_t CURRENT      = 0x04;
    constexpr uint8_t CALIBRATION  = 0x05;
}

constexpr uint16_t RESET_BIT       = 0x8000;
constexpr float    SHUNT_LSB_UV    = 10.0f;
constexpr float    BUS_LSB_MV      = 4.0f;
constexpr float    CAL_CONSTANT    = 0.04096f;
constexpr float    POWER_LSB_MULT  = 20.0f;

INA219::INA219(I2CBus& bus, uint8_t addr)
    : m_bus(bus)
    , m_addr(addr)
    , m_currentLSB(0.0f)
    , m_powerLSB(0.0f)
    , m_shuntOhms(0.1f)
    , m_lastBusV(0.0f)
    , m_lastCurrentMA(0.0f)
    , m_lastPowerMW(0.0f)
    , m_calibrated(false)
{}

void INA219::writeReg(uint8_t reg, uint16_t value) {
    m_bus.writeWord(m_addr, reg, value);
}

bool INA219::readReg(uint8_t reg, uint16_t& value) {
    return m_bus.readWord(m_addr, reg, value);
}

bool INA219::init() {
    reset();

    configure(
        BusRange::RANGE_32V,
        Gain::PGA_320MV,
        Resolution::ADC_12BIT,
        Resolution::ADC_12BIT
    );

    calibrate(0.1f, 3.2f);

    return true;
}

void INA219::shutdown() {
    writeReg(Reg::CONFIG, 0x0000);
}

void INA219::reset() {
    writeReg(Reg::CONFIG, RESET_BIT);
}

void INA219::configure(BusRange vRange, Gain gain, Resolution busRes, Resolution shuntRes) {
    uint16_t config = 0;

    config |= (static_cast<uint16_t>(vRange) & 0x01) << 13;
    config |= (static_cast<uint16_t>(gain)   & 0x03) << 11;

    uint16_t badc = static_cast<uint16_t>(busRes) & 0x0F;
    if (badc > 3) {
        config |= (1 << 10);
        config |= ((badc - 8) & 0x07) << 7;
    } else {
        config |= (badc & 0x03) << 7;
    }

    uint16_t sadc = static_cast<uint16_t>(shuntRes) & 0x0F;
    if (sadc > 3) {
        config |= (1 << 6);
        config |= ((sadc - 8) & 0x07) << 3;
    } else {
        config |= (sadc & 0x03) << 3;
    }

    config |= 0x07;

    writeReg(Reg::CONFIG, config);
}

void INA219::calibrate(float shuntOhms, float maxExpectedAmps) {
    m_shuntOhms = shuntOhms;

    m_currentLSB = maxExpectedAmps / 32768.0f;

    float calFloat = CAL_CONSTANT / (m_currentLSB * m_shuntOhms);
    uint16_t calReg = static_cast<uint16_t>(std::floor(calFloat));

    m_currentLSB = CAL_CONSTANT / (static_cast<float>(calReg) * m_shuntOhms);
    m_powerLSB = POWER_LSB_MULT * m_currentLSB;

    writeReg(Reg::CALIBRATION, calReg);
    m_calibrated = true;
}

bool INA219::readPower(float& busVoltage, float& currentMA, float& powerMW) {
    if (!m_calibrated) return false;

    if (!readBusVoltage(busVoltage)) return false;
    if (!readCurrent(currentMA)) return false;
    if (!readPowerReg(powerMW)) return false;

    m_lastBusV      = busVoltage;
    m_lastCurrentMA = currentMA;
    m_lastPowerMW   = powerMW;

    return true;
}

bool INA219::readBusVoltage(float& volts) {
    uint16_t raw;
    if (!readReg(Reg::BUS_VOLT, raw)) return false;

    int16_t shifted = static_cast<int16_t>(raw >> 3);
    volts = static_cast<float>(shifted) * BUS_LSB_MV * 0.001f;
    return true;
}

bool INA219::readShuntVoltage(float& millivolts) {
    uint16_t raw;
    if (!readReg(Reg::SHUNT_VOLT, raw)) return false;

    int16_t signed_raw = static_cast<int16_t>(raw);
    millivolts = static_cast<float>(signed_raw) * SHUNT_LSB_UV * 0.001f;
    return true;
}

bool INA219::readCurrent(float& milliamps) {
    if (!m_calibrated) return false;

    uint16_t raw;
    if (!readReg(Reg::CURRENT, raw)) return false;

    int16_t signed_raw = static_cast<int16_t>(raw);
    milliamps = static_cast<float>(signed_raw) * m_currentLSB * 1000.0f;
    return true;
}

bool INA219::readPowerReg(float& milliwatts) {
    if (!m_calibrated) return false;

    uint16_t raw;
    if (!readReg(Reg::POWER, raw)) return false;

    milliwatts = static_cast<float>(raw) * m_powerLSB * 1000.0f;
    return true;
}

bool INA219::conversionReady() {
    uint16_t raw;
    if (!readReg(Reg::BUS_VOLT, raw)) return false;
    return (raw & 0x0002) != 0;
}

float INA219::batteryPercent(float fullVoltage, float emptyVoltage) const {
    if (fullVoltage <= emptyVoltage) return 0.0f;
    float pct = (m_lastBusV - emptyVoltage) / (fullVoltage - emptyVoltage) * 100.0f;
    return std::clamp(pct, 0.0f, 100.0f);
}

float INA219::estimateRuntimeHours(float capacityMAh) const {
    if (m_lastCurrentMA <= 1.0f) return 999.0f;
    return capacityMAh / m_lastCurrentMA;
}

float INA219::lastBusVoltage() const {
    return m_lastBusV;
}

float INA219::lastCurrentMA() const {
    return m_lastCurrentMA;
}

float INA219::lastPowerMW() const {
    return m_lastPowerMW;
}
