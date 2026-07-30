#pragma once

#include <cstdint>

class I2CBus;

class INA219 {
public:
    enum class Gain : uint8_t {
        PGA_40MV   = 0,
        PGA_80MV   = 1,
        PGA_160MV  = 2,
        PGA_320MV  = 3
    };

    enum class BusRange : uint8_t {
        RANGE_16V = 0,
        RANGE_32V = 1
    };

    enum class Resolution : uint8_t {
        ADC_9BIT    = 0,
        ADC_10BIT   = 1,
        ADC_11BIT   = 2,
        ADC_12BIT   = 3,
        ADC_2SAMP   = 9,
        ADC_4SAMP   = 10,
        ADC_8SAMP   = 11,
        ADC_16SAMP  = 12,
        ADC_32SAMP  = 13,
        ADC_64SAMP  = 14,
        ADC_128SAMP = 15
    };

    INA219(I2CBus& bus, uint8_t addr);

    bool init();
    void shutdown();

    void configure(BusRange vRange, Gain gain, Resolution busRes, Resolution shuntRes);
    void calibrate(float shuntOhms, float maxExpectedAmps);

    bool readPower(float& busVoltage, float& currentMA, float& powerMW);
    bool readBusVoltage(float& volts);
    bool readShuntVoltage(float& millivolts);
    bool readCurrent(float& milliamps);
    bool readPowerReg(float& milliwatts);

    float batteryPercent(float fullVoltage, float emptyVoltage) const;
    float estimateRuntimeHours(float capacityMAh) const;

    void reset();
    bool conversionReady();

    float lastBusVoltage() const;
    float lastCurrentMA() const;
    float lastPowerMW() const;

private:
    void writeReg(uint8_t reg, uint16_t value);
    bool readReg(uint8_t reg, uint16_t& value);

    I2CBus& m_bus;
    uint8_t m_addr;
    float   m_currentLSB;
    float   m_powerLSB;
    float   m_shuntOhms;
    float   m_lastBusV;
    float   m_lastCurrentMA;
    float   m_lastPowerMW;
    bool    m_calibrated;
};
