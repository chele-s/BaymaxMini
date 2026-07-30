#pragma once

#include <cstdint>
#include <cstddef>

class I2CBus;

class MAX30102 {
public:
    MAX30102(I2CBus& bus, uint8_t addr);

    bool init();
    void shutdown();

    bool readFIFO(double& heartRateBPM, double& spo2Pct);

    bool dataReady();
    uint8_t samplesAvailable();

    void setLedCurrent(uint8_t redMA, uint8_t irMA);
    void setSampleRate(uint16_t sps);
    void setAdcRange(uint8_t range);
    void setPulseWidth(uint16_t us);

    bool readPartId(uint8_t& id);
    bool readRevisionId(uint8_t& rev);

    void reset();

    static constexpr uint8_t  EXPECTED_PART_ID  = 0x15;
    static constexpr std::size_t BUFFER_SIZE     = 32;

private:
    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    void clearFIFO();

    bool readRawSamples(uint32_t* redBuf, uint32_t* irBuf, std::size_t& count);

    void processSamples(const uint32_t* redBuf, const uint32_t* irBuf, std::size_t count,
                        double& heartRateBPM, double& spo2Pct);

    double detectHeartRate(const uint32_t* irBuf, std::size_t count);
    double calculateSpO2(const uint32_t* redBuf, const uint32_t* irBuf, std::size_t count);

    I2CBus&  m_bus;
    uint8_t  m_addr;
    uint16_t m_sampleRate;
    double   m_lastHR;
    double   m_lastSpO2;
    bool     m_initialized;
};
