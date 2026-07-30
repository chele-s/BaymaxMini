#pragma once

#include <cstdint>

class I2CBus;

class PCA9685 {
public:
    PCA9685(I2CBus& bus, uint8_t addr);

    bool init();
    void shutdown();

    void setFrequency(float hz);
    float frequency() const;

    void setPWM(uint8_t channel, uint16_t onTick, uint16_t offTick);
    void setDutyCycle(uint8_t channel, float duty);

    void setServoPulseUs(uint8_t channel, float pulseUs);
    void setServoAngle(uint8_t channel, float angleDeg);
    void setServoNormalized(uint8_t channel, float t);

    void setServoLimits(float minPulseUs, float maxPulseUs,
                        float minAngleDeg, float maxAngleDeg);

    void allOff();
    void channelOff(uint8_t channel);
    void channelFullOn(uint8_t channel);

    void sleep();
    void wake();

    static constexpr uint8_t  NUM_CHANNELS     = 16;
    static constexpr float    DEFAULT_FREQ_HZ   = 50.0f;
    static constexpr float    DEFAULT_MIN_PULSE  = 500.0f;
    static constexpr float    DEFAULT_MAX_PULSE  = 2400.0f;
    static constexpr float    DEFAULT_MIN_ANGLE  = 0.0f;
    static constexpr float    DEFAULT_MAX_ANGLE  = 180.0f;

private:
    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    I2CBus&  m_bus;
    uint8_t  m_addr;
    float    m_freq;
    float    m_usPerTick;
    float    m_minPulseUs;
    float    m_maxPulseUs;
    float    m_minAngleDeg;
    float    m_maxAngleDeg;
};
