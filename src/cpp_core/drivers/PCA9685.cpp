#include "PCA9685.h"
#include "I2C_Bus.h"

#include <cmath>
#include <algorithm>
#include <unistd.h>

namespace Reg {
    constexpr uint8_t MODE1       = 0x00;
    constexpr uint8_t MODE2       = 0x01;
    constexpr uint8_t LED0_ON_L   = 0x06;
    constexpr uint8_t ALL_LED_ON_L  = 0xFA;
    constexpr uint8_t ALL_LED_OFF_L = 0xFC;
    constexpr uint8_t PRESCALE    = 0xFE;
}

namespace Bit {
    constexpr uint8_t RESTART     = 0x80;
    constexpr uint8_t SLEEP       = 0x10;
    constexpr uint8_t AI          = 0x20;
    constexpr uint8_t ALLCALL     = 0x01;
    constexpr uint8_t OUTDRV      = 0x04;
    constexpr uint8_t FULL_ON     = 0x10;
    constexpr uint8_t FULL_OFF    = 0x10;
}

constexpr float    OSC_CLOCK       = 25000000.0f;
constexpr uint16_t RESOLUTION      = 4096;
constexpr float    TICK_CEILING    = 4095.0f;

PCA9685::PCA9685(I2CBus& bus, uint8_t addr)
    : m_bus(bus)
    , m_addr(addr)
    , m_freq(DEFAULT_FREQ_HZ)
    , m_usPerTick(0.0f)
    , m_minPulseUs(DEFAULT_MIN_PULSE)
    , m_maxPulseUs(DEFAULT_MAX_PULSE)
    , m_minAngleDeg(DEFAULT_MIN_ANGLE)
    , m_maxAngleDeg(DEFAULT_MAX_ANGLE)
{}

bool PCA9685::init() {
    writeReg(Reg::MODE1, Bit::SLEEP);
    usleep(500);

    setFrequency(m_freq);

    writeReg(Reg::MODE2, Bit::OUTDRV);
    writeReg(Reg::MODE1, Bit::AI | Bit::ALLCALL);
    usleep(500);

    uint8_t mode1 = readReg(Reg::MODE1);
    mode1 &= ~Bit::SLEEP;
    writeReg(Reg::MODE1, mode1);
    usleep(500);

    mode1 = readReg(Reg::MODE1);
    if (mode1 & Bit::RESTART) {
        writeReg(Reg::MODE1, mode1 | Bit::RESTART);
    }

    allOff();
    return true;
}

void PCA9685::shutdown() {
    allOff();
    sleep();
}

void PCA9685::setFrequency(float hz) {
    hz = std::clamp(hz, 24.0f, 1526.0f);
    m_freq = hz;

    float prescaleVal = std::round(OSC_CLOCK / (static_cast<float>(RESOLUTION) * hz)) - 1.0f;
    uint8_t prescale = static_cast<uint8_t>(std::clamp(prescaleVal, 3.0f, 255.0f));

    float actualFreq = OSC_CLOCK / (static_cast<float>(RESOLUTION) * (static_cast<float>(prescale) + 1.0f));
    float periodUs = 1000000.0f / actualFreq;
    m_usPerTick = periodUs / static_cast<float>(RESOLUTION);

    uint8_t oldMode = readReg(Reg::MODE1);
    writeReg(Reg::MODE1, (oldMode & 0x7F) | Bit::SLEEP);
    writeReg(Reg::PRESCALE, prescale);
    writeReg(Reg::MODE1, oldMode);
    usleep(500);
    writeReg(Reg::MODE1, oldMode | Bit::RESTART);
}

float PCA9685::frequency() const {
    return m_freq;
}

void PCA9685::writeReg(uint8_t reg, uint8_t value) {
    m_bus.writeByte(m_addr, reg, value);
}

uint8_t PCA9685::readReg(uint8_t reg) {
    uint8_t value = 0;
    m_bus.readByte(m_addr, reg, value);
    return value;
}

void PCA9685::setPWM(uint8_t channel, uint16_t onTick, uint16_t offTick) {
    if (channel >= NUM_CHANNELS) return;

    uint8_t base = Reg::LED0_ON_L + 4 * channel;
    uint8_t data[4] = {
        static_cast<uint8_t>(onTick & 0xFF),
        static_cast<uint8_t>((onTick >> 8) & 0x1F),
        static_cast<uint8_t>(offTick & 0xFF),
        static_cast<uint8_t>((offTick >> 8) & 0x1F)
    };
    m_bus.writeBytes(m_addr, base, data, 4);
}

void PCA9685::setDutyCycle(uint8_t channel, float duty) {
    duty = std::clamp(duty, 0.0f, 1.0f);

    if (duty <= 0.0f) {
        channelOff(channel);
        return;
    }
    if (duty >= 1.0f) {
        channelFullOn(channel);
        return;
    }

    uint16_t offTick = static_cast<uint16_t>(duty * TICK_CEILING);
    setPWM(channel, 0, offTick);
}

void PCA9685::setServoPulseUs(uint8_t channel, float pulseUs) {
    pulseUs = std::clamp(pulseUs, m_minPulseUs, m_maxPulseUs);

    if (m_usPerTick <= 0.0f) return;

    uint16_t ticks = static_cast<uint16_t>(std::round(pulseUs / m_usPerTick));
    ticks = std::min(ticks, static_cast<uint16_t>(RESOLUTION - 1));
    setPWM(channel, 0, ticks);
}

void PCA9685::setServoAngle(uint8_t channel, float angleDeg) {
    angleDeg = std::clamp(angleDeg, m_minAngleDeg, m_maxAngleDeg);

    float range = m_maxAngleDeg - m_minAngleDeg;
    float t = (range > 0.0f) ? (angleDeg - m_minAngleDeg) / range : 0.0f;
    float pulseUs = m_minPulseUs + t * (m_maxPulseUs - m_minPulseUs);

    setServoPulseUs(channel, pulseUs);
}

void PCA9685::setServoNormalized(uint8_t channel, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    float pulseUs = m_minPulseUs + t * (m_maxPulseUs - m_minPulseUs);
    setServoPulseUs(channel, pulseUs);
}

void PCA9685::setServoLimits(float minPulseUs, float maxPulseUs,
                              float minAngleDeg, float maxAngleDeg) {
    m_minPulseUs  = minPulseUs;
    m_maxPulseUs  = maxPulseUs;
    m_minAngleDeg = minAngleDeg;
    m_maxAngleDeg = maxAngleDeg;
}

void PCA9685::allOff() {
    uint8_t data[4] = {0x00, 0x00, 0x00, Bit::FULL_OFF};
    m_bus.writeBytes(m_addr, Reg::ALL_LED_ON_L, data, 4);
}

void PCA9685::channelOff(uint8_t channel) {
    if (channel >= NUM_CHANNELS) return;
    setPWM(channel, 0, 0x1000);
}

void PCA9685::channelFullOn(uint8_t channel) {
    if (channel >= NUM_CHANNELS) return;

    uint8_t base = Reg::LED0_ON_L + 4 * channel;
    uint8_t data[4] = {0x00, Bit::FULL_ON, 0x00, 0x00};
    m_bus.writeBytes(m_addr, base, data, 4);
}

void PCA9685::sleep() {
    uint8_t mode1 = readReg(Reg::MODE1);
    writeReg(Reg::MODE1, mode1 | Bit::SLEEP);
}

void PCA9685::wake() {
    uint8_t mode1 = readReg(Reg::MODE1);
    mode1 &= ~Bit::SLEEP;
    writeReg(Reg::MODE1, mode1);
    usleep(500);

    if (mode1 & Bit::RESTART) {
        writeReg(Reg::MODE1, mode1 | Bit::RESTART);
    }
}
