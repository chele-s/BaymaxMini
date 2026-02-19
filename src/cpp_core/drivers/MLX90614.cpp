#include "MLX90614.h"
#include "I2C_Bus.h"

namespace Reg {
    constexpr uint8_t RAM_AMBIENT  = 0x06;
    constexpr uint8_t RAM_OBJECT1  = 0x07;
    constexpr uint8_t RAM_OBJECT2  = 0x08;
    constexpr uint8_t EEPROM_BASE  = 0x20;
    constexpr uint8_t EMISSIVITY   = 0x24;
    constexpr uint8_t FLAGS        = 0xF0;
}

constexpr double KELVIN_OFFSET = 273.15;
constexpr double RESOLUTION    = 0.02;
constexpr uint16_t ERROR_FLAG  = 0x8000;

MLX90614::MLX90614(I2CBus& bus, uint8_t addr)
    : m_bus(bus)
    , m_addr(addr)
    , m_emissivity(1.0f)
{}

bool MLX90614::init() {
    uint16_t raw;
    if (!readWordSMBus(Reg::EMISSIVITY, raw)) return false;

    m_emissivity = static_cast<float>(raw) / 65535.0f;
    return true;
}

void MLX90614::shutdown() {
}

bool MLX90614::readWordSMBus(uint8_t cmd, uint16_t& out) {
    uint8_t buf[3];
    if (!m_bus.writeReadTransaction(m_addr, &cmd, 1, buf, 3)) return false;

    out = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    return true;
}

bool MLX90614::writeWordSMBus(uint8_t cmd, uint16_t value) {
    uint8_t buf[3] = {
        cmd,
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)
    };
    return m_bus.writeRaw(m_addr, buf, 3);
}

double MLX90614::rawToC(uint16_t raw) const {
    if (raw & ERROR_FLAG) return -999.0;
    return (static_cast<double>(raw) * RESOLUTION) - KELVIN_OFFSET;
}

bool MLX90614::readTemperatures(double& objectC, double& ambientC) {
    uint16_t rawObj, rawAmb;
    if (!readWordSMBus(Reg::RAM_OBJECT1, rawObj)) return false;
    if (!readWordSMBus(Reg::RAM_AMBIENT, rawAmb)) return false;

    objectC  = rawToC(rawObj);
    ambientC = rawToC(rawAmb);
    return true;
}

bool MLX90614::readObjectTemp(double& tempC) {
    uint16_t raw;
    if (!readWordSMBus(Reg::RAM_OBJECT1, raw)) return false;
    tempC = rawToC(raw);
    return true;
}

bool MLX90614::readAmbientTemp(double& tempC) {
    uint16_t raw;
    if (!readWordSMBus(Reg::RAM_AMBIENT, raw)) return false;
    tempC = rawToC(raw);
    return true;
}

bool MLX90614::readRawObject(uint16_t& raw) {
    return readWordSMBus(Reg::RAM_OBJECT1, raw);
}

bool MLX90614::readRawAmbient(uint16_t& raw) {
    return readWordSMBus(Reg::RAM_AMBIENT, raw);
}

float MLX90614::emissivity() const {
    return m_emissivity;
}

bool MLX90614::setEmissivity(float emissivity) {
    if (emissivity < 0.1f || emissivity > 1.0f) return false;

    if (!writeWordSMBus(Reg::EMISSIVITY, 0x0000)) return false;

    uint16_t value = static_cast<uint16_t>(emissivity * 65535.0f);
    if (!writeWordSMBus(Reg::EMISSIVITY, value)) return false;

    m_emissivity = emissivity;
    return true;
}
