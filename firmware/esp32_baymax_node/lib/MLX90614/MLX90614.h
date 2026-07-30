#pragma once

#include <cstdint>

class I2CBus;

class MLX90614 {
public:
    MLX90614(I2CBus& bus, uint8_t addr);

    bool init();
    void shutdown();

    bool readTemperatures(double& objectC, double& ambientC);
    bool readObjectTemp(double& tempC);
    bool readAmbientTemp(double& tempC);
    bool readRawObject(uint16_t& raw);
    bool readRawAmbient(uint16_t& raw);

    float emissivity() const;
    bool  setEmissivity(float emissivity);

private:
    bool readWordSMBus(uint8_t cmd, uint16_t& out);
    bool writeWordSMBus(uint8_t cmd, uint16_t value);
    double rawToC(uint16_t raw) const;

    I2CBus& m_bus;
    uint8_t m_addr;
    float   m_emissivity;
};
