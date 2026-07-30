#include "I2CBus.h"

I2CBus::I2CBus(int sda, int scl, uint32_t freqHz)
    : m_sda(sda)
    , m_scl(scl)
    , m_freq(freqHz)
    , m_wire(&Wire)
    , m_mutex(nullptr)
    , m_errors(0)
    , m_open(false)
{}

I2CBus::~I2CBus() {
    close();
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

bool I2CBus::open() {
    if (m_open) return true;

    m_mutex = xSemaphoreCreateMutex();
    if (!m_mutex) return false;

    m_wire->begin(m_sda, m_scl, m_freq);
    m_open = true;
    return true;
}

void I2CBus::close() {
    if (m_open) {
        m_wire->end();
        m_open = false;
    }
}

bool I2CBus::isOpen() const {
    return m_open;
}

bool I2CBus::writeByte(uint8_t addr, uint8_t reg, uint8_t value) {
    if (!m_open) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    m_wire->write(value);
    uint8_t err = m_wire->endTransmission();

    xSemaphoreGive(m_mutex);
    if (err != 0) { ++m_errors; return false; }
    return true;
}

bool I2CBus::writeWord(uint8_t addr, uint8_t reg, uint16_t value) {
    if (!m_open) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    m_wire->write(static_cast<uint8_t>((value >> 8) & 0xFF));
    m_wire->write(static_cast<uint8_t>(value & 0xFF));
    uint8_t err = m_wire->endTransmission();

    xSemaphoreGive(m_mutex);
    if (err != 0) { ++m_errors; return false; }
    return true;
}

bool I2CBus::writeBytes(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len) {
    if (!m_open || len > 128) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    m_wire->write(data, len);
    uint8_t err = m_wire->endTransmission();

    xSemaphoreGive(m_mutex);
    if (err != 0) { ++m_errors; return false; }
    return true;
}

bool I2CBus::writeRaw(uint8_t addr, const uint8_t* data, std::size_t len) {
    if (!m_open || len > 128) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    m_wire->beginTransmission(addr);
    m_wire->write(data, len);
    uint8_t err = m_wire->endTransmission();

    xSemaphoreGive(m_mutex);
    if (err != 0) { ++m_errors; return false; }
    return true;
}

bool I2CBus::readByte(uint8_t addr, uint8_t reg, uint8_t& out) {
    if (!m_open) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    uint8_t err = m_wire->endTransmission(false);
    if (err != 0) {
        xSemaphoreGive(m_mutex);
        ++m_errors;
        return false;
    }

    uint8_t count = m_wire->requestFrom(addr, static_cast<uint8_t>(1));
    if (count < 1) {
        xSemaphoreGive(m_mutex);
        ++m_errors;
        return false;
    }

    out = m_wire->read();
    xSemaphoreGive(m_mutex);
    return true;
}

bool I2CBus::readWord(uint8_t addr, uint8_t reg, uint16_t& out) {
    if (!m_open) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    uint8_t err = m_wire->endTransmission(false);
    if (err != 0) {
        xSemaphoreGive(m_mutex);
        ++m_errors;
        return false;
    }

    uint8_t count = m_wire->requestFrom(addr, static_cast<uint8_t>(2));
    if (count < 2) {
        xSemaphoreGive(m_mutex);
        ++m_errors;
        return false;
    }

    uint8_t hi = m_wire->read();
    uint8_t lo = m_wire->read();
    out = (static_cast<uint16_t>(hi) << 8) | lo;

    xSemaphoreGive(m_mutex);
    return true;
}

bool I2CBus::readBytes(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len) {
    if (!m_open || len > 128) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    uint8_t err = m_wire->endTransmission(false);
    if (err != 0) {
        xSemaphoreGive(m_mutex);
        ++m_errors;
        return false;
    }

    uint8_t count = m_wire->requestFrom(addr, static_cast<uint8_t>(len));
    if (count < len) {
        xSemaphoreGive(m_mutex);
        ++m_errors;
        return false;
    }

    for (std::size_t i = 0; i < len; ++i) {
        out[i] = m_wire->read();
    }

    xSemaphoreGive(m_mutex);
    return true;
}

bool I2CBus::readRaw(uint8_t addr, uint8_t* out, std::size_t len) {
    if (!m_open || len > 128) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    uint8_t count = m_wire->requestFrom(addr, static_cast<uint8_t>(len));
    if (count < len) {
        xSemaphoreGive(m_mutex);
        ++m_errors;
        return false;
    }

    for (std::size_t i = 0; i < len; ++i) {
        out[i] = m_wire->read();
    }

    xSemaphoreGive(m_mutex);
    return true;
}

bool I2CBus::writeReadTransaction(uint8_t addr,
                                   const uint8_t* txBuf, std::size_t txLen,
                                   uint8_t* rxBuf, std::size_t rxLen)
{
    if (!m_open) { ++m_errors; return false; }
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    if (txBuf && txLen > 0) {
        m_wire->beginTransmission(addr);
        m_wire->write(txBuf, txLen);
        uint8_t err = m_wire->endTransmission(rxLen > 0 ? false : true);
        if (err != 0) {
            xSemaphoreGive(m_mutex);
            ++m_errors;
            return false;
        }
    }

    if (rxBuf && rxLen > 0) {
        uint8_t count = m_wire->requestFrom(addr, static_cast<uint8_t>(rxLen));
        if (count < rxLen) {
            xSemaphoreGive(m_mutex);
            ++m_errors;
            return false;
        }
        for (std::size_t i = 0; i < rxLen; ++i) {
            rxBuf[i] = m_wire->read();
        }
    }

    xSemaphoreGive(m_mutex);
    return true;
}

uint32_t I2CBus::errorCount() const {
    return m_errors;
}

void I2CBus::resetErrorCount() {
    m_errors = 0;
}

SemaphoreHandle_t I2CBus::mutexHandle() {
    return m_mutex;
}
