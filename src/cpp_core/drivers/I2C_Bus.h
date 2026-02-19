#pragma once

#include <cstdint>
#include <cstddef>
#include <mutex>

class I2CBus {
public:
    explicit I2CBus(int busId);
    ~I2CBus();

    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

    bool open();
    void close();
    bool isOpen() const;

    bool writeByte(uint8_t addr, uint8_t reg, uint8_t value);
    bool writeWord(uint8_t addr, uint8_t reg, uint16_t value);
    bool writeBytes(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len);
    bool writeRaw(uint8_t addr, const uint8_t* data, std::size_t len);

    bool readByte(uint8_t addr, uint8_t reg, uint8_t& out);
    bool readWord(uint8_t addr, uint8_t reg, uint16_t& out);
    bool readBytes(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len);
    bool readRaw(uint8_t addr, uint8_t* out, std::size_t len);

    bool writeReadTransaction(uint8_t addr,
                              const uint8_t* txBuf, std::size_t txLen,
                              uint8_t* rxBuf, std::size_t rxLen);

    uint32_t errorCount() const;
    void     resetErrorCount();

    std::mutex& mutex();

private:
    int         m_busId;
    int         m_fd;
    std::mutex  m_mutex;
    uint32_t    m_errors;

    bool transfer(uint8_t addr,
                  const uint8_t* txBuf, std::size_t txLen,
                  uint8_t* rxBuf, std::size_t rxLen);
};
