#include "I2C_Bus.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cstdio>
#include <cstring>

I2CBus::I2CBus(int busId)
    : m_busId(busId)
    , m_fd(-1)
    , m_errors(0)
{}

I2CBus::~I2CBus() {
    close();
}

bool I2CBus::open() {
    if (m_fd >= 0) return true;

    char path[32];
    std::snprintf(path, sizeof(path), "/dev/i2c-%d", m_busId);

    m_fd = ::open(path, O_RDWR);
    return m_fd >= 0;
}

void I2CBus::close() {
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool I2CBus::isOpen() const {
    return m_fd >= 0;
}

bool I2CBus::transfer(uint8_t addr,
                      const uint8_t* txBuf, std::size_t txLen,
                      uint8_t* rxBuf, std::size_t rxLen)
{
    if (m_fd < 0) {
        ++m_errors;
        return false;
    }

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data ioData;
    int msgCount = 0;

    if (txBuf && txLen > 0) {
        msgs[msgCount].addr  = addr;
        msgs[msgCount].flags = 0;
        msgs[msgCount].len   = static_cast<__u16>(txLen);
        msgs[msgCount].buf   = const_cast<uint8_t*>(txBuf);
        ++msgCount;
    }

    if (rxBuf && rxLen > 0) {
        msgs[msgCount].addr  = addr;
        msgs[msgCount].flags = I2C_M_RD;
        msgs[msgCount].len   = static_cast<__u16>(rxLen);
        msgs[msgCount].buf   = rxBuf;
        ++msgCount;
    }

    if (msgCount == 0) return false;

    ioData.msgs  = msgs;
    ioData.nmsgs = static_cast<__u32>(msgCount);

    int ret = ::ioctl(m_fd, I2C_RDWR, &ioData);
    if (ret < 0) {
        ++m_errors;
        return false;
    }

    return true;
}

bool I2CBus::writeByte(uint8_t addr, uint8_t reg, uint8_t value) {
    std::lock_guard<std::mutex> lock(m_mutex);
    uint8_t buf[2] = {reg, value};
    return transfer(addr, buf, 2, nullptr, 0);
}

bool I2CBus::writeWord(uint8_t addr, uint8_t reg, uint16_t value) {
    std::lock_guard<std::mutex> lock(m_mutex);
    uint8_t buf[3] = {
        reg,
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF)
    };
    return transfer(addr, buf, 3, nullptr, 0);
}

bool I2CBus::writeBytes(uint8_t addr, uint8_t reg, const uint8_t* data, std::size_t len) {
    std::lock_guard<std::mutex> lock(m_mutex);

    constexpr std::size_t MAX_TX = 256;
    if (len + 1 > MAX_TX) {
        ++m_errors;
        return false;
    }

    uint8_t buf[MAX_TX];
    buf[0] = reg;
    std::memcpy(buf + 1, data, len);
    return transfer(addr, buf, len + 1, nullptr, 0);
}

bool I2CBus::writeRaw(uint8_t addr, const uint8_t* data, std::size_t len) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return transfer(addr, data, len, nullptr, 0);
}

bool I2CBus::readByte(uint8_t addr, uint8_t reg, uint8_t& out) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return transfer(addr, &reg, 1, &out, 1);
}

bool I2CBus::readWord(uint8_t addr, uint8_t reg, uint16_t& out) {
    std::lock_guard<std::mutex> lock(m_mutex);
    uint8_t buf[2] = {0, 0};
    bool ok = transfer(addr, &reg, 1, buf, 2);
    if (ok) {
        out = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    }
    return ok;
}

bool I2CBus::readBytes(uint8_t addr, uint8_t reg, uint8_t* out, std::size_t len) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return transfer(addr, &reg, 1, out, len);
}

bool I2CBus::readRaw(uint8_t addr, uint8_t* out, std::size_t len) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return transfer(addr, nullptr, 0, out, len);
}

bool I2CBus::writeReadTransaction(uint8_t addr,
                                   const uint8_t* txBuf, std::size_t txLen,
                                   uint8_t* rxBuf, std::size_t rxLen)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return transfer(addr, txBuf, txLen, rxBuf, rxLen);
}

uint32_t I2CBus::errorCount() const {
    return m_errors;
}

void I2CBus::resetErrorCount() {
    m_errors = 0;
}

std::mutex& I2CBus::mutex() {
    return m_mutex;
}
