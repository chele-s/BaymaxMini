#pragma once

#include <cstdint>
#include <cstring>

namespace sensor {

enum class Type : uint8_t {
    UNKNOWN         = 0,
    PULSE_OXIMETER  = 1,
    TEMPERATURE     = 2,
    DISTANCE        = 3,
    POWER_MONITOR   = 4,
    SERVO_DRIVER    = 5,
    IMU             = 6,
    MICROPHONE      = 7,
    CAMERA          = 8,
    GENERIC         = 255
};

enum class Status : uint8_t {
    UNINITIALIZED,
    READY,
    BUSY,
    DEGRADED,
    ERROR,
    OFFLINE,
    SELF_TEST
};

enum class ErrorCode : uint8_t {
    NONE              = 0,
    INIT_FAILED       = 1,
    COMM_FAILURE      = 2,
    INVALID_DATA      = 3,
    TIMEOUT           = 4,
    CALIBRATION_LOST  = 5,
    SELF_TEST_FAILED  = 6,
    HARDWARE_FAULT    = 7,
    NOT_PRESENT       = 8,
    OVERRANGE         = 9,
    UNDERRANGE        = 10,
    UNKNOWN_ERROR     = 255
};

struct Diagnostics {
    Status    status          = Status::UNINITIALIZED;
    ErrorCode lastError       = ErrorCode::NONE;
    uint32_t  totalReads      = 0;
    uint32_t  failedReads     = 0;
    uint32_t  totalErrors     = 0;
    uint32_t  consecutiveErrs = 0;
    uint32_t  uptimeMs        = 0;
    float     successRate     = 0.0f;
    float     lastReadMs      = 0.0f;
    float     avgReadMs       = 0.0f;
    float     peakReadMs      = 0.0f;

    void recordSuccess(float readTimeMs) {
        ++totalReads;
        consecutiveErrs = 0;
        lastReadMs = readTimeMs;
        avgReadMs += (readTimeMs - avgReadMs) / static_cast<float>(totalReads);
        if (readTimeMs > peakReadMs) peakReadMs = readTimeMs;
        updateRate();
    }

    void recordFailure(ErrorCode err) {
        ++totalReads;
        ++failedReads;
        ++totalErrors;
        ++consecutiveErrs;
        lastError = err;
        updateRate();
    }

    void updateRate() {
        successRate = (totalReads > 0)
            ? static_cast<float>(totalReads - failedReads) / static_cast<float>(totalReads)
            : 0.0f;
    }

    void reset() {
        *this = Diagnostics{};
    }
};

struct Identity {
    char     name[24]     = {};
    Type     type         = Type::UNKNOWN;
    uint8_t  i2cAddress   = 0;
    uint8_t  busId        = 0;
    uint16_t partId       = 0;
    uint8_t  revisionId   = 0;
    uint8_t  instanceId   = 0;

    void setName(const char* n) {
        std::strncpy(name, n, sizeof(name) - 1);
        name[sizeof(name) - 1] = '\0';
    }
};

}

class I_Sensor {
public:
    virtual ~I_Sensor() = default;

    virtual bool init() = 0;
    virtual void shutdown() = 0;
    virtual bool read() = 0;
    virtual void reset() = 0;

    virtual bool selfTest() { return isHealthy(); }

    virtual bool isHealthy() const {
        const auto& d = diagnostics();
        return d.status == sensor::Status::READY &&
               d.consecutiveErrs < 5;
    }

    virtual bool isPresent() const {
        return diagnostics().status != sensor::Status::OFFLINE &&
               diagnostics().status != sensor::Status::UNINITIALIZED;
    }

    virtual const sensor::Identity& identity() const = 0;
    virtual const sensor::Diagnostics& diagnostics() const = 0;

    virtual sensor::Type type() const { return identity().type; }
    virtual const char* name() const { return identity().name; }
    virtual uint8_t address() const { return identity().i2cAddress; }

    virtual sensor::Status status() const { return diagnostics().status; }
    virtual sensor::ErrorCode lastError() const { return diagnostics().lastError; }

    virtual float readLatencyMs() const { return diagnostics().lastReadMs; }
    virtual float successRate() const { return diagnostics().successRate; }

    virtual bool needsCalibration() const { return false; }
    virtual bool calibrate() { return true; }

    virtual void onError(sensor::ErrorCode) {}
    virtual void onRecovery() {}

protected:
    I_Sensor() = default;
    I_Sensor(const I_Sensor&) = default;
    I_Sensor& operator=(const I_Sensor&) = default;
};

class SensorRegistry {
public:
    static constexpr std::size_t MAX_SENSORS = 16;

    SensorRegistry() = default;

    bool registerSensor(I_Sensor* s) {
        if (!s || m_count >= MAX_SENSORS) return false;
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_sensors[i] == s) return false;
        }
        m_sensors[m_count++] = s;
        return true;
    }

    bool unregister(I_Sensor* s) {
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_sensors[i] == s) {
                for (std::size_t j = i; j < m_count - 1; ++j) {
                    m_sensors[j] = m_sensors[j + 1];
                }
                m_sensors[--m_count] = nullptr;
                return true;
            }
        }
        return false;
    }

    bool initAll() {
        bool allOk = true;
        for (std::size_t i = 0; i < m_count; ++i) {
            if (!m_sensors[i]->init()) {
                allOk = false;
            }
        }
        return allOk;
    }

    void shutdownAll() {
        for (std::size_t i = 0; i < m_count; ++i) {
            m_sensors[i]->shutdown();
        }
    }

    void readAll() {
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_sensors[i]->isHealthy()) {
                m_sensors[i]->read();
            }
        }
    }

    void resetFailed() {
        for (std::size_t i = 0; i < m_count; ++i) {
            if (!m_sensors[i]->isHealthy()) {
                m_sensors[i]->reset();
                m_sensors[i]->init();
            }
        }
    }

    bool selfTestAll() {
        bool allOk = true;
        for (std::size_t i = 0; i < m_count; ++i) {
            if (!m_sensors[i]->selfTest()) {
                allOk = false;
            }
        }
        return allOk;
    }

    std::size_t healthyCount() const {
        std::size_t n = 0;
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_sensors[i]->isHealthy()) ++n;
        }
        return n;
    }

    std::size_t failedCount() const {
        return m_count - healthyCount();
    }

    bool allHealthy() const {
        return healthyCount() == m_count;
    }

    I_Sensor* findByType(sensor::Type type) const {
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_sensors[i]->type() == type) return m_sensors[i];
        }
        return nullptr;
    }

    I_Sensor* findByAddress(uint8_t addr) const {
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_sensors[i]->address() == addr) return m_sensors[i];
        }
        return nullptr;
    }

    I_Sensor* findByName(const char* n) const {
        for (std::size_t i = 0; i < m_count; ++i) {
            if (std::strcmp(m_sensors[i]->name(), n) == 0) return m_sensors[i];
        }
        return nullptr;
    }

    I_Sensor* at(std::size_t idx) const {
        return (idx < m_count) ? m_sensors[idx] : nullptr;
    }

    std::size_t count() const { return m_count; }

    template<typename Fn>
    void forEach(Fn&& fn) {
        for (std::size_t i = 0; i < m_count; ++i) {
            fn(m_sensors[i]);
        }
    }

    template<typename Fn>
    void forEachOfType(sensor::Type type, Fn&& fn) {
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_sensors[i]->type() == type) fn(m_sensors[i]);
        }
    }

private:
    I_Sensor*   m_sensors[MAX_SENSORS] = {};
    std::size_t m_count = 0;
};

template<typename Derived>
class SensorBase : public I_Sensor {
public:
    const sensor::Identity& identity() const override { return m_identity; }
    const sensor::Diagnostics& diagnostics() const override { return m_diag; }

protected:
    SensorBase() = default;

    void setIdentity(const char* n, sensor::Type t, uint8_t addr, uint8_t bus = 0) {
        m_identity.setName(n);
        m_identity.type = t;
        m_identity.i2cAddress = addr;
        m_identity.busId = bus;
    }

    void setPartInfo(uint16_t partId, uint8_t revId) {
        m_identity.partId = partId;
        m_identity.revisionId = revId;
    }

    void setStatus(sensor::Status s) { m_diag.status = s; }
    void setError(sensor::ErrorCode e) { m_diag.lastError = e; }

    void recordSuccess(float ms = 0.0f) { m_diag.recordSuccess(ms); }
    void recordFailure(sensor::ErrorCode e) { m_diag.recordFailure(e); }

    void addUptime(float dtMs) { m_diag.uptimeMs += static_cast<uint32_t>(dtMs); }

    sensor::Identity    m_identity;
    sensor::Diagnostics m_diag;
};
