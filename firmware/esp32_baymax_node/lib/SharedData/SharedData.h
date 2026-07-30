#pragma once

#include <cstdint>
#include <cstring>

#pragma pack(push, 1)

namespace ipc {

constexpr uint32_t PROTOCOL_VERSION = 1;
constexpr uint32_t TELEMETRY_MAGIC  = 0xBABE0001;
constexpr uint32_t COMMAND_MAGIC    = 0xBABE0002;
constexpr uint32_t STATUS_MAGIC     = 0xBABE0003;
constexpr uint32_t HEARTBEAT_MAGIC  = 0xBABE0004;

enum class CommandType : uint8_t {
    NONE            = 0x00,
    SET_EXPRESSION  = 0x01,
    SET_EYELID      = 0x02,
    TRIGGER_BLINK   = 0x03,
    SET_GAZE        = 0x04,
    SET_BREATH      = 0x05,
    SPEAK           = 0x06,
    SET_LED_COLOR   = 0x07,
    PLAY_ANIMATION  = 0x08,
    EMERGENCY_STOP  = 0xFE,
    SHUTDOWN        = 0xFF
};

enum class ExpressionType : uint8_t {
    NEUTRAL     = 0x00,
    HAPPY       = 0x01,
    SAD         = 0x02,
    SURPRISED   = 0x03,
    ANGRY       = 0x04,
    SLEEPY      = 0x05,
    CONCERNED   = 0x06,
    CURIOUS     = 0x07,
    LOVE        = 0x08,
    THINKING    = 0x09
};

enum class SystemStateId : uint8_t {
    BOOT        = 0x00,
    CONNECTED   = 0x01,
    AUTONOMOUS  = 0x02,
    SHUTDOWN    = 0x03,
    ERROR       = 0xFF
};

enum class AlertLevel : uint8_t {
    NONE       = 0x00,
    INFO       = 0x01,
    WARNING    = 0x02,
    CRITICAL   = 0x03
};

struct TelemetryFrame {
    uint32_t magic        = TELEMETRY_MAGIC;
    uint32_t version      = PROTOCOL_VERSION;
    uint64_t timestamp_us = 0;
    uint32_t sequence     = 0;

    float distance_mm     = 0.0f;

    float heart_rate_bpm  = 0.0f;
    float spo2_percent    = 0.0f;
    float skin_temp_c     = 0.0f;
    float ambient_temp_c  = 0.0f;

    float bus_voltage_v   = 0.0f;
    float current_ma      = 0.0f;
    float power_mw        = 0.0f;
    float battery_pct     = 0.0f;

    float eyelid_openness = 1.0f;
    float gaze_x          = 0.0f;
    float gaze_y          = 0.0f;
    float breath_level    = 0.0f;

    uint8_t proximity_valid : 1;
    uint8_t vitals_valid    : 1;
    uint8_t power_valid     : 1;
    uint8_t face_valid      : 1;
    uint8_t _reserved_bits  : 4;

    SystemStateId  state  = SystemStateId::BOOT;
    ExpressionType expression = ExpressionType::NEUTRAL;
    AlertLevel     alert  = AlertLevel::NONE;

    uint8_t _pad[2]       = {0, 0};

    uint32_t checksum     = 0;

    uint32_t computeChecksum() const {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint32_t sum = 0;
        std::size_t len = offsetof(TelemetryFrame, checksum);
        for (std::size_t i = 0; i < len; ++i) {
            sum = ((sum << 5) + sum) + data[i];
        }
        return sum;
    }

    void stamp(uint32_t seq, uint64_t ts) {
        sequence = seq;
        timestamp_us = ts;
        checksum = computeChecksum();
    }

    bool verify() const {
        return magic == TELEMETRY_MAGIC
            && version == PROTOCOL_VERSION
            && checksum == computeChecksum();
    }
};

struct SetExpressionPayload {
    ExpressionType type       = ExpressionType::NEUTRAL;
    float          transition = 0.3f;
};

struct SetEyelidPayload {
    float openness  = 1.0f;
    float duration  = 0.2f;
};

struct SetGazePayload {
    float x         = 0.0f;
    float y         = 0.0f;
    float speed     = 1.0f;
};

struct SetBreathPayload {
    float intensity = 0.5f;
    float cycle_sec = 5.0f;
};

struct SetLedColorPayload {
    uint8_t r       = 255;
    uint8_t g       = 255;
    uint8_t b       = 255;
    uint8_t region  = 0;
    float   fade_sec = 0.5f;
};

struct PlayAnimationPayload {
    uint8_t animation_id = 0;
    float   speed        = 1.0f;
    uint8_t loop         = 0;
};

struct SpeakPayload {
    uint16_t utterance_id  = 0;
    float    volume        = 1.0f;
};

union CommandPayload {
    SetExpressionPayload    expression;
    SetEyelidPayload        eyelid;
    SetGazePayload          gaze;
    SetBreathPayload        breath;
    SetLedColorPayload      led;
    PlayAnimationPayload    animation;
    SpeakPayload            speak;
    uint8_t                 raw[16];

    CommandPayload() { std::memset(raw, 0, sizeof(raw)); }
};

struct CommandFrame {
    uint32_t       magic      = COMMAND_MAGIC;
    uint32_t       version    = PROTOCOL_VERSION;
    uint64_t       timestamp_us = 0;
    uint32_t       sequence   = 0;
    CommandType    type       = CommandType::NONE;
    uint8_t        priority   = 0;
    uint8_t        _pad[2]    = {0, 0};
    CommandPayload payload;
    uint32_t       checksum   = 0;

    uint32_t computeChecksum() const {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint32_t sum = 0;
        std::size_t len = offsetof(CommandFrame, checksum);
        for (std::size_t i = 0; i < len; ++i) {
            sum = ((sum << 5) + sum) + data[i];
        }
        return sum;
    }

    void stamp(uint32_t seq, uint64_t ts) {
        sequence = seq;
        timestamp_us = ts;
        checksum = computeChecksum();
    }

    bool verify() const {
        return magic == COMMAND_MAGIC
            && version == PROTOCOL_VERSION
            && checksum == computeChecksum();
    }
};

struct SystemStatus {
    uint32_t      magic        = STATUS_MAGIC;
    uint32_t      version      = PROTOCOL_VERSION;
    uint64_t      uptime_us    = 0;
    SystemStateId state        = SystemStateId::BOOT;
    AlertLevel    alert        = AlertLevel::NONE;
    uint8_t       cpu_temp_c   = 0;
    uint8_t       cpu_load_pct = 0;
    uint32_t      tick_count   = 0;
    uint32_t      overrun_count = 0;
    float         avg_tick_ms  = 0.0f;
    float         max_tick_ms  = 0.0f;
    uint8_t       i2c_errors   = 0;
    uint8_t       zmq_errors   = 0;
    uint8_t       _pad[2]      = {0, 0};

    uint32_t      checksum     = 0;

    uint32_t computeChecksum() const {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint32_t sum = 0;
        std::size_t len = offsetof(SystemStatus, checksum);
        for (std::size_t i = 0; i < len; ++i) {
            sum = ((sum << 5) + sum) + data[i];
        }
        return sum;
    }

    void stamp() {
        checksum = computeChecksum();
    }

    bool verify() const {
        return magic == STATUS_MAGIC
            && version == PROTOCOL_VERSION
            && checksum == computeChecksum();
    }
};

struct Heartbeat {
    uint32_t      magic       = HEARTBEAT_MAGIC;
    uint64_t      timestamp_us = 0;
    SystemStateId source      = SystemStateId::BOOT;
    uint8_t       _pad[3]     = {0, 0, 0};
};

static_assert(sizeof(TelemetryFrame) % 4 == 0, "TelemetryFrame must be 4-byte aligned");
static_assert(sizeof(CommandFrame) % 4 == 0, "CommandFrame must be 4-byte aligned");
static_assert(sizeof(SystemStatus) % 4 == 0, "SystemStatus must be 4-byte aligned");

template<typename T>
inline bool fromBytes(const uint8_t* data, std::size_t len, T& out) {
    if (len < sizeof(T)) return false;
    std::memcpy(&out, data, sizeof(T));
    return true;
}

template<typename T>
inline std::size_t toBytes(const T& frame, uint8_t* buf, std::size_t bufLen) {
    if (bufLen < sizeof(T)) return 0;
    std::memcpy(buf, &frame, sizeof(T));
    return sizeof(T);
}

}

#pragma pack(pop)
