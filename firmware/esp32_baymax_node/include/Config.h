#pragma once

#include <cstdint>

namespace cfg {

constexpr double   TICK_RATE_HZ       = 50.0;
constexpr int64_t  TICK_PERIOD_US     = static_cast<int64_t>(1'000'000.0 / TICK_RATE_HZ);
constexpr float    TICK_DT            = static_cast<float>(1.0 / TICK_RATE_HZ);

constexpr int      I2C_SDA_PIN        = 8;
constexpr int      I2C_SCL_PIN        = 9;
constexpr uint32_t I2C_FREQ_HZ        = 400000;

constexpr uint8_t  PCA9685_ADDR       = 0x40;
constexpr uint8_t  VL53L1X_ADDR       = 0x29;
constexpr uint8_t  MAX30102_ADDR      = 0x57;
constexpr uint8_t  MLX90614_ADDR      = 0x5A;
constexpr uint8_t  INA219_ADDR        = 0x41;

constexpr long     SERIAL_BAUD        = 921600;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 500;

constexpr double   BRAIN_TIMEOUT_SEC  = 2.0;
constexpr double   BLINK_INTERVAL_SEC = 4.0;
constexpr double   BLINK_DURATION_SEC = 0.15;
constexpr double   BREATH_CYCLE_SEC   = 5.0;

constexpr int      WIFI_CHANNEL       = 1;
constexpr int      CAM_MJPEG_PORT     = 81;
constexpr int      CAM_FRAME_SIZE     = 10;
constexpr int      CAM_JPEG_QUALITY   = 12;
constexpr int      CAM_FB_COUNT       = 2;

constexpr int      CAM_PIN_PWDN       = -1;
constexpr int      CAM_PIN_RESET      = -1;
constexpr int      CAM_PIN_XCLK       = 15;
constexpr int      CAM_PIN_SIOD       = 4;
constexpr int      CAM_PIN_SIOC       = 5;
constexpr int      CAM_PIN_D7         = 16;
constexpr int      CAM_PIN_D6         = 17;
constexpr int      CAM_PIN_D5         = 18;
constexpr int      CAM_PIN_D4         = 12;
constexpr int      CAM_PIN_D3         = 10;
constexpr int      CAM_PIN_D2         = 8;
constexpr int      CAM_PIN_D1         = 9;
constexpr int      CAM_PIN_D0         = 11;
constexpr int      CAM_PIN_VSYNC      = 6;
constexpr int      CAM_PIN_HREF       = 7;
constexpr int      CAM_PIN_PCLK       = 13;

namespace batt {
    constexpr float NOMINAL_VOLTAGE    = 11.1f;
    constexpr float FULL_VOLTAGE       = 12.6f;
    constexpr float EMPTY_VOLTAGE      = 9.0f;
    constexpr float CAPACITY_MAH       = 2200.0f;
    constexpr uint8_t CELL_COUNT       = 3;
}

namespace prot {
    constexpr float OVERCURRENT_TRIP_MA  = 3000.0f;
    constexpr float OVERCURRENT_CLEAR_MA = 2500.0f;
    constexpr float SERVO_STALL_TRIP_MA  = 2000.0f;
    constexpr float SERVO_STALL_CLEAR_MA = 1200.0f;
}

namespace ina {
    constexpr float SHUNT_RESISTANCE_OHM = 0.1f;
    constexpr float MAX_EXPECTED_CURRENT  = 3.2f;
}

}
