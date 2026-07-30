#pragma once

#include <cstdint>

class I2CBus;

class VL53L1X {
public:
    enum class DistanceMode : uint8_t {
        SHORT = 1,
        LONG  = 2
    };

    enum class RangeStatus : uint8_t {
        VALID                 = 0,
        SIGMA_FAIL            = 1,
        SIGNAL_FAIL           = 2,
        OUT_OF_BOUNDS         = 4,
        HARDWARE_FAIL         = 5,
        WRAP_AROUND           = 7,
        INTERNAL_ERROR        = 8,
        NO_MEASUREMENT        = 14,
        NOT_UPDATED           = 255
    };

    struct ROI {
        uint8_t width   = 16;
        uint8_t height  = 16;
        uint8_t center  = 199;
    };

    struct RangingResult {
        uint16_t    distanceMM       = 0;
        uint16_t    signalRateKcps   = 0;
        uint16_t    ambientRateKcps  = 0;
        uint16_t    spadCount         = 0;
        RangeStatus status           = RangeStatus::NOT_UPDATED;
        bool        valid            = false;
    };

    VL53L1X(I2CBus& bus, uint8_t addr);

    bool init();
    void shutdown();

    bool startRanging();
    bool stopRanging();
    bool dataReady(bool& ready);
    bool readDistance(uint16_t& distanceMM);
    bool readResult(RangingResult& result);
    bool clearInterrupt();

    bool setDistanceMode(DistanceMode mode);
    bool setTimingBudgetMs(uint16_t budgetMs);
    bool setInterMeasurementMs(uint16_t periodMs);

    bool setROI(uint8_t width, uint8_t height);
    bool setROICenter(uint8_t center);
    bool getROI(ROI& roi);

    bool setSignalThreshold(uint16_t kcps);
    bool setSigmaThreshold(uint16_t mm);

    bool setI2CAddress(uint8_t newAddr);
    bool getSensorId(uint16_t& id);

    DistanceMode  currentDistanceMode() const;
    uint16_t      currentTimingBudget() const;
    uint16_t      lastDistance() const;
    RangeStatus   lastStatus() const;

private:
    void writeReg(uint16_t reg, uint8_t value);
    void writeReg16(uint16_t reg, uint16_t value);
    void writeReg32(uint16_t reg, uint32_t value);
    bool readReg(uint16_t reg, uint8_t& value);
    bool readReg16(uint16_t reg, uint16_t& value);
    bool waitBoot(uint16_t timeoutMs = 1000);
    bool writeDefaultConfig();

    I2CBus&      m_bus;
    uint8_t      m_addr;
    DistanceMode m_distMode;
    uint16_t     m_timingBudget;
    uint16_t     m_lastDistance;
    RangeStatus  m_lastStatus;
    bool         m_ranging;
};
