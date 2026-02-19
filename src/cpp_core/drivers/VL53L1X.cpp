#include "VL53L1X.h"
#include "I2C_Bus.h"

#include <cstring>
#include <algorithm>
#include <unistd.h>

namespace Reg {
    constexpr uint16_t SOFT_RESET                       = 0x0000;
    constexpr uint16_t I2C_SLAVE_DEVICE_ADDRESS         = 0x0001;
    constexpr uint16_t OSC_MEASURED_FAST_OSC_FREQ       = 0x0006;
    constexpr uint16_t VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
    constexpr uint16_t GPIO_HV_MUX_CTRL                 = 0x0030;
    constexpr uint16_t GPIO_TIO_HV_STATUS               = 0x0031;
    constexpr uint16_t SYSTEM_INTERRUPT_CLEAR            = 0x0086;
    constexpr uint16_t SYSTEM_MODE_START                 = 0x0087;
    constexpr uint16_t RESULT_RANGE_STATUS               = 0x0089;
    constexpr uint16_t RESULT_DSS_ACTUAL_EFFECTIVE_SPADS = 0x008C;
    constexpr uint16_t RESULT_AMBIENT_COUNT_RATE_MCPS    = 0x0090;
    constexpr uint16_t RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM = 0x0096;
    constexpr uint16_t RESULT_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS = 0x0098;
    constexpr uint16_t FIRMWARE_SYSTEM_STATUS            = 0x00E5;
    constexpr uint16_t IDENTIFICATION_MODEL_ID           = 0x010F;
    constexpr uint16_t ROI_CONFIG_USER_ROI_CENTRE_SPAD   = 0x007F;
    constexpr uint16_t ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080;
    constexpr uint16_t RANGE_CONFIG_SIGMA_THRESH         = 0x0064;
    constexpr uint16_t RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066;
    constexpr uint16_t SYSTEM_THRESH_HIGH                = 0x0072;
    constexpr uint16_t SYSTEM_THRESH_LOW                 = 0x0074;
    constexpr uint16_t SD_CONFIG_WOI_SD0                 = 0x0078;
    constexpr uint16_t SD_CONFIG_WOI_SD1                 = 0x0079;
    constexpr uint16_t SD_CONFIG_INITIAL_PHASE_SD0       = 0x007A;
    constexpr uint16_t SD_CONFIG_INITIAL_PHASE_SD1       = 0x007B;
    constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_A     = 0x005E;
    constexpr uint16_t RANGE_CONFIG_TIMEOUT_MACROP_B     = 0x0061;
    constexpr uint16_t RANGE_CONFIG_VCSEL_PERIOD_A       = 0x0060;
    constexpr uint16_t RANGE_CONFIG_VCSEL_PERIOD_B       = 0x0063;
    constexpr uint16_t RANGE_CONFIG_VALID_PHASE_LOW      = 0x0069;
    constexpr uint16_t RANGE_CONFIG_VALID_PHASE_HIGH     = 0x0068;
    constexpr uint16_t INTERMEASUREMENT_MS               = 0x006C;
    constexpr uint16_t SYSTEM_SEED_CONFIG                = 0x0077;
    constexpr uint16_t PHASECAL_CONFIG_TIMEOUT_MACROP    = 0x004B;
}

static const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
    0x00,
    0x00,
    0x00,
    0x01,
    0x02,
    0x00,
    0x02,
    0x08,
    0x00,
    0x08,
    0x10,
    0x01,
    0x01,
    0x00,
    0x00,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x0F,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x20,
    0x0B,
    0x00,
    0x00,
    0x02,
    0x0A,
    0x21,
    0x00,
    0x00,
    0x05,
    0x00,
    0x00,
    0x00,
    0x00,
    0xC8,
    0x00,
    0x00,
    0x38,
    0xFF,
    0x01,
    0x00,
    0x08,
    0x00,
    0x00,
    0x01,
    0xDB,
    0x0F,
    0x01,
    0xF1,
    0x0D,
    0x01,
    0x68,
    0x00,
    0x80,
    0x08,
    0xB8,
    0x00,
    0x00,
    0x00,
    0x00,
    0x0F,
    0x89,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x0F,
    0x0D,
    0x0E,
    0x0E,
    0x00,
    0x00,
    0x02,
    0xC7,
    0xFF,
    0x9B,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x00
};

constexpr uint16_t DEFAULT_CONFIG_START_REG = 0x002D;
constexpr std::size_t DEFAULT_CONFIG_SIZE = sizeof(VL51L1X_DEFAULT_CONFIGURATION);

static const uint8_t RANGE_STATUS_LUT[] = {
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::HARDWARE_FAIL),
    static_cast<uint8_t>(VL53L1X::RangeStatus::SIGMA_FAIL),
    static_cast<uint8_t>(VL53L1X::RangeStatus::OUT_OF_BOUNDS),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::WRAP_AROUND),
    static_cast<uint8_t>(VL53L1X::RangeStatus::INTERNAL_ERROR),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::SIGNAL_FAIL),
    static_cast<uint8_t>(VL53L1X::RangeStatus::SIGNAL_FAIL),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::VALID),
    static_cast<uint8_t>(VL53L1X::RangeStatus::NO_MEASUREMENT),
};

VL53L1X::VL53L1X(I2CBus& bus, uint8_t addr)
    : m_bus(bus)
    , m_addr(addr)
    , m_distMode(DistanceMode::LONG)
    , m_timingBudget(100)
    , m_lastDistance(0)
    , m_lastStatus(RangeStatus::NOT_UPDATED)
    , m_ranging(false)
{}

void VL53L1X::writeReg(uint16_t reg, uint8_t value) {
    uint8_t buf[3];
    buf[0] = static_cast<uint8_t>(reg >> 8);
    buf[1] = static_cast<uint8_t>(reg & 0xFF);
    buf[2] = value;
    m_bus.writeRaw(m_addr, buf, 3);
}

void VL53L1X::writeReg16(uint16_t reg, uint16_t value) {
    uint8_t buf[4];
    buf[0] = static_cast<uint8_t>(reg >> 8);
    buf[1] = static_cast<uint8_t>(reg & 0xFF);
    buf[2] = static_cast<uint8_t>(value >> 8);
    buf[3] = static_cast<uint8_t>(value & 0xFF);
    m_bus.writeRaw(m_addr, buf, 4);
}

void VL53L1X::writeReg32(uint16_t reg, uint32_t value) {
    uint8_t buf[6];
    buf[0] = static_cast<uint8_t>(reg >> 8);
    buf[1] = static_cast<uint8_t>(reg & 0xFF);
    buf[2] = static_cast<uint8_t>((value >> 24) & 0xFF);
    buf[3] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buf[4] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[5] = static_cast<uint8_t>(value & 0xFF);
    m_bus.writeRaw(m_addr, buf, 6);
}

bool VL53L1X::readReg(uint16_t reg, uint8_t& value) {
    uint8_t regBuf[2];
    regBuf[0] = static_cast<uint8_t>(reg >> 8);
    regBuf[1] = static_cast<uint8_t>(reg & 0xFF);

    uint8_t result = 0;
    bool ok = m_bus.writeReadTransaction(m_addr, regBuf, 2, &result, 1);
    if (ok) value = result;
    return ok;
}

bool VL53L1X::readReg16(uint16_t reg, uint16_t& value) {
    uint8_t regBuf[2];
    regBuf[0] = static_cast<uint8_t>(reg >> 8);
    regBuf[1] = static_cast<uint8_t>(reg & 0xFF);

    uint8_t result[2] = {0, 0};
    bool ok = m_bus.writeReadTransaction(m_addr, regBuf, 2, result, 2);
    if (ok) value = (static_cast<uint16_t>(result[0]) << 8) | result[1];
    return ok;
}

bool VL53L1X::waitBoot(uint16_t timeoutMs) {
    uint16_t elapsed = 0;
    while (elapsed < timeoutMs) {
        uint8_t state = 0;
        if (readReg(Reg::FIRMWARE_SYSTEM_STATUS, state) && (state & 0x01)) {
            return true;
        }
        usleep(2000);
        elapsed += 2;
    }
    return false;
}

bool VL53L1X::writeDefaultConfig() {
    uint8_t buf[DEFAULT_CONFIG_SIZE + 2];
    buf[0] = static_cast<uint8_t>(DEFAULT_CONFIG_START_REG >> 8);
    buf[1] = static_cast<uint8_t>(DEFAULT_CONFIG_START_REG & 0xFF);
    std::memcpy(buf + 2, VL51L1X_DEFAULT_CONFIGURATION, DEFAULT_CONFIG_SIZE);
    return m_bus.writeRaw(m_addr, buf, DEFAULT_CONFIG_SIZE + 2);
}

bool VL53L1X::init() {
    writeReg(Reg::SOFT_RESET, 0x00);
    usleep(1000);
    writeReg(Reg::SOFT_RESET, 0x01);
    usleep(1000);

    if (!waitBoot()) return false;

    if (!writeDefaultConfig()) return false;

    writeReg(Reg::GPIO_HV_MUX_CTRL, 0x01);
    writeReg(Reg::GPIO_TIO_HV_STATUS, 0x01);

    clearInterrupt();

    writeReg(Reg::SYSTEM_MODE_START, 0x40);
    uint8_t tmp = 0;
    uint16_t timeout = 0;
    while (timeout < 1000) {
        readReg(Reg::GPIO_TIO_HV_STATUS, tmp);
        if ((tmp & 0x01) == 0) break;
        usleep(1000);
        timeout++;
    }
    clearInterrupt();
    writeReg(Reg::SYSTEM_MODE_START, 0x00);

    writeReg(Reg::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    writeReg(0x000B, 0x00);

    setDistanceMode(DistanceMode::LONG);
    setTimingBudgetMs(100);
    setInterMeasurementMs(0);

    uint16_t osc = 0;
    readReg16(Reg::OSC_MEASURED_FAST_OSC_FREQ, osc);

    return true;
}

void VL53L1X::shutdown() {
    if (m_ranging) stopRanging();
    writeReg(Reg::SOFT_RESET, 0x00);
}

bool VL53L1X::startRanging() {
    clearInterrupt();
    writeReg(Reg::SYSTEM_MODE_START, 0x40);
    m_ranging = true;
    return true;
}

bool VL53L1X::stopRanging() {
    writeReg(Reg::SYSTEM_MODE_START, 0x00);
    m_ranging = false;
    return true;
}

bool VL53L1X::dataReady(bool& ready) {
    uint8_t intPol = 0;
    readReg(Reg::GPIO_HV_MUX_CTRL, intPol);
    intPol = (intPol & 0x10) >> 4;

    uint8_t gpioStatus = 0;
    if (!readReg(Reg::GPIO_TIO_HV_STATUS, gpioStatus)) return false;

    ready = ((gpioStatus & 0x01) == (intPol ^ 0x01));
    return true;
}

bool VL53L1X::clearInterrupt() {
    writeReg(Reg::SYSTEM_INTERRUPT_CLEAR, 0x01);
    return true;
}

bool VL53L1X::readDistance(uint16_t& distanceMM) {
    uint16_t raw = 0;
    if (!readReg16(Reg::RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM, raw)) return false;
    distanceMM = raw;
    m_lastDistance = raw;
    return true;
}

bool VL53L1X::readResult(RangingResult& result) {
    uint8_t regAddr[2];
    regAddr[0] = static_cast<uint8_t>(Reg::RESULT_RANGE_STATUS >> 8);
    regAddr[1] = static_cast<uint8_t>(Reg::RESULT_RANGE_STATUS & 0xFF);

    uint8_t buf[17] = {};
    if (!m_bus.writeReadTransaction(m_addr, regAddr, 2, buf, 17)) return false;

    uint8_t rawStatus = buf[0];
    uint8_t statusIdx = rawStatus >> 3;
    if (statusIdx < sizeof(RANGE_STATUS_LUT)) {
        result.status = static_cast<RangeStatus>(RANGE_STATUS_LUT[statusIdx]);
    } else {
        result.status = RangeStatus::INTERNAL_ERROR;
    }

    result.spadCount = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];

    result.ambientRateKcps = (static_cast<uint16_t>(buf[7]) << 8) | buf[8];

    result.distanceMM = (static_cast<uint16_t>(buf[13]) << 8) | buf[14];

    result.signalRateKcps = (static_cast<uint16_t>(buf[15]) << 8) | buf[16];

    result.valid = (result.status == RangeStatus::VALID);

    m_lastDistance = result.distanceMM;
    m_lastStatus = result.status;

    return true;
}

bool VL53L1X::setDistanceMode(DistanceMode mode) {
    uint16_t budget = m_timingBudget;

    if (mode == DistanceMode::SHORT) {
        writeReg(Reg::PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14);
        writeReg(Reg::RANGE_CONFIG_VCSEL_PERIOD_A, 0x07);
        writeReg(Reg::RANGE_CONFIG_VCSEL_PERIOD_B, 0x05);
        writeReg(Reg::RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg16(Reg::SD_CONFIG_WOI_SD0, 0x0705);
        writeReg16(Reg::SD_CONFIG_INITIAL_PHASE_SD0, 0x0606);
    } else {
        writeReg(Reg::PHASECAL_CONFIG_TIMEOUT_MACROP, 0x0A);
        writeReg(Reg::RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);
        writeReg(Reg::RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);
        writeReg(Reg::RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8);
        writeReg16(Reg::SD_CONFIG_WOI_SD0, 0x0F0D);
        writeReg16(Reg::SD_CONFIG_INITIAL_PHASE_SD0, 0x0E0E);
    }

    m_distMode = mode;
    setTimingBudgetMs(budget);
    return true;
}

bool VL53L1X::setTimingBudgetMs(uint16_t budgetMs) {
    uint16_t a = 0, b = 0;

    if (m_distMode == DistanceMode::SHORT) {
        switch (budgetMs) {
            case 15:  a = 0x001D; b = 0x0027; break;
            case 20:  a = 0x0051; b = 0x006E; break;
            case 33:  a = 0x00D6; b = 0x006E; break;
            case 50:  a = 0x01AE; b = 0x01E8; break;
            case 100: a = 0x02E1; b = 0x0388; break;
            case 200: a = 0x03E1; b = 0x0496; break;
            case 500: a = 0x0591; b = 0x05C1; break;
            default:  a = 0x02E1; b = 0x0388; budgetMs = 100; break;
        }
    } else {
        switch (budgetMs) {
            case 20:  a = 0x001E; b = 0x0022; break;
            case 33:  a = 0x0060; b = 0x006E; break;
            case 50:  a = 0x00AD; b = 0x00C6; break;
            case 100: a = 0x01CC; b = 0x01EA; break;
            case 200: a = 0x02D9; b = 0x02F8; break;
            case 500: a = 0x048F; b = 0x04A4; break;
            default:  a = 0x01CC; b = 0x01EA; budgetMs = 100; break;
        }
    }

    writeReg16(Reg::RANGE_CONFIG_TIMEOUT_MACROP_A, a);
    writeReg16(Reg::RANGE_CONFIG_TIMEOUT_MACROP_B, b);
    m_timingBudget = budgetMs;
    return true;
}

bool VL53L1X::setInterMeasurementMs(uint16_t periodMs) {
    uint16_t osc = 0;
    readReg16(Reg::OSC_MEASURED_FAST_OSC_FREQ, osc);

    float clockPLL = 1.065f;
    if (osc != 0) {
        clockPLL = static_cast<float>(osc) * 1.065f / (1 << 16);
    }

    uint32_t imm = static_cast<uint32_t>(static_cast<float>(periodMs) * clockPLL);
    writeReg32(Reg::INTERMEASUREMENT_MS, imm);
    return true;
}

bool VL53L1X::setROI(uint8_t width, uint8_t height) {
    width  = std::clamp(width, static_cast<uint8_t>(4), static_cast<uint8_t>(16));
    height = std::clamp(height, static_cast<uint8_t>(4), static_cast<uint8_t>(16));

    uint8_t encoded = ((height - 1) << 4) | (width - 1);
    writeReg(Reg::ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, encoded);
    return true;
}

bool VL53L1X::setROICenter(uint8_t center) {
    writeReg(Reg::ROI_CONFIG_USER_ROI_CENTRE_SPAD, center);
    return true;
}

bool VL53L1X::getROI(ROI& roi) {
    uint8_t encoded = 0;
    if (!readReg(Reg::ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, encoded)) return false;
    roi.width  = (encoded & 0x0F) + 1;
    roi.height = ((encoded >> 4) & 0x0F) + 1;

    if (!readReg(Reg::ROI_CONFIG_USER_ROI_CENTRE_SPAD, roi.center)) return false;
    return true;
}

bool VL53L1X::setSignalThreshold(uint16_t kcps) {
    writeReg16(Reg::RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, kcps >> 1);
    return true;
}

bool VL53L1X::setSigmaThreshold(uint16_t mm) {
    writeReg16(Reg::RANGE_CONFIG_SIGMA_THRESH, mm << 2);
    return true;
}

bool VL53L1X::setI2CAddress(uint8_t newAddr) {
    writeReg(Reg::I2C_SLAVE_DEVICE_ADDRESS, newAddr >> 1);
    m_addr = newAddr >> 1;
    return true;
}

bool VL53L1X::getSensorId(uint16_t& id) {
    return readReg16(Reg::IDENTIFICATION_MODEL_ID, id);
}

VL53L1X::DistanceMode VL53L1X::currentDistanceMode() const {
    return m_distMode;
}

uint16_t VL53L1X::currentTimingBudget() const {
    return m_timingBudget;
}

uint16_t VL53L1X::lastDistance() const {
    return m_lastDistance;
}

VL53L1X::RangeStatus VL53L1X::lastStatus() const {
    return m_lastStatus;
}
