#include "MAX30102.h"
#include "I2C_Bus.h"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <unistd.h>

namespace Reg {
    constexpr uint8_t INT_STATUS1   = 0x00;
    constexpr uint8_t INT_STATUS2   = 0x01;
    constexpr uint8_t INT_ENABLE1   = 0x02;
    constexpr uint8_t INT_ENABLE2   = 0x03;
    constexpr uint8_t FIFO_WR_PTR   = 0x04;
    constexpr uint8_t OVF_COUNTER   = 0x05;
    constexpr uint8_t FIFO_RD_PTR   = 0x06;
    constexpr uint8_t FIFO_DATA     = 0x07;
    constexpr uint8_t FIFO_CONFIG   = 0x08;
    constexpr uint8_t MODE_CONFIG   = 0x09;
    constexpr uint8_t SPO2_CONFIG   = 0x0A;
    constexpr uint8_t LED1_PA       = 0x0C;
    constexpr uint8_t LED2_PA       = 0x0D;
    constexpr uint8_t MULTI_LED_1   = 0x11;
    constexpr uint8_t MULTI_LED_2   = 0x12;
    constexpr uint8_t TEMP_INT      = 0x1F;
    constexpr uint8_t TEMP_FRAC     = 0x20;
    constexpr uint8_t TEMP_CONFIG   = 0x21;
    constexpr uint8_t REV_ID        = 0xFE;
    constexpr uint8_t PART_ID       = 0xFF;
}

namespace Mode {
    constexpr uint8_t HEART_RATE    = 0x02;
    constexpr uint8_t SPO2          = 0x03;
    constexpr uint8_t MULTI_LED     = 0x07;
    constexpr uint8_t RESET         = 0x40;
    constexpr uint8_t SHDN          = 0x80;
}

namespace Fifo {
    constexpr uint8_t A_FULL_EN     = 0x80;
    constexpr uint8_t DATA_RDY_EN   = 0x40;
}

constexpr uint8_t  ADC_RANGE_4096   = 0x00;
constexpr uint8_t  ADC_RANGE_8192   = 0x20;
constexpr uint8_t  ADC_RANGE_16384  = 0x40;
constexpr uint8_t  ADC_RANGE_32768  = 0x60;

constexpr uint8_t  SAMPLE_AVG_4     = 0x40;
constexpr uint8_t  FIFO_ROLLOVER_EN = 0x10;

constexpr uint32_t ADC_MASK         = 0x0003FFFF;

MAX30102::MAX30102(I2CBus& bus, uint8_t addr)
    : m_bus(bus)
    , m_addr(addr)
    , m_sampleRate(100)
    , m_lastHR(0.0)
    , m_lastSpO2(0.0)
    , m_initialized(false)
{}

void MAX30102::writeReg(uint8_t reg, uint8_t value) {
    m_bus.writeByte(m_addr, reg, value);
}

uint8_t MAX30102::readReg(uint8_t reg) {
    uint8_t val = 0;
    m_bus.readByte(m_addr, reg, val);
    return val;
}

bool MAX30102::init() {
    reset();
    usleep(50000);

    uint8_t partId;
    if (!readPartId(partId) || partId != EXPECTED_PART_ID) return false;

    writeReg(Reg::INT_ENABLE1, Fifo::A_FULL_EN | Fifo::DATA_RDY_EN);
    writeReg(Reg::INT_ENABLE2, 0x00);

    clearFIFO();

    writeReg(Reg::FIFO_CONFIG, SAMPLE_AVG_4 | FIFO_ROLLOVER_EN | 0x0F);

    writeReg(Reg::MODE_CONFIG, Mode::SPO2);

    writeReg(Reg::SPO2_CONFIG, ADC_RANGE_16384 | 0x0C | 0x03);

    writeReg(Reg::LED1_PA, 0x24);
    writeReg(Reg::LED2_PA, 0x24);

    readReg(Reg::INT_STATUS1);
    readReg(Reg::INT_STATUS2);

    m_sampleRate = 100;
    m_initialized = true;
    return true;
}

void MAX30102::shutdown() {
    if (!m_initialized) return;
    uint8_t mode = readReg(Reg::MODE_CONFIG);
    writeReg(Reg::MODE_CONFIG, mode | Mode::SHDN);
    m_initialized = false;
}

void MAX30102::reset() {
    writeReg(Reg::MODE_CONFIG, Mode::RESET);
}

void MAX30102::clearFIFO() {
    writeReg(Reg::FIFO_WR_PTR, 0x00);
    writeReg(Reg::OVF_COUNTER, 0x00);
    writeReg(Reg::FIFO_RD_PTR, 0x00);
}

bool MAX30102::dataReady() {
    uint8_t status = readReg(Reg::INT_STATUS1);
    return (status & Fifo::DATA_RDY_EN) != 0;
}

uint8_t MAX30102::samplesAvailable() {
    uint8_t wrPtr = readReg(Reg::FIFO_WR_PTR) & 0x1F;
    uint8_t rdPtr = readReg(Reg::FIFO_RD_PTR) & 0x1F;
    int diff = static_cast<int>(wrPtr) - static_cast<int>(rdPtr);
    if (diff < 0) diff += 32;
    return static_cast<uint8_t>(diff);
}

bool MAX30102::readRawSamples(uint32_t* redBuf, uint32_t* irBuf, std::size_t& count) {
    count = samplesAvailable();
    if (count == 0) return false;
    if (count > BUFFER_SIZE) count = BUFFER_SIZE;

    for (std::size_t i = 0; i < count; ++i) {
        uint8_t raw[6];
        if (!m_bus.readBytes(m_addr, Reg::FIFO_DATA, raw, 6)) {
            count = i;
            return i > 0;
        }

        redBuf[i] = ((static_cast<uint32_t>(raw[0]) << 16) |
                      (static_cast<uint32_t>(raw[1]) << 8) |
                       static_cast<uint32_t>(raw[2])) & ADC_MASK;

        irBuf[i]  = ((static_cast<uint32_t>(raw[3]) << 16) |
                      (static_cast<uint32_t>(raw[4]) << 8) |
                       static_cast<uint32_t>(raw[5])) & ADC_MASK;
    }

    return true;
}

bool MAX30102::readFIFO(double& heartRateBPM, double& spo2Pct) {
    if (!m_initialized) {
        heartRateBPM = 0.0;
        spo2Pct = 0.0;
        return false;
    }

    uint32_t redBuf[BUFFER_SIZE];
    uint32_t irBuf[BUFFER_SIZE];
    std::size_t count = 0;

    if (!readRawSamples(redBuf, irBuf, count) || count < 4) {
        heartRateBPM = m_lastHR;
        spo2Pct = m_lastSpO2;
        return false;
    }

    processSamples(redBuf, irBuf, count, heartRateBPM, spo2Pct);

    if (heartRateBPM > 0.0) m_lastHR = heartRateBPM;
    if (spo2Pct > 0.0) m_lastSpO2 = spo2Pct;

    return true;
}

void MAX30102::processSamples(const uint32_t* redBuf, const uint32_t* irBuf, std::size_t count,
                               double& heartRateBPM, double& spo2Pct) {
    heartRateBPM = detectHeartRate(irBuf, count);
    spo2Pct = calculateSpO2(redBuf, irBuf, count);
}

double MAX30102::detectHeartRate(const uint32_t* irBuf, std::size_t count) {
    if (count < 8) return 0.0;

    double mean = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        mean += static_cast<double>(irBuf[i]);
    }
    mean /= static_cast<double>(count);

    std::size_t peakCount = 0;
    std::size_t firstPeak = 0;
    std::size_t lastPeak  = 0;
    constexpr double PEAK_THRESHOLD_FACTOR = 1.02;

    for (std::size_t i = 2; i < count - 2; ++i) {
        double val  = static_cast<double>(irBuf[i]);
        double prev = static_cast<double>(irBuf[i - 1]);
        double next = static_cast<double>(irBuf[i + 1]);
        double prev2 = static_cast<double>(irBuf[i - 2]);
        double next2 = static_cast<double>(irBuf[i + 2]);

        bool isPeak = val > prev && val > next &&
                      val > prev2 && val > next2 &&
                      val > mean * PEAK_THRESHOLD_FACTOR;

        if (isPeak) {
            if (peakCount == 0) firstPeak = i;
            lastPeak = i;
            ++peakCount;
        }
    }

    if (peakCount < 2) return 0.0;

    double effectiveSampleRate = static_cast<double>(m_sampleRate) / 4.0;
    double peakSpanSamples = static_cast<double>(lastPeak - firstPeak);
    double intervals = static_cast<double>(peakCount - 1);
    double avgPeriod = peakSpanSamples / intervals;
    double bpm = (effectiveSampleRate / avgPeriod) * 60.0;

    if (bpm < 30.0 || bpm > 220.0) return 0.0;

    return bpm;
}

double MAX30102::calculateSpO2(const uint32_t* redBuf, const uint32_t* irBuf, std::size_t count) {
    if (count < 4) return 0.0;

    double redAC = 0.0, redDC = 0.0;
    double irAC  = 0.0, irDC  = 0.0;

    double redMean = 0.0, irMean = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        redMean += static_cast<double>(redBuf[i]);
        irMean  += static_cast<double>(irBuf[i]);
    }
    redMean /= static_cast<double>(count);
    irMean  /= static_cast<double>(count);

    redDC = redMean;
    irDC  = irMean;

    if (redDC < 1.0 || irDC < 1.0) return 0.0;

    double redVar = 0.0, irVar = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        double rd = static_cast<double>(redBuf[i]) - redMean;
        double id = static_cast<double>(irBuf[i])  - irMean;
        redVar += rd * rd;
        irVar  += id * id;
    }

    redAC = std::sqrt(redVar / static_cast<double>(count));
    irAC  = std::sqrt(irVar / static_cast<double>(count));

    if (irAC < 1.0) return 0.0;

    double R = (redAC / redDC) / (irAC / irDC);

    double spo2 = 110.0 - 25.0 * R;

    spo2 = std::clamp(spo2, 0.0, 100.0);

    return spo2;
}

void MAX30102::setLedCurrent(uint8_t redMA, uint8_t irMA) {
    writeReg(Reg::LED1_PA, redMA);
    writeReg(Reg::LED2_PA, irMA);
}

void MAX30102::setSampleRate(uint16_t sps) {
    uint8_t val;
    switch (sps) {
        case 50:   val = 0x00; break;
        case 100:  val = 0x04; break;
        case 200:  val = 0x08; break;
        case 400:  val = 0x0C; break;
        case 800:  val = 0x10; break;
        case 1000: val = 0x14; break;
        case 1600: val = 0x18; break;
        case 3200: val = 0x1C; break;
        default:   val = 0x04; sps = 100; break;
    }

    m_sampleRate = sps;
    uint8_t cfg = readReg(Reg::SPO2_CONFIG);
    cfg = (cfg & 0xE3) | val;
    writeReg(Reg::SPO2_CONFIG, cfg);
}

void MAX30102::setAdcRange(uint8_t range) {
    uint8_t cfg = readReg(Reg::SPO2_CONFIG);
    cfg = (cfg & 0x9F) | ((range & 0x03) << 5);
    writeReg(Reg::SPO2_CONFIG, cfg);
}

void MAX30102::setPulseWidth(uint16_t us) {
    uint8_t val;
    switch (us) {
        case 69:  val = 0x00; break;
        case 118: val = 0x01; break;
        case 215: val = 0x02; break;
        case 411: val = 0x03; break;
        default:  val = 0x03; break;
    }

    uint8_t cfg = readReg(Reg::SPO2_CONFIG);
    cfg = (cfg & 0xFC) | val;
    writeReg(Reg::SPO2_CONFIG, cfg);
}

bool MAX30102::readPartId(uint8_t& id) {
    return m_bus.readByte(m_addr, Reg::PART_ID, id);
}

bool MAX30102::readRevisionId(uint8_t& rev) {
    return m_bus.readByte(m_addr, Reg::REV_ID, rev);
}
