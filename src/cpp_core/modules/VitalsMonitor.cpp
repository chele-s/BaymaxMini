#include "VitalsMonitor.h"
#include "../drivers/MAX30102.h"
#include "../drivers/MLX90614.h"

#include <cmath>
#include <algorithm>

VitalsMonitor::VitalsMonitor(MAX30102& pulseOx, MLX90614& tempSensor)
    : m_pulseOx(pulseOx)
    , m_tempSensor(tempSensor)
    , m_state(vitals::MeasureState::IDLE)
    , m_stateTimer(0.0f)
    , m_measureTimer(0.0f)
    , m_fingerDebounce(0.0f)
    , m_noFingerTimer(0.0f)
    , m_rawHR(0.0)
    , m_rawSpO2(0.0)
    , m_rawBodyTemp(0.0)
    , m_rawAmbientTemp(0.0)
    , m_validSampleCount(0)
    , m_totalSampleCount(0)
    , m_totalMeasurements(0)
    , m_successMeasurements(0)
    , m_fingerPresent(false)
    , m_measureActive(false)
    , m_initialized(false)
{
    m_hrFilter.alpha   = m_config.hrFilterAlpha;
    m_spo2Filter.alpha = m_config.spo2FilterAlpha;
    m_tempFilter.alpha = m_config.tempFilterAlpha;
}

bool VitalsMonitor::init() {
    m_hrFilter.alpha   = m_config.hrFilterAlpha;
    m_spo2Filter.alpha = m_config.spo2FilterAlpha;
    m_tempFilter.alpha = m_config.tempFilterAlpha;

    m_initialized = true;
    return true;
}

void VitalsMonitor::update(float dt) {
    if (!m_initialized) return;

    readSensors();

    switch (m_state) {
        case vitals::MeasureState::IDLE:              stateIdle(dt); break;
        case vitals::MeasureState::DETECTING_FINGER:  stateDetecting(dt); break;
        case vitals::MeasureState::STABILIZING:       stateStabilizing(dt); break;
        case vitals::MeasureState::MEASURING:         stateMeasuring(dt); break;
        case vitals::MeasureState::COMPLETE:           stateComplete(dt); break;
        case vitals::MeasureState::ERROR_NO_FINGER:
        case vitals::MeasureState::ERROR_POOR_SIGNAL:
            break;
    }
}

void VitalsMonitor::readSensors() {
    double hr = 0.0, spo2 = 0.0;
    if (m_pulseOx.readFIFO(hr, spo2)) {
        if (hr > 0.0) m_rawHR = hr;
        if (spo2 > 0.0) m_rawSpO2 = spo2;
        ++m_totalSampleCount;
    }

    double bodyT = 0.0, ambientT = 0.0;
    m_tempSensor.readTemperatures(bodyT, ambientT);
    if (bodyT > 0.0) m_rawBodyTemp = bodyT;
    if (ambientT > 0.0) m_rawAmbientTemp = ambientT;
}

bool VitalsMonitor::checkFingerPresent() {
    uint8_t avail = m_pulseOx.samplesAvailable();
    return avail > 0 && m_rawHR > 0.0;
}

void VitalsMonitor::stateIdle(float dt) {
    if (!m_measureActive) return;

    transitionTo(vitals::MeasureState::DETECTING_FINGER);
    (void)dt;
}

void VitalsMonitor::stateDetecting(float dt) {
    m_stateTimer += dt;

    bool fingerNow = checkFingerPresent();

    if (fingerNow) {
        m_fingerDebounce += dt;
        m_noFingerTimer = 0.0f;

        if (m_fingerDebounce >= 0.5f) {
            if (!m_fingerPresent && m_callbacks.onFingerDetect) {
                m_callbacks.onFingerDetect(true);
            }
            m_fingerPresent = true;
            transitionTo(vitals::MeasureState::STABILIZING);
            return;
        }
    } else {
        m_fingerDebounce = 0.0f;
        m_noFingerTimer += dt;
    }

    if (m_stateTimer > m_config.timeoutSec) {
        transitionTo(vitals::MeasureState::ERROR_NO_FINGER);
        ++m_totalMeasurements;
    }
}

void VitalsMonitor::stateStabilizing(float dt) {
    m_stateTimer += dt;

    bool fingerNow = checkFingerPresent();
    if (!fingerNow) {
        m_noFingerTimer += dt;
        if (m_noFingerTimer > 1.0f) {
            m_fingerPresent = false;
            if (m_callbacks.onFingerDetect) m_callbacks.onFingerDetect(false);
            transitionTo(vitals::MeasureState::DETECTING_FINGER);
            return;
        }
    } else {
        m_noFingerTimer = 0.0f;
    }

    if (m_rawHR > 30.0 && m_rawHR < 220.0) {
        m_hrFilter.process(m_rawHR);
    }
    if (m_rawSpO2 > 70.0 && m_rawSpO2 <= 100.0) {
        m_spo2Filter.process(m_rawSpO2);
    }
    if (m_rawBodyTemp > 25.0 && m_rawBodyTemp < 45.0) {
        m_tempFilter.process(m_rawBodyTemp);
    }

    if (m_stateTimer >= m_config.stabilizationTimeSec) {
        transitionTo(vitals::MeasureState::MEASURING);
    }
}

void VitalsMonitor::stateMeasuring(float dt) {
    m_stateTimer += dt;
    m_measureTimer += dt;

    bool fingerNow = checkFingerPresent();
    if (!fingerNow) {
        m_noFingerTimer += dt;
        if (m_noFingerTimer > 2.0f) {
            m_fingerPresent = false;
            if (m_callbacks.onFingerDetect) m_callbacks.onFingerDetect(false);

            if (m_measureTimer >= m_config.minimumMeasureTimeSec) {
                transitionTo(vitals::MeasureState::COMPLETE);
            } else {
                transitionTo(vitals::MeasureState::ERROR_NO_FINGER);
                ++m_totalMeasurements;
            }
            return;
        }
    } else {
        m_noFingerTimer = 0.0f;
    }

    if (m_rawHR > 30.0 && m_rawHR < 220.0) {
        double filteredHR = m_hrFilter.process(m_rawHR);
        m_hrQuality.push(filteredHR);
        m_current.heartRateBPM = filteredHR;
        ++m_validSampleCount;
    }

    if (m_rawSpO2 > 70.0 && m_rawSpO2 <= 100.0) {
        double filteredSpO2 = m_spo2Filter.process(m_rawSpO2);
        m_spo2Quality.push(filteredSpO2);
        m_current.spo2Percent = filteredSpO2;
    }

    if (m_rawBodyTemp > 25.0 && m_rawBodyTemp < 45.0) {
        m_current.bodyTempC = m_tempFilter.process(m_rawBodyTemp);
    }

    m_current.ambientTempC = m_rawAmbientTemp;
    m_current.fingerDetected = m_fingerPresent;
    m_current.measureTimeSec = m_measureTimer;

    updateQuality();

    float progress = m_measureTimer / m_config.targetMeasureTimeSec;
    m_current.measureProgress = std::min(progress, 1.0f);

    if (m_measureTimer >= m_config.targetMeasureTimeSec) {
        transitionTo(vitals::MeasureState::COMPLETE);
    }

    if (m_stateTimer > m_config.timeoutSec) {
        if (m_measureTimer >= m_config.minimumMeasureTimeSec) {
            transitionTo(vitals::MeasureState::COMPLETE);
        } else {
            transitionTo(vitals::MeasureState::ERROR_POOR_SIGNAL);
            ++m_totalMeasurements;
        }
    }
}

void VitalsMonitor::stateComplete(float dt) {
    (void)dt;

    assessHealth();

    m_current.overallQuality = computeOverallQuality();
    m_current.readingValid = m_current.overallQuality >= m_config.qualityThreshold;
    m_current.measureProgress = 1.0f;

    m_lastComplete = m_current;

    ++m_totalMeasurements;
    if (m_current.readingValid) ++m_successMeasurements;

    if (m_callbacks.onMeasureComplete) {
        m_callbacks.onMeasureComplete(m_current);
    }

    if (m_current.assessment == vitals::HealthAssessment::CRITICAL ||
        m_current.assessment == vitals::HealthAssessment::LOW_SPO2 ||
        m_current.assessment == vitals::HealthAssessment::FEVER) {
        if (m_callbacks.onHealthAlert) {
            m_callbacks.onHealthAlert(m_current.assessment);
        }
    }

    m_measureActive = false;
    transitionTo(vitals::MeasureState::IDLE);
}

void VitalsMonitor::updateQuality() {
    float hrConf = 0.0f;
    if (m_hrQuality.count >= 4) {
        double hrVar = m_hrQuality.variance();
        double hrMean = m_hrQuality.mean();
        if (hrMean > 1.0) {
            double cv = std::sqrt(hrVar) / hrMean;
            hrConf = static_cast<float>(std::max(0.0, 1.0 - cv * 5.0));
        }
    }
    m_current.hrConfidence = hrConf;

    float spo2Conf = 0.0f;
    if (m_spo2Quality.count >= 4) {
        double spo2Var = m_spo2Quality.variance();
        if (m_current.spo2Percent > 80.0) {
            double cv = std::sqrt(spo2Var) / m_current.spo2Percent;
            spo2Conf = static_cast<float>(std::max(0.0, 1.0 - cv * 10.0));
        }
    }
    m_current.spo2Confidence = spo2Conf;

    float tempConf = 0.0f;
    if (m_current.bodyTempC > 30.0 && m_current.bodyTempC < 42.0) {
        tempConf = 0.9f;
        float diff = static_cast<float>(std::abs(m_current.bodyTempC - m_rawBodyTemp));
        if (diff < 0.5f) tempConf = 1.0f;
        else if (diff < 1.0f) tempConf = 0.7f;
        else tempConf = 0.3f;
    }
    m_current.tempConfidence = tempConf;

    if (m_totalSampleCount > 0) {
        float ratio = static_cast<float>(m_validSampleCount) / static_cast<float>(m_totalSampleCount);
        m_current.perfusionIndex = static_cast<double>(ratio);
    }
}

float VitalsMonitor::computeOverallQuality() const {
    float quality = 0.0f;
    quality += m_current.hrConfidence * 0.4f;
    quality += m_current.spo2Confidence * 0.35f;
    quality += m_current.tempConfidence * 0.25f;

    float timeFactor = std::min(m_measureTimer / m_config.targetMeasureTimeSec, 1.0f);
    quality *= (0.5f + 0.5f * timeFactor);

    return std::clamp(quality, 0.0f, 1.0f);
}

void VitalsMonitor::assessHealth() {
    double hr = m_current.heartRateBPM;
    double spo2 = m_current.spo2Percent;
    double temp = m_current.bodyTempC;

    if ((spo2 > 0.0 && spo2 < m_config.criticalSpO2) ||
        (hr > 0.0 && (hr < m_config.criticalHRLow || hr > m_config.criticalHRHigh))) {
        m_current.assessment = vitals::HealthAssessment::CRITICAL;
        return;
    }

    if (spo2 > 0.0 && spo2 < m_config.normalSpO2Min) {
        m_current.assessment = vitals::HealthAssessment::LOW_SPO2;
        return;
    }

    if (temp > m_config.feverThreshold) {
        m_current.assessment = vitals::HealthAssessment::FEVER;
        return;
    }

    if (temp > 0.0 && temp < m_config.normalTempMin - 1.0) {
        m_current.assessment = vitals::HealthAssessment::HYPOTHERMIA;
        return;
    }

    if (hr > m_config.normalHRMax) {
        m_current.assessment = vitals::HealthAssessment::ELEVATED_HR;
        return;
    }

    if (hr > 0.0 && hr < m_config.normalHRMin) {
        m_current.assessment = vitals::HealthAssessment::LOW_HR;
        return;
    }

    m_current.assessment = vitals::HealthAssessment::NORMAL;
}

void VitalsMonitor::transitionTo(vitals::MeasureState newState) {
    vitals::MeasureState prev = m_state;
    m_state = newState;
    m_stateTimer = 0.0f;

    if (newState == vitals::MeasureState::MEASURING) {
        m_measureTimer = 0.0f;
        m_validSampleCount = 0;
        m_totalSampleCount = 0;
        m_hrQuality.reset();
        m_spo2Quality.reset();
    }

    if (newState == vitals::MeasureState::DETECTING_FINGER) {
        m_hrFilter.reset();
        m_spo2Filter.reset();
        m_tempFilter.reset();
        m_fingerDebounce = 0.0f;
        m_noFingerTimer = 0.0f;
        m_current = vitals::VitalsData{};
    }

    if (m_callbacks.onStateChange && prev != newState) {
        m_callbacks.onStateChange(newState);
    }
}

void VitalsMonitor::startMeasurement() {
    m_measureActive = true;
    if (m_state == vitals::MeasureState::IDLE ||
        m_state == vitals::MeasureState::ERROR_NO_FINGER ||
        m_state == vitals::MeasureState::ERROR_POOR_SIGNAL ||
        m_state == vitals::MeasureState::COMPLETE) {
        transitionTo(vitals::MeasureState::DETECTING_FINGER);
    }
}

void VitalsMonitor::stopMeasurement() {
    m_measureActive = false;
    m_fingerPresent = false;
    transitionTo(vitals::MeasureState::IDLE);
}

void VitalsMonitor::setConfig(const vitals::MeasureConfig& config) {
    m_config = config;
    m_hrFilter.alpha   = config.hrFilterAlpha;
    m_spo2Filter.alpha = config.spo2FilterAlpha;
    m_tempFilter.alpha = config.tempFilterAlpha;
}

void VitalsMonitor::setCallbacks(const vitals::VitalsCallbacks& cb) {
    m_callbacks = cb;
}

vitals::MeasureState  VitalsMonitor::currentState()         const { return m_state; }
vitals::VitalsData    VitalsMonitor::currentVitals()        const { return m_current; }
vitals::VitalsData    VitalsMonitor::lastCompleteReading()  const { return m_lastComplete; }

bool   VitalsMonitor::fingerDetected()     const { return m_fingerPresent; }
bool   VitalsMonitor::measurementActive()  const { return m_measureActive; }
bool   VitalsMonitor::hasValidReading()    const { return m_lastComplete.readingValid; }
float  VitalsMonitor::measureProgress()    const { return m_current.measureProgress; }

double VitalsMonitor::instantHeartRate()   const { return m_current.heartRateBPM; }
double VitalsMonitor::instantSpO2()        const { return m_current.spo2Percent; }
double VitalsMonitor::bodyTemperature()    const { return m_current.bodyTempC; }

uint32_t VitalsMonitor::totalMeasurements()      const { return m_totalMeasurements; }
uint32_t VitalsMonitor::successfulMeasurements() const { return m_successMeasurements; }
