#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <functional>

class MAX30102;
class MLX90614;

namespace vitals {

enum class MeasureState : uint8_t {
    IDLE,
    DETECTING_FINGER,
    STABILIZING,
    MEASURING,
    COMPLETE,
    ERROR_NO_FINGER,
    ERROR_POOR_SIGNAL
};

enum class HealthAssessment : uint8_t {
    UNKNOWN,
    NORMAL,
    ELEVATED_HR,
    LOW_HR,
    LOW_SPO2,
    FEVER,
    HYPOTHERMIA,
    CRITICAL
};

struct VitalsData {
    double heartRateBPM     = 0.0;
    double spo2Percent      = 0.0;
    double bodyTempC        = 0.0;
    double ambientTempC     = 0.0;
    double perfusionIndex   = 0.0;

    float  hrConfidence     = 0.0f;
    float  spo2Confidence   = 0.0f;
    float  tempConfidence   = 0.0f;
    float  overallQuality   = 0.0f;

    HealthAssessment assessment = HealthAssessment::UNKNOWN;

    bool   fingerDetected   = false;
    bool   readingValid     = false;
    float  measureProgress  = 0.0f;
    float  measureTimeSec   = 0.0f;
};

struct MeasureConfig {
    float fingerDetectThresholdIR = 50000.0f;
    float fingerRemoveThresholdIR = 20000.0f;
    float stabilizationTimeSec    = 2.0f;
    float minimumMeasureTimeSec   = 5.0f;
    float targetMeasureTimeSec    = 10.0f;
    float timeoutSec              = 30.0f;
    float qualityThreshold        = 0.5f;
    float hrFilterAlpha           = 0.15f;
    float spo2FilterAlpha         = 0.10f;
    float tempFilterAlpha         = 0.08f;

    float normalHRMin             = 50.0f;
    float normalHRMax             = 110.0f;
    float normalSpO2Min           = 95.0f;
    float normalTempMin           = 36.0f;
    float normalTempMax           = 37.5f;
    float feverThreshold          = 38.0f;
    float criticalSpO2            = 90.0f;
    float criticalHRLow           = 40.0f;
    float criticalHRHigh          = 180.0f;
};

struct VitalsCallbacks {
    std::function<void(MeasureState)>           onStateChange;
    std::function<void(const VitalsData&)>      onMeasureComplete;
    std::function<void(bool)>                   onFingerDetect;
    std::function<void(HealthAssessment)>       onHealthAlert;
};

}

class VitalsMonitor {
public:
    VitalsMonitor(MAX30102& pulseOx, MLX90614& tempSensor);

    bool init();
    void update(float dt);

    void startMeasurement();
    void stopMeasurement();

    void setConfig(const vitals::MeasureConfig& config);
    void setCallbacks(const vitals::VitalsCallbacks& cb);

    vitals::MeasureState  currentState() const;
    vitals::VitalsData    currentVitals() const;
    vitals::VitalsData    lastCompleteReading() const;

    bool   fingerDetected() const;
    bool   measurementActive() const;
    bool   hasValidReading() const;
    float  measureProgress() const;

    double instantHeartRate() const;
    double instantSpO2() const;
    double bodyTemperature() const;

    uint32_t totalMeasurements() const;
    uint32_t successfulMeasurements() const;

private:
    struct EMADouble {
        double value = 0.0;
        double alpha = 0.1;
        bool   init  = false;

        double process(double input) {
            if (!init) {
                value = input;
                init = true;
                return value;
            }
            value += alpha * (input - value);
            return value;
        }

        void reset() {
            value = 0.0;
            init  = false;
        }
    };

    struct SignalQualityTracker {
        static constexpr std::size_t WINDOW = 8;
        double samples[WINDOW] = {};
        std::size_t writeIdx   = 0;
        std::size_t count      = 0;

        void push(double val) {
            samples[writeIdx] = val;
            writeIdx = (writeIdx + 1) % WINDOW;
            if (count < WINDOW) ++count;
        }

        double variance() const {
            if (count < 2) return 0.0;
            double mean = 0.0;
            for (std::size_t i = 0; i < count; ++i) mean += samples[i];
            mean /= static_cast<double>(count);
            double var = 0.0;
            for (std::size_t i = 0; i < count; ++i) {
                double d = samples[i] - mean;
                var += d * d;
            }
            return var / static_cast<double>(count);
        }

        double mean() const {
            if (count == 0) return 0.0;
            double s = 0.0;
            for (std::size_t i = 0; i < count; ++i) s += samples[i];
            return s / static_cast<double>(count);
        }

        void reset() {
            writeIdx = 0;
            count = 0;
        }
    };

    void stateIdle(float dt);
    void stateDetecting(float dt);
    void stateStabilizing(float dt);
    void stateMeasuring(float dt);
    void stateComplete(float dt);

    void readSensors();
    void updateQuality();
    void assessHealth();
    void transitionTo(vitals::MeasureState newState);

    bool checkFingerPresent();
    float computeOverallQuality() const;

    MAX30102& m_pulseOx;
    MLX90614& m_tempSensor;

    vitals::MeasureConfig  m_config;
    vitals::VitalsCallbacks m_callbacks;

    vitals::MeasureState m_state;
    vitals::VitalsData   m_current;
    vitals::VitalsData   m_lastComplete;

    EMADouble m_hrFilter;
    EMADouble m_spo2Filter;
    EMADouble m_tempFilter;

    SignalQualityTracker m_hrQuality;
    SignalQualityTracker m_spo2Quality;

    float m_stateTimer;
    float m_measureTimer;
    float m_fingerDebounce;
    float m_noFingerTimer;

    double m_rawHR;
    double m_rawSpO2;
    double m_rawBodyTemp;
    double m_rawAmbientTemp;

    uint32_t m_validSampleCount;
    uint32_t m_totalSampleCount;
    uint32_t m_totalMeasurements;
    uint32_t m_successMeasurements;

    bool m_fingerPresent;
    bool m_measureActive;
    bool m_initialized;
};
