#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <numeric>

struct HRVAnalysisResult {
    double meanRRMs       = 0.0;
    double sdnnMs         = 0.0;
    double rmssdMs        = 0.0;
    double pnn50Pct       = 0.0;
    double lfPower        = 0.0;
    double hfPower        = 0.0;
    double lfHfRatio      = 0.0;
    double stressIndex    = 0.0;
    bool   valid          = false;
};

struct VitalsRiskAssessment {
    float overallRiskScore   = 0.0f;
    bool  cardiacRisk        = false;
    bool  respiratoryRisk    = false;
    bool  thermalRisk        = false;
    bool  vitalAlertTriggered = false;
};

class AdvancedVitalsAnalyzer {
public:
    AdvancedVitalsAnalyzer();

    void addHeartRateSample(double bpm, double timestampSec);
    void addSpO2Sample(double spo2, double timestampSec);
    void addTemperatureSample(double tempC, double timestampSec);

    HRVAnalysisResult analyzeHRV();
    VitalsRiskAssessment evaluateHealthRisk(double hr, double spo2, double tempC);

    void resetHistory();

private:
    struct TimedSample {
        double value;
        double timestampSec;
    };

    std::vector<TimedSample> m_hrHistory;
    std::vector<TimedSample> m_spo2History;
    std::vector<TimedSample> m_tempHistory;

    static constexpr std::size_t MAX_HISTORY_SIZE = 1200;
};
