#include "AdvancedVitalsAnalyzer.h"

AdvancedVitalsAnalyzer::AdvancedVitalsAnalyzer() {}

void AdvancedVitalsAnalyzer::addHeartRateSample(double bpm, double timestampSec) {
    if (bpm <= 0.0) return;
    m_hrHistory.push_back({bpm, timestampSec});
    if (m_hrHistory.size() > MAX_HISTORY_SIZE) {
        m_hrHistory.erase(m_hrHistory.begin());
    }
}

void AdvancedVitalsAnalyzer::addSpO2Sample(double spo2, double timestampSec) {
    if (spo2 <= 0.0) return;
    m_spo2History.push_back({spo2, timestampSec});
    if (m_spo2History.size() > MAX_HISTORY_SIZE) {
        m_spo2History.erase(m_spo2History.begin());
    }
}

void AdvancedVitalsAnalyzer::addTemperatureSample(double tempC, double timestampSec) {
    if (tempC <= 0.0) return;
    m_tempHistory.push_back({tempC, timestampSec});
    if (m_tempHistory.size() > MAX_HISTORY_SIZE) {
        m_tempHistory.erase(m_tempHistory.begin());
    }
}

HRVAnalysisResult AdvancedVitalsAnalyzer::analyzeHRV() {
    HRVAnalysisResult res;
    if (m_hrHistory.size() < 10) {
        res.valid = false;
        return res;
    }

    std::vector<double> rrMs;
    rrMs.reserve(m_hrHistory.size());

    for (const auto& sample : m_hrHistory) {
        if (sample.value > 30.0 && sample.value < 220.0) {
            double rr = 60000.0 / sample.value;
            rrMs.push_back(rr);
        }
    }

    if (rrMs.size() < 10) {
        res.valid = false;
        return res;
    }

    double sum = std::accumulate(rrMs.begin(), rrMs.end(), 0.0);
    double mean = sum / static_cast<double>(rrMs.size());
    res.meanRRMs = mean;

    double sqDiffSum = 0.0;
    for (double rr : rrMs) {
        double diff = rr - mean;
        sqDiffSum += diff * diff;
    }
    res.sdnnMs = std::sqrt(sqDiffSum / static_cast<double>(rrMs.size()));

    double diffSqSum = 0.0;
    std::size_t nn50Count = 0;

    for (std::size_t i = 1; i < rrMs.size(); ++i) {
        double diff = std::abs(rrMs[i] - rrMs[i - 1]);
        diffSqSum += diff * diff;
        if (diff > 50.0) {
            ++nn50Count;
        }
    }

    res.rmssdMs = std::sqrt(diffSqSum / static_cast<double>(rrMs.size() - 1));
    res.pnn50Pct = (static_cast<double>(nn50Count) / static_cast<double>(rrMs.size() - 1)) * 100.0;

    double lowFreqBand = 0.0;
    double highFreqBand = 0.0;

    for (double rr : rrMs) {
        double dev = std::abs(rr - mean);
        if (dev > 20.0 && dev < 80.0) {
            lowFreqBand += dev;
        } else if (dev >= 80.0) {
            highFreqBand += dev;
        }
    }

    res.lfPower = lowFreqBand;
    res.hfPower = highFreqBand;
    res.lfHfRatio = (highFreqBand > 0.001) ? (lowFreqBand / highFreqBand) : 1.0;

    double amo = (static_cast<double>(rrMs.size()) * 100.0) / static_cast<double>(m_hrHistory.size());
    double mxDMx = res.sdnnMs * 2.0;
    res.stressIndex = (mxDMx > 0.001) ? (amo / (2.0 * (mean / 1000.0) * (mxDMx / 1000.0))) : 0.0;

    res.valid = true;
    return res;
}

VitalsRiskAssessment AdvancedVitalsAnalyzer::evaluateHealthRisk(double hr, double spo2, double tempC) {
    VitalsRiskAssessment risk;
    float score = 0.0f;

    if (hr > 0.0) {
        if (hr < 50.0 || hr > 120.0) {
            risk.cardiacRisk = true;
            score += 0.35f;
        } else if (hr < 60.0 || hr > 100.0) {
            score += 0.15f;
        }
    }

    if (spo2 > 0.0) {
        if (spo2 < 90.0) {
            risk.respiratoryRisk = true;
            score += 0.45f;
        } else if (spo2 < 95.0) {
            score += 0.20f;
        }
    }

    if (tempC > 0.0) {
        if (tempC > 38.0 || tempC < 35.0) {
            risk.thermalRisk = true;
            score += 0.35f;
        } else if (tempC > 37.5 || tempC < 36.0) {
            score += 0.10f;
        }
    }

    risk.overallRiskScore = std::min(score, 1.0f);
    risk.vitalAlertTriggered = (risk.cardiacRisk || risk.respiratoryRisk || risk.thermalRisk || risk.overallRiskScore >= 0.6f);

    return risk;
}

void AdvancedVitalsAnalyzer::resetHistory() {
    m_hrHistory.clear();
    m_spo2History.clear();
    m_tempHistory.clear();
}
