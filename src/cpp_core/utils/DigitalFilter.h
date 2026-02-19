#pragma once

#include "MathUtils.h"
#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>
#include <type_traits>
#include <limits>

namespace dsp {
namespace oxi {

template<typename T, std::size_t N = 50>
class RingBuffer {
    static_assert(std::is_floating_point_v<T>);
    static_assert(N > 0);
public:
    RingBuffer() { m_data.fill(T(0)); }

    void push(T value) {
        m_head = (m_head + 1) % N;
        m_data[m_head] = value;
        if (m_count < N) ++m_count;
    }

    T operator[](std::size_t age) const {
        std::size_t idx = (m_head + N - age) % N;
        return m_data[idx];
    }

    T newest() const { return m_data[m_head]; }

    T oldest() const {
        if (m_count < N) {
            std::size_t idx = (m_head + N - m_count + 1) % N;
            return m_data[idx];
        }
        return m_data[(m_head + 1) % N];
    }

    T mean() const {
        if (m_count == 0) return T(0);
        T sum = T(0);
        for (std::size_t i = 0; i < m_count; ++i) {
            sum += (*this)[i];
        }
        return sum / static_cast<T>(m_count);
    }

    T min() const {
        if (m_count == 0) return T(0);
        T result = (*this)[0];
        for (std::size_t i = 1; i < m_count; ++i) {
            T val = (*this)[i];
            if (val < result) result = val;
        }
        return result;
    }

    T max() const {
        if (m_count == 0) return T(0);
        T result = (*this)[0];
        for (std::size_t i = 1; i < m_count; ++i) {
            T val = (*this)[i];
            if (val > result) result = val;
        }
        return result;
    }

    T peakToPeak() const {
        if (m_count < 2) return T(0);
        T lo = (*this)[0];
        T hi = lo;
        for (std::size_t i = 1; i < m_count; ++i) {
            T val = (*this)[i];
            if (val < lo) lo = val;
            if (val > hi) hi = val;
        }
        return hi - lo;
    }

    T variance() const {
        if (m_count < 2) return T(0);
        T avg = mean();
        T sumSq = T(0);
        for (std::size_t i = 0; i < m_count; ++i) {
            T diff = (*this)[i] - avg;
            sumSq += diff * diff;
        }
        return sumSq / static_cast<T>(m_count);
    }

    bool full() const { return m_count == N; }
    std::size_t count() const { return m_count; }
    constexpr std::size_t capacity() const { return N; }

    void reset() {
        m_data.fill(T(0));
        m_head  = 0;
        m_count = 0;
    }

    const std::array<T, N>& raw() const { return m_data; }

private:
    std::array<T, N> m_data{};
    std::size_t      m_head  = 0;
    std::size_t      m_count = 0;
};

template<typename T>
class DCRemovalFilter {
    static_assert(std::is_floating_point_v<T>);
public:
    explicit DCRemovalFilter(T R = T(0.992))
        : m_blocker(R)
    {}

    T process(T input) {
        m_rawDC += T(0.001) * (input - m_rawDC);
        m_ac = m_blocker.process(input);
        return m_ac;
    }

    T ac() const { return m_ac; }
    T dc() const { return m_rawDC; }

    void setR(T R) {
        m_blocker = DCBlocker<T>(R);
    }

    void reset() {
        m_blocker.reset();
        m_ac    = T(0);
        m_rawDC = T(0);
    }

private:
    DCBlocker<T> m_blocker;
    T            m_ac    = T(0);
    T            m_rawDC = T(0);
};

template<typename T>
class LowPassIIR {
    static_assert(std::is_floating_point_v<T>);
public:
    LowPassIIR() = default;

    explicit LowPassIIR(T alpha)
        : m_smoother(clamp(alpha, T(0), T(1)))
    {}

    static LowPassIIR fromCutoff(T cutoffHz, T sampleRateHz) {
        T alpha = ExponentialSmoother<T>::alphaFromCutoff(cutoffHz, sampleRateHz);
        return LowPassIIR(alpha);
    }

    T process(T input) {
        return m_smoother.process(input);
    }

    void setAlpha(T alpha) { m_smoother.setAlpha(alpha); }

    void configureCutoff(T cutoffHz, T sampleRateHz) {
        T alpha = ExponentialSmoother<T>::alphaFromCutoff(cutoffHz, sampleRateHz);
        m_smoother.setAlpha(alpha);
    }

    T value() const { return m_smoother.value(); }

    void reset() { m_smoother.reset(); }

private:
    ExponentialSmoother<T> m_smoother{T(0.2)};
};

template<typename T>
class ButterworthLPF {
    static_assert(std::is_floating_point_v<T>);
public:
    ButterworthLPF() = default;

    ButterworthLPF(T sampleRate, T cutoffHz) {
        configure(sampleRate, cutoffHz);
    }

    void configure(T sampleRate, T cutoffHz) {
        m_biquad.configure(BiquadFilter<T>::Type::LOWPASS, sampleRate, cutoffHz, T(0.70710678118));
    }

    T process(T input) {
        return m_biquad.process(input);
    }

    void reset() { m_biquad.reset(); }

private:
    BiquadFilter<T> m_biquad;
};

template<typename T>
class PeakDetector {
    static_assert(std::is_floating_point_v<T>);
public:
    PeakDetector() = default;

    explicit PeakDetector(T sampleRate, T refractoryMs = T(300))
        : m_sampleRate(sampleRate)
        , m_refractorySamples(static_cast<uint32_t>(sampleRate * refractoryMs / T(1000)))
    {}

    void configure(T sampleRate, T refractoryMs = T(300)) {
        m_sampleRate = sampleRate;
        m_refractorySamples = static_cast<uint32_t>(sampleRate * refractoryMs / T(1000));
        reset();
    }

    bool process(T sample) {
        bool peakFound = false;

        if (m_samplesSinceLastPeak < m_refractorySamples) {
            ++m_samplesSinceLastPeak;
            m_prevSlope = sample - m_prevSample;
            m_prevSample = sample;
            return false;
        }

        T slope = sample - m_prevSample;

        if (m_prevSlope > T(0) && slope <= T(0)) {
            if (m_prevSample > m_adaptiveThreshold) {
                peakFound = true;

                if (m_lastPeakInterval > 0) {
                    m_instantBPM = (m_sampleRate * T(60)) / static_cast<T>(m_samplesSinceLastPeak);
                    m_instantBPM = clamp(m_instantBPM, T(30), T(240));
                    m_bpmSmoothed += T(0.3) * (m_instantBPM - m_bpmSmoothed);
                }

                m_lastPeakInterval = m_samplesSinceLastPeak;
                m_samplesSinceLastPeak = 0;

                m_adaptiveThreshold = T(0.7) * m_adaptiveThreshold + T(0.3) * m_prevSample;
            }
        }

        m_adaptiveThreshold *= T(0.998);

        m_prevSlope  = slope;
        m_prevSample = sample;
        ++m_samplesSinceLastPeak;

        return peakFound;
    }

    T bpm() const { return m_bpmSmoothed; }
    T instantBPM() const { return m_instantBPM; }
    uint32_t intervalSamples() const { return m_lastPeakInterval; }

    T intervalSeconds() const {
        if (m_lastPeakInterval == 0 || m_sampleRate <= T(0)) return T(0);
        return static_cast<T>(m_lastPeakInterval) / m_sampleRate;
    }

    void reset() {
        m_prevSample         = T(0);
        m_prevSlope          = T(0);
        m_adaptiveThreshold  = T(0);
        m_samplesSinceLastPeak = m_refractorySamples;
        m_lastPeakInterval   = 0;
        m_instantBPM         = T(0);
        m_bpmSmoothed        = T(0);
    }

private:
    T        m_sampleRate        = T(100);
    uint32_t m_refractorySamples = 30;

    T        m_prevSample        = T(0);
    T        m_prevSlope         = T(0);
    T        m_adaptiveThreshold = T(0);
    uint32_t m_samplesSinceLastPeak = 30;
    uint32_t m_lastPeakInterval  = 0;
    T        m_instantBPM        = T(0);
    T        m_bpmSmoothed       = T(0);
};

template<typename T, std::size_t BUFFER_SIZE = 50>
class PPGFilterChain {
    static_assert(std::is_floating_point_v<T>);
public:
    PPGFilterChain() = default;

    explicit PPGFilterChain(T sampleRate)
        : m_sampleRate(sampleRate)
        , m_dcFilter(T(0.992))
        , m_lpf(sampleRate, T(5))
        , m_peakDet(sampleRate, T(300))
    {}

    void configure(T sampleRate) {
        m_sampleRate = sampleRate;
        m_dcFilter   = DCRemovalFilter<T>(T(0.992));
        m_lpf        = ButterworthLPF<T>(sampleRate, T(5));
        m_peakDet.configure(sampleRate, T(300));
        reset();
    }

    T process(T rawSample) {
        T median   = m_median.process(rawSample);
        T acSignal = m_dcFilter.process(median);
        T filtered = m_lpf.process(acSignal);

        m_buffer.push(filtered);
        m_peakDetected = m_peakDet.process(filtered);

        m_filtered = filtered;
        return filtered;
    }

    T ac() const { return m_dcFilter.ac(); }
    T dc() const { return m_dcFilter.dc(); }
    T filtered() const { return m_filtered; }
    bool peakDetected() const { return m_peakDetected; }
    T bpm() const { return m_peakDet.bpm(); }

    const RingBuffer<T, BUFFER_SIZE>& buffer() const { return m_buffer; }
    const PeakDetector<T>& peakDetector() const { return m_peakDet; }

    T signalQuality() const {
        if (!m_buffer.full()) return T(0);

        T pp = m_buffer.peakToPeak();
        T dcVal = m_dcFilter.dc();

        if (dcVal <= T(0)) return T(0);

        T acDcRatio = pp / dcVal;

        T ratioScore = clamp(acDcRatio / T(0.02), T(0), T(1));

        T var = m_buffer.variance();
        T mean = m_buffer.mean();
        T cv = (std::abs(mean) > T(1e-9)) ? std::sqrt(var) / std::abs(mean) : T(0);
        T stabilityScore = clamp(T(1) - cv * T(0.5), T(0), T(1));

        return ratioScore * T(0.6) + stabilityScore * T(0.4);
    }

    void reset() {
        m_median = MedianFilter3<T>();
        m_dcFilter.reset();
        m_lpf.reset();
        m_buffer.reset();
        m_peakDet.reset();
        m_filtered     = T(0);
        m_peakDetected = false;
    }

private:
    T m_sampleRate = T(100);

    MedianFilter3<T>             m_median;
    DCRemovalFilter<T>           m_dcFilter{T(0.992)};
    ButterworthLPF<T>            m_lpf;
    RingBuffer<T, BUFFER_SIZE>   m_buffer;
    PeakDetector<T>              m_peakDet;

    T    m_filtered     = T(0);
    bool m_peakDetected = false;
};

}
}
