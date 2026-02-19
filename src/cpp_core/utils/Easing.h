#pragma once

#include "MathUtils.h"
#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>
#include <type_traits>
#include <functional>
#include <limits>

namespace dsp {
namespace anim {

template<typename T>
using EasingFn = T(*)(T);

enum class Curve : uint8_t {
    LINEAR,
    IN_SINE,       OUT_SINE,       IN_OUT_SINE,
    IN_QUAD,       OUT_QUAD,       IN_OUT_QUAD,
    IN_CUBIC,      OUT_CUBIC,      IN_OUT_CUBIC,
    IN_QUART,      OUT_QUART,      IN_OUT_QUART,
    IN_QUINT,      OUT_QUINT,      IN_OUT_QUINT,
    IN_EXPO,       OUT_EXPO,       IN_OUT_EXPO,
    IN_CIRC,       OUT_CIRC,       IN_OUT_CIRC,
    IN_BACK,       OUT_BACK,       IN_OUT_BACK,
    IN_ELASTIC,    OUT_ELASTIC,    IN_OUT_ELASTIC,
    IN_BOUNCE,     OUT_BOUNCE,     IN_OUT_BOUNCE,
    SMOOTH_STEP,
    SMOOTHER_STEP,
    COUNT
};

template<typename T>
constexpr T evaluateCurve(Curve curve, T t) {
    t = clamp(t, T(0), T(1));
    switch (curve) {
        case Curve::LINEAR:         return t;
        case Curve::IN_SINE:        return ease::inSine(t);
        case Curve::OUT_SINE:       return ease::outSine(t);
        case Curve::IN_OUT_SINE:    return ease::inOutSine(t);
        case Curve::IN_QUAD:        return ease::inQuad(t);
        case Curve::OUT_QUAD:       return ease::outQuad(t);
        case Curve::IN_OUT_QUAD:    return ease::inOutQuad(t);
        case Curve::IN_CUBIC:       return ease::inCubic(t);
        case Curve::OUT_CUBIC:      return ease::outCubic(t);
        case Curve::IN_OUT_CUBIC:   return ease::inOutCubic(t);
        case Curve::IN_QUART:       return ease::inQuart(t);
        case Curve::OUT_QUART:      return ease::outQuart(t);
        case Curve::IN_OUT_QUART:   return ease::inOutQuart(t);
        case Curve::IN_QUINT:       return ease::inQuint(t);
        case Curve::OUT_QUINT:      return ease::outQuint(t);
        case Curve::IN_OUT_QUINT:   return ease::inOutQuint(t);
        case Curve::IN_EXPO:        return ease::inExpo(t);
        case Curve::OUT_EXPO:       return ease::outExpo(t);
        case Curve::IN_OUT_EXPO:    return ease::inOutExpo(t);
        case Curve::IN_CIRC:        return ease::inCirc(t);
        case Curve::OUT_CIRC:       return ease::outCirc(t);
        case Curve::IN_OUT_CIRC:    return ease::inOutCirc(t);
        case Curve::IN_BACK:        return ease::inBack(t);
        case Curve::OUT_BACK:       return ease::outBack(t);
        case Curve::IN_OUT_BACK:    return ease::inOutBack(t);
        case Curve::IN_ELASTIC:     return ease::inElastic(t);
        case Curve::OUT_ELASTIC:    return ease::outElastic(t);
        case Curve::IN_OUT_ELASTIC: return ease::inOutElastic(t);
        case Curve::IN_BOUNCE:      return ease::inBounce(t);
        case Curve::OUT_BOUNCE:     return ease::outBounce(t);
        case Curve::IN_OUT_BOUNCE:  return ease::inOutBounce(t);
        case Curve::SMOOTH_STEP:    return smoothStep(T(0), T(1), t);
        case Curve::SMOOTHER_STEP:  return smootherStep(T(0), T(1), t);
        default:                    return t;
    }
}

template<typename T>
class CubicBezierEasing {
    static_assert(std::is_floating_point_v<T>);
public:
    CubicBezierEasing() = default;

    CubicBezierEasing(T x1, T y1, T x2, T y2)
        : m_x1(x1), m_y1(y1), m_x2(x2), m_y2(y2)
    {}

    T evaluate(T t) const {
        t = clamp(t, T(0), T(1));
        T param = solveCurveX(t);
        return sampleCurveY(param);
    }

    static CubicBezierEasing easeInOut() {
        return CubicBezierEasing(T(0.42), T(0), T(0.58), T(1));
    }

    static CubicBezierEasing easeIn() {
        return CubicBezierEasing(T(0.42), T(0), T(1), T(1));
    }

    static CubicBezierEasing easeOut() {
        return CubicBezierEasing(T(0), T(0), T(0.58), T(1));
    }

    static CubicBezierEasing snappy() {
        return CubicBezierEasing(T(0.17), T(0.67), T(0.12), T(0.99));
    }

    static CubicBezierEasing organic() {
        return CubicBezierEasing(T(0.25), T(0.1), T(0.25), T(1.0));
    }

    static CubicBezierEasing overshoot() {
        return CubicBezierEasing(T(0.34), T(1.56), T(0.64), T(1));
    }

    static CubicBezierEasing gentle() {
        return CubicBezierEasing(T(0.4), T(0), T(0.2), T(1));
    }

    static CubicBezierEasing anticipate() {
        return CubicBezierEasing(T(0.36), T(-0.2), T(0.7), T(0.18));
    }

    static CubicBezierEasing heavyLanding() {
        return CubicBezierEasing(T(0.6), T(0.04), T(0.98), T(0.335));
    }

    static CubicBezierEasing rubberBand() {
        return CubicBezierEasing(T(0.175), T(0.885), T(0.32), T(1.275));
    }

private:
    T m_x1 = T(0.25), m_y1 = T(0.1);
    T m_x2 = T(0.25), m_y2 = T(1.0);

    T sampleCurveX(T t) const {
        return ((( T(1) - T(3) * m_x2 + T(3) * m_x1) * t
               + (T(3) * m_x2 - T(6) * m_x1)) * t
               + (T(3) * m_x1)) * t;
    }

    T sampleCurveY(T t) const {
        return ((( T(1) - T(3) * m_y2 + T(3) * m_y1) * t
               + (T(3) * m_y2 - T(6) * m_y1)) * t
               + (T(3) * m_y1)) * t;
    }

    T sampleCurveDerivX(T t) const {
        return (T(3) * (T(1) - T(3) * m_x2 + T(3) * m_x1) * t
              + T(2) * (T(3) * m_x2 - T(6) * m_x1)) * t
              + (T(3) * m_x1);
    }

    T solveCurveX(T x) const {
        T t = x;
        for (int i = 0; i < 8; ++i) {
            T residual = sampleCurveX(t) - x;
            if (std::abs(residual) < T(1e-7)) return t;
            T deriv = sampleCurveDerivX(t);
            if (std::abs(deriv) < T(1e-7)) break;
            t -= residual / deriv;
        }

        T lo = T(0), hi = T(1);
        t = x;
        for (int i = 0; i < 20; ++i) {
            T val = sampleCurveX(t);
            if (std::abs(val - x) < T(1e-7)) return t;
            if (x > val) lo = t; else hi = t;
            t = (lo + hi) * T(0.5);
        }
        return t;
    }
};

template<typename T>
class SpringAnimator {
    static_assert(std::is_floating_point_v<T>);
public:
    SpringAnimator() = default;

    SpringAnimator(T stiffness, T damping, T mass = T(1))
        : m_stiffness(stiffness)
        , m_damping(damping)
        , m_mass(mass)
    {
        recompute();
    }

    static SpringAnimator criticallyDamped(T frequency) {
        T omega = TAU<T> * frequency;
        T stiffness = omega * omega;
        T damping = T(2) * omega;
        return SpringAnimator(stiffness, damping);
    }

    static SpringAnimator underdamped(T frequency, T dampingRatio = T(0.5)) {
        T omega = TAU<T> * frequency;
        T stiffness = omega * omega;
        T damping = T(2) * dampingRatio * omega;
        return SpringAnimator(stiffness, damping);
    }

    static SpringAnimator overdamped(T frequency, T dampingRatio = T(2)) {
        T omega = TAU<T> * frequency;
        T stiffness = omega * omega;
        T damping = T(2) * dampingRatio * omega;
        return SpringAnimator(stiffness, damping);
    }

    static SpringAnimator bouncy(T frequency = T(4), T dampingRatio = T(0.3)) {
        return underdamped(frequency, dampingRatio);
    }

    static SpringAnimator stiff(T frequency = T(10)) {
        return criticallyDamped(frequency);
    }

    static SpringAnimator gentle(T frequency = T(2)) {
        return criticallyDamped(frequency);
    }

    static SpringAnimator wobbly(T frequency = T(3), T dampingRatio = T(0.15)) {
        return underdamped(frequency, dampingRatio);
    }

    static SpringAnimator heavyImpact(T frequency = T(6), T dampingRatio = T(0.4)) {
        return underdamped(frequency, dampingRatio);
    }

    void setTarget(T target) { m_target = target; }
    void setStiffness(T k) { m_stiffness = k; recompute(); }
    void setDamping(T c) { m_damping = c; recompute(); }

    void snap(T value) {
        m_position = value;
        m_velocity = T(0);
        m_target = value;
    }

    T update(T dt) {
        T x = m_position - m_target;
        T springForce = -m_stiffness * x;
        T dampForce   = -m_damping * m_velocity;
        T accel       = (springForce + dampForce) / m_mass;

        T halfDtAccel = accel * dt * T(0.5);
        T midVel      = m_velocity + halfDtAccel;
        m_position   += midVel * dt;

        x = m_position - m_target;
        springForce = -m_stiffness * x;
        dampForce   = -m_damping * midVel;
        accel       = (springForce + dampForce) / m_mass;
        m_velocity   = midVel + accel * dt * T(0.5);

        return m_position;
    }

    bool settled(T tolerance = T(0.001)) const {
        return std::abs(m_position - m_target) < tolerance &&
               std::abs(m_velocity) < tolerance;
    }

    T position() const { return m_position; }
    T velocity() const { return m_velocity; }
    T target()   const { return m_target; }
    T dampingRatio() const { return m_zeta; }
    T naturalFrequency() const { return m_omega; }

private:
    T m_stiffness = T(100);
    T m_damping   = T(10);
    T m_mass      = T(1);
    T m_position  = T(0);
    T m_velocity  = T(0);
    T m_target    = T(0);
    T m_omega     = T(10);
    T m_zeta      = T(0.5);

    void recompute() {
        m_omega = std::sqrt(m_stiffness / m_mass);
        if (m_omega > T(0))
            m_zeta = m_damping / (T(2) * m_mass * m_omega);
        else
            m_zeta = T(0);
    }
};

template<typename T>
struct DisneyMotion {
    static_assert(std::is_floating_point_v<T>);

    static T anticipateAndSettle(T t, T anticipation = T(0.15), T overshootAmt = T(0.05)) {
        t = clamp(t, T(0), T(1));
        T anticipEnd = T(0.2);
        T overshootStart = T(0.8);

        if (t < anticipEnd) {
            T phase = t / anticipEnd;
            return -anticipation * ease::inQuad(phase);
        }

        if (t > overshootStart) {
            T phase = (t - overshootStart) / (T(1) - overshootStart);
            T overshoot = overshootAmt * std::sin(phase * PI<T>) * (T(1) - phase);
            T base = lerp(evaluateCurve(Curve::OUT_CUBIC, (t - anticipEnd) / (T(1) - anticipEnd)), T(1), phase);
            return base + overshoot;
        }

        T normalT = (t - anticipEnd) / (overshootStart - anticipEnd);
        return lerp(T(-anticipation), T(1), evaluateCurve(Curve::IN_OUT_CUBIC, normalT));
    }

    static T dropAndBounce(T t, T gravity = T(2.5), T bounciness = T(0.4)) {
        t = clamp(t, T(0), T(1));
        T fallEnd = T(0.4);
        if (t < fallEnd) {
            T phase = t / fallEnd;
            return ease::inQuad(phase);
        }

        T phase = (t - fallEnd) / (T(1) - fallEnd);
        T bounceDecay = std::exp(-gravity * phase);
        T freq = T(3);
        T bounce = bounceDecay * bounciness * std::abs(std::sin(phase * freq * PI<T>));
        return T(1) - bounce;
    }

    static T rubberSnap(T t, T elasticity = T(0.25), T oscillations = T(3)) {
        t = clamp(t, T(0), T(1));
        T base = ease::outQuint(t);
        T decay = std::exp(T(-6) * t);
        T wobble = std::sin(t * oscillations * TAU<T>) * elasticity * decay;
        return base + wobble;
    }

    static T breathe(T t, T depth = T(1)) {
        t = clamp(t, T(0), T(1));
        T inhale = std::sin(t * PI<T>);
        T organic = T(0.7) * inhale + T(0.3) * inhale * inhale;
        return organic * depth;
    }

    static T gentleNod(T t, T amplitude = T(1), T asymmetry = T(0.6)) {
        t = clamp(t, T(0), T(1));
        T peak = asymmetry;
        if (t < peak) {
            T phase = t / peak;
            return amplitude * ease::inOutSine(phase);
        }
        T phase = (t - peak) / (T(1) - peak);
        return amplitude * (T(1) - ease::inOutCubic(phase));
    }

    static T headTilt(T t, T angle = T(1), T settleOscillations = T(2)) {
        t = clamp(t, T(0), T(1));
        T base = ease::outBack(t) * angle;
        T settle = std::exp(T(-4) * t) * std::sin(t * settleOscillations * TAU<T>) * T(0.1) * angle;
        return base + settle;
    }

    static T armDrop(T t) {
        return dropAndBounce(t, T(3.0), T(0.25));
    }

    static T armRaise(T t) {
        return anticipateAndSettle(t, T(0.08), T(0.03));
    }

    static T lookAt(T t) {
        t = clamp(t, T(0), T(1));
        T saccade = ease::outQuart(t);
        T microCorrection = T(0.02) * std::sin(t * T(8) * PI<T>) * std::exp(T(-5) * t);
        return saccade + microCorrection;
    }

    static T surprise(T t) {
        t = clamp(t, T(0), T(1));
        T jolt = ease::outExpo(std::min(t * T(3), T(1)));
        T settle = T(1);
        if (t > T(0.3)) {
            T phase = (t - T(0.3)) / T(0.7);
            settle = T(1) - T(0.15) * ease::inOutSine(phase);
        }
        return jolt * settle;
    }

    static T wave(T t, T frequency = T(2), T decay = T(3)) {
        t = clamp(t, T(0), T(1));
        T envelope = std::exp(-decay * t);
        return std::sin(t * frequency * TAU<T>) * envelope;
    }

    static T squashAndStretch(T t, T squashAmount = T(0.2), T stretchAmount = T(0.15)) {
        t = clamp(t, T(0), T(1));
        if (t < T(0.15)) {
            T phase = t / T(0.15);
            return T(1) - squashAmount * ease::inQuad(phase);
        }
        if (t < T(0.4)) {
            T phase = (t - T(0.15)) / T(0.25);
            return (T(1) - squashAmount) + (squashAmount + stretchAmount) * ease::outQuad(phase);
        }
        if (t < T(0.65)) {
            T phase = (t - T(0.4)) / T(0.25);
            return (T(1) + stretchAmount) - stretchAmount * ease::inOutSine(phase);
        }
        T phase = (t - T(0.65)) / T(0.35);
        T residual = T(0.03) * std::sin(phase * PI<T> * T(2)) * (T(1) - phase);
        return T(1) + residual;
    }

    static T followThrough(T t, T overshootPct = T(0.12), T settleSpeed = T(5)) {
        t = clamp(t, T(0), T(1));
        T base = ease::outCubic(t);
        if (t > T(0.5)) {
            T phase = (t - T(0.5)) / T(0.5);
            T over = overshootPct * std::sin(phase * PI<T>) * std::exp(-settleSpeed * phase);
            base += over;
        }
        return base;
    }

    static T slowInSlowOut(T t, T accelWeight = T(0.5)) {
        t = clamp(t, T(0), T(1));
        T sharp = ease::inOutQuad(t);
        T soft = ease::inOutSine(t);
        return lerp(soft, sharp, accelWeight);
    }

    static T characterStep(T t) {
        t = clamp(t, T(0), T(1));
        if (t < T(0.1)) {
            T phase = t / T(0.1);
            return T(-0.05) * ease::inQuad(phase);
        }
        if (t < T(0.5)) {
            T phase = (t - T(0.1)) / T(0.4);
            return lerp(T(-0.05), T(1.08), ease::outQuad(phase));
        }
        if (t < T(0.75)) {
            T phase = (t - T(0.5)) / T(0.25);
            return lerp(T(1.08), T(0.96), ease::inOutSine(phase));
        }
        T phase = (t - T(0.75)) / T(0.25);
        return lerp(T(0.96), T(1), ease::outSine(phase));
    }

    static T emotionTransition(T t) {
        t = clamp(t, T(0), T(1));
        T smooth = ease::inOutCubic(t);
        T organic = smooth + T(0.02) * std::sin(t * T(5) * PI<T>) * (T(1) - t) * t * T(4);
        return clamp(organic, T(0), T(1));
    }

    static T servoSafe(T t, T maxAccel = T(8)) {
        t = clamp(t, T(0), T(1));
        T raw = ease::inOutQuad(t);
        T secondDeriv = T(4) * (t < T(0.5) ? T(1) : T(-1));
        T scale = std::min(T(1), maxAccel / (std::abs(secondDeriv) + T(0.001)));
        return lerp(t, raw, scale);
    }
};

template<typename T>
class Tween {
    static_assert(std::is_floating_point_v<T>);
public:
    Tween() = default;

    Tween(T from, T to, T durationSec, Curve curve = Curve::IN_OUT_QUAD)
        : m_from(from)
        , m_to(to)
        , m_duration(durationSec)
        , m_curve(curve)
        , m_active(true)
    {}

    void start(T from, T to, T durationSec, Curve curve = Curve::IN_OUT_QUAD) {
        m_from     = from;
        m_to       = to;
        m_duration = durationSec;
        m_curve    = curve;
        m_elapsed  = T(0);
        m_active   = true;
    }

    void startFrom(T to, T durationSec, Curve curve = Curve::IN_OUT_QUAD) {
        start(m_current, to, durationSec, curve);
    }

    T update(T dt) {
        if (!m_active) return m_current;

        m_elapsed += dt;
        T t = (m_duration > T(0)) ? clamp(m_elapsed / m_duration, T(0), T(1)) : T(1);
        T eased = evaluateCurve(m_curve, t);
        m_current = lerp(m_from, m_to, eased);

        if (t >= T(1)) {
            m_active = false;
            m_current = m_to;
        }

        return m_current;
    }

    bool active()  const { return m_active; }
    bool done()    const { return !m_active; }
    T    value()   const { return m_current; }
    T    progress() const {
        return (m_duration > T(0)) ? clamp(m_elapsed / m_duration, T(0), T(1)) : T(1);
    }

    void cancel() { m_active = false; }

    void snap(T value) {
        m_current = value;
        m_from    = value;
        m_to      = value;
        m_active  = false;
        m_elapsed = T(0);
    }

private:
    T     m_from     = T(0);
    T     m_to       = T(0);
    T     m_current  = T(0);
    T     m_duration = T(0);
    T     m_elapsed  = T(0);
    Curve m_curve    = Curve::IN_OUT_QUAD;
    bool  m_active   = false;
};

template<typename T, std::size_t MAX_KEYS = 16>
class KeyframeTimeline {
    static_assert(std::is_floating_point_v<T>);
    static_assert(MAX_KEYS >= 2);
public:
    struct Key {
        T     time   = T(0);
        T     value  = T(0);
        Curve curve  = Curve::IN_OUT_QUAD;
    };

    KeyframeTimeline() = default;

    bool addKey(T time, T value, Curve curve = Curve::IN_OUT_QUAD) {
        if (m_count >= MAX_KEYS) return false;

        std::size_t insertIdx = m_count;
        for (std::size_t i = 0; i < m_count; ++i) {
            if (m_keys[i].time > time) {
                insertIdx = i;
                break;
            }
        }

        for (std::size_t i = m_count; i > insertIdx; --i) {
            m_keys[i] = m_keys[i - 1];
        }

        m_keys[insertIdx] = {time, value, curve};
        ++m_count;
        return true;
    }

    void clear() { m_count = 0; }

    T evaluate(T time) const {
        if (m_count == 0) return T(0);
        if (m_count == 1) return m_keys[0].value;

        if (time <= m_keys[0].time) return m_keys[0].value;
        if (time >= m_keys[m_count - 1].time) return m_keys[m_count - 1].value;

        std::size_t seg = 0;
        for (std::size_t i = 0; i < m_count - 1; ++i) {
            if (time >= m_keys[i].time && time < m_keys[i + 1].time) {
                seg = i;
                break;
            }
        }

        T segDuration = m_keys[seg + 1].time - m_keys[seg].time;
        T localT = (segDuration > T(0)) ? (time - m_keys[seg].time) / segDuration : T(1);
        T eased = evaluateCurve(m_keys[seg + 1].curve, localT);

        return lerp(m_keys[seg].value, m_keys[seg + 1].value, eased);
    }

    T totalDuration() const {
        return (m_count > 0) ? m_keys[m_count - 1].time : T(0);
    }

    std::size_t keyCount() const { return m_count; }

private:
    std::array<Key, MAX_KEYS> m_keys{};
    std::size_t               m_count = 0;
};

template<typename T, std::size_t MAX_KEYS = 16>
class TimelinePlayer {
    static_assert(std::is_floating_point_v<T>);
public:
    TimelinePlayer() = default;

    explicit TimelinePlayer(const KeyframeTimeline<T, MAX_KEYS>& timeline)
        : m_timeline(&timeline)
    {}

    void setTimeline(const KeyframeTimeline<T, MAX_KEYS>& timeline) {
        m_timeline = &timeline;
        m_elapsed = T(0);
        m_playing = false;
    }

    void play() {
        m_elapsed = T(0);
        m_playing = true;
    }

    void playReverse() {
        m_elapsed = m_timeline ? m_timeline->totalDuration() : T(0);
        m_playbackRate = T(-1);
        m_playing = true;
    }

    void pause()  { m_playing = false; }
    void resume() { m_playing = true; }

    void setPlaybackRate(T rate) { m_playbackRate = rate; }
    void setLooping(bool loop) { m_looping = loop; }
    void setPingPong(bool pp) { m_pingPong = pp; }

    T update(T dt) {
        if (!m_playing || !m_timeline) return m_value;

        m_elapsed += dt * m_playbackRate;
        T duration = m_timeline->totalDuration();

        if (duration <= T(0)) {
            m_playing = false;
            return m_value;
        }

        if (m_looping) {
            if (m_pingPong) {
                T cycle = duration * T(2);
                T wrapped = std::fmod(m_elapsed, cycle);
                if (wrapped < T(0)) wrapped += cycle;
                m_elapsed = (wrapped > duration) ? cycle - wrapped : wrapped;
            } else {
                m_elapsed = std::fmod(m_elapsed, duration);
                if (m_elapsed < T(0)) m_elapsed += duration;
            }
        } else {
            if (m_elapsed >= duration) {
                m_elapsed = duration;
                m_playing = false;
            } else if (m_elapsed < T(0)) {
                m_elapsed = T(0);
                m_playing = false;
            }
        }

        m_value = m_timeline->evaluate(m_elapsed);
        return m_value;
    }

    bool playing()  const { return m_playing; }
    T    value()    const { return m_value; }
    T    elapsed()  const { return m_elapsed; }
    T    progress() const {
        if (!m_timeline) return T(0);
        T dur = m_timeline->totalDuration();
        return (dur > T(0)) ? clamp(m_elapsed / dur, T(0), T(1)) : T(1);
    }

private:
    const KeyframeTimeline<T, MAX_KEYS>* m_timeline = nullptr;
    T    m_elapsed      = T(0);
    T    m_value        = T(0);
    T    m_playbackRate = T(1);
    bool m_playing      = false;
    bool m_looping      = false;
    bool m_pingPong     = false;
};

template<typename T, std::size_t MAX_SEGMENTS = 8>
class MotionSequence {
    static_assert(std::is_floating_point_v<T>);
public:
    struct Segment {
        T     targetValue = T(0);
        T     duration    = T(0);
        Curve curve       = Curve::IN_OUT_QUAD;
        T     delayBefore = T(0);
    };

    MotionSequence() = default;

    bool addSegment(T targetValue, T duration, Curve curve = Curve::IN_OUT_QUAD, T delayBefore = T(0)) {
        if (m_segCount >= MAX_SEGMENTS) return false;
        m_segments[m_segCount++] = {targetValue, duration, curve, delayBefore};
        return true;
    }

    void clear() {
        m_segCount  = 0;
        m_active    = false;
        m_elapsed   = T(0);
        m_segIndex  = 0;
    }

    void play(T startValue) {
        m_startValue = startValue;
        m_value      = startValue;
        m_elapsed    = T(0);
        m_segIndex   = 0;
        m_active     = true;
        m_segElapsed = T(0);
        m_inDelay    = (m_segCount > 0 && m_segments[0].delayBefore > T(0));
        m_segFrom    = startValue;
    }

    T update(T dt) {
        if (!m_active || m_segCount == 0) return m_value;

        m_elapsed    += dt;
        m_segElapsed += dt;

        if (m_inDelay) {
            if (m_segElapsed >= m_segments[m_segIndex].delayBefore) {
                m_segElapsed -= m_segments[m_segIndex].delayBefore;
                m_inDelay = false;
            } else {
                return m_value;
            }
        }

        const Segment& seg = m_segments[m_segIndex];
        T t = (seg.duration > T(0)) ? clamp(m_segElapsed / seg.duration, T(0), T(1)) : T(1);
        T eased = evaluateCurve(seg.curve, t);
        m_value = lerp(m_segFrom, seg.targetValue, eased);

        if (t >= T(1)) {
            m_value = seg.targetValue;
            ++m_segIndex;

            if (m_segIndex >= m_segCount) {
                m_active = false;
                return m_value;
            }

            m_segFrom    = m_value;
            m_segElapsed = T(0);
            m_inDelay    = (m_segments[m_segIndex].delayBefore > T(0));
        }

        return m_value;
    }

    bool active() const { return m_active; }
    bool done()   const { return !m_active && m_segCount > 0; }
    T    value()  const { return m_value; }
    std::size_t currentSegment() const { return m_segIndex; }

private:
    std::array<Segment, MAX_SEGMENTS> m_segments{};
    std::size_t m_segCount  = 0;
    std::size_t m_segIndex  = 0;
    T           m_startValue = T(0);
    T           m_segFrom    = T(0);
    T           m_value      = T(0);
    T           m_elapsed    = T(0);
    T           m_segElapsed = T(0);
    bool        m_active     = false;
    bool        m_inDelay    = false;
};

template<typename T>
class MotionBlender {
    static_assert(std::is_floating_point_v<T>);
public:
    MotionBlender() = default;

    T blend(T a, T b, T weight) const {
        weight = clamp(weight, T(0), T(1));
        return lerp(a, b, weight);
    }

    T crossfade(T a, T b, T t, Curve curve = Curve::IN_OUT_QUAD) const {
        T eased = evaluateCurve(curve, clamp(t, T(0), T(1)));
        return lerp(a, b, eased);
    }

    T additiveBlend(T base, T overlay, T weight) const {
        return base + overlay * clamp(weight, T(0), T(1));
    }

    T multiplicativeBlend(T a, T b, T weight) const {
        weight = clamp(weight, T(0), T(1));
        return a * lerp(T(1), b, weight);
    }

    T smoothTransition(T from, T to, T t, T smoothness = T(0.5)) const {
        t = clamp(t, T(0), T(1));
        T hermiteT = smoothStep(T(0), T(1), t);
        T linearT = t;
        return lerp(from, to, lerp(linearT, hermiteT, clamp(smoothness, T(0), T(1))));
    }
};

template<typename T>
class LFO {
    static_assert(std::is_floating_point_v<T>);
public:
    enum class Shape : uint8_t {
        SINE,
        TRIANGLE,
        SAWTOOTH,
        SQUARE,
        SMOOTH_RANDOM
    };

    LFO() = default;

    LFO(T frequencyHz, T amplitude = T(1), T offset = T(0), Shape shape = Shape::SINE)
        : m_freq(frequencyHz)
        , m_amp(amplitude)
        , m_offset(offset)
        , m_shape(shape)
    {}

    T update(T dt) {
        m_phase += m_freq * dt;
        if (m_phase > T(1)) m_phase -= std::floor(m_phase);

        T raw = T(0);
        switch (m_shape) {
            case Shape::SINE:
                raw = std::sin(m_phase * TAU<T>);
                break;
            case Shape::TRIANGLE:
                raw = T(4) * std::abs(m_phase - T(0.5)) - T(1);
                break;
            case Shape::SAWTOOTH:
                raw = T(2) * m_phase - T(1);
                break;
            case Shape::SQUARE:
                raw = m_phase < T(0.5) ? T(1) : T(-1);
                break;
            case Shape::SMOOTH_RANDOM:
                raw = smoothRandom();
                break;
        }

        m_value = m_offset + raw * m_amp;
        return m_value;
    }

    void setFrequency(T hz) { m_freq = hz; }
    void setAmplitude(T amp) { m_amp = amp; }
    void setOffset(T off) { m_offset = off; }
    void setShape(Shape s) { m_shape = s; }
    void setPhase(T p) { m_phase = p; }

    T    value() const { return m_value; }
    T    phase() const { return m_phase; }

    void reset() {
        m_phase = T(0);
        m_value = m_offset;
        m_rngState = 0x12345678;
        m_smoothTargetA = T(0);
        m_smoothTargetB = T(0);
        m_smoothPos = T(0);
    }

private:
    T m_freq   = T(1);
    T m_amp    = T(1);
    T m_offset = T(0);
    T m_phase  = T(0);
    T m_value  = T(0);
    Shape m_shape = Shape::SINE;

    uint32_t m_rngState = 0x12345678;
    T        m_smoothTargetA = T(0);
    T        m_smoothTargetB = T(0);
    T        m_smoothPos     = T(0);

    uint32_t xorshift() {
        m_rngState ^= m_rngState << 13;
        m_rngState ^= m_rngState >> 17;
        m_rngState ^= m_rngState << 5;
        return m_rngState;
    }

    T randomNorm() {
        return static_cast<T>(xorshift()) / static_cast<T>(0xFFFFFFFF) * T(2) - T(1);
    }

    T smoothRandom() {
        if (m_phase < m_smoothPos) {
            m_smoothTargetA = m_smoothTargetB;
            m_smoothTargetB = randomNorm();
        }
        m_smoothPos = m_phase;
        T t = m_phase;
        T cubic = t * t * (T(3) - T(2) * t);
        return lerp(m_smoothTargetA, m_smoothTargetB, cubic);
    }
};

template<typename T>
class OrganicNoise {
    static_assert(std::is_floating_point_v<T>);
public:
    OrganicNoise() = default;

    OrganicNoise(T baseFreq, T amplitude)
        : m_lfo1(baseFreq, amplitude * T(0.6))
        , m_lfo2(baseFreq * T(2.17), amplitude * T(0.25))
        , m_lfo3(baseFreq * T(4.73), amplitude * T(0.15))
    {}

    T update(T dt) {
        T v1 = m_lfo1.update(dt);
        T v2 = m_lfo2.update(dt);
        T v3 = m_lfo3.update(dt);
        m_value = v1 + v2 + v3;
        return m_value;
    }

    T value() const { return m_value; }

    void reset() {
        m_lfo1.reset();
        m_lfo2.reset();
        m_lfo3.reset();
        m_value = T(0);
    }

private:
    LFO<T> m_lfo1{T(0.3), T(0.6)};
    LFO<T> m_lfo2{T(0.651), T(0.25)};
    LFO<T> m_lfo3{T(1.419), T(0.15)};
    T      m_value = T(0);
};

template<typename T>
class TrajectoryPlanner {
    static_assert(std::is_floating_point_v<T>);
public:
    TrajectoryPlanner() = default;

    TrajectoryPlanner(T maxVelocity, T maxAcceleration)
        : m_maxVel(std::abs(maxVelocity))
        , m_maxAcc(std::abs(maxAcceleration))
    {}

    void setTarget(T pos) { m_target = pos; }
    void setLimits(T maxVel, T maxAcc) { m_maxVel = std::abs(maxVel); m_maxAcc = std::abs(maxAcc); }

    T update(T dt) {
        T error = m_target - m_position;
        T dir = (error > T(0)) ? T(1) : T(-1);
        T absError = std::abs(error);

        T brakingDist = (m_velocity * m_velocity) / (T(2) * m_maxAcc);

        T desiredVel;
        if (absError <= brakingDist + T(0.001)) {
            desiredVel = dir * std::sqrt(T(2) * m_maxAcc * absError + T(0.0001));
            desiredVel = clamp(desiredVel, -m_maxVel, m_maxVel);
        } else {
            desiredVel = dir * m_maxVel;
        }

        T velError = desiredVel - m_velocity;
        T maxDeltaV = m_maxAcc * dt;
        m_velocity += clamp(velError, -maxDeltaV, maxDeltaV);
        m_velocity = clamp(m_velocity, -m_maxVel, m_maxVel);

        m_position += m_velocity * dt;

        if (absError < T(0.001) && std::abs(m_velocity) < T(0.001)) {
            m_position = m_target;
            m_velocity = T(0);
        }

        return m_position;
    }

    void snap(T pos) {
        m_position = pos;
        m_velocity = T(0);
        m_target   = pos;
    }

    bool settled(T tolerance = T(0.001)) const {
        return std::abs(m_position - m_target) < tolerance &&
               std::abs(m_velocity) < tolerance;
    }

    T position() const { return m_position; }
    T velocity() const { return m_velocity; }
    T target()   const { return m_target; }

private:
    T m_maxVel    = T(100);
    T m_maxAcc    = T(200);
    T m_position  = T(0);
    T m_velocity  = T(0);
    T m_target    = T(0);
};

template<typename T>
class ServoProfile {
    static_assert(std::is_floating_point_v<T>);
public:
    ServoProfile() = default;

    ServoProfile(T minAngleDeg, T maxAngleDeg, T maxDegreesPerSec, T maxAccelDegPerSec2)
        : m_minAngle(minAngleDeg)
        , m_maxAngle(maxAngleDeg)
        , m_planner(maxDegreesPerSec, maxAccelDegPerSec2)
    {}

    void setTarget(T angleDeg) {
        m_planner.setTarget(clamp(angleDeg, m_minAngle, m_maxAngle));
    }

    void setTargetNormalized(T t) {
        setTarget(lerp(m_minAngle, m_maxAngle, clamp(t, T(0), T(1))));
    }

    T update(T dt) {
        return m_planner.update(dt);
    }

    void snap(T angleDeg) {
        m_planner.snap(clamp(angleDeg, m_minAngle, m_maxAngle));
    }

    bool settled(T tolerance = T(0.1)) const { return m_planner.settled(tolerance); }

    T angle()      const { return m_planner.position(); }
    T velocity()   const { return m_planner.velocity(); }
    T normalized() const {
        T range = m_maxAngle - m_minAngle;
        return (range > T(0)) ? (m_planner.position() - m_minAngle) / range : T(0);
    }

    T toPulseUs(T minPulse = T(500), T maxPulse = T(2500)) const {
        return lerp(minPulse, maxPulse, normalized());
    }

private:
    T m_minAngle = T(0);
    T m_maxAngle = T(180);
    TrajectoryPlanner<T> m_planner{T(200), T(400)};
};

template<typename T>
struct EasingPresets {
    static Curve headNod()         { return Curve::IN_OUT_QUAD; }
    static Curve headTilt()        { return Curve::IN_OUT_CUBIC; }
    static Curve eyeBlink()        { return Curve::IN_OUT_SINE; }
    static Curve eyeGaze()         { return Curve::OUT_QUART; }
    static Curve armRaise()        { return Curve::IN_OUT_CUBIC; }
    static Curve armDrop()         { return Curve::IN_QUAD; }
    static Curve armWave()         { return Curve::IN_OUT_SINE; }
    static Curve expressionChange(){ return Curve::IN_OUT_QUAD; }
    static Curve breathe()         { return Curve::IN_OUT_SINE; }
    static Curve wakeUp()          { return Curve::OUT_EXPO; }
    static Curve fallAsleep()      { return Curve::IN_CUBIC; }
    static Curve alert()           { return Curve::OUT_BACK; }
    static Curve relaxed()         { return Curve::OUT_SINE; }
    static Curve curious()         { return Curve::IN_OUT_BACK; }
    static Curve surprised()       { return Curve::OUT_ELASTIC; }
    static Curve sad()             { return Curve::IN_OUT_SINE; }
    static Curve happy()           { return Curve::OUT_BOUNCE; }
    static Curve thinking()        { return Curve::IN_OUT_QUART; }
    static Curve servoDefault()    { return Curve::IN_OUT_QUAD; }
    static Curve ledFade()         { return Curve::OUT_EXPO; }
    static Curve motorRamp()       { return Curve::SMOOTH_STEP; }
};

template<typename T>
struct EasingChains {
    static_assert(std::is_floating_point_v<T>);

    static T anticipateAction(T t) {
        return DisneyMotion<T>::anticipateAndSettle(t, T(0.12), T(0.04));
    }

    static T bounceSettle(T t) {
        return DisneyMotion<T>::dropAndBounce(t, T(2.5), T(0.35));
    }

    static T elasticSnap(T t) {
        return DisneyMotion<T>::rubberSnap(t, T(0.2), T(3));
    }

    static T heavyDrop(T t) {
        t = clamp(t, T(0), T(1));
        T fall = ease::inQuad(std::min(t * T(2), T(1)));
        if (t < T(0.5)) return fall;
        T phase = (t - T(0.5)) / T(0.5);
        T squash = T(1) - T(0.15) * std::exp(T(-8) * phase) * std::cos(phase * T(4) * PI<T>);
        return squash;
    }

    static T gentleFloat(T t) {
        t = clamp(t, T(0), T(1));
        T base = ease::inOutSine(t);
        T float_ = T(0.03) * std::sin(t * T(6) * PI<T>) * (T(1) - t);
        return base + float_;
    }

    static T mechanicalToOrganic(T t, T organicWeight = T(0.7)) {
        t = clamp(t, T(0), T(1));
        T mechanical = t;
        T organic = ease::inOutCubic(t);
        return lerp(mechanical, organic, clamp(organicWeight, T(0), T(1)));
    }

    static T saccadicLook(T t) {
        return DisneyMotion<T>::lookAt(t);
    }

    static T expressionMorph(T t) {
        return DisneyMotion<T>::emotionTransition(t);
    }

    static T blinkCurve(T t, bool closing) {
        t = clamp(t, T(0), T(1));
        if (closing) {
            return T(1) - ease::inQuad(t);
        }
        T base = ease::outCubic(t);
        T overshoot = T(0.04) * std::sin(t * PI<T>) * (T(1) - t);
        return base + overshoot;
    }

    static T breathingCurve(T t) {
        return DisneyMotion<T>::breathe(t);
    }

    static T asymmetricPulse(T t, T riseRatio = T(0.3)) {
        t = clamp(t, T(0), T(1));
        if (t < riseRatio) {
            return ease::outQuad(t / riseRatio);
        }
        return ease::inCubic(T(1) - (t - riseRatio) / (T(1) - riseRatio));
    }
};

template<typename T>
class MultiChannelAnimator {
    static_assert(std::is_floating_point_v<T>);
    static constexpr std::size_t MAX_CHANNELS = 16;
public:
    MultiChannelAnimator() = default;

    void setChannelCount(std::size_t n) {
        m_channelCount = (n <= MAX_CHANNELS) ? n : MAX_CHANNELS;
    }

    void startTween(std::size_t ch, T from, T to, T duration, Curve curve = Curve::IN_OUT_QUAD) {
        if (ch < m_channelCount) {
            m_tweens[ch].start(from, to, duration, curve);
        }
    }

    void startTweenFrom(std::size_t ch, T to, T duration, Curve curve = Curve::IN_OUT_QUAD) {
        if (ch < m_channelCount) {
            m_tweens[ch].startFrom(to, duration, curve);
        }
    }

    void snapChannel(std::size_t ch, T value) {
        if (ch < m_channelCount) {
            m_tweens[ch].snap(value);
        }
    }

    void update(T dt) {
        m_anyActive = false;
        for (std::size_t i = 0; i < m_channelCount; ++i) {
            m_tweens[i].update(dt);
            if (m_tweens[i].active()) m_anyActive = true;
        }
    }

    T channelValue(std::size_t ch) const {
        return (ch < m_channelCount) ? m_tweens[ch].value() : T(0);
    }

    bool channelActive(std::size_t ch) const {
        return (ch < m_channelCount) ? m_tweens[ch].active() : false;
    }

    bool anyActive() const { return m_anyActive; }

    bool allSettled() const {
        for (std::size_t i = 0; i < m_channelCount; ++i) {
            if (m_tweens[i].active()) return false;
        }
        return true;
    }

    std::size_t channelCount() const { return m_channelCount; }

    void stopAll() {
        for (std::size_t i = 0; i < m_channelCount; ++i) {
            m_tweens[i].cancel();
        }
        m_anyActive = false;
    }

private:
    std::array<Tween<T>, MAX_CHANNELS> m_tweens{};
    std::size_t m_channelCount = 0;
    bool        m_anyActive    = false;
};

template<typename T>
class DelayedAction {
    static_assert(std::is_floating_point_v<T>);
public:
    DelayedAction() = default;

    void schedule(T delaySec) {
        m_delay   = delaySec;
        m_elapsed = T(0);
        m_pending = true;
        m_fired   = false;
    }

    bool update(T dt) {
        if (!m_pending) return false;
        m_elapsed += dt;
        if (m_elapsed >= m_delay) {
            m_pending = false;
            m_fired   = true;
            return true;
        }
        return false;
    }

    void cancel() {
        m_pending = false;
        m_fired   = false;
    }

    bool pending() const { return m_pending; }
    bool fired()   const { return m_fired; }
    T    remaining() const { return m_pending ? std::max(T(0), m_delay - m_elapsed) : T(0); }

private:
    T    m_delay   = T(0);
    T    m_elapsed = T(0);
    bool m_pending = false;
    bool m_fired   = false;
};

template<typename T>
class PulseGenerator {
    static_assert(std::is_floating_point_v<T>);
public:
    PulseGenerator() = default;

    PulseGenerator(T periodSec, T dutyCycle = T(0.5), Curve riseCurve = Curve::IN_OUT_SINE, Curve fallCurve = Curve::IN_OUT_SINE)
        : m_period(periodSec)
        , m_duty(clamp(dutyCycle, T(0), T(1)))
        , m_riseCurve(riseCurve)
        , m_fallCurve(fallCurve)
    {}

    T update(T dt) {
        m_phase += dt / m_period;
        if (m_phase > T(1)) m_phase -= std::floor(m_phase);

        T riseEnd = m_duty * T(0.5);
        T highEnd = m_duty;
        T fallEnd = m_duty + (T(1) - m_duty) * T(0.5);

        if (m_phase < riseEnd) {
            m_value = evaluateCurve(m_riseCurve, m_phase / riseEnd);
        } else if (m_phase < highEnd) {
            m_value = T(1);
        } else if (m_phase < fallEnd) {
            T t = (m_phase - highEnd) / (fallEnd - highEnd);
            m_value = T(1) - evaluateCurve(m_fallCurve, t);
        } else {
            m_value = T(0);
        }

        return m_value;
    }

    T value() const { return m_value; }

    void setPeriod(T sec) { m_period = sec; }
    void setDuty(T duty) { m_duty = clamp(duty, T(0), T(1)); }

    void reset() {
        m_phase = T(0);
        m_value = T(0);
    }

private:
    T     m_period    = T(1);
    T     m_duty      = T(0.5);
    Curve m_riseCurve = Curve::IN_OUT_SINE;
    Curve m_fallCurve = Curve::IN_OUT_SINE;
    T     m_phase     = T(0);
    T     m_value     = T(0);
};

template<typename T>
T chain(T t, T splitPoint, T(*firstHalf)(T), T(*secondHalf)(T)) {
    t = clamp(t, T(0), T(1));
    if (t < splitPoint) {
        return firstHalf(t / splitPoint) * splitPoint;
    }
    T phase = (t - splitPoint) / (T(1) - splitPoint);
    return splitPoint + secondHalf(phase) * (T(1) - splitPoint);
}

template<typename T>
T mirror(T t, T(*func)(T)) {
    t = clamp(t, T(0), T(1));
    if (t < T(0.5)) {
        return func(t * T(2));
    }
    return func((T(1) - t) * T(2));
}

template<typename T>
T reverse(T t, T(*func)(T)) {
    return func(T(1) - clamp(t, T(0), T(1)));
}

template<typename T>
T power(T t, T exponent) {
    t = clamp(t, T(0), T(1));
    return std::pow(t, exponent);
}

template<typename T>
T inOut(T t, T(*inFunc)(T), T(*outFunc)(T)) {
    t = clamp(t, T(0), T(1));
    if (t < T(0.5)) {
        return inFunc(t * T(2)) * T(0.5);
    }
    return T(0.5) + outFunc((t - T(0.5)) * T(2)) * T(0.5);
}

template<typename T>
T blend(T t, T(*funcA)(T), T(*funcB)(T), T weight) {
    t = clamp(t, T(0), T(1));
    return lerp(funcA(t), funcB(t), clamp(weight, T(0), T(1)));
}

template<typename T>
T scale(T t, T(*func)(T), T amplitude, T offset = T(0)) {
    return offset + func(clamp(t, T(0), T(1))) * amplitude;
}

template<typename T>
T repeat(T t, T(*func)(T), int count) {
    t = clamp(t, T(0), T(1));
    T scaled = t * static_cast<T>(count);
    T local = scaled - std::floor(scaled);
    return func(local);
}

template<typename T>
T pingPong(T t, T(*func)(T)) {
    t = clamp(t, T(0), T(1));
    T doubled = t * T(2);
    if (doubled > T(1)) doubled = T(2) - doubled;
    return func(doubled);
}

}
}
