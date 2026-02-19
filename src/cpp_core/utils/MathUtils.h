#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <array>
#include <algorithm>
#include <type_traits>

namespace dsp {

template<typename T = double>
constexpr T PI = T(3.14159265358979323846264338327950288);

template<typename T = double>
constexpr T TAU = T(6.28318530717958647692528676655900577);

template<typename T = double>
constexpr T HALF_PI = T(1.57079632679489661923132169163975144);

template<typename T>
constexpr T clamp(T value, T lo, T hi) {
    return (value < lo) ? lo : (value > hi) ? hi : value;
}

template<typename T>
constexpr T lerp(T a, T b, T t) {
    return a + (b - a) * t;
}

template<typename T>
constexpr T inverseLerp(T a, T b, T value) {
    return (b - a) != T(0) ? (value - a) / (b - a) : T(0);
}

template<typename T>
constexpr T remap(T inLo, T inHi, T outLo, T outHi, T value) {
    T t = inverseLerp(inLo, inHi, value);
    return lerp(outLo, outHi, t);
}

template<typename T>
constexpr T smoothStep(T edge0, T edge1, T x) {
    T t = clamp((x - edge0) / (edge1 - edge0), T(0), T(1));
    return t * t * (T(3) - T(2) * t);
}

template<typename T>
constexpr T smootherStep(T edge0, T edge1, T x) {
    T t = clamp((x - edge0) / (edge1 - edge0), T(0), T(1));
    return t * t * t * (t * (t * T(6) - T(15)) + T(10));
}

template<typename T>
constexpr T hermite(T p0, T m0, T p1, T m1, T t) {
    T t2 = t * t;
    T t3 = t2 * t;
    T h00 = T(2) * t3 - T(3) * t2 + T(1);
    T h10 = t3 - T(2) * t2 + t;
    T h01 = T(-2) * t3 + T(3) * t2;
    T h11 = t3 - t2;
    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;
}

template<typename T>
constexpr T catmullRom(T p0, T p1, T p2, T p3, T t) {
    T m0 = (p2 - p0) * T(0.5);
    T m1 = (p3 - p1) * T(0.5);
    return hermite(p1, m0, p2, m1, t);
}

namespace ease {

template<typename T>
constexpr T inSine(T t) {
    return T(1) - std::cos(t * HALF_PI<T>);
}

template<typename T>
constexpr T outSine(T t) {
    return std::sin(t * HALF_PI<T>);
}

template<typename T>
constexpr T inOutSine(T t) {
    return T(-0.5) * (std::cos(PI<T> * t) - T(1));
}

template<typename T>
constexpr T inQuad(T t) {
    return t * t;
}

template<typename T>
constexpr T outQuad(T t) {
    return t * (T(2) - t);
}

template<typename T>
constexpr T inOutQuad(T t) {
    return t < T(0.5) ? T(2) * t * t : T(-1) + (T(4) - T(2) * t) * t;
}

template<typename T>
constexpr T inCubic(T t) {
    return t * t * t;
}

template<typename T>
constexpr T outCubic(T t) {
    T u = t - T(1);
    return u * u * u + T(1);
}

template<typename T>
constexpr T inOutCubic(T t) {
    return t < T(0.5) ? T(4) * t * t * t : T(1) + (t - T(1)) * (T(2) * t - T(2)) * (T(2) * t - T(2));
}

template<typename T>
constexpr T inQuart(T t) {
    return t * t * t * t;
}

template<typename T>
constexpr T outQuart(T t) {
    T u = t - T(1);
    return T(1) - u * u * u * u;
}

template<typename T>
constexpr T inOutQuart(T t) {
    T u = t - T(1);
    return t < T(0.5) ? T(8) * t * t * t * t : T(1) - T(8) * u * u * u * u;
}

template<typename T>
constexpr T inQuint(T t) {
    return t * t * t * t * t;
}

template<typename T>
constexpr T outQuint(T t) {
    T u = t - T(1);
    return T(1) + u * u * u * u * u;
}

template<typename T>
constexpr T inOutQuint(T t) {
    T u = t - T(1);
    return t < T(0.5) ? T(16) * t * t * t * t * t : T(1) + T(16) * u * u * u * u * u;
}

template<typename T>
constexpr T inExpo(T t) {
    return t <= T(0) ? T(0) : std::pow(T(2), T(10) * (t - T(1)));
}

template<typename T>
constexpr T outExpo(T t) {
    return t >= T(1) ? T(1) : T(1) - std::pow(T(2), T(-10) * t);
}

template<typename T>
constexpr T inOutExpo(T t) {
    if (t <= T(0)) return T(0);
    if (t >= T(1)) return T(1);
    return t < T(0.5)
        ? std::pow(T(2), T(20) * t - T(10)) * T(0.5)
        : (T(2) - std::pow(T(2), T(-20) * t + T(10))) * T(0.5);
}

template<typename T>
constexpr T inCirc(T t) {
    return T(1) - std::sqrt(T(1) - t * t);
}

template<typename T>
constexpr T outCirc(T t) {
    T u = t - T(1);
    return std::sqrt(T(1) - u * u);
}

template<typename T>
constexpr T inOutCirc(T t) {
    T u = t * T(2);
    if (u < T(1)) return T(-0.5) * (std::sqrt(T(1) - u * u) - T(1));
    u -= T(2);
    return T(0.5) * (std::sqrt(T(1) - u * u) + T(1));
}

template<typename T>
constexpr T inBack(T t) {
    constexpr T s = T(1.70158);
    return t * t * ((s + T(1)) * t - s);
}

template<typename T>
constexpr T outBack(T t) {
    constexpr T s = T(1.70158);
    T u = t - T(1);
    return u * u * ((s + T(1)) * u + s) + T(1);
}

template<typename T>
constexpr T inOutBack(T t) {
    constexpr T s = T(1.70158) * T(1.525);
    T u = t * T(2);
    if (u < T(1)) return T(0.5) * (u * u * ((s + T(1)) * u - s));
    u -= T(2);
    return T(0.5) * (u * u * ((s + T(1)) * u + s) + T(2));
}

template<typename T>
constexpr T inElastic(T t) {
    if (t <= T(0)) return T(0);
    if (t >= T(1)) return T(1);
    return -std::pow(T(2), T(10) * t - T(10)) * std::sin((t * T(10) - T(10.75)) * TAU<T> / T(3));
}

template<typename T>
constexpr T outElastic(T t) {
    if (t <= T(0)) return T(0);
    if (t >= T(1)) return T(1);
    return std::pow(T(2), T(-10) * t) * std::sin((t * T(10) - T(0.75)) * TAU<T> / T(3)) + T(1);
}

template<typename T>
constexpr T inOutElastic(T t) {
    if (t <= T(0)) return T(0);
    if (t >= T(1)) return T(1);
    constexpr T c = TAU<T> / T(4.5);
    return t < T(0.5)
        ? -(std::pow(T(2), T(20) * t - T(10)) * std::sin((T(20) * t - T(11.125)) * c)) * T(0.5)
        : (std::pow(T(2), T(-20) * t + T(10)) * std::sin((T(20) * t - T(11.125)) * c)) * T(0.5) + T(1);
}

template<typename T>
constexpr T outBounce(T t) {
    constexpr T n1 = T(7.5625);
    constexpr T d1 = T(2.75);
    if (t < T(1) / d1)         return n1 * t * t;
    if (t < T(2) / d1)       { t -= T(1.5) / d1;   return n1 * t * t + T(0.75); }
    if (t < T(2.5) / d1)     { t -= T(2.25) / d1;  return n1 * t * t + T(0.9375); }
    t -= T(2.625) / d1;
    return n1 * t * t + T(0.984375);
}

template<typename T>
constexpr T inBounce(T t) {
    return T(1) - outBounce(T(1) - t);
}

template<typename T>
constexpr T inOutBounce(T t) {
    return t < T(0.5)
        ? (T(1) - outBounce(T(1) - T(2) * t)) * T(0.5)
        : (T(1) + outBounce(T(2) * t - T(1))) * T(0.5);
}

}

template<typename T>
class ExponentialSmoother {
    static_assert(std::is_floating_point_v<T>);
public:
    ExponentialSmoother() = default;

    explicit ExponentialSmoother(T alpha, T initial = T(0))
        : m_alpha(clamp(alpha, T(0), T(1)))
        , m_value(initial)
        , m_initialized(true)
    {}

    void setAlpha(T alpha) { m_alpha = clamp(alpha, T(0), T(1)); }

    static T alphaFromCutoff(T cutoffHz, T sampleRateHz) {
        T rc = T(1) / (TAU<T> * cutoffHz);
        T dt = T(1) / sampleRateHz;
        return dt / (rc + dt);
    }

    T process(T input) {
        if (!m_initialized) {
            m_value = input;
            m_initialized = true;
            return m_value;
        }
        m_value += m_alpha * (input - m_value);
        return m_value;
    }

    void reset(T value = T(0)) {
        m_value = value;
        m_initialized = false;
    }

    T value() const { return m_value; }

private:
    T    m_alpha       = T(0.1);
    T    m_value       = T(0);
    bool m_initialized = false;
};

template<typename T, std::size_t N>
class MovingAverage {
    static_assert(std::is_floating_point_v<T>);
    static_assert(N > 0);
public:
    MovingAverage() { m_buffer.fill(T(0)); }

    T process(T input) {
        m_sum -= m_buffer[m_index];
        m_buffer[m_index] = input;
        m_sum += input;
        m_index = (m_index + 1) % N;
        if (m_count < N) ++m_count;
        return m_sum / static_cast<T>(m_count);
    }

    void reset() {
        m_buffer.fill(T(0));
        m_sum   = T(0);
        m_index = 0;
        m_count = 0;
    }

    T value() const { return m_count > 0 ? m_sum / static_cast<T>(m_count) : T(0); }
    std::size_t count() const { return m_count; }

private:
    std::array<T, N> m_buffer{};
    T                m_sum   = T(0);
    std::size_t      m_index = 0;
    std::size_t      m_count = 0;
};

template<typename T>
class BiquadFilter {
    static_assert(std::is_floating_point_v<T>);
public:
    enum class Type : uint8_t {
        LOWPASS,
        HIGHPASS,
        BANDPASS,
        NOTCH,
        ALLPASS
    };

    BiquadFilter() = default;

    void configure(Type type, T sampleRate, T cutoffHz, T Q = T(0.7071)) {
        T w0    = TAU<T> * cutoffHz / sampleRate;
        T sinW0 = std::sin(w0);
        T cosW0 = std::cos(w0);
        T alpha  = sinW0 / (T(2) * Q);

        T b0, b1, b2, a0, a1, a2;

        switch (type) {
            case Type::LOWPASS:
                b0 = (T(1) - cosW0) * T(0.5);
                b1 =  T(1) - cosW0;
                b2 = (T(1) - cosW0) * T(0.5);
                a0 =  T(1) + alpha;
                a1 = T(-2) * cosW0;
                a2 =  T(1) - alpha;
                break;

            case Type::HIGHPASS:
                b0 =  (T(1) + cosW0) * T(0.5);
                b1 = -(T(1) + cosW0);
                b2 =  (T(1) + cosW0) * T(0.5);
                a0 =  T(1) + alpha;
                a1 = T(-2) * cosW0;
                a2 =  T(1) - alpha;
                break;

            case Type::BANDPASS:
                b0 =  alpha;
                b1 =  T(0);
                b2 = -alpha;
                a0 =  T(1) + alpha;
                a1 = T(-2) * cosW0;
                a2 =  T(1) - alpha;
                break;

            case Type::NOTCH:
                b0 =  T(1);
                b1 = T(-2) * cosW0;
                b2 =  T(1);
                a0 =  T(1) + alpha;
                a1 = T(-2) * cosW0;
                a2 =  T(1) - alpha;
                break;

            case Type::ALLPASS:
                b0 =  T(1) - alpha;
                b1 = T(-2) * cosW0;
                b2 =  T(1) + alpha;
                a0 =  T(1) + alpha;
                a1 = T(-2) * cosW0;
                a2 =  T(1) - alpha;
                break;
        }

        T invA0 = T(1) / a0;
        m_b0 = b0 * invA0;
        m_b1 = b1 * invA0;
        m_b2 = b2 * invA0;
        m_a1 = a1 * invA0;
        m_a2 = a2 * invA0;
    }

    T process(T input) {
        T output = m_b0 * input + m_b1 * m_x1 + m_b2 * m_x2
                                - m_a1 * m_y1 - m_a2 * m_y2;
        m_x2 = m_x1;
        m_x1 = input;
        m_y2 = m_y1;
        m_y1 = output;
        return output;
    }

    void reset() {
        m_x1 = m_x2 = m_y1 = m_y2 = T(0);
    }

private:
    T m_b0 = T(1), m_b1 = T(0), m_b2 = T(0);
    T m_a1 = T(0), m_a2 = T(0);
    T m_x1 = T(0), m_x2 = T(0);
    T m_y1 = T(0), m_y2 = T(0);
};

template<typename T, std::size_t ORDER>
class CascadedBiquad {
    static_assert(std::is_floating_point_v<T>);
    static_assert(ORDER > 0 && ORDER % 2 == 0);
public:
    void configureLowpass(T sampleRate, T cutoffHz) {
        constexpr std::size_t numSections = ORDER / 2;
        for (std::size_t i = 0; i < numSections; ++i) {
            T theta = PI<T> * (T(2) * T(i) + T(1)) / (T(2) * T(ORDER));
            T Q = T(1) / (T(2) * std::cos(theta));
            m_sections[i].configure(BiquadFilter<T>::Type::LOWPASS, sampleRate, cutoffHz, Q);
        }
    }

    void configureHighpass(T sampleRate, T cutoffHz) {
        constexpr std::size_t numSections = ORDER / 2;
        for (std::size_t i = 0; i < numSections; ++i) {
            T theta = PI<T> * (T(2) * T(i) + T(1)) / (T(2) * T(ORDER));
            T Q = T(1) / (T(2) * std::cos(theta));
            m_sections[i].configure(BiquadFilter<T>::Type::HIGHPASS, sampleRate, cutoffHz, Q);
        }
    }

    T process(T input) {
        T value = input;
        for (auto& section : m_sections) {
            value = section.process(value);
        }
        return value;
    }

    void reset() {
        for (auto& section : m_sections) {
            section.reset();
        }
    }

private:
    std::array<BiquadFilter<T>, ORDER / 2> m_sections;
};

template<typename T>
class CriticallyDampedSpring {
    static_assert(std::is_floating_point_v<T>);
public:
    CriticallyDampedSpring() = default;

    explicit CriticallyDampedSpring(T omega, T initial = T(0))
        : m_omega(omega)
        , m_pos(initial)
        , m_target(initial)
    {}

    void setOmega(T omega) { m_omega = omega; }

    void setTarget(T target) { m_target = target; }

    void snap(T value) {
        m_pos = value;
        m_vel = T(0);
        m_target = value;
    }

    T update(T dt) {
        T x  = m_pos - m_target;
        T w  = m_omega;
        T wd = w * dt;

        T expTerm = std::exp(-wd);
        T newX = (x + (m_vel + w * x) * dt) * expTerm;

        m_vel = (m_vel - w * (m_vel + w * x) * dt) * expTerm;
        m_pos = newX + m_target;

        return m_pos;
    }

    T position() const { return m_pos; }
    T velocity() const { return m_vel; }
    T target()   const { return m_target; }

    bool settled(T tolerance = T(0.001)) const {
        return std::abs(m_pos - m_target) < tolerance && std::abs(m_vel) < tolerance;
    }

private:
    T m_omega  = T(10);
    T m_pos    = T(0);
    T m_vel    = T(0);
    T m_target = T(0);
};

template<typename T>
class SecondOrderDynamics {
    static_assert(std::is_floating_point_v<T>);
public:
    SecondOrderDynamics() = default;

    SecondOrderDynamics(T frequency, T damping, T response, T initial = T(0))
        : m_xp(initial)
        , m_y(initial)
        , m_yd(T(0))
    {
        recompute(frequency, damping, response);
    }

    void recompute(T frequency, T damping, T response) {
        T w = TAU<T> * frequency;
        m_k1 = damping / (PI<T> * frequency);
        m_k2 = T(1) / (w * w);
        m_k3 = response * damping / (TAU<T> * frequency);
    }

    T update(T dt, T x) {
        T xd = (x - m_xp) / dt;
        m_xp = x;
        return updateWithVelocity(dt, x, xd);
    }

    T updateWithVelocity(T dt, T x, T xd) {
        m_xp = x;
        T k2Stable = std::max(m_k2, std::max(
            dt * dt / T(2) + dt * m_k1 / T(2),
            dt * m_k1
        ));
        m_y  = m_y + dt * m_yd;
        m_yd = m_yd + dt * (x + m_k3 * xd - m_y - m_k1 * m_yd) / k2Stable;
        return m_y;
    }

    void snap(T value) {
        m_xp = value;
        m_y  = value;
        m_yd = T(0);
    }

    T value() const { return m_y; }

private:
    T m_k1 = T(0);
    T m_k2 = T(0);
    T m_k3 = T(0);
    T m_xp = T(0);
    T m_y  = T(0);
    T m_yd = T(0);
};

template<typename T>
class MedianFilter3 {
    static_assert(std::is_floating_point_v<T>);
public:
    T process(T input) {
        m_buf[m_idx] = input;
        m_idx = (m_idx + 1) % 3;

        T a = m_buf[0], b = m_buf[1], c = m_buf[2];
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
        return b;
    }

    void reset() {
        m_buf[0] = m_buf[1] = m_buf[2] = T(0);
        m_idx = 0;
    }

private:
    T           m_buf[3] = {T(0), T(0), T(0)};
    std::size_t m_idx    = 0;
};

template<typename T>
class RateOfChange {
    static_assert(std::is_floating_point_v<T>);
public:
    T process(T input, T dt) {
        T rate = T(0);
        if (m_initialized && dt > T(0)) {
            rate = (input - m_prev) / dt;
        }
        m_prev = input;
        m_initialized = true;
        return rate;
    }

    void reset() {
        m_prev = T(0);
        m_initialized = false;
    }

private:
    T    m_prev        = T(0);
    bool m_initialized = false;
};

template<typename T>
class SlewRateLimiter {
    static_assert(std::is_floating_point_v<T>);
public:
    SlewRateLimiter() = default;

    SlewRateLimiter(T maxRiseRate, T maxFallRate)
        : m_riseRate(std::abs(maxRiseRate))
        , m_fallRate(std::abs(maxFallRate))
    {}

    T process(T input, T dt) {
        if (!m_initialized) {
            m_value = input;
            m_initialized = true;
            return m_value;
        }
        T delta  = input - m_value;
        T maxUp  =  m_riseRate * dt;
        T maxDn  = -m_fallRate * dt;
        m_value += clamp(delta, maxDn, maxUp);
        return m_value;
    }

    void reset() {
        m_value = T(0);
        m_initialized = false;
    }

    T value() const { return m_value; }

private:
    T    m_riseRate    = T(1000);
    T    m_fallRate    = T(1000);
    T    m_value       = T(0);
    bool m_initialized = false;
};

template<typename T>
class Hysteresis {
    static_assert(std::is_floating_point_v<T>);
public:
    Hysteresis(T thresholdHigh, T thresholdLow)
        : m_high(thresholdHigh)
        , m_low(thresholdLow)
    {}

    bool process(T input) {
        if (m_state) {
            if (input < m_low)  m_state = false;
        } else {
            if (input > m_high) m_state = true;
        }
        return m_state;
    }

    bool state() const { return m_state; }

    void reset() { m_state = false; }

private:
    T    m_high  = T(0);
    T    m_low   = T(0);
    bool m_state = false;
};

template<typename T>
class OnePoleAllpass {
    static_assert(std::is_floating_point_v<T>);
public:
    explicit OnePoleAllpass(T coefficient = T(0))
        : m_coeff(coefficient)
    {}

    T process(T input) {
        T output = m_coeff * (input - m_y1) + m_x1;
        m_x1 = input;
        m_y1 = output;
        return output;
    }

    void reset() {
        m_x1 = m_y1 = T(0);
    }

private:
    T m_coeff = T(0);
    T m_x1    = T(0);
    T m_y1    = T(0);
};

template<typename T>
class DCBlocker {
    static_assert(std::is_floating_point_v<T>);
public:
    explicit DCBlocker(T R = T(0.995))
        : m_R(R)
    {}

    T process(T input) {
        T output = input - m_x1 + m_R * m_y1;
        m_x1 = input;
        m_y1 = output;
        return output;
    }

    void reset() {
        m_x1 = m_y1 = T(0);
    }

private:
    T m_R  = T(0.995);
    T m_x1 = T(0);
    T m_y1 = T(0);
};

template<typename T>
struct Vec2 {
    T x = T(0);
    T y = T(0);

    constexpr Vec2() = default;
    constexpr Vec2(T _x, T _y) : x(_x), y(_y) {}

    constexpr Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(T s) const { return {x * s, y * s}; }

    constexpr T dot(const Vec2& o)  const { return x * o.x + y * o.y; }
    T          length()             const { return std::sqrt(x * x + y * y); }
    T          lengthSq()           const { return x * x + y * y; }

    Vec2 normalized() const {
        T len = length();
        return len > T(0) ? Vec2{x / len, y / len} : Vec2{};
    }

    static constexpr Vec2 lerpV(const Vec2& a, const Vec2& b, T t) {
        return {lerp(a.x, b.x, t), lerp(a.y, b.y, t)};
    }
};

template<typename T>
T degreesToRadians(T degrees) {
    return degrees * PI<T> / T(180);
}

template<typename T>
T radiansToDegrees(T radians) {
    return radians * T(180) / PI<T>;
}

template<typename T>
T wrapAngle(T radians) {
    radians = std::fmod(radians + PI<T>, TAU<T>);
    if (radians < T(0)) radians += TAU<T>;
    return radians - PI<T>;
}

template<typename T>
T pulseWidthFromAngle(T angleDeg, T minPulseUs = T(500), T maxPulseUs = T(2500),
                       T minAngleDeg = T(0), T maxAngleDeg = T(180)) {
    T t = inverseLerp(minAngleDeg, maxAngleDeg, clamp(angleDeg, minAngleDeg, maxAngleDeg));
    return lerp(minPulseUs, maxPulseUs, t);
}

template<typename T>
uint16_t pulseUsToTicks(T pulseUs, T frequency = T(50), uint16_t resolution = 4096) {
    T periodUs = T(1'000'000) / frequency;
    return static_cast<uint16_t>(clamp(pulseUs / periodUs * T(resolution), T(0), T(resolution - 1)));
}

}