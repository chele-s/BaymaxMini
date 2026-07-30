#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <array>
#include <functional>

class PCA9685;

namespace face {

enum class Expression : uint8_t {
    NEUTRAL    = 0,
    HAPPY      = 1,
    SAD        = 2,
    SURPRISED  = 3,
    ANGRY      = 4,
    SLEEPY     = 5,
    CONCERNED  = 6,
    CURIOUS    = 7,
    LOVE       = 8,
    THINKING   = 9,
    COUNT      = 10
};

enum class AnimState : uint8_t {
    IDLE,
    BLINKING,
    WINKING,
    EXPRESSION_TRANSITION,
    SLEEPING,
    ALERT,
    MICRO_MOVEMENT
};

struct EyePose {
    float upperLid   = 1.0f;
    float lowerLid   = 0.0f;
    float gazeX       = 0.0f;
    float gazeY       = 0.0f;
};

struct ExpressionPose {
    EyePose leftEye;
    EyePose rightEye;
    float   cheekL     = 0.0f;
    float   cheekR     = 0.0f;
    float   browL      = 0.5f;
    float   browR      = 0.5f;
};

struct ChannelMap {
    uint8_t leftUpperLid   = 0;
    uint8_t leftLowerLid   = 1;
    uint8_t rightUpperLid  = 2;
    uint8_t rightLowerLid  = 3;
    uint8_t leftGazeX      = 4;
    uint8_t leftGazeY      = 5;
    uint8_t rightGazeX     = 6;
    uint8_t rightGazeY     = 7;
    uint8_t leftCheek      = 8;
    uint8_t rightCheek     = 9;
    uint8_t leftBrow       = 10;
    uint8_t rightBrow      = 11;
};

struct BlinkTimings {
    float closeDuration    = 0.075f;
    float holdDuration     = 0.035f;
    float openDuration     = 0.130f;
    float closeOpenRatio   = 1.73f;
    float minInterval      = 2.0f;
    float maxInterval      = 6.0f;
    float variancePct      = 0.18f;
};

struct MicroMovementConfig {
    float gazeJitterAmp       = 0.02f;
    float gazeJitterFreqHz    = 0.3f;
    float gazeDriftAmp        = 0.05f;
    float gazeDriftFreqHz     = 0.08f;
    float lidTwitchAmp        = 0.015f;
    float lidTwitchFreqHz     = 0.15f;
    float browSubtleAmp       = 0.01f;
    float browSubtleFreqHz    = 0.06f;
};

}

class FaceController {
public:
    FaceController(PCA9685& pwm);

    bool init();
    void update(double dt);

    void triggerBlink();
    void triggerWink(bool leftEye);
    void triggerDoubleBlink();

    void setExpression(face::Expression expr, float transitionSec = 0.3f);
    void setEyelidOpenness(float openness, float durationSec = 0.2f);
    void setGaze(float x, float y, float speedNorm = 1.0f);
    void setBreathIntensity(float intensity);
    void setBrowPosition(float left, float right, float durationSec = 0.3f);

    void enterSleepMode(float transitionSec = 1.5f);
    void exitSleepMode(float transitionSec = 0.8f);
    void enterAlertMode();
    void exitAlertMode();

    void setAutoBlinkEnabled(bool enabled);
    void setMicroMovementsEnabled(bool enabled);

    face::AnimState    currentState() const;
    face::Expression   currentExpression() const;
    face::EyePose      leftEyePose() const;
    face::EyePose      rightEyePose() const;
    float              breathLevel() const;

    void setChannelMap(const face::ChannelMap& map);
    void setBlinkTimings(const face::BlinkTimings& timings);
    void setMicroMovementConfig(const face::MicroMovementConfig& config);

private:
    struct SpringChannel {
        float current  = 0.0f;
        float target   = 0.0f;
        float velocity = 0.0f;
        float omega    = 12.0f;

        float update(float dt) {
            float x  = current - target;
            float wd = omega * dt;
            float expTerm = std::exp(-wd);
            float newX = (x + (velocity + omega * x) * dt) * expTerm;
            velocity = (velocity - omega * (velocity + omega * x) * dt) * expTerm;
            current = newX + target;
            return current;
        }

        void snap(float val) {
            current = val;
            target  = val;
            velocity = 0.0f;
        }

        bool settled(float tol = 0.001f) const {
            return std::abs(current - target) < tol && std::abs(velocity) < tol;
        }
    };

    struct BlinkState {
        bool   active       = false;
        bool   isWink       = false;
        bool   winkLeft     = false;
        bool   doubleBlink  = false;
        bool   secondPhase  = false;
        float  elapsed      = 0.0f;
        float  totalClose   = 0.075f;
        float  totalHold    = 0.035f;
        float  totalOpen    = 0.130f;
        float  startOpenness = 1.0f;
    };

    struct ExpressionBlend {
        face::Expression from   = face::Expression::NEUTRAL;
        face::Expression to     = face::Expression::NEUTRAL;
        float            blend  = 1.0f;
        float            speed  = 0.0f;
        bool             active = false;
    };

    struct SleepState {
        bool  sleeping       = false;
        bool  transitioning  = false;
        float breathPhase    = 0.0f;
        float lidDrift       = 0.0f;
    };

    void updateAutoBlink(double dt);
    void updateBlinkAnimation(double dt);
    void updateExpressionBlend(double dt);
    void updateGaze(double dt);
    void updateMicroMovements(double dt);
    void updateBreathing(double dt);
    void updateSleep(double dt);
    void updateAlert(double dt);
    void composeFinalPose();
    void applyToServos();

    float asymmetricBlinkCurve(float t, bool closing) const;
    float organicNoise(float phase) const;
    float nextBlinkInterval();

    const face::ExpressionPose& getExpressionPose(face::Expression expr) const;
    face::ExpressionPose blendPoses(const face::ExpressionPose& a,
                                      const face::ExpressionPose& b,
                                      float t) const;

    PCA9685&                m_pwm;
    face::ChannelMap        m_channels;
    face::BlinkTimings      m_blinkTimings;
    face::MicroMovementConfig m_microConfig;

    face::AnimState         m_state;
    face::Expression        m_expression;
    face::ExpressionPose    m_currentPose;

    SpringChannel           m_leftUpperLid;
    SpringChannel           m_leftLowerLid;
    SpringChannel           m_rightUpperLid;
    SpringChannel           m_rightLowerLid;
    SpringChannel           m_gazeX;
    SpringChannel           m_gazeY;
    SpringChannel           m_leftCheek;
    SpringChannel           m_rightCheek;
    SpringChannel           m_leftBrow;
    SpringChannel           m_rightBrow;

    BlinkState              m_blink;
    ExpressionBlend         m_exprBlend;
    SleepState              m_sleep;

    float   m_breathIntensity;
    float   m_breathPhase;

    bool    m_autoBlinkEnabled;
    float   m_blinkTimer;
    float   m_nextBlinkTime;

    bool    m_microEnabled;
    float   m_microPhase;

    bool    m_alertActive;
    float   m_alertPulse;

    float   m_manualLidTarget;
    bool    m_manualLidActive;

    float   m_totalTime;
    uint32_t m_rngState;

    uint32_t xorshift32();
    float    randomFloat();
    float    randomRange(float lo, float hi);

    static const std::array<face::ExpressionPose, static_cast<std::size_t>(face::Expression::COUNT)> s_expressionTable;
};
