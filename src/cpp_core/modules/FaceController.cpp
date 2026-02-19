#include "FaceController.h"
#include "../drivers/PCA9685.h"
#include "../utils/MathUtils.h"

using namespace face;

const std::array<ExpressionPose, static_cast<std::size_t>(Expression::COUNT)>
FaceController::s_expressionTable = {{
    {{ {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.5f, 0.5f }},

    {{ {1.0f, 0.15f, 0.0f, 0.05f}, {1.0f, 0.15f, 0.0f, 0.05f}, 0.6f, 0.6f, 0.65f, 0.65f }},

    {{ {0.55f, 0.0f, 0.0f, -0.1f}, {0.55f, 0.0f, 0.0f, -0.1f}, 0.0f, 0.0f, 0.7f, 0.7f }},

    {{ {1.0f, 0.3f, 0.0f, 0.0f}, {1.0f, 0.3f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.85f, 0.85f }},

    {{ {0.6f, 0.0f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.2f, 0.2f }},

    {{ {0.2f, 0.0f, 0.0f, -0.05f}, {0.2f, 0.0f, 0.0f, -0.05f}, 0.0f, 0.0f, 0.4f, 0.4f }},

    {{ {0.75f, 0.0f, 0.0f, 0.0f}, {0.75f, 0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.65f, 0.55f }},

    {{ {1.0f, 0.1f, 0.15f, 0.05f}, {0.85f, 0.0f, 0.15f, 0.05f}, 0.0f, 0.0f, 0.7f, 0.45f }},

    {{ {0.85f, 0.1f, 0.0f, 0.0f}, {0.85f, 0.1f, 0.0f, 0.0f}, 0.5f, 0.5f, 0.6f, 0.6f }},

    {{ {0.9f, 0.0f, -0.2f, 0.1f}, {0.9f, 0.0f, -0.2f, 0.1f}, 0.0f, 0.0f, 0.55f, 0.45f }}
}};

FaceController::FaceController(PCA9685& pwm)
    : m_pwm(pwm)
    , m_state(AnimState::IDLE)
    , m_expression(Expression::NEUTRAL)
    , m_currentPose(s_expressionTable[0])
    , m_breathIntensity(0.5f)
    , m_breathPhase(0.0f)
    , m_autoBlinkEnabled(true)
    , m_blinkTimer(0.0f)
    , m_nextBlinkTime(3.0f)
    , m_microEnabled(true)
    , m_microPhase(0.0f)
    , m_alertActive(false)
    , m_alertPulse(0.0f)
    , m_manualLidTarget(1.0f)
    , m_manualLidActive(false)
    , m_totalTime(0.0f)
    , m_rngState(0xDEADBEEF)
{
    m_leftUpperLid.snap(1.0f);
    m_leftLowerLid.snap(0.0f);
    m_rightUpperLid.snap(1.0f);
    m_rightLowerLid.snap(0.0f);
    m_gazeX.snap(0.0f);
    m_gazeY.snap(0.0f);
    m_leftCheek.snap(0.0f);
    m_rightCheek.snap(0.0f);
    m_leftBrow.snap(0.5f);
    m_rightBrow.snap(0.5f);

    m_gazeX.omega = 8.0f;
    m_gazeY.omega = 8.0f;
    m_leftCheek.omega = 6.0f;
    m_rightCheek.omega = 6.0f;
    m_leftBrow.omega = 5.0f;
    m_rightBrow.omega = 5.0f;
}

bool FaceController::init() {
    m_nextBlinkTime = nextBlinkInterval();
    composeFinalPose();
    applyToServos();
    return true;
}

void FaceController::update(double dt) {
    float fdt = static_cast<float>(dt);
    m_totalTime += fdt;

    updateAutoBlink(dt);
    updateBlinkAnimation(dt);
    updateExpressionBlend(dt);
    updateGaze(dt);
    updateSleep(dt);
    updateAlert(dt);
    updateMicroMovements(dt);
    updateBreathing(dt);

    m_leftUpperLid.update(fdt);
    m_leftLowerLid.update(fdt);
    m_rightUpperLid.update(fdt);
    m_rightLowerLid.update(fdt);
    m_gazeX.update(fdt);
    m_gazeY.update(fdt);
    m_leftCheek.update(fdt);
    m_rightCheek.update(fdt);
    m_leftBrow.update(fdt);
    m_rightBrow.update(fdt);

    composeFinalPose();
    applyToServos();
}

uint32_t FaceController::xorshift32() {
    m_rngState ^= m_rngState << 13;
    m_rngState ^= m_rngState >> 17;
    m_rngState ^= m_rngState << 5;
    return m_rngState;
}

float FaceController::randomFloat() {
    return static_cast<float>(xorshift32()) / static_cast<float>(0xFFFFFFFF);
}

float FaceController::randomRange(float lo, float hi) {
    return lo + randomFloat() * (hi - lo);
}

float FaceController::nextBlinkInterval() {
    float base = randomRange(m_blinkTimings.minInterval, m_blinkTimings.maxInterval);
    float variance = base * m_blinkTimings.variancePct;
    return base + randomRange(-variance, variance);
}

float FaceController::asymmetricBlinkCurve(float t, bool closing) const {
    t = dsp::clamp(t, 0.0f, 1.0f);

    if (closing) {
        return 1.0f - dsp::ease::inQuart(t);
    }

    float fast = dsp::ease::outCubic(t);
    float overshoot = 1.0f + 0.03f * std::sin(t * dsp::PI<float>);
    return fast * overshoot;
}

float FaceController::organicNoise(float phase) const {
    float n1 = std::sin(phase * 1.0f);
    float n2 = std::sin(phase * 2.37f) * 0.5f;
    float n3 = std::sin(phase * 5.13f) * 0.25f;
    float n4 = std::sin(phase * 11.7f) * 0.1f;
    return (n1 + n2 + n3 + n4) / 1.85f;
}

void FaceController::triggerBlink() {
    if (m_blink.active || m_sleep.sleeping) return;

    m_blink.active       = true;
    m_blink.isWink       = false;
    m_blink.doubleBlink  = false;
    m_blink.secondPhase  = false;
    m_blink.elapsed      = 0.0f;
    m_blink.startOpenness = m_leftUpperLid.current;

    float variance = 1.0f + randomRange(-m_blinkTimings.variancePct, m_blinkTimings.variancePct);
    m_blink.totalClose = m_blinkTimings.closeDuration * variance;
    m_blink.totalHold  = m_blinkTimings.holdDuration * variance;
    m_blink.totalOpen  = m_blinkTimings.openDuration * variance;

    m_blinkTimer = 0.0f;
    m_state = AnimState::BLINKING;
}

void FaceController::triggerWink(bool leftEye) {
    if (m_blink.active) return;

    m_blink.active       = true;
    m_blink.isWink       = true;
    m_blink.winkLeft     = leftEye;
    m_blink.doubleBlink  = false;
    m_blink.secondPhase  = false;
    m_blink.elapsed      = 0.0f;
    m_blink.startOpenness = leftEye ? m_leftUpperLid.current : m_rightUpperLid.current;
    m_blink.totalClose   = m_blinkTimings.closeDuration * 1.2f;
    m_blink.totalHold    = m_blinkTimings.holdDuration * 1.5f;
    m_blink.totalOpen    = m_blinkTimings.openDuration * 1.1f;

    m_state = AnimState::WINKING;
}

void FaceController::triggerDoubleBlink() {
    if (m_blink.active) return;

    m_blink.active       = true;
    m_blink.isWink       = false;
    m_blink.doubleBlink  = true;
    m_blink.secondPhase  = false;
    m_blink.elapsed      = 0.0f;
    m_blink.startOpenness = m_leftUpperLid.current;
    m_blink.totalClose   = m_blinkTimings.closeDuration * 0.85f;
    m_blink.totalHold    = m_blinkTimings.holdDuration * 0.6f;
    m_blink.totalOpen    = m_blinkTimings.openDuration * 0.7f;

    m_state = AnimState::BLINKING;
}

void FaceController::updateAutoBlink(double dt) {
    if (!m_autoBlinkEnabled || m_blink.active || m_sleep.sleeping) return;

    m_blinkTimer += static_cast<float>(dt);
    if (m_blinkTimer >= m_nextBlinkTime) {
        float roll = randomFloat();
        if (roll < 0.08f) {
            triggerDoubleBlink();
        } else {
            triggerBlink();
        }
        m_nextBlinkTime = nextBlinkInterval();
    }
}

void FaceController::updateBlinkAnimation(double dt) {
    if (!m_blink.active) return;

    m_blink.elapsed += static_cast<float>(dt);

    float closeEnd = m_blink.totalClose;
    float holdEnd  = closeEnd + m_blink.totalHold;
    float totalEnd = holdEnd + m_blink.totalOpen;

    float lid = m_blink.startOpenness;

    if (m_blink.elapsed < closeEnd) {
        float t = m_blink.elapsed / closeEnd;
        lid = m_blink.startOpenness * asymmetricBlinkCurve(t, true);
    } else if (m_blink.elapsed < holdEnd) {
        lid = 0.0f;
    } else if (m_blink.elapsed < totalEnd) {
        float t = (m_blink.elapsed - holdEnd) / m_blink.totalOpen;
        lid = m_blink.startOpenness * asymmetricBlinkCurve(t, false);
    } else {
        lid = m_blink.startOpenness;

        if (m_blink.doubleBlink && !m_blink.secondPhase) {
            m_blink.secondPhase = true;
            m_blink.elapsed = 0.0f;
            m_blink.totalClose *= 0.9f;
            m_blink.totalHold  *= 0.8f;
            m_blink.totalOpen  *= 0.95f;
            return;
        }

        m_blink.active = false;
        m_state = AnimState::IDLE;
    }

    if (m_blink.isWink) {
        if (m_blink.winkLeft) {
            m_leftUpperLid.current = lid;
        } else {
            m_rightUpperLid.current = lid;
        }
    } else {
        float asymmetry = 1.0f + organicNoise(m_totalTime * 0.7f) * 0.02f;
        m_leftUpperLid.current  = lid;
        m_rightUpperLid.current = lid * asymmetry;
    }
}

void FaceController::setExpression(Expression expr, float transitionSec) {
    if (expr == m_expression && !m_exprBlend.active) return;

    m_exprBlend.from   = m_expression;
    m_exprBlend.to     = expr;
    m_exprBlend.blend  = 0.0f;
    m_exprBlend.speed  = (transitionSec > 0.001f) ? (1.0f / transitionSec) : 100.0f;
    m_exprBlend.active = true;
    m_expression = expr;
    m_state = AnimState::EXPRESSION_TRANSITION;
}

void FaceController::updateExpressionBlend(double dt) {
    if (!m_exprBlend.active) return;

    m_exprBlend.blend += m_exprBlend.speed * static_cast<float>(dt);

    if (m_exprBlend.blend >= 1.0f) {
        m_exprBlend.blend = 1.0f;
        m_exprBlend.active = false;
        if (m_state == AnimState::EXPRESSION_TRANSITION) {
            m_state = AnimState::IDLE;
        }
    }

    float t = dsp::ease::inOutCubic(m_exprBlend.blend);
    const ExpressionPose& fromPose = getExpressionPose(m_exprBlend.from);
    const ExpressionPose& toPose   = getExpressionPose(m_exprBlend.to);
    ExpressionPose blended = blendPoses(fromPose, toPose, t);

    if (!m_blink.active && !m_manualLidActive) {
        m_leftUpperLid.target  = blended.leftEye.upperLid;
        m_leftLowerLid.target  = blended.leftEye.lowerLid;
        m_rightUpperLid.target = blended.rightEye.upperLid;
        m_rightLowerLid.target = blended.rightEye.lowerLid;
    }

    m_leftCheek.target  = blended.cheekL;
    m_rightCheek.target = blended.cheekR;
    m_leftBrow.target   = blended.browL;
    m_rightBrow.target  = blended.browR;
}

void FaceController::setEyelidOpenness(float openness, float durationSec) {
    openness = dsp::clamp(openness, 0.0f, 1.0f);
    m_manualLidTarget = openness;
    m_manualLidActive = true;

    float omega = (durationSec > 0.001f) ? (4.6f / durationSec) : 50.0f;
    m_leftUpperLid.omega  = omega;
    m_rightUpperLid.omega = omega;
    m_leftUpperLid.target  = openness;
    m_rightUpperLid.target = openness;
}

void FaceController::setGaze(float x, float y, float speedNorm) {
    m_gazeX.target = dsp::clamp(x, -1.0f, 1.0f);
    m_gazeY.target = dsp::clamp(y, -1.0f, 1.0f);

    float omega = dsp::lerp(4.0f, 20.0f, dsp::clamp(speedNorm, 0.0f, 1.0f));
    m_gazeX.omega = omega;
    m_gazeY.omega = omega;
}

void FaceController::updateGaze(double dt) {
    (void)dt;
}

void FaceController::setBreathIntensity(float intensity) {
    m_breathIntensity = dsp::clamp(intensity, 0.0f, 1.0f);
}

void FaceController::updateBreathing(double dt) {
    m_breathPhase += static_cast<float>(dt);
    if (m_breathPhase > 100000.0f) m_breathPhase -= 100000.0f;
}

void FaceController::setBrowPosition(float left, float right, float durationSec) {
    float omega = (durationSec > 0.001f) ? (4.6f / durationSec) : 50.0f;
    m_leftBrow.omega  = omega;
    m_rightBrow.omega = omega;
    m_leftBrow.target  = dsp::clamp(left, 0.0f, 1.0f);
    m_rightBrow.target = dsp::clamp(right, 0.0f, 1.0f);
}

void FaceController::enterSleepMode(float transitionSec) {
    m_sleep.sleeping = true;
    m_sleep.transitioning = true;

    float omega = (transitionSec > 0.001f) ? (4.6f / transitionSec) : 50.0f;
    m_leftUpperLid.omega  = omega * 0.3f;
    m_rightUpperLid.omega = omega * 0.3f;
    m_leftUpperLid.target  = 0.05f;
    m_rightUpperLid.target = 0.05f;
    m_leftBrow.target = 0.35f;
    m_rightBrow.target = 0.35f;

    m_autoBlinkEnabled = false;
    m_state = AnimState::SLEEPING;
}

void FaceController::exitSleepMode(float transitionSec) {
    m_sleep.sleeping = false;
    m_sleep.transitioning = true;

    float omega = (transitionSec > 0.001f) ? (4.6f / transitionSec) : 50.0f;
    m_leftUpperLid.omega  = omega;
    m_rightUpperLid.omega = omega;

    const ExpressionPose& pose = getExpressionPose(m_expression);
    m_leftUpperLid.target  = pose.leftEye.upperLid;
    m_rightUpperLid.target = pose.rightEye.upperLid;
    m_leftBrow.target = pose.browL;
    m_rightBrow.target = pose.browR;

    m_autoBlinkEnabled = true;
    m_state = AnimState::IDLE;
}

void FaceController::updateSleep(double dt) {
    if (!m_sleep.sleeping) return;

    m_sleep.breathPhase += static_cast<float>(dt) * 0.2f;
    if (m_sleep.breathPhase > dsp::TAU<float>) m_sleep.breathPhase -= dsp::TAU<float>;

    float sleepBreath = 0.05f + 0.03f * std::sin(m_sleep.breathPhase);
    m_leftUpperLid.target  = sleepBreath;
    m_rightUpperLid.target = sleepBreath + 0.005f * organicNoise(m_totalTime * 0.3f);

    m_sleep.lidDrift += static_cast<float>(dt);
    if (m_sleep.lidDrift > 8.0f + randomFloat() * 6.0f) {
        m_sleep.lidDrift = 0.0f;
        float flutter = 0.15f + randomFloat() * 0.15f;
        m_leftUpperLid.target = flutter;
        m_rightUpperLid.target = flutter * (0.9f + randomFloat() * 0.2f);
    }
}

void FaceController::enterAlertMode() {
    m_alertActive = true;
    m_alertPulse = 0.0f;
    m_state = AnimState::ALERT;

    m_leftUpperLid.omega  = 25.0f;
    m_rightUpperLid.omega = 25.0f;
    m_leftUpperLid.target  = 1.0f;
    m_rightUpperLid.target = 1.0f;
    m_leftBrow.target = 0.8f;
    m_rightBrow.target = 0.8f;
}

void FaceController::exitAlertMode() {
    m_alertActive = false;
    m_leftUpperLid.omega  = 12.0f;
    m_rightUpperLid.omega = 12.0f;

    const ExpressionPose& pose = getExpressionPose(m_expression);
    m_leftUpperLid.target  = pose.leftEye.upperLid;
    m_rightUpperLid.target = pose.rightEye.upperLid;
    m_leftBrow.target = pose.browL;
    m_rightBrow.target = pose.browR;

    m_state = AnimState::IDLE;
}

void FaceController::updateAlert(double dt) {
    if (!m_alertActive) return;

    m_alertPulse += static_cast<float>(dt) * 3.0f;
    if (m_alertPulse > dsp::TAU<float>) m_alertPulse -= dsp::TAU<float>;

    float pulse = 0.85f + 0.15f * std::sin(m_alertPulse);
    m_leftUpperLid.target  = pulse;
    m_rightUpperLid.target = pulse;
}

void FaceController::updateMicroMovements(double dt) {
    if (!m_microEnabled || m_sleep.sleeping || m_blink.active) return;

    m_microPhase += static_cast<float>(dt);
    if (m_microPhase > 100000.0f) m_microPhase -= 100000.0f;

    float gazeJitterX = organicNoise(m_microPhase * m_microConfig.gazeJitterFreqHz * dsp::TAU<float>)
                       * m_microConfig.gazeJitterAmp;
    float gazeJitterY = organicNoise(m_microPhase * m_microConfig.gazeJitterFreqHz * dsp::TAU<float> + 1.7f)
                       * m_microConfig.gazeJitterAmp;

    float gazeDriftX = std::sin(m_microPhase * m_microConfig.gazeDriftFreqHz * dsp::TAU<float>)
                      * m_microConfig.gazeDriftAmp;
    float gazeDriftY = std::sin(m_microPhase * m_microConfig.gazeDriftFreqHz * dsp::TAU<float> * 0.7f + 0.9f)
                      * m_microConfig.gazeDriftAmp;

    m_gazeX.target = dsp::clamp(m_gazeX.target + gazeJitterX + gazeDriftX, -1.0f, 1.0f);
    m_gazeY.target = dsp::clamp(m_gazeY.target + gazeJitterY + gazeDriftY, -1.0f, 1.0f);

    if (!m_blink.active && !m_manualLidActive && !m_alertActive && !m_exprBlend.active) {
        float lidMicro = organicNoise(m_microPhase * m_microConfig.lidTwitchFreqHz * dsp::TAU<float> + 3.1f)
                        * m_microConfig.lidTwitchAmp;
        m_leftUpperLid.target  += lidMicro;
        m_rightUpperLid.target += lidMicro * 0.85f;
    }

    float browMicro = organicNoise(m_microPhase * m_microConfig.browSubtleFreqHz * dsp::TAU<float> + 5.3f)
                     * m_microConfig.browSubtleAmp;
    m_leftBrow.target  += browMicro * 1.1f;
    m_rightBrow.target += browMicro * 0.9f;
}

void FaceController::composeFinalPose() {
    float breathOffset = std::sin(m_breathPhase * 0.4f * dsp::TAU<float>) * m_breathIntensity * 0.02f;

    m_currentPose.leftEye.upperLid  = dsp::clamp(m_leftUpperLid.current + breathOffset, 0.0f, 1.0f);
    m_currentPose.leftEye.lowerLid  = dsp::clamp(m_leftLowerLid.current, 0.0f, 1.0f);
    m_currentPose.rightEye.upperLid = dsp::clamp(m_rightUpperLid.current + breathOffset, 0.0f, 1.0f);
    m_currentPose.rightEye.lowerLid = dsp::clamp(m_rightLowerLid.current, 0.0f, 1.0f);

    m_currentPose.leftEye.gazeX  = dsp::clamp(m_gazeX.current, -1.0f, 1.0f);
    m_currentPose.leftEye.gazeY  = dsp::clamp(m_gazeY.current, -1.0f, 1.0f);
    m_currentPose.rightEye.gazeX = dsp::clamp(m_gazeX.current, -1.0f, 1.0f);
    m_currentPose.rightEye.gazeY = dsp::clamp(m_gazeY.current, -1.0f, 1.0f);

    m_currentPose.cheekL = dsp::clamp(m_leftCheek.current, 0.0f, 1.0f);
    m_currentPose.cheekR = dsp::clamp(m_rightCheek.current, 0.0f, 1.0f);
    m_currentPose.browL  = dsp::clamp(m_leftBrow.current, 0.0f, 1.0f);
    m_currentPose.browR  = dsp::clamp(m_rightBrow.current, 0.0f, 1.0f);
}

void FaceController::applyToServos() {
    m_pwm.setServoNormalized(m_channels.leftUpperLid,   m_currentPose.leftEye.upperLid);
    m_pwm.setServoNormalized(m_channels.leftLowerLid,   m_currentPose.leftEye.lowerLid);
    m_pwm.setServoNormalized(m_channels.rightUpperLid,  m_currentPose.rightEye.upperLid);
    m_pwm.setServoNormalized(m_channels.rightLowerLid,  m_currentPose.rightEye.lowerLid);

    float gazeXServo = m_currentPose.leftEye.gazeX * 0.5f + 0.5f;
    float gazeYServo = m_currentPose.leftEye.gazeY * 0.5f + 0.5f;
    m_pwm.setServoNormalized(m_channels.leftGazeX,  gazeXServo);
    m_pwm.setServoNormalized(m_channels.leftGazeY,  gazeYServo);
    m_pwm.setServoNormalized(m_channels.rightGazeX, gazeXServo);
    m_pwm.setServoNormalized(m_channels.rightGazeY, gazeYServo);

    m_pwm.setServoNormalized(m_channels.leftCheek,  m_currentPose.cheekL);
    m_pwm.setServoNormalized(m_channels.rightCheek, m_currentPose.cheekR);
    m_pwm.setServoNormalized(m_channels.leftBrow,   m_currentPose.browL);
    m_pwm.setServoNormalized(m_channels.rightBrow,  m_currentPose.browR);
}

const ExpressionPose& FaceController::getExpressionPose(Expression expr) const {
    auto idx = static_cast<std::size_t>(expr);
    if (idx >= s_expressionTable.size()) idx = 0;
    return s_expressionTable[idx];
}

ExpressionPose FaceController::blendPoses(const ExpressionPose& a,
                                            const ExpressionPose& b,
                                            float t) const {
    ExpressionPose result;
    result.leftEye.upperLid  = dsp::lerp(a.leftEye.upperLid,  b.leftEye.upperLid,  t);
    result.leftEye.lowerLid  = dsp::lerp(a.leftEye.lowerLid,  b.leftEye.lowerLid,  t);
    result.leftEye.gazeX     = dsp::lerp(a.leftEye.gazeX,     b.leftEye.gazeX,     t);
    result.leftEye.gazeY     = dsp::lerp(a.leftEye.gazeY,     b.leftEye.gazeY,     t);
    result.rightEye.upperLid = dsp::lerp(a.rightEye.upperLid, b.rightEye.upperLid, t);
    result.rightEye.lowerLid = dsp::lerp(a.rightEye.lowerLid, b.rightEye.lowerLid, t);
    result.rightEye.gazeX    = dsp::lerp(a.rightEye.gazeX,    b.rightEye.gazeX,    t);
    result.rightEye.gazeY    = dsp::lerp(a.rightEye.gazeY,    b.rightEye.gazeY,    t);
    result.cheekL = dsp::lerp(a.cheekL, b.cheekL, t);
    result.cheekR = dsp::lerp(a.cheekR, b.cheekR, t);
    result.browL  = dsp::lerp(a.browL,  b.browL,  t);
    result.browR  = dsp::lerp(a.browR,  b.browR,  t);
    return result;
}

void FaceController::setAutoBlinkEnabled(bool enabled) {
    m_autoBlinkEnabled = enabled;
}

void FaceController::setMicroMovementsEnabled(bool enabled) {
    m_microEnabled = enabled;
}

AnimState FaceController::currentState() const {
    return m_state;
}

Expression FaceController::currentExpression() const {
    return m_expression;
}

EyePose FaceController::leftEyePose() const {
    return m_currentPose.leftEye;
}

EyePose FaceController::rightEyePose() const {
    return m_currentPose.rightEye;
}

float FaceController::breathLevel() const {
    return m_breathIntensity;
}

void FaceController::setChannelMap(const ChannelMap& map) {
    m_channels = map;
}

void FaceController::setBlinkTimings(const BlinkTimings& timings) {
    m_blinkTimings = timings;
}

void FaceController::setMicroMovementConfig(const MicroMovementConfig& config) {
    m_microConfig = config;
}
