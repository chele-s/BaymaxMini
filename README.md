<p align="center">
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%205-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white" alt="Platform"/>
  <img src="https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=cplusplus&logoColor=white" alt="C++17"/>
  <img src="https://img.shields.io/badge/Python-3.11+-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python"/>
  <img src="https://img.shields.io/badge/Build-CMake%203.22+-064F8C?style=for-the-badge&logo=cmake&logoColor=white" alt="CMake"/>
  <img src="https://img.shields.io/badge/IPC-ZeroMQ-DF0000?style=for-the-badge&logo=zeromq&logoColor=white" alt="ZeroMQ"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="License"/>
</p>

<h1 align="center">BaymaxMini</h1>

<p align="center">
  <strong>A State-of-the-Art Healthcare Companion Robot</strong><br>
  <em>Real-time biometric monitoring · Expressive face animation · Intelligent medical assessment</em><br>
  <em>Powered by a dual-layer C++17 / Python 3.11 architecture on Raspberry Pi 5</em>
</p>

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Layer](#hardware-layer)
- [Mathematical Foundations](#mathematical-foundations)
  - [Digital Signal Processing](#1-digital-signal-processing-dsp)
  - [Photoplethysmography Pipeline](#2-photoplethysmography-ppg-pipeline)
  - [Spring-Damper Dynamics](#3-spring-damper-dynamics)
  - [Second-Order Dynamics System](#4-second-order-dynamics-system)
  - [Easing & Interpolation Theory](#5-easing--interpolation-theory)
  - [Cubic Bézier Curve Solver](#6-cubic-bézier-curve-solver)
  - [Power Management Mathematics](#7-power-management-mathematics)
  - [SpO₂ Computation (Beer-Lambert)](#8-spo₂-computation-beer-lambert-law)
  - [INA219 Calibration Mathematics](#9-ina219-calibration-mathematics)
  - [Servo Kinematics](#10-servo-kinematics)
- [C++ Real-Time Core](#c-real-time-core-src-cpp_core)
- [Python Brain](#python-brain-src-python_brain)
- [IPC Protocol](#ipc-protocol)
- [Build & Deploy](#build--deploy)
- [Configuration](#configuration)
- [Project Structure](#project-structure)

---

## Overview

**BaymaxMini** is a healthcare companion robot inspired by the fictional nurse-bot _Baymax_ from Disney's _Big Hero 6_. It implements a **hard real-time control loop** in C++17 running at 50 Hz on bare Linux, coupled with a **high-level Python brain** that handles natural language understanding, computer vision, medical reasoning, and emotional intelligence.

The system runs on a **Raspberry Pi 5** and communicates across layers via **ZeroMQ PUB/SUB** with checksummed binary telemetry frames, ensuring < 1ms IPC latency while maintaining full decoupling between the deterministic hardware layer and the non-deterministic AI layer.

### Key Capabilities

| Domain | Capability | Implementation |
|--------|-----------|----------------|
| **Biometrics** | Heart rate, SpO₂, skin temperature | MAX30102 PPG + MLX90614 IR thermometry |
| **Proximity** | Time-of-Flight distance sensing | VL53L1X SPAD array (up to 4m) |
| **Expression** | 10 emotions, blinks, gaze, micro-movements | PCA9685 12-channel servo via spring dynamics |
| **Power** | Real-time energy monitoring & protection | INA219 current/voltage sensing with hysteresis FSM |
| **Vision** | Person detection, face analysis | YOLOv8-nano INT8 TFLite + OpenCV |
| **Audio** | Speech recognition, TTS, intent parsing | Whisper STT + edge TTS |
| **Medical** | Health assessment, medication reminders | Rule-based clinical heuristics + patient history DB |
| **Intelligence** | State machine, event bus, scheduling | Finite automaton with priority event dispatch |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Python Brain (non-RT)                     │
│  ┌─────────┐ ┌──────────┐ ┌────────┐ ┌───────────────────┐ │
│  │ Vision  │ │  Audio   │ │Medical │ │   Core Logic      │ │
│  │ YOLOv8  │ │STT / TTS │ │Assess. │ │ StateMachine      │ │
│  │ FaceAn. │ │IntentPrs │ │History │ │ EventBus/Sched.   │ │
│  └────┬────┘ └────┬─────┘ └───┬────┘ └────────┬──────────┘ │
│       └───────────┴────────────┴───────────────┘            │
│                         │ ZMQ PUB/SUB                       │
│                    tcp://127.0.0.1:5555 (CMD ↓)             │
│                    tcp://127.0.0.1:5556 (TELEM ↑)           │
├─────────────────────────────────────────────────────────────┤
│                  C++ Real-Time Core (50 Hz)                 │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                   Orchestrator                      │    │
│  │   tick() → readSensors → processCmd → logic → out   │    │
│  └──────┬──────────┬──────────┬────────────────────────┘    │
│  ┌──────┴───┐ ┌────┴─────┐ ┌─┴──────────┐                  │
│  │  Face    │ │ Vitals   │ │  Power     │   Modules         │
│  │Controller│ │ Monitor  │ │  System    │                   │
│  └──────┬───┘ └────┬─────┘ └─┬──────────┘                  │
│  ┌──────┴──────────┴─────────┴──────────────────────┐      │
│  │              I²C Bus  (400 kHz)                   │      │
│  │  PCA9685 · VL53L1X · MAX30102 · MLX90614 · INA219│      │
│  └──────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

---

## Hardware Layer

### I²C Sensor Suite

| Chip | Type | Address | Function | Key Specs |
|------|------|---------|----------|-----------|
| **PCA9685** | PWM Driver | `0x40` | 12-channel servo control | 12-bit resolution, 50 Hz PWM |
| **VL53L1X** | ToF Sensor | `0x29` | Proximity detection | 4m range, SPAD array, ±3% accuracy |
| **MAX30102** | Pulse Oximeter | `0x57` | Heart rate + SpO₂ | Dual LED (660nm red / 880nm IR), 18-bit ADC |
| **MLX90614** | IR Thermometer | `0x5A` | Contactless body temp | ±0.5°C accuracy, 0.02°C resolution |
| **INA219** | Power Monitor | `0x41` | Battery voltage/current | 12-bit ADC, ±1% current accuracy |

### Battery System

- **Chemistry**: 3S LiPo (Lithium Polymer)
- **Nominal**: 11.1V · **Full**: 12.6V · **Empty**: 9.0V
- **Capacity**: 2200 mAh
- **Protection**: Overcurrent 3A trip / 2.5A clear, servo stall 2A / 1.2A, shutdown at 8.5V

---

## Mathematical Foundations

### 1. Digital Signal Processing (DSP)

#### 1.1 Biquad Filter (Second-Order IIR)

The core DSP primitive is a **Direct Form I biquad filter**, implementing the general second-order transfer function:

$$H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}$$

The difference equation computed each sample:

$$y[n] = b_0 \cdot x[n] + b_1 \cdot x[n-1] + b_2 \cdot x[n-2] - a_1 \cdot y[n-1] - a_2 \cdot y[n-2]$$

Coefficients are derived from the analog prototype via the **bilinear transform**. For a **lowpass** Butterworth with cutoff frequency $f_c$ and sample rate $f_s$:

$$\omega_0 = 2\pi \frac{f_c}{f_s}, \quad \alpha = \frac{\sin(\omega_0)}{2Q}$$

$$b_0 = \frac{1 - \cos(\omega_0)}{2}, \quad b_1 = 1 - \cos(\omega_0), \quad b_2 = b_0$$

$$a_0 = 1 + \alpha, \quad a_1 = -2\cos(\omega_0), \quad a_2 = 1 - \alpha$$

All coefficients are normalized by $a_0$ before storage. The system supports **lowpass, highpass, bandpass, notch**, and **allpass** topologies.

#### 1.2 Cascaded Butterworth (Nth-Order)

For steeper roll-off, biquad sections are **cascaded** to form higher-order Butterworth filters. An $N$th-order filter uses $N/2$ biquad sections, each with a unique $Q$ factor derived from Butterworth pole placement:

$$\theta_k = \frac{\pi(2k + 1)}{2N}, \quad Q_k = \frac{1}{2\cos(\theta_k)}, \quad k = 0, 1, \ldots, \frac{N}{2} - 1$$

This ensures **maximally flat magnitude response** in the passband — zero ripple by construction.

#### 1.3 Exponential Moving Average (EMA)

Used throughout for smoothing sensor data with minimal latency:

$$y[n] = \alpha \cdot x[n] + (1 - \alpha) \cdot y[n-1]$$

The smoothing constant $\alpha$ can be derived from a desired cutoff frequency:

$$\alpha = \frac{\Delta t}{RC + \Delta t}, \quad \text{where } RC = \frac{1}{2\pi f_c}$$

#### 1.4 DC Blocker (First-Order Highpass)

Removes DC offset from AC-coupled signals (critical for PPG processing):

$$y[n] = x[n] - x[n-1] + R \cdot y[n-1], \quad R \approx 0.995$$

The pole at $z = R$ creates a highpass with cutoff at:

$$f_c = \frac{f_s}{2\pi} \cdot \arccos\left(\frac{2R}{1 + R^2}\right)$$

#### 1.5 Median Filter (Order 3)

Non-linear filter for impulse noise rejection. For three samples $\{a, b, c\}$:

$$y = \text{median}(a, b, c)$$

Implemented via a sorting network (3 conditional swaps) for branchless execution on ARM.

#### 1.6 Slew Rate Limiter

Constrains the rate of change to protect servo mechanisms:

$$y[n] = y[n-1] + \text{clamp}\big(x[n] - y[n-1],\ -R_{\text{fall}} \cdot \Delta t,\ R_{\text{rise}} \cdot \Delta t\big)$$

---

### 2. Photoplethysmography (PPG) Pipeline

The MAX30102 outputs raw photon count data from red (660nm) and infrared (880nm) LEDs. The full signal processing chain:

```
Raw IR/Red → Median₃ → DC Removal → Butterworth LPF (5 Hz) → Peak Detection → BPM
```

#### 2.1 Signal Quality Metric

A composite quality score determines reading validity:

$$Q = 0.6 \cdot Q_{\text{ratio}} + 0.4 \cdot Q_{\text{stability}}$$

Where:

$$Q_{\text{ratio}} = \text{clamp}\left(\frac{\text{peak-to-peak}(AC)}{DC} \cdot 50, \ 0, \ 1\right)$$

$$Q_{\text{stability}} = \text{clamp}\left(1 - \frac{\sigma}{\mu} \cdot 0.5, \ 0, \ 1\right)$$

The ratio score validates adequate perfusion (AC/DC > 0.02), while the stability score penalizes high coefficient of variation.

#### 2.2 Adaptive Peak Detection

Heart beats are detected via zero-crossing of the signal slope with a refractory period and adaptive threshold:

1. **Refractory guard**: Ignore peaks within 300ms of the last detection
2. **Slope analysis**: detect sign change $\text{slope}_{n-1} > 0 \land \text{slope}_n \leq 0$
3. **Adaptive threshold**: $\tau[n] = 0.7 \cdot \tau[n-1] + 0.3 \cdot \text{peak amplitude}$
4. **Threshold decay**: $\tau[n] \leftarrow 0.998 \cdot \tau[n]$ (adapts to signal loss)
5. **BPM computation**: $\text{BPM} = \frac{60 \cdot f_s}{\Delta_{\text{samples}}}$, smoothed with EMA ($\alpha = 0.3$)

---

### 3. Spring-Damper Dynamics

All servo channels (eyelids, gaze, brows, cheeks) are driven by **critically damped spring** dynamics, ensuring smooth, natural motion without overshoot:

$$\ddot{x} + 2\omega\dot{x} + \omega^2 x = 0$$

The exact closed-form solution for each timestep $\Delta t$:

$$e = \exp(-\omega \Delta t)$$
$$x_{\text{new}} = \big[x + (\dot{x} + \omega x)\Delta t\big] \cdot e$$
$$\dot{x}_{\text{new}} = \big[\dot{x} - \omega(\dot{x} + \omega x)\Delta t\big] \cdot e$$

This provides:
- **Zero overshoot** (critically damped: $\zeta = 1$)
- **Exponential convergence** at rate $\omega$ (default $\omega = 12$ rad/s)
- **Exact integration** — no numerical drift regardless of timestep

The `SpringAnimator` further extends this with configurable damping ratios for **underdamped** ($\zeta < 1$, bouncy), **critically damped** ($\zeta = 1$, smooth), and **overdamped** ($\zeta > 1$, sluggish) responses. The spring force is integrated via **Velocity Verlet**:

$$\vec{F} = -k(\vec{x} - \vec{x}_{\text{target}}) - c\dot{\vec{x}}$$

$$\vec{v}_{1/2} = \vec{v}_n + \frac{\vec{a}_n \Delta t}{2}, \quad \vec{x}_{n+1} = \vec{x}_n + \vec{v}_{1/2}\Delta t$$

---

### 4. Second-Order Dynamics System

A procedural animation system providing frequency-controlled, stability-guaranteed smoothing with anticipation:

$$\ddot{y} = \frac{x + k_3 \dot{x} - y - k_1 \dot{y}}{k_2}$$

Where the constants derive from desired natural frequency $f$, damping $\zeta$, and response $r$:

$$\omega = 2\pi f, \quad k_1 = \frac{\zeta}{\pi f}, \quad k_2 = \frac{1}{\omega^2}, \quad k_3 = \frac{r \cdot \zeta}{2\pi f}$$

The $k_2$ parameter is clamped for **unconditional stability**:

$$k_2^* = \max\left(k_2, \quad \frac{\Delta t^2}{2} + \frac{\Delta t \cdot k_1}{2}, \quad \Delta t \cdot k_1\right)$$

---

### 5. Easing & Interpolation Theory

#### 5.1 Hermite Spline

Cubic Hermite interpolation between two points $p_0, p_1$ with tangents $m_0, m_1$:

$$H(t) = h_{00}(t) \cdot p_0 + h_{10}(t) \cdot m_0 + h_{01}(t) \cdot p_1 + h_{11}(t) \cdot m_1$$

With basis functions:

$$h_{00} = 2t^3 - 3t^2 + 1, \quad h_{10} = t^3 - 2t^2 + t$$
$$h_{01} = -2t^3 + 3t^2, \quad h_{11} = t^3 - t^2$$

#### 5.2 Catmull-Rom Spline

A special case of Hermite interpolation with auto-computed tangents:

$$m_i = \frac{p_{i+1} - p_{i-1}}{2}$$

#### 5.3 Smoothstep & Smootherstep

Hermite-basis polynomial interpolation with clamped input:

$$\text{smoothstep}(t) = 3t^2 - 2t^3 \quad \text{(}C^1\text{ continuous)}$$

$$\text{smootherstep}(t) = 6t^5 - 15t^4 + 10t^3 \quad \text{(}C^2\text{ continuous)}$$

#### 5.4 Complete Easing Library (32 functions)

| Family | Mathematical Form |
|--------|------------------|
| **Sine** | $f(t) = 1 - \cos\left(\frac{\pi t}{2}\right)$ |
| **Quadratic** | $f(t) = t^2$ |
| **Cubic** | $f(t) = t^3$ |
| **Quartic** | $f(t) = t^4$ |
| **Quintic** | $f(t) = t^5$ |
| **Exponential** | $f(t) = 2^{10(t-1)}$ |
| **Circular** | $f(t) = 1 - \sqrt{1 - t^2}$ |
| **Back** | $f(t) = t^2[(s+1)t - s], \quad s = 1.70158$ |
| **Elastic** | $f(t) = -2^{10t-10}\sin\left(\frac{(10t - 10.75) \cdot 2\pi}{3}\right)$ |
| **Bounce** | Piecewise parabolic with $n_1 = 7.5625$ |

Each family provides **in**, **out**, and **inOut** variants (30 total), plus smoothstep and smootherstep.

---

### 6. Cubic Bézier Curve Solver

CSS-style timing functions via parametric cubic Bézier $B(t) = (X(t), Y(t))$:

$$X(t) = 3(1-3x_2+3x_1)t^3 + 3(3x_2-6x_1)t^2 + 3x_1 t$$

Given a target $x$, the parameter $t$ is found via **Newton-Raphson** iteration (8 steps) with bisection fallback (20 steps):

$$t_{n+1} = t_n - \frac{X(t_n) - x}{X'(t_n)}, \quad \text{tolerance} = 10^{-7}$$

Pre-built presets: `easeInOut`, `snappy`, `organic`, `overshoot`, `rubberBand`, `anticipate`, `heavyLanding`.

---

### 7. Power Management Mathematics

#### 7.1 Battery State-of-Charge (SoC)

Estimated via a **piecewise-linear lookup table** against cell voltage, derived from empirical Li-ion discharge curves:

| Cell V | 3.00 | 3.30 | 3.50 | 3.60 | 3.70 | 3.75 | 3.80 | 3.85 | 3.95 | 4.10 | 4.20 |
|--------|------|------|------|------|------|------|------|------|------|------|------|
| SoC %  | 0    | 5    | 10   | 20   | 30   | 40   | 50   | 60   | 70   | 90   | 100  |

Cell voltage derived from bus voltage: $V_{\text{cell}} = V_{\text{bus}} / N_{\text{cells}}$

#### 7.2 Runtime Estimation

$$t_{\text{remaining}} = \frac{C_{\text{mAh}} \cdot \text{SoC} / 100}{\bar{I}_{\text{mA}}}$$

#### 7.3 Hysteresis State Machine

All protection thresholds use **Schmitt-trigger hysteresis** to prevent chattering:

```
        ┌─────────────────┐
        │   INACTIVE       │
        │                  │ value > trip_threshold
        │    ────────────► │──────────────────────►┐
        │                  │                        │
        └─────────────────┘                        ▼
                  ▲                         ┌──────┴─────┐
                  │  value < clear_threshold│   ACTIVE    │
                  └─────────────────────────│             │
                                            └─────────────┘
```

---

### 8. SpO₂ Computation (Beer-Lambert Law)

Blood oxygen saturation is derived from the **ratio of ratios (R)** of the red and infrared PPG signals:

$$R = \frac{AC_{\text{red}} / DC_{\text{red}}}{AC_{\text{IR}} / DC_{\text{IR}}}$$

SpO₂ is then estimated via the empirically calibrated linear approximation:

$$\text{SpO}_2 = 110 - 25R$$

This derives from the Beer-Lambert law of optical absorption:

$$I = I_0 \cdot e^{-\epsilon(\lambda) \cdot c \cdot d}$$

Where $\epsilon(\lambda)$ is the molar extinction coefficient at wavelength $\lambda$, $c$ is concentration, and $d$ is optical path length. The dual-wavelength ratiometric approach cancels out path length and tissue geometry.

---

### 9. INA219 Calibration Mathematics

The INA219 current sense amplifier requires precise register calibration:

$$\text{Current\_LSB} = \frac{I_{\text{max\_expected}}}{2^{15}} \quad [\text{A/bit}]$$

$$\text{CAL} = \text{trunc}\left(\frac{0.04096}{I_{\text{LSB}} \cdot R_{\text{shunt}}}\right)$$

$$\text{Power\_LSB} = 20 \times \text{Current\_LSB} \quad [\text{W/bit}]$$

With the configured $R_{\text{shunt}} = 0.1\Omega$ and $I_{\text{max}} = 3.2$A:

$$I_{\text{LSB}} = \frac{3.2}{32768} \approx 97.66 \; \mu\text{A/bit}$$

---

### 10. Servo Kinematics

#### Pulse Width Mapping

Servo angle $\theta$ maps to pulse width via linear interpolation:

$$\text{PW}(\theta) = \text{PW}_{\min} + \frac{\theta - \theta_{\min}}{\theta_{\max} - \theta_{\min}} \cdot (\text{PW}_{\max} - \text{PW}_{\min})$$

Default range: $500\mu s$ (0°) → $2500\mu s$ (180°)

#### PCA9685 Tick Conversion

$$\text{ticks} = \text{round}\left(\frac{\text{PW}_{\mu s} \cdot f_{\text{PWM}} \cdot 4096}{10^6}\right)$$

At $f_{\text{PWM}} = 50$Hz: $\text{ticks} = \text{PW}_{\mu s} \times 0.2048$

---

## C++ Real-Time Core (`src/cpp_core`)

### Tick Architecture

The core runs a **fixed-timestep loop** at 50 Hz ($\Delta t = 20$ms) with overrun detection:

```
while (running) {
    nextTick += 20ms
    readSensors()        ─► I²C reads from all 5 chips
    processCommands()    ─► ZMQ SUB (non-blocking)
    updateLogic()        ─► FSM + spring dynamics + animation
    writeOutputs()       ─► Telemetry PUB + servo PWM
    sleep_until(nextTick)
    if (overrun > 3 ticks) { resync; overrun_count++ }
}
```

### State Machine

```
    BOOT ──(ZMQ connected)──► CONNECTED ──(brain timeout 2s)──► AUTONOMOUS
      ▲                           ▲                                  │
      │                           └──────(CMD received)──────────────┘
      │
      └──────────────────── SHUTDOWN ◄──(SIGINT/SIGTERM or critical battery)
```

### Module: FaceController

Manages **12 servo channels** mapped to facial features via spring-damped dynamics:

| Channel | Servo | Feature | Spring ω |
|---------|-------|---------|----------|
| 0-1 | Left eyelids | Upper/Lower lid | 12 rad/s |
| 2-3 | Right eyelids | Upper/Lower lid | 12 rad/s |
| 4-5 | Left gaze | Horizontal/Vertical | 12 rad/s |
| 6-7 | Right gaze | Horizontal/Vertical | 12 rad/s |
| 8-9 | Cheeks | Left/Right raise | 12 rad/s |
| 10-11 | Brows | Left/Right position | 12 rad/s |

**10 Expression Poses**: Neutral, Happy, Sad, Surprised, Angry, Sleepy, Concerned, Curious, Love, Thinking — each defined as a target pose for all 12 channels with smooth spring-interpolated transitions.

**Blink System**: Asymmetric timing modeled after human physiology — close: 75ms, hold: 35ms, open: 130ms, ratio 1.73:1 (close:open speed). Auto-blinks at 2-6s intervals with 18% variance.

**Micro-Movements**: Continuous subtle animation to avoid the uncanny valley:
- Gaze jitter: 0.02 amplitude, 0.3 Hz
- Gaze drift: 0.05 amplitude, 0.08 Hz
- Lid twitch: 0.015 amplitude, 0.15 Hz
- Brow drift: 0.01 amplitude, 0.06 Hz

### Module: VitalsMonitor

State machine for clinical-grade vital sign acquisition:

```
IDLE → DETECTING_FINGER → STABILIZING (2s) → MEASURING (5-10s) → COMPLETE
                                                    ↓
                                         ERROR_NO_FINGER / ERROR_POOR_SIGNAL
```

Health assessment heuristics:

| Condition | Threshold | Assessment |
|-----------|-----------|------------|
| Normal HR | 50-110 BPM | `NORMAL` |
| Tachycardia | > 110 BPM | `ELEVATED_HR` |
| Bradycardia | < 50 BPM | `LOW_HR` |
| Hypoxemia | SpO₂ < 95% | `LOW_SPO2` |
| Fever | > 38.0°C | `FEVER` |
| Hypothermia | < 36.0°C | `HYPOTHERMIA` |
| Critical | HR < 40 or > 180, SpO₂ < 90% | `CRITICAL` |

### Module: PowerSystem

Real-time energy management with multi-layered protection:

- **EMA-filtered** voltage, current, power readings
- **Coulomb counting** for energy accumulation: $E += P \cdot \Delta t$
- **7-state FSM**: NOMINAL → LOW_BATTERY → CRITICAL → OVERCURRENT → SERVO_STALL → OVERTEMP → SHUTDOWN
- **Callbacks** for face alert mode and emergency shutdown

---

## Python Brain (`src/python_brain`)

| Subsystem | Modules | Purpose |
|-----------|---------|---------|
| **core** | `brain.py`, `event_bus.py`, `logger.py` | Central coordinator, pub/sub events, structured logging |
| **vision** | `camera.py`, `camera_stream.py`, `detector.py`, `face_analyzer.py`, `object_detector.py`, `visual_memory.py` | Camera capture, YOLO inference, facial expression analysis, persistent memory |
| **audio** | `stt_engine.py`, `tts_engine.py`, `intent_parser.py`, `sound_fx.py`, `audio_alerts.py` | Speech-to-text, text-to-speech, NLU intent extraction, sound effects |
| **medical** | `patient_history.py`, `pharmacist.py`, `scheduler.py` | Patient records, medication database, reminder scheduling |
| **communication** | `zmq_link.py`, `telemetry.py` | ZMQ bridge to C++ core, telemetry deserialization |
| **logic** | `state_machine.py`, `scheduler.py` | Behavioral FSM, task scheduling |
| **config** | `settings.py`, `db_migrate.py` | YAML config loader, database migrations |
| **utils** | `time_utils.py` | Timezone-aware time helpers |

---

## IPC Protocol

### Telemetry Frame (C++ → Python, 50 Hz)

Binary packed struct with **djb2 checksum** verification:

| Field | Type | Offset | Description |
|-------|------|--------|-------------|
| `magic` | u32 | 0 | `0xBABE0001` |
| `version` | u32 | 4 | Protocol version |
| `timestamp_us` | u64 | 8 | Microsecond timestamp |
| `sequence` | u32 | 16 | Monotonic counter |
| `distance_mm` | f32 | 20 | VL53L1X range |
| `heart_rate_bpm` | f32 | 24 | Filtered heart rate |
| `spo2_percent` | f32 | 28 | Blood oxygen % |
| `skin_temp_c` | f32 | 32 | Body temperature |
| `bus_voltage_v` | f32 | 40 | Battery voltage |
| `battery_pct` | f32 | 52 | State of charge |
| `eyelid_openness` | f32 | 56 | Current lid state |
| `gaze_x` / `gaze_y` | f32 | 60, 64 | Eye direction |
| `state` | u8 | — | System state enum |
| `expression` | u8 | — | Current face expression |
| `alert` | u8 | — | Alert level |
| `checksum` | u32 | last | djb2 integrity check |

### Command Frame (Python → C++)

Supports: `SET_EXPRESSION`, `SET_EYELID`, `TRIGGER_BLINK`, `SET_GAZE`, `SET_BREATH`, `SPEAK`, `SET_LED_COLOR`, `PLAY_ANIMATION`, `EMERGENCY_STOP`, `SHUTDOWN`

---

## Build & Deploy

### Prerequisites

```bash
sudo apt-get install build-essential cmake libzmq3-dev nlohmann-json3-dev
pip install pyzmq pyyaml numpy opencv-python-headless tflite-runtime
```

### Build C++ Core

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Run

```bash
./build/baymax_core &
python3 src/python_brain/main.py
```

---

## Configuration

| File | Purpose |
|------|---------|
| `configs/sensors_config.yaml` | I²C addresses, sample rates, filter parameters |
| `configs/vision_params.yaml` | YOLO confidence thresholds, camera resolution |
| `configs/reminders.json` | Medication and appointment schedule |

---

## Project Structure

```
BaymaxMini/
├── CMakeLists.txt
├── README.md
├── requirements.txt
├── configs/
│   ├── sensors_config.yaml
│   ├── vision_params.yaml
│   └── reminders.json
├── models/
│   └── yolo_nano_int8.tflite
├── scripts/
├── src/
│   ├── cpp_core/
│   │   ├── main.cpp                    Orchestrator + tick loop
│   │   ├── drivers/
│   │   │   ├── I2C_Bus.{cpp,h}        Linux I²C ioctl wrapper
│   │   │   ├── PCA9685.{cpp,h}        16-ch PWM servo driver
│   │   │   ├── VL53L1X.{cpp,h}        Time-of-Flight ranging
│   │   │   ├── MAX30102.{cpp,h}       Pulse oximetry + HR
│   │   │   ├── MLX90614.{cpp,h}       IR thermometry
│   │   │   └── INA219.{cpp,h}         Power monitoring
│   │   ├── modules/
│   │   │   ├── FaceController.{cpp,h}  Expressive servo animation
│   │   │   ├── VitalsMonitor.{cpp,h}   Clinical vitals pipeline
│   │   │   └── PowerSystem.{cpp,h}     Energy management FSM
│   │   ├── interfaces/
│   │   │   └── I_Sensor.h              Abstract sensor + registry
│   │   ├── ipc/
│   │   │   └── SharedData.h            Binary IPC protocol
│   │   └── utils/
│   │       ├── MathUtils.h             DSP, filters, springs, Vec2
│   │       ├── DigitalFilter.h         PPG chain, peak detection
│   │       └── Easing.h                Animation, Bézier, timelines
│   └── python_brain/
│       ├── main.py                     Brain entry point
│       ├── core/                       Event bus, state, logging
│       ├── vision/                     Camera, YOLO, face analysis
│       ├── audio/                      STT, TTS, intent parsing
│       ├── medical/                    Patient history, meds
│       ├── communication/             ZMQ link, telemetry decode
│       ├── logic/                      Behavioral state machine
│       ├── config/                     Settings, DB migration
│       └── utils/                      Time utilities
```

---

<p align="center">
  <em>"Hello. I am Baymax, your personal healthcare companion."</em>
</p>
