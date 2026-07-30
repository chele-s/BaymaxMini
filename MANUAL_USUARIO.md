<p align="center">
  <img src="img/Baymax_reference.jpeg" alt="Baymax Reference" width="220"/>
</p>

<h1 align="center">Manual de Usuario — BaymaxMini v1.0</h1>

<p align="center">
  <strong>Robot Asistente de Salud con Inteligencia Artificial</strong><br>
  <em>Plataforma Raspberry Pi 5 · C++17 / Python 3.11 · Arquitectura de Doble Capa en Tiempo Real</em><br>
  <em>Creado por Gabriel Calderon · Solicitado por Elias Bautista</em><br>
  <a href="https://github.com/chele-s/BaymaxMini.git">https://github.com/chele-s/BaymaxMini.git</a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Plataforma-Raspberry%20Pi%205-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white" alt="Plataforma"/>
  <img src="https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=cplusplus&logoColor=white" alt="C++17"/>
  <img src="https://img.shields.io/badge/Python-3.11+-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python"/>
  <img src="https://img.shields.io/badge/Build-CMake%203.16+-064F8C?style=for-the-badge&logo=cmake&logoColor=white" alt="CMake"/>
  <img src="https://img.shields.io/badge/IPC-ZeroMQ-DF0000?style=for-the-badge&logo=zeromq&logoColor=white" alt="ZeroMQ"/>
  <img src="https://img.shields.io/badge/Licencia-MIT-green?style=for-the-badge" alt="Licencia"/>
</p>

---

## Índice General

1. [Introducción](#1-introducción)
2. [Requisitos del Sistema](#2-requisitos-del-sistema)
   - 2.1 [Hardware Requerido](#21-hardware-requerido)
   - 2.2 [Software y Dependencias](#22-software-y-dependencias)
3. [Instalación](#3-instalación)
   - 3.1 [Clonar el Repositorio](#31-clonar-el-repositorio)
   - 3.2 [Instalar Dependencias del Sistema](#32-instalar-dependencias-del-sistema)
   - 3.3 [Instalar Dependencias de Python](#33-instalar-dependencias-de-python)
   - 3.4 [Compilar el Núcleo C++](#34-compilar-el-núcleo-c)
4. [Conexión de Hardware](#4-conexión-de-hardware)
   - 4.1 [Bus I²C y Sensores](#41-bus-i²c-y-sensores)
   - 4.2 [Sistema de Batería](#42-sistema-de-batería)
   - 4.3 [Diagrama de Cableado](#43-diagrama-de-cableado)
5. [Configuración](#5-configuración)
   - 5.1 [Archivo de Configuración Principal](#51-archivo-de-configuración-principal)
   - 5.2 [Parámetros de Red ZeroMQ](#52-parámetros-de-red-zeromq)
   - 5.3 [Parámetros de Visión](#53-parámetros-de-visión)
   - 5.4 [Parámetros de Audio](#54-parámetros-de-audio)
   - 5.5 [Parámetros Médicos](#55-parámetros-médicos)
   - 5.6 [Parámetros de Energía](#56-parámetros-de-energía)
6. [Puesta en Marcha](#6-puesta-en-marcha)
   - 6.1 [Iniciar el Núcleo C++ (Tiempo Real)](#61-iniciar-el-núcleo-c-tiempo-real)
   - 6.2 [Iniciar el Cerebro Python](#62-iniciar-el-cerebro-python)
   - 6.3 [Secuencia de Arranque Completa](#63-secuencia-de-arranque-completa)
   - 6.4 [Apagado Seguro](#64-apagado-seguro)
7. [Arquitectura del Sistema](#7-arquitectura-del-sistema)
   - 7.1 [Diagrama de Arquitectura General](#71-diagrama-de-arquitectura-general)
   - 7.2 [Núcleo de Tiempo Real (C++17)](#72-núcleo-de-tiempo-real-c17)
   - 7.3 [Cerebro Inteligente (Python 3.11)](#73-cerebro-inteligente-python-311)
   - 7.4 [Protocolo de Comunicación IPC](#74-protocolo-de-comunicación-ipc)
8. [Módulos del Núcleo C++](#8-módulos-del-núcleo-c)
   - 8.1 [Orchestrator (main.cpp)](#81-orchestrator-maincpp)
   - 8.2 [Bus I²C (I2C_Bus)](#82-bus-i²c-i2c_bus)
   - 8.3 [Driver PWM — PCA9685](#83-driver-pwm--pca9685)
   - 8.4 [Sensor de Distancia — VL53L1X](#84-sensor-de-distancia--vl53l1x)
   - 8.5 [Pulsioxímetro — MAX30102](#85-pulsioxímetro--max30102)
   - 8.6 [Termómetro IR — MLX90614](#86-termómetro-ir--mlx90614)
   - 8.7 [Monitor de Energía — INA219](#87-monitor-de-energía--ina219)
   - 8.8 [Controlador Facial (FaceController)](#88-controlador-facial-facecontroller)
   - 8.9 [Monitor de Signos Vitales (VitalsMonitor)](#89-monitor-de-signos-vitales-vitalsmonitor)
   - 8.10 [Sistema de Energía (PowerSystem)](#810-sistema-de-energía-powersystem)
9. [Módulos del Cerebro Python](#9-módulos-del-cerebro-python)
   - 9.1 [Punto de Entrada (main.py)](#91-punto-de-entrada-mainpy)
   - 9.2 [Core — Event Bus y Logger](#92-core--event-bus-y-logger)
   - 9.3 [Comunicación — ZmqLink y Telemetry](#93-comunicación--zmqlink-y-telemetry)
   - 9.4 [Visión — Detección de Objetos y Rostros](#94-visión--detección-de-objetos-y-rostros)
   - 9.5 [Audio — Sistema de Voz Completo](#95-audio--sistema-de-voz-completo)
   - 9.6 [Módulo Médico — Farmacéutico y Paciente](#96-módulo-médico--farmacéutico-y-paciente)
   - 9.7 [Lógica — Máquina de Estados Finitos](#97-lógica--máquina-de-estados-finitos)
   - 9.8 [Configuración y Base de Datos](#98-configuración-y-base-de-datos)
10. [Expresiones Faciales](#10-expresiones-faciales)
11. [Sistema de Signos Vitales](#11-sistema-de-signos-vitales)
12. [Sistema de Gestión de Energía](#12-sistema-de-gestión-de-energía)
13. [Protocolo IPC Detallado](#13-protocolo-ipc-detallado)
    - 13.1 [Trama de Telemetría (C++ → Python)](#131-trama-de-telemetría-c--python)
    - 13.2 [Trama de Comandos (Python → C++)](#132-trama-de-comandos-python--c)
    - 13.3 [Tabla de Comandos Soportados](#133-tabla-de-comandos-soportados)
14. [Interfaz de Sensores](#14-interfaz-de-sensores)
15. [Fundamentos Matemáticos](#15-fundamentos-matemáticos)
    - 15.1 [Procesamiento Digital de Señales (DSP)](#151-procesamiento-digital-de-señales-dsp)
    - 15.2 [Fotopletismografía (PPG)](#152-fotopletismografía-ppg)
    - 15.3 [Dinámica Resorte-Amortiguador](#153-dinámica-resorte-amortiguador)
    - 15.4 [Sistema de Dinámicas de Segundo Orden](#154-sistema-de-dinámicas-de-segundo-orden)
    - 15.5 [Teoría de Easing e Interpolación](#155-teoría-de-easing-e-interpolación)
    - 15.6 [Cálculo de SpO₂ (Ley de Beer-Lambert)](#156-cálculo-de-spo₂-ley-de-beer-lambert)
    - 15.7 [Calibración del INA219](#157-calibración-del-ina219)
    - 15.8 [Cinemática de Servos](#158-cinemática-de-servos)
16. [Estructura del Proyecto](#16-estructura-del-proyecto)
17. [Solución de Problemas](#17-solución-de-problemas)
18. [Mantenimiento](#18-mantenimiento)
19. [Glosario](#19-glosario)
20. [Licencia y Créditos](#20-licencia-y-créditos)

---

## 1. Introducción

**BaymaxMini** es un robot compañero de salud inspirado en el personaje _Baymax_ de la película _Big Hero 6_ de Disney. Su propósito es actuar como un asistente personal de cuidado médico capaz de:

- **Monitorear signos vitales** en tiempo real (frecuencia cardíaca, SpO₂, temperatura corporal)
- **Detectar personas y rostros** mediante visión por computadora con YOLOv8
- **Comunicarse por voz** usando reconocimiento de habla (Vosk STT) y síntesis de voz (Piper TTS)
- **Gestionar medicamentos** con recordatorios programados e interacciones clínicas
- **Expresar emociones** a través de 10 expresiones faciales animadas con servomotores
- **Monitorear la energía** del sistema con protección multicapa contra sobrecorriente, sobrevoltaje y batería baja

El sistema opera sobre una **Raspberry Pi 5** utilizando una arquitectura de doble capa:

| Capa | Lenguaje | Responsabilidad | Frecuencia |
|------|----------|-----------------|------------|
| **Núcleo de Tiempo Real** | C++17 | Control de hardware, sensores, servos, DSP | 50 Hz (20 ms) |
| **Cerebro Inteligente** | Python 3.11 | IA, visión, voz, lógica médica, planificación | Asíncrono basado en eventos |

Ambas capas se comunican mediante **ZeroMQ PUB/SUB** con tramas binarias verificadas por checksum, garantizando una latencia IPC inferior a 1 ms.

---

## 2. Requisitos del Sistema

### 2.1 Hardware Requerido

| Componente | Especificación | Cantidad |
|------------|---------------|----------|
| **Raspberry Pi 5** | 4 GB / 8 GB RAM | 1 |
| **PCA9685** — Driver PWM 16 canales | Dirección I²C: `0x40` | 1 |
| **VL53L1X** — Sensor Time-of-Flight | Dirección I²C: `0x29` | 1 |
| **MAX30102** — Pulsioxímetro | Dirección I²C: `0x57` | 1 |
| **MLX90614** — Termómetro IR | Dirección I²C: `0x5A` | 1 |
| **INA219** — Monitor de potencia | Dirección I²C: `0x41` | 1 |
| **Servomotores SG90/MG90S** | Para expresiones faciales | Hasta 12 |
| **Batería LiPo 3S** | 11.1V nominal, 2200 mAh | 1 |
| **Micrófono USB** | Para reconocimiento de voz | 1 |
| **Altavoz** (3.5mm o USB) | Para síntesis de voz | 1 |
| **Cámara Pi / USB** | 640×480 mínimo, 30 FPS | 1 |

### 2.2 Software y Dependencias

| Categoría | Paquete | Versión Mínima |
|-----------|---------|----------------|
| **Sistema Operativo** | Raspberry Pi OS (64-bit) | Bookworm |
| **Compilador** | GCC / G++ | 12.0+ |
| **Build System** | CMake | 3.16+ |
| **Bibliotecas C++** | libzmq, nlohmann-json, Boost | libzmq 4.3+, Boost 1.74+ |
| **Python** | Python | 3.11+ |
| **Comunicación** | pyzmq | 25+ |
| **Configuración** | PyYAML, pydantic | — |
| **Visión** | OpenCV, pycoral, numpy | — |
| **Audio** | Vosk, Piper TTS, pyaudio, webrtcvad | — |
| **Médico** | APScheduler, sqlite3 | — |

---

## 3. Instalación

### 3.1 Clonar el Repositorio

```bash
git clone https://github.com/chele-s/BaymaxMini.git
cd BaymaxMini
```

### 3.2 Instalar Dependencias del Sistema

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
    build-essential \
    cmake \
    libzmq3-dev \
    nlohmann-json3-dev \
    libboost-all-dev \
    python3-pip \
    python3-venv \
    portaudio19-dev \
    libasound2-dev \
    i2c-tools
```

Asegúrese de habilitar el bus I²C en la Raspberry Pi:

```bash
sudo raspi-config
```

Navegue a **Interface Options → I2C → Enable**.

Verifique que los sensores están conectados correctamente:

```bash
sudo i2cdetect -y 1
```

Debería ver las direcciones `0x29`, `0x40`, `0x41`, `0x57` y `0x5A` en la tabla de resultados.

### 3.3 Instalar Dependencias de Python

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3.4 Compilar el Núcleo C++

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

El ejecutable resultante será `build/baymax_core`.

**Opciones de compilación:**

| Modo | Flags | Uso |
|------|-------|-----|
| `Release` | `-O3 -march=native -flto -ffast-math` | Producción: máxima velocidad |
| `Debug` | `-O0 -g3 -fsanitize=address,undefined` | Desarrollo: detección de errores |

---

## 4. Conexión de Hardware

### 4.1 Bus I²C y Sensores

Todos los sensores se comunican por el bus **I²C-1** de la Raspberry Pi 5 a **400 kHz**:

| Sensor | Dirección I²C | Pin SDA | Pin SCL | Función Principal |
|--------|---------------|---------|---------|-------------------|
| **PCA9685** | `0x40` | GPIO 2 | GPIO 3 | Control de 12 servomotores para expresiones faciales |
| **VL53L1X** | `0x29` | GPIO 2 | GPIO 3 | Detección de proximidad con láser (hasta 4 metros) |
| **MAX30102** | `0x57` | GPIO 2 | GPIO 3 | Medición de frecuencia cardíaca y SpO₂ |
| **MLX90614** | `0x5A` | GPIO 2 | GPIO 3 | Termometría infrarroja sin contacto |
| **INA219** | `0x41` | GPIO 2 | GPIO 3 | Monitoreo de voltaje, corriente y potencia de la batería |

### 4.2 Sistema de Batería

| Parámetro | Valor |
|-----------|-------|
| **Tipo de Celda** | LiPo (Polímero de Litio) |
| **Configuración** | 3S (3 celdas en serie) |
| **Voltaje Nominal** | 11.1 V |
| **Voltaje Completo** | 12.6 V |
| **Voltaje Vacío** | 9.0 V |
| **Capacidad** | 2200 mAh |
| **Corte de Sobrecorriente** | 3.0 A (activa) / 2.5 A (desactiva) |
| **Detección de Servo Bloqueado** | 2.0 A (activa) / 1.2 A (desactiva) |
| **Apagado de Emergencia** | < 8.5 V |

### 4.3 Diagrama de Cableado

```
┌──────────────────────────────────────────────────┐
│                 Raspberry Pi 5                    │
│                                                   │
│   GPIO 2 (SDA) ──┬──┬──┬──┬──── Sensores I²C    │
│   GPIO 3 (SCL) ──┤  │  │  │                      │
│                   │  │  │  │                      │
│     3.3V ────────┤  │  │  │                      │
│     GND ─────────┤  │  │  │                      │
│                   │  │  │  │                      │
│                   │  │  │  └── INA219 (0x41)      │
│                   │  │  └───── MLX90614 (0x5A)    │
│                   │  └──────── MAX30102 (0x57)    │
│                   └─────────── VL53L1X (0x29)     │
│                                                   │
│   GPIO 2 (SDA) ──── PCA9685 (0x40)               │
│   GPIO 3 (SCL) ──┘                               │
│                      │                            │
│                      └── CH0-CH11 → Servomotores  │
│                                                   │
│   USB ────── Micrófono                            │
│   3.5mm ──── Altavoz                              │
│   CSI/USB ── Cámara                               │
│                                                   │
│   Batería 3S LiPo ──→ INA219 (shunt 0.1Ω)       │
└──────────────────────────────────────────────────┘
```

---

## 5. Configuración

### 5.1 Archivo de Configuración Principal

Toda la configuración del cerebro Python se centraliza en `config/settings.yaml`:

```yaml
system:
  name: BaymaxMini
  version: "1.0.0"
  log_level: INFO
  log_dir: /var/log/baymax
  locale: es_MX
```

### 5.2 Parámetros de Red ZeroMQ

```yaml
zmq:
  telemetry_endpoint: "tcp://127.0.0.1:5556"
  command_endpoint: "tcp://127.0.0.1:5555"
  topic_telem: "TELEM"
  topic_cmd: "CMD"
  heartbeat_interval_s: 1.0
  brain_timeout_s: 2.0
```

| Parámetro | Descripción | Valor por Defecto |
|-----------|-------------|-------------------|
| `telemetry_endpoint` | Puerto donde el núcleo C++ publica telemetría | `tcp://127.0.0.1:5556` |
| `command_endpoint` | Puerto donde el cerebro Python envía comandos | `tcp://127.0.0.1:5555` |
| `brain_timeout_s` | Tiempo sin comandos antes de modo autónomo | 2.0 segundos |

### 5.3 Parámetros de Visión

```yaml
vision:
  camera_width: 1280
  camera_height: 720
  camera_fps: 30
  detector_confidence: 0.55
  detector_iou: 0.45
  face_min_detection_confidence: 0.6
```

| Parámetro | Descripción |
|-----------|-------------|
| `camera_width` / `camera_height` | Resolución de captura de la cámara |
| `detector_confidence` | Umbral mínimo de confianza para detecciones YOLOv8 |
| `detector_iou` | Umbral de IoU para Non-Maximum Suppression |
| `face_min_detection_confidence` | Confianza mínima para detección de rostro |

### 5.4 Parámetros de Audio

```yaml
audio:
  stt_model_path: /opt/vosk/model-es
  stt_sample_rate: 16000
  tts_model_path: /opt/piper/es_MX-claude-high.onnx
  tts_sample_rate: 22050
  sounds_volume: 0.8
```

| Parámetro | Descripción |
|-----------|-------------|
| `stt_model_path` | Ruta al modelo de reconocimiento de habla Vosk |
| `tts_model_path` | Ruta al modelo de síntesis de voz Piper |
| `stt_sample_rate` | Frecuencia de muestreo de audio de entrada (16 kHz estándar) |
| `tts_sample_rate` | Frecuencia de muestreo de síntesis (22.05 kHz) |

### 5.5 Parámetros Médicos

```yaml
medical:
  fever_threshold_c: 37.5
  hypothermia_threshold_c: 35.0
  tachycardia_bpm: 100
  bradycardia_bpm: 50
  low_spo2_pct: 94.0
  reminder_lead_time_s: 30
  missed_dose_window_s: 300
```

| Parámetro | Descripción | Valor |
|-----------|-------------|-------|
| `fever_threshold_c` | Temperatura a partir de la cual se declara fiebre | 37.5°C |
| `bradycardia_bpm` | Frecuencia cardíaca baja | < 50 BPM |
| `tachycardia_bpm` | Frecuencia cardíaca alta | > 100 BPM |
| `low_spo2_pct` | Umbral de hipoxemia | < 94% |
| `missed_dose_window_s` | Ventana de tiempo para dosis omitida | 300 s (5 min) |

### 5.6 Parámetros de Energía

```yaml
power:
  nominal_voltage_v: 11.1
  full_voltage_v: 12.6
  empty_voltage_v: 9.0
  capacity_mah: 2200.0
  cell_count: 3
  overcurrent_trip_ma: 3000.0
  overcurrent_clear_ma: 2500.0
  low_battery_pct: 15.0
  critical_battery_pct: 5.0
```

---

## 6. Puesta en Marcha

### 6.1 Iniciar el Núcleo C++ (Tiempo Real)

```bash
./build/baymax_core &
```

El núcleo ejecutará un **bucle fijo a 50 Hz** que:
1. Lee todos los sensores I²C (VL53L1X, MAX30102, MLX90614, INA219)
2. Recibe comandos del cerebro Python vía ZeroMQ
3. Actualiza la lógica interna (animaciones faciales, sistema de protección energética)
4. Publica telemetría a 50 Hz en el socket ZeroMQ

### 6.2 Iniciar el Cerebro Python

```bash
python3 src/python_brain/main.py
```

El cerebro inicializa todos los subsistemas en orden:
1. Máquina de estados finitos (FSM)
2. Planificador de tareas
3. Historial de paciente + Farmacéutico
4. Efectos de sonido + Alertas de audio
5. Motores STT y TTS
6. Enlace ZMQ con el núcleo C++
7. Cámara + Pipeline de visión (en hilo separado)

### 6.3 Secuencia de Arranque Completa

```
Terminal 1:
$ cd BaymaxMini
$ ./build/baymax_core &

Terminal 2:
$ cd BaymaxMini
$ source venv/bin/activate
$ python3 src/python_brain/main.py
```

**Flujo de estados durante el arranque:**

```
BOOT ──(ZMQ conectado)──► CONNECTED ──(timeout 2s sin CMD)──► AUTONOMOUS
  ▲                           ▲                                    │
  │                           └──────(CMD recibido)────────────────┘
  │
  └──────────────────── SHUTDOWN ◄──(SIGINT/SIGTERM o batería crítica)
```

### 6.4 Apagado Seguro

**Desde la terminal:**
```bash
kill -SIGINT $(pgrep baymax_core)
```

**Desde el cerebro Python (comando ZMQ):**
```json
{"cmd": "shutdown", "data": {}}
```

**Secuencia de apagado del núcleo C++:**
1. Los ojos se cierran suavemente (animación de 0.5 s)
2. Se detiene la medición de signos vitales
3. Se apaga el sensor ToF
4. Se apagan todos los drivers de hardware
5. Se cierra la conexión ZMQ
6. Se libera el bus I²C

---

## 7. Arquitectura del Sistema

### 7.1 Diagrama de Arquitectura General

```
┌─────────────────────────────────────────────────────────────┐
│                    Cerebro Python (no-RT)                    │
│  ┌─────────┐ ┌──────────┐ ┌────────┐ ┌───────────────────┐ │
│  │ Visión  │ │  Audio   │ │Médico  │ │   Lógica Central  │ │
│  │ YOLOv8  │ │STT / TTS │ │Farmac. │ │ StateMachine      │ │
│  │ FaceAn. │ │VoiceSys  │ │History │ │ EventBus/Sched.   │ │
│  └────┬────┘ └────┬─────┘ └───┬────┘ └────────┬──────────┘ │
│       └───────────┴────────────┴───────────────┘            │
│                         │ ZMQ PUB/SUB                       │
│                    tcp://127.0.0.1:5555 (CMD ↓)             │
│                    tcp://127.0.0.1:5556 (TELEM ↑)           │
├─────────────────────────────────────────────────────────────┤
│                Núcleo C++ Tiempo Real (50 Hz)               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                  Orchestrator                       │    │
│  │   tick() → readSensors → processCmd → logic → out   │    │
│  └──────┬──────────┬──────────┬────────────────────────┘    │
│  ┌──────┴───┐ ┌────┴─────┐ ┌─┴──────────┐                  │
│  │  Face    │ │ Vitals   │ │  Power     │   Módulos         │
│  │Controller│ │ Monitor  │ │  System    │                   │
│  └──────┬───┘ └────┬─────┘ └─┬──────────┘                  │
│  ┌──────┴──────────┴─────────┴──────────────────────┐      │
│  │              Bus I²C  (400 kHz)                   │      │
│  │  PCA9685 · VL53L1X · MAX30102 · MLX90614 · INA219│      │
│  └──────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Núcleo de Tiempo Real (C++17)

El núcleo ejecuta un **bucle de paso fijo** (fixed-timestep loop) a exactamente **50 Hz** (Δt = 20 ms). Cada iteración (_tick_) ejecuta cuatro fases en secuencia:

| Fase | Función | Operación |
|------|---------|-----------|
| **1. readSensors()** | Lectura I²C | Lee distancia, pulsioximetría, temperatura y potencia |
| **2. processCommands()** | Recepción ZMQ | Procesa comandos no-bloqueantes del cerebro Python |
| **3. updateLogic()** | Lógica interna | Actualiza animaciones faciales, FSM, protección energética |
| **4. writeOutputs()** | Escritura | Publica telemetría + actualiza servos PWM |

Si el sistema detecta un **overrun** (tick que tarda más de 3 períodos), resincroniza el reloj automáticamente.

### 7.3 Cerebro Inteligente (Python 3.11)

El cerebro opera de forma **asíncrona basada en eventos** mediante un bus de eventos (`EventBus`) con patrón publicador/suscriptor:

| Subsistema | Módulos | Función |
|------------|---------|---------|
| **Core** | `brain.py`, `event_bus.py`, `logger.py` | Coordinador central, bus de eventos pub/sub, logging estructurado |
| **Visión** | `camera.py`, `detector.py`, `tpu_pipeline.py`, `face_analyzer.py` | YOLOv8 en Edge TPU, detección de rostros, análisis geométrico |
| **Audio** | `voice_system.py`, `stt_engine.py`, `tts_engine.py`, `sound_fx.py` | VAD + Vosk STT + Piper TTS en hilos separados |
| **Médico** | `pharmacist.py`, `patient_history.py` | Base de datos SQLite, recordatorios, interacciones clínicas |
| **Comunicación** | `zmq_link.py`, `telemetry.py` | Puente ZMQ, deserialización binaria, validación Pydantic |
| **Lógica** | `state_machine.py`, `scheduler.py` | FSM conductual con 5 estados, planificador de tareas |
| **Config** | `settings.py`, `db_migrate.py` | Cargador YAML, migraciones de base de datos |

### 7.4 Protocolo de Comunicación IPC

La comunicación entre las dos capas utiliza **ZeroMQ** con patrón PUB/SUB en dos canales unidireccionales:

| Canal | Dirección | Puerto | Topic | Formato | Frecuencia |
|-------|-----------|--------|-------|---------|------------|
| **Telemetría** | C++ → Python | `5556` | `TELEM` | Struct binario empaquetado | 50 Hz |
| **Comandos** | Python → C++ | `5555` | `CMD` | JSON UTF-8 | Bajo demanda |

Cada trama incluye verificación de integridad mediante **checksum djb2** y número de secuencia monótonamente creciente.

---

## 8. Módulos del Núcleo C++

### 8.1 Orchestrator (main.cpp)

El `Orchestrator` es la clase principal que contiene y coordina todos los componentes del sistema. Su ciclo de vida es:

1. **bootstrap()**: Inicializa bus I²C, todos los sensores, módulos, batería y conexión ZMQ
2. **tick()** (repetido a 50 Hz): Ejecuta las 4 fases de lectura-proceso-lógica-escritura
3. **teardown()**: Apaga todo de forma segura al recibir SIGINT/SIGTERM

**Máquina de estados interna:**

| Estado | Descripción | Transición |
|--------|-------------|------------|
| `BOOT` | Inicio del sistema | → `CONNECTED` si ZMQ conecta |
| `CONNECTED` | Cerebro Python activo | → `AUTONOMOUS` si no hay CMD en 2s |
| `AUTONOMOUS` | Sin cerebro, auto-parpadeo y respiración | → `CONNECTED` al recibir CMD |
| `SHUTDOWN` | Apagado seguro | Terminal |

### 8.2 Bus I²C (I2C_Bus)

Wrapper seguro para operaciones I²C de Linux usando `ioctl()`:

| Operación | Método | Descripción |
|-----------|--------|-------------|
| Escritura de 1 byte | `writeByte(addr, reg, value)` | Escribe un registro de 8 bits |
| Escritura de 2 bytes | `writeWord(addr, reg, value)` | Escribe un registro de 16 bits |
| Lectura de 1 byte | `readByte(addr, reg, out)` | Lee un registro de 8 bits |
| Lectura de 2 bytes | `readWord(addr, reg, out)` | Lee un registro de 16 bits |
| Transacción combinada | `writeReadTransaction(...)` | Escritura+lectura atómica |

El bus es **thread-safe** gracias a un `std::mutex` interno y cuenta con un **contador de errores** para diagnóstico.

### 8.3 Driver PWM — PCA9685

Controla hasta **16 canales PWM** para manejo de servomotores:

| Función | Descripción |
|---------|-------------|
| `setFrequency(hz)` | Establece la frecuencia PWM (por defecto 50 Hz para servos) |
| `setServoAngle(ch, deg)` | Posiciona un servo por ángulo (0°–180°) |
| `setServoNormalized(ch, t)` | Posiciona un servo con valor normalizado [0.0 – 1.0] |
| `setServoPulseUs(ch, us)` | Control directo por ancho de pulso en microsegundos |
| `allOff()` | Apaga todos los canales simultáneamente |
| `sleep()` / `wake()` | Modo de bajo consumo del chip |

**Rango de pulso por defecto:** 500 μs (0°) a 2400 μs (180°), resolución de 12 bits (4096 ticks).

### 8.4 Sensor de Distancia — VL53L1X

Sensor Time-of-Flight basado en matriz SPAD con láser VCSEL:

| Función | Descripción |
|---------|-------------|
| `startRanging()` / `stopRanging()` | Inicia/detiene mediciones continuas |
| `readDistance(distMM)` | Lee distancia en milímetros |
| `readResult(result)` | Lee resultado completo con señal, ambiente y estado |
| `setDistanceMode(mode)` | `SHORT` (hasta 1.3m, mayor precisión) o `LONG` (hasta 4m) |
| `setTimingBudgetMs(ms)` | Tiempo de integración (20–200 ms) |
| `setROI(w, h)` | Región de interés para detección focalizada |

**Estados de rango posibles:**

| Estado | Código | Significado |
|--------|--------|-------------|
| `VALID` | 0 | Medición exitosa |
| `SIGMA_FAIL` | 1 | Ruido excesivo |
| `SIGNAL_FAIL` | 2 | Señal insuficiente |
| `OUT_OF_BOUNDS` | 4 | Fuera de rango |
| `HARDWARE_FAIL` | 5 | Error de hardware |

### 8.5 Pulsioxímetro — MAX30102

Sensor de fotopletismografía con doble LED (rojo 660nm + infrarrojo 880nm):

| Función | Descripción |
|---------|-------------|
| `readFIFO(hr, spo2)` | Lee FIFO del sensor y calcula HR y SpO₂ |
| `setLedCurrent(redMA, irMA)` | Ajusta la corriente de los LEDs |
| `setSampleRate(sps)` | Configura la frecuencia de muestreo |
| `setAdcRange(range)` | Rango del ADC (18 bits máximo) |
| `setPulseWidth(us)` | Ancho de pulso del LED |

**Pipeline de procesamiento de señal:**

```
Raw IR/Red → Mediana₃ → Remoción DC → LPF Butterworth (5 Hz) → Detección de Picos → BPM
```

### 8.6 Termómetro IR — MLX90614

Sensor de temperatura infrarroja sin contacto:

| Función | Descripción |
|---------|-------------|
| `readTemperatures(objC, ambC)` | Lee temperatura de objeto y ambiente simultáneamente |
| `readObjectTemp(tempC)` | Solo temperatura del objeto |
| `readAmbientTemp(tempC)` | Solo temperatura ambiente |
| `setEmissivity(e)` | Ajusta la emisividad del objeto (0.0 – 1.0) |

**Precisión:** ±0.5°C con resolución de 0.02°C. El sensor se comunica por protocolo SMBus.

### 8.7 Monitor de Energía — INA219

Monitor de voltaje, corriente y potencia del bus de alimentación:

| Función | Descripción |
|---------|-------------|
| `calibrate(shuntOhms, maxAmps)` | Calibra el sensor con resistencia shunt y corriente máxima |
| `readPower(busV, curMA, powMW)` | Lee voltaje, corriente y potencia simultáneamente |
| `batteryPercent(fullV, emptyV)` | Estima porcentaje de carga |
| `estimateRuntimeHours(capMAh)` | Estima tiempo de ejecución restante |
| `configure(vRange, gain, busRes, shuntRes)` | Configuración avanzada del ADC |

**Configuración por defecto:**

| Parámetro | Valor |
|-----------|-------|
| Resistencia shunt | 0.1 Ω |
| Corriente máxima esperada | 3.2 A |
| Resolución del ADC | 12 bits |
| LSB de corriente | ≈ 97.66 μA/bit |

### 8.8 Controlador Facial (FaceController)

Gestiona **12 canales de servo** mapeados a características faciales con dinámica de resorte amortiguado:

**Mapa de canales:**

| Canal | Servo | Característica | Frecuencia Angular ω |
|-------|-------|----------------|---------------------|
| 0–1 | Párpados izquierdos | Superior / Inferior | 12 rad/s |
| 2–3 | Párpados derechos | Superior / Inferior | 12 rad/s |
| 4–5 | Mirada izquierda | Horizontal / Vertical | 12 rad/s |
| 6–7 | Mirada derecha | Horizontal / Vertical | 12 rad/s |
| 8–9 | Mejillas | Izquierda / Derecha | 12 rad/s |
| 10–11 | Cejas | Izquierda / Derecha | 12 rad/s |

**Funciones principales:**

| Función | Descripción |
|---------|-------------|
| `setExpression(expr, transitionSec)` | Cambia a una de las 10 expresiones con transición suave |
| `triggerBlink()` | Dispara un parpadeo asimétrico natural |
| `triggerWink(leftEye)` | Guiño con un solo ojo |
| `triggerDoubleBlink()` | Parpadeo doble |
| `setGaze(x, y, speed)` | Dirige la mirada a una posición normalizada |
| `setEyelidOpenness(openness, duration)` | Abre/cierra párpados manualmente |
| `setBrowPosition(left, right, duration)` | Posiciona las cejas |
| `setBreathIntensity(intensity)` | Intensidad del efecto de respiración en ojos |
| `enterSleepMode(transition)` | Modo dormido (cierra ojos gradualmente) |
| `exitSleepMode(transition)` | Despertar |
| `enterAlertMode()` | Modo alerta (pulso visual para alertas de energía) |

**Sistema de Parpadeo:**

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| Duración de cierre | 75 ms | Velocidad de cierre del párpado |
| Duración de sostén | 35 ms | Tiempo con ojo cerrado |
| Duración de apertura | 130 ms | Velocidad de apertura del párpado |
| Razón cierre/apertura | 1.73:1 | Simula fisiología humana real |
| Intervalo entre parpadeos | 2–6 s | Con 18% de varianza aleatoria |

**Micro-Movimientos** (evitan el efecto "valle inquietante"):

| Movimiento | Amplitud | Frecuencia |
|------------|----------|------------|
| Jitter de mirada | 0.02 | 0.3 Hz |
| Deriva de mirada | 0.05 | 0.08 Hz |
| Tic de párpado | 0.015 | 0.15 Hz |
| Deriva de cejas | 0.01 | 0.06 Hz |

### 8.9 Monitor de Signos Vitales (VitalsMonitor)

Máquina de estados para adquisición de signos vitales con calidad clínica:

**Flujo de medición:**

```
IDLE → DETECTING_FINGER → STABILIZING (2s) → MEASURING (5–10s) → COMPLETE
                                                     ↓
                                          ERROR_NO_FINGER / ERROR_POOR_SIGNAL
```

| Estado | Duración | Acción |
|--------|----------|--------|
| `IDLE` | — | Esperando instrucción de medición |
| `DETECTING_FINGER` | Variable | Detecta dedo sobre el sensor (IR > 50,000) |
| `STABILIZING` | 2 s | Espera que la señal se estabilice |
| `MEASURING` | 5–10 s | Acumulando lecturas válidas con filtro EMA |
| `COMPLETE` | — | Lectura finalizada con evaluación de salud |

**Evaluación de salud automática:**

| Condición | Criterio | Evaluación |
|-----------|----------|------------|
| Normal | HR 50–110, SpO₂ ≥ 95%, Temp 36–37.5°C | `NORMAL` |
| Taquicardia | HR > 110 BPM | `ELEVATED_HR` |
| Bradicardia | HR < 50 BPM | `LOW_HR` |
| Hipoxemia | SpO₂ < 95% | `LOW_SPO2` |
| Fiebre | Temp > 38.0°C | `FEVER` |
| Hipotermia | Temp < 36.0°C | `HYPOTHERMIA` |
| Crítico | HR < 40 o > 180, SpO₂ < 90% | `CRITICAL` |

**Seguimiento de calidad de señal** usando `boost::accumulators` con ventana rodante de 8 muestras para cálculo de media y varianza en O(1).

### 8.10 Sistema de Energía (PowerSystem)

Gestión de energía en tiempo real con protección multicapa:

**Máquina de estados de potencia:**

```
NOMINAL → LOW_BATTERY → CRITICAL_BATTERY → SHUTDOWN_IMMINENT
    ↕          ↕              ↕
OVERCURRENT  SERVO_STALL  OVERTEMPERATURE
```

| Estado | Condición de Entrada | Acción |
|--------|---------------------|--------|
| `NOMINAL` | Operación normal | Monitoreo continuo |
| `LOW_BATTERY` | Voltaje < 10.2V | Alerta visual y auditiva |
| `CRITICAL_BATTERY` | Voltaje < 9.6V | Restricción de operaciones |
| `OVERCURRENT` | Corriente > 3.0A | Alerta facial + restricción |
| `SERVO_STALL` | Corriente > 2.0A sostenida | Alerta facial |
| `OVERTEMPERATURE` | Temperatura > 60°C | Reducción de potencia |
| `SHUTDOWN_IMMINENT` | Voltaje < 8.5V | Apagado de emergencia |

**Características avanzadas:**
- Filtrado EMA de voltaje, corriente y potencia para eliminar ruido
- Histéresis tipo Schmitt-trigger en todos los umbrales de protección
- Contador Coulomb para acumulación de energía: $E += P \cdot \Delta t$
- Tabla de búsqueda (LUT) de 11 puntos para estimación de SoC por voltaje de celda
- Estimación de tiempo de ejecución restante

---

## 9. Módulos del Cerebro Python

### 9.1 Punto de Entrada (main.py)

La clase `BaymaxBrain` es el orquestador del cerebro Python. Administra el ciclo de vida de todos los subsistemas:

**Secuencia de inicio (`start()`):**

| Orden | Subsistema | Descripción |
|-------|------------|-------------|
| 1 | `StateMachine` | Máquina de estados finitos conductual |
| 2 | `GlobalScheduler` | Planificador de tareas periódicas |
| 3 | `PatientHistory` | Historial clínico del paciente |
| 4 | `Pharmacist` | Sistema de medicamentos y recordatorios |
| 5 | `SoundFX` + `AudioAlerts` | Precarga de efectos de sonido y alertas |
| 6 | `STTEngine` + `TTSEngine` | Motores de voz |
| 7 | `ZmqLink` | Enlace de comunicación con el núcleo C++ |
| 8 | `CameraController` | Inicio de la captura de video |
| 9 | `VisionModule` (hilo separado) | Pipeline de detección en hilo daemon |

**Secuencia de apagado (`stop()`):** Orden inverso para garantizar liberación segura de recursos.

### 9.2 Core — Event Bus y Logger

**Event Bus** (`event_bus.py`): Sistema de comunicación interna con patrón publicador/suscriptor.

| Método | Descripción |
|--------|-------------|
| `subscribe(topic, callback)` | Registra un callback para un tipo de evento |
| `unsubscribe(topic, callback)` | Elimina la suscripción |
| `publish(topic, data)` | Publica un evento a todos los suscriptores |

**Eventos principales del sistema:**

| Evento | Origen | Datos | Descripción |
|--------|--------|-------|-------------|
| `TELEMETRY_UPDATE` | ZmqLink | Dict de telemetría | Actualización de sensores from C++ |
| `FACE_DETECTED` | VisionModule | Detecciones | Persona/cara detectada por visión |
| `WAKEWORD_DETECTED` | VoiceSystem | Texto reconocido | Palabra de activación detectada |
| `COMMAND_CPP_CMD` | Cualquier módulo | JSON de comando | Comando para enviar al núcleo C++ |
| `COMMAND_SPEAK` | Cualquier módulo | `{"text": "..."}` | Solicitud de síntesis de voz |
| `HIGH_TEMPERATURE` | ZmqLink | Temperatura | Alerta de temperatura alta |
| `BATTERY_CRITICAL` | ZmqLink | Nivel de batería | Batería por debajo del 15% |
| `MEDICATION_TIME` | Pharmacist | Medicamento | Hora de tomar medicamento |
| `STATE_CHANGED` | StateMachine | Nuevo estado | Transición de estado FSM |
| `TICK` | ZmqLink | Timestamp | Tick de 50 Hz para módulos que requieren temporización |

### 9.3 Comunicación — ZmqLink y Telemetry

**ZmqLink** (`zmq_link.py`): Puente bidireccional entre Python y C++ mediante ZeroMQ.

| Dirección | Socket | Función |
|-----------|--------|---------|
| C++ → Python | SUB con `CONFLATE=1` | Recibe solo la telemetría más reciente |
| Python → C++ | PUB con `HWM=1` | Envía comandos JSON sin bloquear |

El hilo receptor (`_rx_loop`) opera un poller que:
1. Espera telemetría del núcleo C++ con timeout calculado
2. Deserializa la trama binaria usando `struct.unpack()`
3. Publica `TELEMETRY_UPDATE` en el EventBus
4. Genera `TICK` a 50 Hz para sincronización de módulos
5. Detecta automáticamente alertas de temperatura y batería

**Telemetry** (`telemetry.py`): Modelo de validación Pydantic para garantizar integridad de datos con restricciones en rangos físicos.

### 9.4 Visión — Detección de Objetos y Rostros

**Pipeline de Visión:**

```
Cámara → SharedMemory (zero-copy) → Resize → Quantize → Edge TPU → NMS → EventBus
```

| Componente | Archivo | Función |
|------------|---------|---------|
| `CameraStream` | `camera_stream.py` | Captura de frames con memoria compartida DMA |
| `CameraController` | `camera.py` | Control de la cámara como proceso separado |
| `EdgeTPUPipeline` | `tpu_pipeline.py` | Inferencia YOLOv8n en Coral TPU |
| `VisionModule` | `tpu_pipeline.py` | Orquestador que formatea y publica detecciones |
| `FaceAnalyzer` | `face_analyzer.py` | Análisis geométrico ligero de rostros |

**Non-Maximum Suppression (NMS)** implementado con algoritmo de IoU para eliminar detecciones duplicadas.

### 9.5 Audio — Sistema de Voz Completo

**VoiceSystem** (`voice_system.py`): Sistema unificado de reconocimiento y síntesis de voz con 3 hilos independientes:

| Hilo | Función | Tecnología |
|------|---------|------------|
| `ListenThread` | Captura PCM + VAD (Voice Activity Detection) | WebRTC VAD |
| `STTThread` | Reconocimiento de habla en tiempo real | Vosk (offline) |
| `TTSThread` | Síntesis de voz bajo demanda | Piper TTS (offline) |

**Palabras de activación (wake words):** `baymax`, `help`, `pain`, `doctor`

**Flujo de reconocimiento:**
1. El micrófono captura audio PCM a 16 kHz
2. WebRTC VAD detecta actividad de voz (agresividad nivel 3)
3. Las tramas de audio se envían al motor Vosk
4. Al detectar silencio de 1 segundo, se procesa el resultado final
5. Si contiene una wake word, publica `WAKEWORD_DETECTED`
6. En caso contrario, publica `VOICE_HEARD`

**Flujo de síntesis:**
1. Un módulo publica `COMMAND_SPEAK` con el texto
2. Piper TTS genera audio en archivo temporal `.raw`
3. El audio se reproduce por el stream de salida PyAudio
4. Se publican `SPEECH_START` y `SPEECH_END` para sincronización

### 9.6 Módulo Médico — Farmacéutico y Paciente

**Pharmacist** (`pharmacist.py`): Sistema de gestión de medicamentos con base de datos SQLite.

**Base de datos médica (4 tablas):**

| Tabla | Campos Clave | Función |
|-------|-------------|---------|
| `medications` | nombre, dosificación, requiere_comida, requiere_agua | Catálogo de medicamentos |
| `schedules` | medication_id, hora | Horarios programados |
| `interactions` | medicamento, tipo, mensaje_advertencia | Interacciones clínicas |
| `logs` | timestamp, tipo_evento, detalles | Registro de actividad médica |

**Funcionalidades:**
- **Recordatorios automáticos:** Basados en horarios (APScheduler con cron). Cuando llega la hora, Baymax despierta, cambia a expresión "curious" y anuncia el medicamento por voz.
- **Interacciones clínicas:** Si la cámara detecta un frasco de ibuprofeno y el paciente no ha comido en > 4 horas, Baymax advierte sobre el riesgo gastrointestinal con expresión "concerned".
- **Registro:** Toda actividad médica (comidas, agua, advertencias, recordatorios) queda registrada en la tabla `logs`.

**PatientHistory** (`patient_history.py`): Historial clínico persistente del paciente.

### 9.7 Lógica — Máquina de Estados Finitos

**StateMachine** (`state_machine.py`): FSM conductual con 5 estados:

```
                         ┌──────────────────┐
                         │    BOOTING       │
                         │ (wake + init)    │
                         └───────┬──────────┘
                                 │ SYSTEM_READY
                                 ▼
┌──────────────┐         ┌──────────────────┐         ┌──────────────────┐
│ MEDICAL_SCAN │◄────────│      IDLE        │────────►│  INTERACTING     │
│ (concerned)  │  HIGH_  │   (neutral)      │ FACE_   │  (curious)       │
│ "Scan..."    │  TEMP   │                  │ DETECTED│  "Hello, I am    │
└──────┬───────┘         └────────┬─────────┘         │   Baymax..."     │
       │ SCAN_                    │                    └────────┬─────────┘
       │ COMPLETE                 │ BATTERY_                   │ 30s timeout
       └──────────┐               │ CRITICAL                   │
                  ▼               ▼                            ▼
                  IDLE    ┌──────────────────┐           IDLE
                          │ EMERGENCY_       │
                          │ SHUTDOWN         │
                          │ "Battery         │
                          │  critical..."    │
                          └──────────────────┘
```

| Estado | Expresión Facial | Comportamiento |
|--------|-----------------|----------------|
| `BOOTING` | Ojos abriendo | Envía comando `wake`, espera `SYSTEM_READY` |
| `IDLE` | Neutral | Espera rostro, wake word o alerta de temperatura |
| `INTERACTING` | Curious | Saluda por voz, escucha comandos, timeout de 30s |
| `MEDICAL_SCAN` | Concerned | Anuncia anomalía, inicia medición de signos vitales |
| `EMERGENCY_SHUTDOWN` | Sleep | Anuncia batería crítica, cierra ojos en 3s |

### 9.8 Configuración y Base de Datos

**Settings** (`settings.py`): Cargador de configuración YAML con acceso por notación punto (e.g., `CONFIG.get('network.host')`).

**DB Migrate** (`db_migrate.py`): Módulo de migraciones de base de datos para mantener esquemas actualizados.

---

## 10. Expresiones Faciales

BaymaxMini soporta **10 expresiones faciales** preconfiguradas, cada una definida como un conjunto de posiciones objetivo para los 12 canales de servo:

| # | Expresión | Descripción Visual | Uso Típico |
|---|-----------|-------------------|------------|
| 0 | **Neutral** | Ojos abiertos, cejas centradas | Estado por defecto |
| 1 | **Happy** (Feliz) | Ojos entrecerrados, mejillas arriba | Saludo, buenas noticias |
| 2 | **Sad** (Triste) | Párpados bajos, cejas caídas | Malas noticias, empatía |
| 3 | **Surprised** (Sorprendido) | Ojos muy abiertos, cejas altas | Detección inesperada |
| 4 | **Angry** (Enojado) | Cejas fruncidas, ojos entrecerrados firmes | Advertencia seria |
| 5 | **Sleepy** (Somnoliento) | Párpados casi cerrados | Modo dormido / ahorro |
| 6 | **Concerned** (Preocupado) | Cejas asimétricas, mirada enfocada | Lectura médica anormal |
| 7 | **Curious** (Curioso) | Ojos algo abiertos, cejas ligeramente elevadas | Escuchando al paciente |
| 8 | **Love** (Amor) | Ojos semicerrados, mejillas arriba | Expresión de cariño |
| 9 | **Thinking** (Pensando) | Mirada lateral, ceja elevada | Procesando información |

Todas las transiciones entre expresiones se realizan con **interpolación por resorte críticamente amortiguado** con velocidad configurable (por defecto 0.3 segundos).

---

## 11. Sistema de Signos Vitales

**Procedimiento para tomar signos vitales:**

1. Enviar comando `start_vitals` desde el cerebro Python o esperar activación automática
2. Colocar el dedo índice sobre el sensor MAX30102
3. Esperar a que el sistema detecte el dedo (umbral IR > 50,000)
4. Mantener el dedo inmóvil durante **2 segundos** de estabilización
5. Continuar inmóvil durante **5–10 segundos** de medición
6. El sistema reporta automáticamente:
   - **Frecuencia cardíaca** (BPM) con filtrado EMA (α = 0.15)
   - **Saturación de oxígeno** (SpO₂ %) con filtrado EMA (α = 0.10)
   - **Temperatura corporal** (°C) del sensor MLX90614 con filtrado EMA (α = 0.08)
7. Se genera una evaluación de salud automática

**Indicadores de calidad de señal:**

| Indicador | Fórmula | Peso |
|-----------|---------|------|
| Calidad de ratio | `clamp(peak-to-peak(AC)/DC × 50, 0, 1)` | 60% |
| Calidad de estabilidad | `clamp(1 - σ/μ × 0.5, 0, 1)` | 40% |

La lectura se considera válida cuando la calidad general supera el umbral de **0.5** (50%).

---

## 12. Sistema de Gestión de Energía

**Tabla de Estado de Carga (SoC) por voltaje de celda:**

| Voltaje de Celda (V) | 3.00 | 3.30 | 3.50 | 3.60 | 3.70 | 3.75 | 3.80 | 3.85 | 3.95 | 4.10 | 4.20 |
|-----------------------|------|------|------|------|------|------|------|------|------|------|------|
| Estado de Carga (%) | 0 | 5 | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 90 | 100 |

El voltaje de celda se calcula como: $V_{celda} = V_{bus} / N_{celdas}$

**Estimación de tiempo restante:**

$$t_{restante} = \frac{C_{mAh} \times SoC / 100}{\bar{I}_{mA}}$$

**Protecciones del sistema (todas con histéresis Schmitt-trigger):**

| Protección | Activación | Desactivación | Acción |
|------------|------------|---------------|--------|
| Sobrecorriente | > 3000 mA | < 2500 mA | Alerta facial + notificación |
| Servo bloqueado | > 2000 mA | < 1200 mA | Alerta facial |
| Batería baja | < 10.2V | > 10.8V | Alerta visual/auditiva |
| Batería crítica | < 9.6V | > 10.2V | Restricción de operaciones |
| Apagado de emergencia | < 8.5V | — | Apagado inmediato del sistema |
| Sobretemperatura | > 60°C | < 50°C | Reducción de servicios |

---

## 13. Protocolo IPC Detallado

### 13.1 Trama de Telemetría (C++ → Python)

Struct binario empaquetado (`#pragma pack(push, 1)`) verificado con checksum djb2:

| Campo | Tipo | Desplazamiento | Descripción |
|-------|------|----------------|-------------|
| `magic` | u32 | 0 | `0xBABE0001` — Identificador de trama |
| `version` | u32 | 4 | Versión del protocolo (1) |
| `timestamp_us` | u64 | 8 | Timestamp en microsegundos |
| `sequence` | u32 | 16 | Contador de secuencia monótono |
| `distance_mm` | f32 | 20 | Distancia del sensor VL53L1X |
| `heart_rate_bpm` | f32 | 24 | Frecuencia cardíaca filtrada |
| `spo2_percent` | f32 | 28 | Saturación de oxígeno |
| `skin_temp_c` | f32 | 32 | Temperatura corporal |
| `ambient_temp_c` | f32 | 36 | Temperatura ambiente |
| `bus_voltage_v` | f32 | 40 | Voltaje del bus de batería |
| `current_ma` | f32 | 44 | Corriente en miliamperios |
| `power_mw` | f32 | 48 | Potencia en miliwatts |
| `battery_pct` | f32 | 52 | Porcentaje de carga |
| `eyelid_openness` | f32 | 56 | Estado actual del párpado |
| `gaze_x` | f32 | 60 | Dirección horizontal de mirada |
| `gaze_y` | f32 | 64 | Dirección vertical de mirada |
| `breath_level` | f32 | 68 | Nivel de respiración |
| Flags de validez | u8 | 72 | Bits: proximity, vitals, power, face |
| `state` | u8 | — | Estado del sistema (enum) |
| `expression` | u8 | — | Expresión facial actual (enum) |
| `alert` | u8 | — | Nivel de alerta (`NONE/INFO/WARNING/CRITICAL`) |
| `checksum` | u32 | último | Verificación de integridad djb2 |

### 13.2 Trama de Comandos (Python → C++)

Los comandos se envían como mensajes JSON codificados en UTF-8 por el socket PUB con topic `CMD`:

```json
{
    "cmd": "nombre_del_comando",
    "data": {
        "param1": valor1,
        "param2": valor2
    }
}
```

### 13.3 Tabla de Comandos Soportados

| Comando | Parámetros | Descripción |
|---------|-----------|-------------|
| `set_expression` | `type` (string), `transition` (float, seg) | Cambia la expresión facial |
| `set_eyelid` | `openness` (0.0–1.0), `duration` (seg) | Control directo de párpados |
| `trigger_blink` | — | Parpadeo natural |
| `set_gaze` | `x` (-1 a 1), `y` (-1 a 1), `speed` (multiplicador) | Dirige la mirada |
| `set_brow` | `left` (0–1), `right` (0–1), `duration` (seg) | Posición de cejas |
| `sleep` | `transition` (seg) | Modo dormido |
| `wake` | `transition` (seg) | Despertar |
| `start_vitals` | — | Inicia medición de signos vitales |
| `stop_vitals` | — | Detiene medición |
| `shutdown` | — | Apagado completo del núcleo C++ |

**Ejemplo — Cambiar a expresión feliz con transición de 0.5 segundos:**
```json
{"cmd": "set_expression", "data": {"type": "happy", "transition": 0.5}}
```

**Ejemplo — Mirar hacia la derecha:**
```json
{"cmd": "set_gaze", "data": {"x": 0.8, "y": 0.0, "speed": 1.0}}
```

**Ejemplo — Iniciar modo dormido en 2 segundos:**
```json
{"cmd": "sleep", "data": {"transition": 2.0}}
```

---

## 14. Interfaz de Sensores

El sistema define una **interfaz abstracta** (`I_Sensor`) y un **registro de sensores** (`SensorRegistry`) para gestión centralizada:

**Interfaz I_Sensor — Métodos base:**

| Método | Descripción |
|--------|-------------|
| `init()` | Inicializa el sensor y configura registros |
| `shutdown()` | Apaga el sensor de forma segura |
| `read()` | Lee datos del sensor |
| `reset()` | Reinicia el sensor |
| `selfTest()` | Ejecuta autodiagnóstico |
| `isHealthy()` | Verifica si el sensor está funcionando correctamente |
| `isPresent()` | Verifica si el sensor está conectado |

**Diagnósticos por sensor:**

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `status` | Enum | `UNINITIALIZED`, `READY`, `BUSY`, `DEGRADED`, `ERROR`, `OFFLINE` |
| `lastError` | Enum | Último código de error (0–255) |
| `totalReads` | u32 | Total de lecturas realizadas |
| `failedReads` | u32 | Lecturas fallidas |
| `successRate` | float | Tasa de éxito (0.0–1.0) |
| `avgReadMs` | float | Tiempo promedio de lectura |
| `peakReadMs` | float | Tiempo pico de lectura |

**SensorRegistry — Gestión centralizada:**

| Función | Descripción |
|---------|-------------|
| `registerSensor(s)` | Registra un sensor (máximo 16) |
| `initAll()` | Inicializa todos los sensores registrados |
| `readAll()` | Lee todos los sensores sanos |
| `resetFailed()` | Reintentar sensores fallidos |
| `selfTestAll()` | Autodiagnóstico de todos los sensores |
| `findByType(type)` | Buscar sensor por tipo |
| `findByAddress(addr)` | Buscar sensor por dirección I²C |
| `allHealthy()` | Verificar si todos están operativos |

---

## 15. Fundamentos Matemáticos

### 15.1 Procesamiento Digital de Señales (DSP)

**Filtro Biquad (IIR de Segundo Orden):**

Función de transferencia:

$$H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}$$

Ecuación en diferencias:

$$y[n] = b_0 \cdot x[n] + b_1 \cdot x[n-1] + b_2 \cdot x[n-2] - a_1 \cdot y[n-1] - a_2 \cdot y[n-2]$$

**Filtro Butterworth Cascado (Orden N):** Cada sección biquad usa un factor Q derivado de la colocación de polos Butterworth:

$$Q_k = \frac{1}{2\cos(\theta_k)}, \quad \theta_k = \frac{\pi(2k + 1)}{2N}$$

**Media Móvil Exponencial (EMA):**

$$y[n] = \alpha \cdot x[n] + (1 - \alpha) \cdot y[n-1]$$

**Bloqueador DC:**

$$y[n] = x[n] - x[n-1] + R \cdot y[n-1], \quad R \approx 0.995$$

**Limitador de Tasa de Cambio (Slew Rate):**

$$y[n] = y[n-1] + \text{clamp}\big(x[n] - y[n-1],\ -R_{fall} \cdot \Delta t,\ R_{rise} \cdot \Delta t\big)$$

### 15.2 Fotopletismografía (PPG)

Cadena de procesamiento para los LEDs rojo (660nm) e infrarrojo (880nm) del MAX30102:

```
Raw IR/Red → Mediana₃ → Remoción DC → LPF Butterworth (5 Hz) → Detección de Picos → BPM
```

**Detección adaptativa de picos:**
1. Guardia refractaria: ignorar picos dentro de 300ms del último
2. Análisis de pendiente: detectar cambio de signo
3. Umbral adaptativo: $\tau[n] = 0.7 \cdot \tau[n-1] + 0.3 \cdot \text{amplitud del pico}$
4. Decaimiento del umbral: $\tau[n] \leftarrow 0.998 \cdot \tau[n]$
5. Cálculo de BPM: $BPM = \frac{60 \cdot f_s}{\Delta_{muestras}}$, suavizado con EMA (α = 0.3)

### 15.3 Dinámica Resorte-Amortiguador

Todos los servos se mueven con **dinámica de resorte críticamente amortiguado** (ζ = 1):

$$\ddot{x} + 2\omega\dot{x} + \omega^2 x = 0$$

Solución exacta para cada paso de tiempo Δt:

$$e = \exp(-\omega \Delta t)$$
$$x_{new} = [x + (\dot{x} + \omega x)\Delta t] \cdot e$$
$$\dot{x}_{new} = [\dot{x} - \omega(\dot{x} + \omega x)\Delta t] \cdot e$$

Propiedades:
- **Cero sobrepaso** (críticamente amortiguado)
- **Convergencia exponencial** a tasa ω (12 rad/s por defecto)
- **Integración exacta** sin deriva numérica

### 15.4 Sistema de Dinámicas de Segundo Orden

Animación procedural con control de frecuencia, amortiguamiento y respuesta:

$$\ddot{y} = \frac{x + k_3 \dot{x} - y - k_1 \dot{y}}{k_2}$$

Con constantes derivadas de frecuencia natural $f$, amortiguamiento $\zeta$ y respuesta $r$:

$$\omega = 2\pi f, \quad k_1 = \frac{\zeta}{\pi f}, \quad k_2 = \frac{1}{\omega^2}, \quad k_3 = \frac{r \cdot \zeta}{2\pi f}$$

### 15.5 Teoría de Easing e Interpolación

**32 funciones de easing** organizadas en 10 familias:

| Familia | Función Matemática |
|---------|--------------------|
| Sine | $f(t) = 1 - \cos(\pi t / 2)$ |
| Quadratic | $f(t) = t^2$ |
| Cubic | $f(t) = t^3$ |
| Exponential | $f(t) = 2^{10(t-1)}$ |
| Elastic | $f(t) = -2^{10t-10}\sin\left(\frac{(10t - 10.75) \cdot 2\pi}{3}\right)$ |
| Bounce | Parabólica por partes |

Más smoothstep ($C^1$) y smootherstep ($C^2$), splines Hermite y Catmull-Rom.

**Solver de Curvas Bézier Cúbicas:** Resolución por Newton-Raphson (8 iteraciones) con respaldo por bisección (20 pasos), tolerancia $10^{-7}$.

### 15.6 Cálculo de SpO₂ (Ley de Beer-Lambert)

Ratio de ratios:

$$R = \frac{AC_{red} / DC_{red}}{AC_{IR} / DC_{IR}}$$

Estimación de saturación:

$$SpO_2 = 110 - 25R$$

Derivada de la ley de Beer-Lambert: $I = I_0 \cdot e^{-\epsilon(\lambda) \cdot c \cdot d}$

### 15.7 Calibración del INA219

$$Current\_LSB = \frac{I_{max}}{2^{15}} \quad [A/bit]$$

$$CAL = \text{trunc}\left(\frac{0.04096}{I_{LSB} \cdot R_{shunt}}\right)$$

Con $R_{shunt} = 0.1\Omega$ y $I_{max} = 3.2A$: $I_{LSB} \approx 97.66 \; \mu A/bit$

### 15.8 Cinemática de Servos

Mapeo de ángulo a ancho de pulso:

$$PW(\theta) = PW_{min} + \frac{\theta - \theta_{min}}{\theta_{max} - \theta_{min}} \cdot (PW_{max} - PW_{min})$$

Conversión a ticks del PCA9685:

$$ticks = \text{round}\left(\frac{PW_{\mu s} \cdot f_{PWM} \cdot 4096}{10^6}\right)$$

A 50 Hz: $ticks = PW_{\mu s} \times 0.2048$

---

## 16. Estructura del Proyecto

```
BaymaxMini/
├── CMakeLists.txt                    Sistema de compilación
├── README.md                         Documentación técnica en inglés
├── MANUAL_USUARIO.md                 Este manual
├── requirements.txt                  Dependencias de Python
├── configs/
│   ├── sensors_config.yaml           Direcciones I²C, filtros
│   ├── vision_params.yaml            Umbrales de detección
│   └── reminders.json                Horarios de medicamentos
├── models/
│   └── yolo_nano_int8.tflite         Modelo YOLOv8n cuantizado
├── src/
│   ├── cpp_core/                     ── NÚCLEO DE TIEMPO REAL ──
│   │   ├── main.cpp                  Orchestrator + bucle principal
│   │   ├── drivers/
│   │   │   ├── I2C_Bus.{cpp,h}       Wrapper I²C Linux (ioctl)
│   │   │   ├── PCA9685.{cpp,h}       Driver PWM 16 canales
│   │   │   ├── VL53L1X.{cpp,h}       Sensor Time-of-Flight
│   │   │   ├── MAX30102.{cpp,h}      Pulsioxímetro + HR
│   │   │   ├── MLX90614.{cpp,h}      Termómetro infrarrojo
│   │   │   └── INA219.{cpp,h}        Monitor de potencia
│   │   ├── modules/
│   │   │   ├── FaceController.{cpp,h}  Animación facial por resortes
│   │   │   ├── VitalsMonitor.{cpp,h}   Pipeline de signos vitales
│   │   │   └── PowerSystem.{cpp,h}     FSM de gestión energética
│   │   ├── interfaces/
│   │   │   └── I_Sensor.h              Interfaz abstracta + registro
│   │   ├── ipc/
│   │   │   └── SharedData.h            Protocolo binario IPC
│   │   └── utils/
│   │       ├── MathUtils.h             DSP, filtros, Vec2, springs
│   │       ├── DigitalFilter.h         Cadena PPG, detección picos
│   │       └── Easing.h               Bézier, 32 easings, timeline
│   └── python_brain/                 ── CEREBRO INTELIGENTE ──
│       ├── main.py                   Punto de entrada del cerebro
│       ├── core/
│       │   ├── brain.py              Coordinador central
│       │   ├── event_bus.py          Bus de eventos pub/sub
│       │   └── logger.py            Logging estructurado
│       ├── vision/
│       │   ├── camera.py             Control de cámara
│       │   ├── camera_stream.py      Captura con SharedMemory
│       │   ├── detector.py           Orquestador de detección
│       │   ├── tpu_pipeline.py       YOLOv8 + Edge TPU + NMS
│       │   ├── face_analyzer.py      Análisis geométrico facial
│       │   ├── object_detector.py    Detección de objetos genéricos
│       │   └── visual_memory.py      Memoria visual temporal
│       ├── audio/
│       │   ├── voice_system.py       Sistema de voz unificado
│       │   ├── stt_engine.py         Reconocimiento de habla (Vosk)
│       │   ├── tts_engine.py         Síntesis de voz (Piper)
│       │   ├── sound_fx.py           Efectos de sonido
│       │   ├── audio_alerts.py       Alertas auditivas
│       │   └── intent_parser.py      Extracción de intenciones
│       ├── medical/
│       │   ├── pharmacist.py         Medicamentos + recordatorios
│       │   ├── patient_history.py    Historial clínico
│       │   └── scheduler.py          Planificador médico
│       ├── communication/
│       │   ├── zmq_link.py           Puente ZMQ bidireccional
│       │   └── telemetry.py          Modelo Pydantic de telemetría
│       ├── logic/
│       │   ├── state_machine.py      FSM conductual (5 estados)
│       │   └── scheduler.py          Planificador global
│       ├── config/
│       │   ├── settings.py           Cargador YAML
│       │   ├── settings.yaml         Configuración principal
│       │   ├── db_migrate.py         Migraciones de BD
│       │   └── medicines.db          Base de datos de medicamentos
│       └── utils/
│           ├── logger.py             Utilidades de logging
│           └── time_utils.py         Helpers de zona horaria
```

---

## 17. Solución de Problemas

### Problemas de Hardware

| Problema | Causa Probable | Solución |
|----------|---------------|----------|
| Sensores no detectados en `i2cdetect` | Cables SDA/SCL sueltos | Verificar conexiones GPIO 2 y 3 |
| | I²C no habilitado | Ejecutar `sudo raspi-config` → I2C → Enable |
| | Dirección I²C incorrecta | Verificar con `i2cdetect -y 1` |
| Servos no se mueven | PCA9685 sin alimentación externa | Conectar fuente de 5V al pin V+ del PCA9685 |
| | Frecuencia PWM incorrecta | Verificar que esté a 50 Hz |
| Lecturas de HR inconsistentes | Dedo mal colocado sobre MAX30102 | Presionar firme pero sin apretar excesivamente |
| | Luz ambiental excesiva | Cubrir el sensor durante la medición |
| Batería reporta 0% | INA219 sin calibrar | Verificar `calibrate(0.1f, 3.2f)` en bootstrap |
| | Resistencia shunt incorrecta | Confirmar 0.1Ω conectada correctamente |

### Problemas de Software

| Problema | Causa Probable | Solución |
|----------|---------------|----------|
| Error de compilación C++ | Dependencias faltantes | `sudo apt install libzmq3-dev nlohmann-json3-dev libboost-all-dev` |
| Core entra en modo AUTONOMOUS | Cerebro Python no iniciado o sin conexión ZMQ | Verificar que ambos procesos estén ejecutándose |
| Sin audio de TTS | Piper no instalado o modelo no encontrado | Verificar ruta en `settings.yaml` |
| Sin reconocimiento de voz | Modelo Vosk no descargado | Descargar modelo desde vosk.io |
| Visión no detecta objetos | Modelo YOLOv8 no encontrado | Verificar `config/neural_net/yolo_v8n.pt` |
| ZMQ connection refused | Puertos ya en uso | Verificar que no haya otra instancia ejecutándose |

### Logs y Diagnóstico

```bash
journalctl -u baymax_core -f

tail -f /var/log/baymax/brain.log

i2cdetect -y 1

htop
```

---

## 18. Mantenimiento

### Actualizaciones de Software

```bash
cd BaymaxMini
git pull origin main
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
pip install -r requirements.txt --upgrade
```

### Calibración del INA219

Si se cambia la resistencia shunt o la batería, recalibrar editando en `main.cpp`:

```cpp
m_ina219.calibrate(SHUNT_OHMS, MAX_AMPS);
```

### Mantenimiento de Batería

- Almacenar al 50–60% de carga si no se usa por períodos prolongados
- No descargar por debajo de 9.0V (el sistema apaga a 8.5V)
- Temperatura de operación: 10°C – 45°C
- Verificar periódicamente los voltajes de celda con `i2cget`

### Limpieza de Base de Datos Médica

```bash
sqlite3 baymax_medical.db "DELETE FROM logs WHERE timestamp < strftime('%s','now','-30 days')"
```

---

## 19. Glosario

| Término | Definición |
|---------|------------|
| **BPM** | Pulsaciones por minuto (Beats Per Minute) |
| **DSP** | Procesamiento Digital de Señales |
| **EMA** | Media Móvil Exponencial |
| **FSM** | Máquina de Estados Finitos |
| **I²C** | Inter-Integrated Circuit, protocolo de comunicación serial |
| **IPC** | Comunicación Inter-Procesos |
| **LiPo** | Polímero de Litio (tipo de batería) |
| **NMS** | Non-Maximum Suppression (supresión de no-máximos) |
| **PPG** | Fotopletismografía |
| **PWM** | Modulación por Ancho de Pulso |
| **RT** | Tiempo Real (Real-Time) |
| **SoC** | Estado de Carga (State of Charge) |
| **SpO₂** | Saturación Periférica de Oxígeno |
| **SPAD** | Single-Photon Avalanche Diode |
| **STT** | Speech-to-Text (habla a texto) |
| **ToF** | Time-of-Flight (tiempo de vuelo) |
| **TTS** | Text-to-Speech (texto a habla) |
| **VAD** | Voice Activity Detection (detección de actividad de voz) |
| **VCSEL** | Vertical-Cavity Surface-Emitting Laser |
| **YOLOv8** | You Only Look Once v8 (modelo de detección de objetos) |
| **ZMQ** | ZeroMQ (librería de mensajería de alto rendimiento) |

---

## 20. Licencia y Créditos

**Licencia:** MIT

**Creador:** Gabriel Calderon

**Solicitado por:** Elias Bautista

**Repositorio:** [https://github.com/chele-s/BaymaxMini.git](https://github.com/chele-s/BaymaxMini.git)

**Tecnologías clave utilizadas:**
- C++17 con Boost Math y Boost Accumulators
- Python 3.11 con asyncio
- ZeroMQ para comunicación IPC
- YOLOv8n con Google Coral Edge TPU
- Vosk para STT offline
- Piper para TTS offline
- SQLite3 para persistencia médica
- APScheduler para recordatorios cron
- Pydantic para validación de datos

---

<p align="center">
  <em>«Hola. Soy Baymax, tu asistente personal de salud.»</em>
</p>
