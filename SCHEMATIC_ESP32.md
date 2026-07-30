<p align="center">
  <img src="img/Baymax_reference.jpeg" alt="Baymax Reference" width="180"/>
</p>

<h1 align="center">Esquemático de Conexiones — BaymaxMini × ESP32-S3</h1>

<p align="center">
  <strong>Diagrama completo de cableado para el nodo distribuido de hardware ESP32-S3-CAM</strong><br>
  <em>Creado por Gabriel Calderon · Solicitado por Elias Bautista</em>
</p>

---

## Arquitectura Distribuida

En la arquitectura distribuida v2.0 de BaymaxMini, el **ESP32-S3** actúa como el nodo de tiempo real determinista (Core 1 a 50Hz para I²C, servos y sensores; Core 0 para comunicación serie USB a 921600 baudios y servidor MJPEG por Wi-Fi). La **Laptop** ejecuta el cerebro en Python y el procesamiento numérico avanzado en C++.

---

## Pinout ESP32-S3-CAM (N16R8)

```
                    ┌─────────────────────────┐
                    │      ESP32-S3-CAM       │
                    │       (N16R8)           │
                    │                         │
            5V  [1] │■                       ○│ [28] GND
           3V3  [2] │■                       ○│ [27] GPIO47
           GND  [3] │○                       ○│ [26] GPIO48
   I2C SDA GPIO8[4] │●                       ○│ [25] GPIO45
   I2C SCL GPIO9[5] │●                       ○│ [24] GPIO0
        GPIO10 [6] │○                       ○│ [23] GPIO35
        GPIO11 [7] │○                       ○│ [22] GPIO36
        GPIO12 [8] │○                       ○│ [21] GPIO37
        GPIO13 [9] │○                       ○│ [20] GPIO38
        GPIO14 [10]│○                       ○│ [19] GPIO39
        GPIO15 [11]│○                       ○│ [18] GPIO40
        GPIO16 [12]│○                       ○│ [17] GPIO41
        GPIO17 [13]│○                       ○│ [16] GPIO42
                    └─────────────────────────┘

  ■ = Alimentación    ● = Usado para Bus I²C    ○ = Usado por DVP Cam / Libre
```

---

## Diagrama de Cableado I²C (Bus Único a 400 kHz)

```
 ┌───────────────────────────────────────────────────────────────────────────┐
 │                            ESP32-S3 NODE                                 │
 │                                                                          │
 │  ┌──────────────────── BUS I²C-1 (400 kHz) ──────────────────────┐      │
 │  │         GPIO 8 (SDA) ──────────┬──────┬──────┬──────┬─────┐   │      │
 │  │         GPIO 9 (SCL) ───────┬──┤      │      │      │     │   │      │
 │  │                             │  │      │      │      │     │   │      │
 │  │                     4.7kΩ pull-up a 3.3V (cada línea)     │   │      │
 │  │                             │  │      │      │      │     │   │      │
 │  │    ┌────────┐  ┌────────┐  ┌┴──┴─┐ ┌──┴──┐ ┌─┴───┐ ┌┴────┴┐  │      │
 │  │    │PCA9685 │  │VL53L1X │  │MAX  │ │MLX  │ │INA  │ │ Libre│  │      │
 │  │    │ 0x40   │  │ 0x29   │  │30102│ │90614│ │219  │ │      │  │      │
 │  │    │PWM Drv │  │  ToF   │  │0x57 │ │0x5A │ │0x41 │ │      │  │      │
 │  │    └───┬────┘  └────────┘  └─────┘ └─────┘ └──┬──┘ └──────┘  │      │
 │  │        │                                       │             │      │
 │  └────────┼───────────────────────────────────────┼─────────────┘      │
 │           │                                       │                    │
 │    CH0-CH11 (PWM)                          Shunt 0.1Ω                 │
 │    ┌──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┐   ┌──┴──────┐               │
 │    │S0│S1│S2│S3│S4│S5│S6│S7│S8│S9│SA│SB│   │ Batería │               │
 │    └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘   │3S LiPo  │               │
 │     12 Servomotores SG90/MG90S              │11.1V    │               │
 │     (Expresiones faciales)                  │2200mAh  │               │
 │                                             └─────────┘               │
 │  ┌──── Enlace con Laptop ──────────────────────────────────────┐     │
 │  │  USB-C (Serial @ 921600 Baud) ─── Telemetría / Comandos   │     │
 │  │  Wi-Fi (MJPEG Server @ Port 81) ─ Streaming de Video        │     │
 │  └─────────────────────────────────────────────────────────────┘     │
 └────────────────────────────────────────────────────────────────────────┘
```

---

## Tabla de Mapeo de Sensores y Direcciones I²C

| Dispositivo | Tipo | Dirección I²C | Pin SDA | Pin SCL | Frecuencia |
|---|---|---|---|---|---|
| **PCA9685** | Controlador PWM Servos (16 ch) | `0x40` | GPIO 8 | GPIO 9 | 400 kHz |
| **VL53L1X** | Sensor de distancia ToF | `0x29` | GPIO 8 | GPIO 9 | 400 kHz |
| **MAX30102** | Pulsioxímetro (SpO₂ / HR) | `0x57` | GPIO 8 | GPIO 9 | 400 kHz |
| **MLX90614** | Termómetro Infrarrojo | `0x5A` | GPIO 8 | GPIO 9 | 100 kHz |
| **INA219** | Monitor de voltaje / corriente | `0x41` | GPIO 8 | GPIO 9 | 400 kHz |

---

## Mapeo de Canales PCA9685 para Servos Faciales

| Canal | Componente Facial | Rango Ángulo | PWM Min (us) | PWM Max (us) |
|---|---|---|---|---|
| `0` | Párpado Superior Izquierdo | 0° - 90° | 500 | 2500 |
| `1` | Párpado Inferior Izquierdo | 0° - 90° | 500 | 2500 |
| `2` | Párpado Superior Derecho | 0° - 90° | 500 | 2500 |
| `3` | Párpado Inferior Derecho | 0° - 90° | 500 | 2500 |
| `4` | Mirada Izquierda (Eje X) | -45° - +45° | 1000 | 2000 |
| `5` | Mirada Izquierda (Eje Y) | -45° - +45° | 1000 | 2000 |
| `6` | Mirada Derecha (Eje X) | -45° - +45° | 1000 | 2000 |
| `7` | Mirada Derecha (Eje Y) | -45° - +45° | 1000 | 2000 |
| `8` | Mejilla Izquierda | 0° - 45° | 500 | 2500 |
| `9` | Mejilla Derecha | 0° - 45° | 500 | 2500 |
| `10` | Ceja Izquierda | 0° - 60° | 500 | 2500 |
| `11` | Ceja Derecha | 0° - 60° | 500 | 2500 |

---

## Mapeo de Pines de la Cámara DVP (OV2640 / OV5640)

| Función DVP | Pin ESP32-S3 |
|---|---|
| `XCLK` | GPIO 15 |
| `PCLK` | GPIO 13 |
| `VSYNC` | GPIO 6 |
| `HREF` | GPIO 7 |
| `SIOD (SDA)` | GPIO 4 |
| `SIOC (SCL)` | GPIO 5 |
| `D0 - D7` | GPIO 11, 9, 8, 10, 12, 18, 17, 16 |
| `PWDN / RESET` | -1 (No conectados / habilitados internamente) |

---

## Alimentación Eléctrica Recomendada

- **Batería principal**: LiPo 3S (11.1V nominal, 12.6V carga máxima).
- **Convertidor Buck DC-DC**: 12V a 5V / 5A continuos (alimenta el riel de servos PCA9685 y el puerto 5V del ESP32-S3).
- **Fusible de protección**: Fusible cerámico de 5A en la salida de la batería LiPo.
