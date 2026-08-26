# IoT-Based Wearable Fetal Kick Detection Device

A wearable, ESP32-based data logger that detects and records **fetal kicks** by correlating a body-worn **vibration sensor** with an **inertial measurement unit (IMU)**. The device runs sensor acquisition on one CPU core and storage/communication on the other, logs to a microSD card, and is controlled over Bluetooth. Wi-Fi upload to a server is the planned next stage.

> **Research prototype — not a medical device.** This project is for engineering and academic purposes only. It is not validated for clinical use and must not be relied upon for medical decisions or fetal-wellbeing monitoring.

---

## Table of Contents

- [Overview](#overview)
- [How Fetal Kick Detection Works](#how-fetal-kick-detection-works)
- [Hardware](#hardware)
- [Wiring / Pinout](#wiring--pinout)
- [Repository Structure](#repository-structure)
- [Active Firmware: `05_simplified_dualcore`](#active-firmware-05_simplified_dualcore)
- [Building & Flashing](#building--flashing)
- [Bluetooth / Serial Command Reference](#bluetooth--serial-command-reference)
- [Data Format (CSV)](#data-format-csv)
- [Quick Start & Testing](#quick-start--testing)
- [Roadmap](#roadmap)
- [Development History](#development-history)
- [License & Disclaimer](#license--disclaimer)

---

## Overview

The device continuously watches for two things at once:

1. **Mechanical vibration** on the mother's abdomen (via a SW-18010P vibration switch), and
2. **Whole-body motion** of the mother (via a GY-87 IMU: accelerometer + gyroscope).

A **fetal kick** is inferred when a **vibration event occurs while the mother is otherwise still** (i.e. the IMU shows no significant maternal movement). The same IMU data is also intended to power **fall detection** for the pregnant woman.

All raw signals are timestamped onto a single uniform time grid and written to a microSD card as CSV, so the kick-vs-movement classification can be done offline during analysis. The card is treated as a durable local backlog; the roadmap adds Wi-Fi upload of these sessions to a server.

The firmware is built for an **ESP32** and uses both CPU cores so that the timing-critical heart-rate sensor is never starved by SD or Bluetooth work.

---

## How Fetal Kick Detection Works

| Observation | Vibration sensor | IMU (GY-87) | Interpretation |
|---|---|---|---|
| Fetal kick | **active** | still | **Kick** |
| Mother moving/walking | active | **moving** | Maternal motion (not a kick) |
| Resting | quiet | still | No event |
| Fall | (any) | **sudden large spike / free-fall** | Fall event |

The vibration sensor alone cannot tell a kick apart from the mother simply moving — the IMU is what disambiguates them. This is why the IMU must be logged at a high rate (see [Data Format](#data-format-csv)).

---

## Hardware

| Component | Role | Interface | Status |
|---|---|---|---|
| **ESP32 dev board** | Main MCU (dual-core, Wi-Fi + Bluetooth) | — | Active |
| **MAX30100** | Heart rate & SpO₂ | I²C | Active |
| **SW-18010P** | Vibration / kick detection | Digital GPIO | Active |
| **GY-87** (MPU6050 + HMC5883L + BMP180) | Accel / gyro / mag / barometer | I²C | **Disabled — hardware damaged** |
| **microSD module** | Session storage (64 GB card) | SPI (VSPI) | Active |
| **DS18B20** | Temperature | 1-Wire | Explored, not integrated |
| **ESP-01 / ESP8266** | Alt. Wi-Fi transceiver | Serial | Explored, superseded by ESP32 Wi-Fi |

The GY-87 accelerometer/gyroscope is central to the kick logic and to fall detection, but the physical module is currently damaged. The firmware keeps its columns in the data schema (zero-filled) so nothing changes when it is repaired — see [re-enabling the GY-87](#re-enabling-the-gy-87).

---

## Wiring / Pinout

Pins as defined in the active firmware (`05_simplified_dualcore`):

| Signal | ESP32 GPIO |
|---|---|
| MAX30100 SDA | 25 |
| MAX30100 SCL | 26 |
| Vibration (SW-18010P) | 27 |
| SD — CS | 5 |
| SD — SCK | 18 |
| SD — MISO | 19 |
| SD — MOSI | 23 |
| GY-87 SDA *(disabled)* | 21 |
| GY-87 SCL *(disabled)* | 22 |

The MAX30100 and the GY-87 sit on **two separate I²C buses** (different SDA/SCL pairs) so the always-on pulse oximeter is never blocked by IMU traffic. The SD card uses the VSPI bus at 20 MHz.

---

## Repository Structure

The numbered folders trace the development path: each sensor was brought up and tested in isolation first, then progressively combined. Each folder is a self-contained Arduino sketch.

```
IoT_Based_Wearable_Fetal_kick_detection_device/
├── 00_Testing_I2C_Communication/          I²C bring-up (Arduino)
├── 00_Testing_I2C_Communication_ESP32/    I²C bring-up (ESP32)
├── 00_DS18B20_Temperature/                DS18B20 first test
├── i2c_scanner_uno/                        I²C address scanner
├── 01_GY-87/                               IMU bring-up
│   ├── 00_Enabling_magnetometer/
│   ├── 01_Testing_MPU6050/
│   ├── 02_Testing_BMP180/
│   └── 03_reading_input/
├── 02_ESP-01_ESP8266_..._Transceiver/     ESP-01 Wi-Fi experiments
├── 03_Heart_Rate_Sensor/
│   └── sensor_testing/                     MAX30100 reference sketch
├── 04_SW-18010P_Vibration_Sensor/
│   ├── 00_sensor_Testing/
│   ├── 01_interrupt_to_register_data/
│   └── 02_Sense_vibration_hardness/        Vibration "hardness" reference
├── 05_DS18B20_Temp_Sensor/                 DS18B20 revisited
├── 06_Switching_TO_ESP8266/                ESP8266 port experiments
├── 07_Combined_All_Sensors/                Integrated firmware evolution
│   ├── 00_allinone/
│   ├── allinone_Optimized/
│   ├── 01_allinone_Optimized_Serial/       Verbose serial logger (legacy, ~3177 lines)
│   ├── 03_allinone_optimized_Bluetooth/
│   ├── 04_allinone_optimized_wifi/
│   └── 05_simplified_dualcore/   ★ ACTIVE FIRMWARE
├── Goal.txt                                Target system architecture notes
└── README.md
```

**Two sketches in `07_` are the ones that matter today:**

- `01_allinone_Optimized_Serial/` — the older, feature-complete but verbose logger. Kept as reference.
- **`05_simplified_dualcore/` — the current, actively developed firmware.** Everything below refers to this one.

---

## Active Firmware: `05_simplified_dualcore`

A deliberately simplified rewrite that enforces a strict split of work across the ESP32's two cores.

```
                 Bluetooth (SPP)  ──── control ────┐
                                                   │
   ┌──────────────────── ESP32 ───────────────────┼────────────────┐
   │                                               ▼                │
   │  CORE 0 — sensorTask()               CORE 1 — loop()           │
   │  ────────────────────────            ───────────────────────   │
   │  • MAX30100  pox.update()   ┌──────► • parse BT / USB commands  │
   │    (as fast as possible)    │        • drain queue → SD (CSV)   │
   │  • SW-18010P vibration      │        • (future) → Wi-Fi server  │
   │  • GY-87 accel/gyro*        │                                   │
   │        │                    │                                   │
   │        └──── FreeRTOS queue ┘  (cross-core, non-blocking)       │
   └────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
                               microSD  (/SCAN_00N.csv)
                     * GY-87 currently disabled → columns zero-filled
```

**Core 0 — sensing only.** Calls `pox.update()` (MAX30100) in a tight loop because that sensor's small FIFO must be serviced far faster than it fills. It also runs the non-blocking vibration window and, while a scan is active, packs each sample and pushes it onto a FreeRTOS queue. It never touches the SD card or Bluetooth.

**Core 1 — storage & control.** The Arduino `loop()` parses commands (from Bluetooth *and* USB serial), drains the queue, and writes rows to the SD card. This is also where Wi-Fi upload will live.

**Why the split:** the heart-rate algorithm is timing-sensitive; sharing a core with SD writes and Bluetooth caused it to miss samples. Dedicating Core 0 to sensing fixes that.

**`vTaskDelay(1)` in the sensor loop** yields ≤1 ms each iteration so the Bluetooth stack and task watchdog stay healthy, while still calling `pox.update()` ~1000×/s — roughly 10× the sample rate, so no samples are lost.

**Always-on sensors.** The MAX30100 is initialized once at boot (7.6 mA IR current, empty beat callback). `START` only opens a file and begins logging — it does **not** re-initialize the sensor — so heart rate and SpO₂ are already locked on when a session begins.

---

## Building & Flashing

**Prerequisites**

1. **Arduino IDE** with the **ESP32 board package** (Boards Manager → "esp32" by Espressif Systems).
2. A **MAX30100 library** that provides `MAX30100_PulseOximeter.h` (e.g. *MAX30100lib* by OXullo Intersecans).
3. `SD`, `SPI`, `Wire`, and `BluetoothSerial` ship with the ESP32 core — no separate install.

**Steps**

1. Open `07_Combined_All_Sensors/05_simplified_dualcore/05_simplified_dualcore.ino`.
2. Select your ESP32 board and serial port.
3. Compile and upload.
4. Open the Serial Monitor at **921600 baud**.

On boot you should see the sensor/Bluetooth init lines followed by a `READY.` banner listing the commands. There is intentionally **no further serial output until a scan is started**.

---

## Bluetooth / Serial Command Reference

Commands are accepted over **Bluetooth (SPP)** and over **USB serial**, are **case-insensitive**, and are dispatched on newline.

| Command | Action | Typical response |
|---|---|---|
| `SD_INIT` | Mount the SD card | `SD OK` + total/used/free KB |
| `START` | Open `/SCAN_00N.csv` and begin logging | `SCAN STARTED -> /SCAN_001.csv (10 Hz)` |
| `STOP` | Stop logging; flush and close the file | `SCAN STOPPED. FILE: /SCAN_001.csv` |
| `LIST` | List CSV files with an index and size | `0: /SCAN_001.csv (1234 B)` |
| `READ <n>` | Stream the file at index `n` over the link | file contents between markers |
| `SD_RELEASE` | Unmount the card (safe to remove) | `SD RELEASED` |
| `STATUS` | Report SD/scan/queue state | `--- status ---` block |

**Bluetooth device name:** `ESP32_SENSOR_LOGGER`. This uses **Classic Bluetooth SPP**, so it pairs with **Android or a PC** (e.g. the "Serial Bluetooth Terminal" app), **not iPhone**.

---

## Data Format (CSV)

Each session is written to `/SCAN_00N.csv` (auto-incrementing number). One header row, then one row per sample on a uniform time grid.

**Columns**

```
scan_time_ms, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, vibration, heart_rate, spo2
```

| Column | Meaning | Now (GY-87 disabled) |
|---|---|---|
| `scan_time_ms` | Milliseconds since `START` | ✓ |
| `accel_x/y/z` | Accelerometer (g), GY-87 | `0` (placeholder) |
| `gyro_x/y/z` | Gyroscope (°/s), GY-87 | `0` (placeholder) |
| `vibration` | Transition count of the 100 ms vibration window (intensity); `0` when no vibration | ✓ |
| `heart_rate` | Latest MAX30100 BPM | ✓ |
| `spo2` | Latest MAX30100 SpO₂ (%) | ✓ |

**Sample rate.** A single constant, `SAMPLE_INTERVAL_MS`, sets the master rate:

- **10 Hz now** (`= 100` ms) — sufficient while the accelerometer is out.
- **100 Hz** (`= 10` ms) once the GY-87 is restored — needed to separate kicks from maternal movement and to catch falls.

Heart rate and SpO₂ are written on every row (their underlying value updates ~once per second, so 10 Hz is plenty). Vibration intensity is emitted on the single row where its 100 ms window closes, and `0` otherwise.

**Storage.** Roughly 2–3 MB/hour at 10 Hz and ~25 MB/hour at 100 Hz, so a 64 GB card holds **months** of continuous recording. The card is a durable backlog for later server upload; capacity is not a constraint.

---

## Quick Start & Testing

You can test the entire command flow over the **USB Serial Monitor alone** — no phone pairing needed, because the parser reads USB serial too.

**Serial Monitor settings that trip people up**

- Baud: **921600**.
- Line ending: **Newline** (or "Both NL & CR"). On "No Line Ending", commands never dispatch and it looks frozen.

**Smoke test**

1. `SD_INIT` → prints card size. `SD INIT FAILED` here means wiring/card, not code.
2. `START` → begins logging; a status line prints once per second (`t= HR= SpO2= vib= queued=`).
3. Place a finger on the MAX30100 → HR/SpO₂ rise from 0 after ~10–15 s.
4. Tap/shake the vibration sensor → the `vib=` value jumps, returns to 0 when still.
5. `STOP` → finalizes the file.
6. `LIST` → confirms the file exists.
7. `READ 0` → dumps the CSV so you can eyeball the rows.
8. `SD_RELEASE` → safe to remove the card.

For Bluetooth, pair to `ESP32_SENSOR_LOGGER` from an Android SPP terminal and send the same commands.

---

## Roadmap

Derived from `Goal.txt`, the target is a field-deployable logger where **Bluetooth is control/diagnostics** and **Wi-Fi is the data channel**:

- **Wi-Fi upload manager** on Core 1 — batch-POST completed sessions to a server; the per-row line is already formatted so it can be reused for upload. Non-blocking, with `PENDING → IN_PROGRESS → COMPLETE` states.
- **Automatic upload** — flush the SD backlog whenever Wi-Fi becomes available, without blocking measurement.
- **Upload bookkeeping** — track which session files are already uploaded (e.g. a `/state/` marker) so nothing is sent twice or lost.
- **Richer diagnostics** — a `SENSORS` command returning per-sensor health (connected / last-reading age) distinct from live values.
- **Wi-Fi provisioning over Bluetooth** — scan for SSIDs and set credentials from the app.
- **Re-enable the GY-87** at 100 Hz for kick/movement separation and fall detection.
- **Modular refactor & binary logging** — split the firmware into managers (Sensor / Measurement / SD / WiFi / Upload / BLE) and consider a compact binary session format with an RTC timestamp.

### Re-enabling the GY-87

1. Uncomment the `GY_SDA (21)` / `GY_SCL (22)` defines.
2. Initialize the MPU6050 (and BMP180 if needed) on the second I²C bus in `setup()`.
3. In `sensorTask()`, replace the zero-fill block (marked `// --- GY-87 (add later) ---`) with real accel/gyro reads.
4. Set `SAMPLE_INTERVAL_MS = 10` for 100 Hz. The CSV schema does not change.

---

## Development History

The `00`–`06` folders are the incremental bring-up of each subsystem — I²C communication, the GY-87 IMU, the MAX30100 heart-rate sensor, the SW-18010P vibration sensor, the DS18B20 temperature probe, and ESP-01/ESP8266 Wi-Fi experiments. `07_Combined_All_Sensors` is where they were integrated, evolving from a verbose serial logger (`01_...`) through Bluetooth and Wi-Fi variants to the current simplified dual-core design (`05_...`).

---

## License & Disclaimer

This repository is an academic/research project. Unless a `LICENSE` file states otherwise, treat the code as "all rights reserved" and contact the author before reuse.

**Not a medical device.** The device is an engineering prototype and has not been clinically validated. It must not be used for medical diagnosis, fetal monitoring, or any health decision.
