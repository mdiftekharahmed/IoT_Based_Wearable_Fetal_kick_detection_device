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
2. **Whole-body motion** of the mother (via a GY-521 IMU: accelerometer + gyroscope).

A **fetal kick** is inferred when a **vibration event occurs while the mother is otherwise still** (i.e. the IMU shows no significant maternal movement). The same IMU data is also intended to power **fall detection** for the pregnant woman.

All raw signals are timestamped onto a single uniform time grid and written to a microSD card as CSV, so the kick-vs-movement classification can be done offline during analysis. The card is treated as a durable local backlog; the roadmap adds Wi-Fi upload of these sessions to a server.

The firmware is built for an **ESP32** and uses both CPU cores so that the timing-critical heart-rate sensor is never starved by SD or Bluetooth work.

---

## How Fetal Kick Detection Works

| Observation | Vibration sensor | IMU (GY-521) | Interpretation |
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
| **MAX30100** | Heart rate & SpO₂ | I²C bus 0 | Active |
| **SW-18010P** | Vibration / kick detection | Digital GPIO | Active |
| **GY-521** (MPU-6050) | Accel / gyro | I²C bus 1 | Active |
| **microSD module** | Session storage (64 GB card) | SPI (VSPI) | Active |
| **GY-87** (MPU6050 + HMC5883L + BMP180) | Previous IMU | I²C | **Retired — hardware damaged** |
| **DS18B20** | Temperature | 1-Wire | Explored, not integrated |
| **ESP-01 / ESP8266** | Alt. Wi-Fi transceiver | Serial | Explored, superseded by ESP32 Wi-Fi |

The GY-87 was replaced by a **GY-521**, which carries the same MPU-6050 accelerometer/gyroscope but no magnetometer (HMC5883L) and no barometer (BMP180). Neither of those was used by the kick or fall logic, so nothing was lost — the GY-521 drops onto the same two pins and the firmware treats it as the project's IMU.

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
| GY-521 SDA | 21 |
| GY-521 SCL | 22 |

The GY-521's `AD0` pin is left unconnected (or tied low), giving it I²C address `0x68`. Power it from **3V3**.

The MAX30100 and the GY-521 sit on the ESP32's **two separate I²C peripherals** — `Wire` (bus 0) for the pulse oximeter, `Wire1` (bus 1) for the IMU. The pulse oximeter is polled ~1000×/s, so giving the IMU its own bus means neither sensor ever waits on the other. Both run at 400 kHz, which is the MPU-6050's maximum. The SD card uses the VSPI bus at 20 MHz.

---

## Repository Structure

The numbered folders trace the development path: each sensor was brought up and tested in isolation first, then progressively combined. Each folder is a self-contained Arduino sketch.

```
IoT_Based_Wearable_Fetal_kick_detection_device/
├── 00_Testing_I2C_Communication/          I²C bring-up (Arduino)
├── 00_Testing_I2C_Communication_ESP32/    I²C bring-up (ESP32)
├── 00_DS18B20_Temperature/                DS18B20 first test
├── i2c_scanner_uno/                        I²C address scanner
├── 01_6DOF/                                IMU bring-up
│   ├── 00_Enabling_magnetometer/            (GY-87 era)
│   ├── 01_Testing_MPU6050/
│   ├── 02_Testing_BMP180/
│   ├── 03_reading_input/
│   ├── 04_Shifted_to_GY-521/                GY-521 sanity check (Arduino)
│   └── 05_GY-521_on_ESP32/                  GY-521 bring-up on ESP32 bus 1
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
   │  • GY-521 accel/gyro        │                                   │
   │        │                    │                                   │
   │        └──── FreeRTOS queue ┘  (cross-core, non-blocking)       │
   └────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
                               microSD  (/SCAN_00N.csv)
```

**Core 0 — sensing only.** Calls `pox.update()` (MAX30100) in a tight loop because that sensor's small FIFO must be serviced far faster than it fills. It also runs the non-blocking vibration window and, while a scan is active, reads the IMU, packs each sample and pushes it onto a FreeRTOS queue. It never touches the SD card or Bluetooth. Core 0 is the **only** core that talks to the IMU, so no locking is needed on `Wire1`.

**Core 1 — storage & control.** The Arduino `loop()` parses commands (from Bluetooth *and* USB serial), drains the queue, and writes rows to the SD card. This is also where Wi-Fi upload will live.

**Why the split:** the heart-rate algorithm is timing-sensitive; sharing a core with SD writes and Bluetooth caused it to miss samples. Dedicating Core 0 to sensing fixes that.

**`vTaskDelay(1)` in the sensor loop** yields ≤1 ms each iteration so the Bluetooth stack and task watchdog stay healthy, while still calling `pox.update()` ~1000×/s — roughly 10× the sample rate, so no samples are lost.

**Always-on sensors.** The MAX30100 and the GY-521 are both initialized once at boot (MAX30100 at 7.6 mA IR current with an empty beat callback). `START` only opens a file and begins logging — it does **not** re-initialize the sensors — so heart rate and SpO₂ are already locked on when a session begins.

**IMU dropout handling.** A wearable on jumper leads will lose the IMU occasionally. If a burst read fails, the firmware marks the IMU down, keeps logging rows with zeroed accel/gyro columns (so the time grid and the other sensors are never interrupted), and retries initialization every 2 s in the background. `STATUS` reports the state and a cumulative dropout count, and `START` warns if the IMU is down.

### IMU configuration

Set once in `imuInit()` and mirrored in `01_6DOF/05_GY-521_on_ESP32`:

| Register | Value | Meaning |
|---|---|---|
| `PWR_MGMT_1` (0x6B) | reset, then `0x01` | Clock source = gyro X PLL (more stable than the internal 8 MHz oscillator) |
| `CONFIG` (0x1A) | `DLPF_CFG = 3` | ~44 Hz bandwidth — passes kick/fall transients, limits aliasing at 100 Hz |
| `SMPLRT_DIV` (0x19) | `9` | Internal ODR = 1 kHz / (1+9) = **100 Hz**, matching the log grid |
| `GYRO_CONFIG` (0x1B) | `FS_SEL = 0` | ±250 °/s → **131 LSB/(°/s)** |
| `ACCEL_CONFIG` (0x1C) | `AFS_SEL = 0` | ±2 g → **16384 LSB/g** |

±2 g is the most sensitive range, which is what picking a faint kick out of the noise floor needs. A hard fall impact will saturate it — that saturation is itself a usable fall cue, but if fall magnitude matters more than kick sensitivity, raise `ACCEL_FS_SEL` and update the scale factor in both places.

Data is read as **one 14-byte burst** from `0x3B`. `TEMP_OUT` (0x41–0x42) sits between the accel and gyro registers, so a 12-byte read silently shifts every gyro axis by one register pair — a trap worth remembering, since the accel values still look perfectly correct when it happens. The die temperature that comes free with the burst is reported by `STATUS` but not logged.

---

## Building & Flashing

**Prerequisites**

1. **Arduino IDE** with the **ESP32 board package** (Boards Manager → "esp32" by Espressif Systems).
2. A **MAX30100 library** that provides `MAX30100_PulseOximeter.h` (e.g. *MAX30100lib* by OXullo Intersecans).
3. `SD`, `SPI`, `Wire`, and `BluetoothSerial` ship with the ESP32 core — no separate install.

The GY-521 is driven by direct register writes over `Wire1`, so **no MPU6050 library is needed** — one less dependency, and the exact configuration is visible in `imuInit()` instead of buried in someone else's defaults.

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
| `START` | Open `/SCAN_00N.csv` and begin logging | `SCAN STARTED -> /SCAN_001.csv (100 Hz)` + the accel/gyro scale factors |
| `STOP` | Stop logging; flush and close the file | `SCAN STOPPED. FILE: /SCAN_001.csv` |
| `LIST` | List CSV files with an index and size | `0: /SCAN_001.csv (1234 B)` |
| `READ <n>` | Stream the file at index `n` over the link | file contents between markers |
| `SD_RELEASE` | Unmount the card (safe to remove) | `SD RELEASED` |
| `STATUS` | Report SD / scan / queue / IMU state | `--- status ---` block |

**Bluetooth device name:** `ESP32_SENSOR_LOGGER`. This uses **Classic Bluetooth SPP**, so it pairs with **Android or a PC** (e.g. the "Serial Bluetooth Terminal" app), **not iPhone**.

---

## Data Format (CSV)

Each session is written to `/SCAN_00N.csv` (auto-incrementing number). One header row, then one row per sample on a uniform time grid.

**Columns**

```
scan_time_ms, accel_x_raw, accel_y_raw, accel_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw, vibration, heart_rate, spo2
```

| Column | Meaning | Unit |
|---|---|---|
| `scan_time_ms` | Milliseconds since `START` | ms |
| `accel_x/y/z_raw` | Accelerometer, GY-521 | **raw int16 counts** — divide by **16384** for g |
| `gyro_x/y/z_raw` | Gyroscope, GY-521 | **raw int16 counts** — divide by **131** for °/s |
| `vibration` | Transition count of the 100 ms vibration window (intensity); `0` when no vibration | count |
| `heart_rate` | Latest MAX30100 BPM | BPM |
| `spo2` | Latest MAX30100 SpO₂ | % |

**Why raw counts.** The card stores exactly what the sensor's registers held, so nothing is lost to on-device float rounding and rows stay short. The cost is that the CSV carries no units of its own — the scale factors above are the only key to the file, and they are only valid while `ACCEL_FS_SEL`/`GYRO_FS_SEL` stay at `0`. The firmware prints them on every `START`, so a session transcript always records which scaling applied. In pandas:

```python
df = pd.read_csv("SCAN_001.csv")
df[["accel_x_g", "accel_y_g", "accel_z_g"]] = df[["accel_x_raw", "accel_y_raw", "accel_z_raw"]] / 16384.0
df[["gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]] = df[["gyro_x_raw", "gyro_y_raw", "gyro_z_raw"]] / 131.0
```

A resting sensor reads ≈ ±16384 on whichever axis faces gravity and ≈ 0 on the other two, so a quick sanity check on any session is that the accel magnitude sits near 16384 while the wearer is still.

**Sample rate.** A single constant, `SAMPLE_INTERVAL_MS`, sets the master rate — now **10 ms = 100 Hz**, which is what separating a kick from maternal movement (and catching a fall) requires. The MPU-6050's internal output rate is configured to match at 100 Hz, so every logged row is a fresh conversion rather than the same register value read twice.

Heart rate and SpO₂ are written on every row (their underlying value updates ~once per second, so they simply repeat). Vibration intensity is emitted on the single row where its 100 ms window closes, and `0` otherwise.

**Storage.** Rows are ~45 bytes, so 100 Hz is roughly **16 MB/hour** — a 64 GB card holds months of continuous recording. The card is a durable backlog for later server upload; capacity is not a constraint.

---

## Quick Start & Testing

You can test the entire command flow over the **USB Serial Monitor alone** — no phone pairing needed, because the parser reads USB serial too.

**Serial Monitor settings that trip people up**

- Baud: **921600**.
- Line ending: **Newline** (or "Both NL & CR"). On "No Line Ending", commands never dispatch and it looks frozen.

**Smoke test**

1. `SD_INIT` → prints card size. `SD INIT FAILED` here means wiring/card, not code.
2. `START` → begins logging; a status line prints once per second (`t= A= G= HR= SpO2= vib= queued=`).
3. Tilt the board → the `A=` triplet should swing, with ≈16384 on whichever axis points down. `[IMU DOWN]` at the end of the line means the GY-521 isn't answering.
4. Place a finger on the MAX30100 → HR/SpO₂ rise from 0 after ~10–15 s.
5. Tap/shake the vibration sensor → the `vib=` value jumps, returns to 0 when still.
6. `STOP` → finalizes the file.
7. `LIST` → confirms the file exists.
8. `READ 0` → dumps the CSV so you can eyeball the rows.
9. `SD_RELEASE` → safe to remove the card.

If the IMU is the only thing misbehaving, flash `01_6DOF/05_GY-521_on_ESP32` instead — it scans I²C bus 1, prints `WHO_AM_I`, and shows raw and scaled values side by side, which isolates a wiring fault from a firmware one.

For Bluetooth, pair to `ESP32_SENSOR_LOGGER` from an Android SPP terminal and send the same commands.

---

## Roadmap

Derived from `Goal.txt`, the target is a field-deployable logger where **Bluetooth is control/diagnostics** and **Wi-Fi is the data channel**:

- **Wi-Fi upload manager** on Core 1 — batch-POST completed sessions to a server; the per-row line is already formatted so it can be reused for upload. Non-blocking, with `PENDING → IN_PROGRESS → COMPLETE` states.
- **Automatic upload** — flush the SD backlog whenever Wi-Fi becomes available, without blocking measurement.
- **Upload bookkeeping** — track which session files are already uploaded (e.g. a `/state/` marker) so nothing is sent twice or lost.
- **Richer diagnostics** — a `SENSORS` command returning per-sensor health (connected / last-reading age) distinct from live values.
- **Wi-Fi provisioning over Bluetooth** — scan for SSIDs and set credentials from the app.
- **Gyro/accel bias calibration** — record a short still-baseline at the start of a session (or a stored per-device offset) so "mother is still" can be judged against a real zero rather than the factory bias.
- **Kick classifier** — the actual vibration-while-still detection, run offline over the CSVs first, then ported on-device once the thresholds are known.
- **Modular refactor & binary logging** — split the firmware into managers (Sensor / Measurement / SD / WiFi / Upload / BLE) and consider a compact binary session format with an RTC timestamp.

**Done:** the IMU is live again — the damaged GY-87 was replaced with a GY-521 on the same pins, and logging runs at 100 Hz with real accel/gyro columns.

---

## Development History

The `00`–`06` folders are the incremental bring-up of each subsystem — I²C communication, the IMU (GY-87, later GY-521), the MAX30100 heart-rate sensor, the SW-18010P vibration sensor, the DS18B20 temperature probe, and ESP-01/ESP8266 Wi-Fi experiments. `07_Combined_All_Sensors` is where they were integrated, evolving from a verbose serial logger (`01_...`) through Bluetooth and Wi-Fi variants to the current simplified dual-core design (`05_...`).

---

## License & Disclaimer

This repository is an academic/research project. Unless a `LICENSE` file states otherwise, treat the code as "all rights reserved" and contact the author before reuse.

**Not a medical device.** The device is an engineering prototype and has not been clinically validated. It must not be used for medical diagnosis, fetal monitoring, or any health decision.
