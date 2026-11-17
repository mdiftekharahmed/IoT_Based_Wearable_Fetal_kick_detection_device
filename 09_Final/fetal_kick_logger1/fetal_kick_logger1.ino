/*
 * =====================================================================
 *  Fetal Kick Detection Device  -  STAGE 1: always-on sensing, serial only
 * =====================================================================
 *
 *  This is the "from scratch" rebuild, stripped down to the essentials
 *  so we can verify sensor behavior with zero interference.
 *
 *  WHAT'S ACTIVE
 *  --------------
 *   - MAX30100 heart rate / SpO2  (I2C bus 0, SDA 25 / SCL 26)
 *   - SW-18010P vibration          (GPIO 27)
 *   - GY-521 (MPU-6050) accel/gyro (I2C bus 1, SDA 21 / SCL 22 -
 *     separate hardware peripheral, Wire1, so it never waits behind
 *     the MAX30100's ~1000x/s polling on bus 0)
 *   - sensorTask() on CORE 1, always sampling, always printing to Serial
 *
 *  WHAT'S DISABLED (but left in place, commented, for later)
 *  -----------------------------------------------------------
 *   - Bluetooth (BluetoothSerial, SerialBT, all BT command handling)
 *   - SD card writing (SD.h, file open/write/flush, queue -> file)
 *
 *  Original GY-87 (damaged) has been swapped for a GY-521 (MPU-6050
 *  only, no magnetometer/barometer) - register map below matches the
 *  bring-up sketch that confirmed wiring and config on this board.
 *
 *  WHY CORE 1 FOR sensorTask
 *  --------------------------
 *   ESP-IDF pins the Bluetooth controller/host tasks to core 0
 *   unconditionally (CONFIG_BT_CTRL_PINNED_TO_CORE), and those tasks
 *   run at high priority for radio timing. Even with BT "disabled" in
 *   this sketch (no .begin() call), we're keeping sensorTask on core 1
 *   as the standing convention, since that's where it needs to live
 *   once BT comes back. loop() also defaults to core 1 (Arduino's
 *   loopTask), but that contention is mild - short, bursty, non-RF work.
 *
 *  NEXT STAGES (uncomment when ready)
 *  ------------------------------------
 *   Stage 2: re-enable SD writing (search "SD WRITE" below)
 *   Stage 3: re-enable Bluetooth (search "BLUETOOTH" below)
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ---- BLUETOOTH DISABLED (Stage 3) ----
#include <BluetoothSerial.h>

// ---- SD WRITE DISABLED (Stage 2) ----
#include <SPI.h>
#include <SD.h>

// ---------------------------------------------------------------- pins
  #define MAX_SDA 25
  #define MAX_SCL 26
  #define VIBRATION_PIN 27

  // ---- SD WRITE DISABLED (Stage 2) ----
  #define SD_CS   19
  #define SD_SCK  21
  #define SD_MISO 5
  #define SD_MOSI 18

  // GY-521 (MPU-6050) - bus 1, its own peripheral (see header note)
  #define GY_SDA 21
  #define GY_SCL 22

// ------------------------------------------------------- MPU-6050 regs
  #define MPU_ADDR          0x68
  #define REG_SMPLRT_DIV    0x19
  #define REG_CONFIG        0x1A
  #define REG_GYRO_CONFIG   0x1B
  #define REG_ACCEL_CONFIG  0x1C
  #define REG_ACCEL_XOUT_H  0x3B
  #define REG_PWR_MGMT_1    0x6B
  #define REG_WHO_AM_I      0x75

// -------------------------------------------------------------- config
  const uint32_t SAMPLE_INTERVAL_MS  = 100;            // 100 ms = 10 Hz
  const uint32_t SERIAL_INTERVAL_MS  = 1000;           // USB report cadence
  const uint32_t VIBRATION_WINDOW_MS = 100;            // vibration hardness window
  const int      QUEUE_LENGTH        = 400;            // cross-core buffer

  // MPU-6050 scale factors - keep these in sync with ACCEL_FS_SEL/GYRO_FS_SEL
  // below, they're what turns raw counts into g / dps.
  //   ACCEL +/-2g   -> 16384 LSB/g   (best resolution; a hard impact clips,
  //                                   which is itself a usable fall cue)
  //   GYRO  +/-250dps -> 131 LSB/(dps)
  const uint8_t  ACCEL_FS_SEL     = 0;      // 0=+/-2g 1=+/-4g 2=+/-8g 3=+/-16g
  const uint8_t  GYRO_FS_SEL      = 0;      // 0=250 1=500 2=1000 3=2000 dps
  const float    ACCEL_LSB_PER_G  = 16384.0f;
  const float    GYRO_LSB_PER_DPS = 131.0f;

  // DLPF_CFG = 4 -> ~21 Hz bandwidth, 1 kHz internal rate. Kick/movement/
  // fall energy sits below 20 Hz, and Nyquist at our 100 Hz sample rate
  // is 50 Hz, so this keeps the signal and rejects what would alias down.
  const uint8_t  DLPF_CFG   = 4;
  // Internal ODR = 1kHz / (1 + SMPLRT_DIV) with DLPF on. 4 -> 200 Hz,
  // deliberately ahead of our 100 Hz reads so we never re-read a stale sample.
  const uint8_t  SMPLRT_DIV = 4;

  // ---- SD WRITE DISABLED (Stage 2) ----
  // const uint32_t FLUSH_INTERVAL_MS = 1000;
  // const uint64_t MIN_FREE_SPACE    = 16ULL * 1024;

// -------------------------------------------------------------- objects
  PulseOximeter   pox;
  QueueHandle_t   sensorQueue;
  TwoWire        &GYWire = Wire1;    // second hardware I2C peripheral, MPU-6050 only
  bool            imuOk  = false;    // sensorTask retries init if this is false

  // ---- BLUETOOTH DISABLED (Stage 3) ----
  BluetoothSerial SerialBT;

  // ---- SD WRITE DISABLED (Stage 2) ----
  SPIClass SD_SPI(VSPI);

// --------------------------------------------------------- sample record
  struct SensorData {
    uint32_t scanTime;      // ms since boot-of-sensing
    float    ax, ay, az;    // accel g    (GY-521 / MPU-6050)
    float    gx, gy, gz;    // gyro dps   (GY-521 / MPU-6050)
    float    tempC;         // MPU-6050 die temperature, deg C
    int      vibration;     // window transition count (intensity) or 0
    float    heartRate;
    float    spo2;
  };

// ------------------------------------------------- shared / sense state
  volatile uint32_t senseStartTime = 0;   // set once in setup()

// live values (written on core 1 sensorTask, read on core 1 loop() for report)
  float heartRate = 0.0f;
  float spo2      = 0.0f;
  float imuTempC  = 0.0f;

// vibration state (sensorTask only)
  int      prevVibState = LOW;
  bool     vibMeasuring = false;
  uint32_t vibStart     = 0;
  int      vibHigh      = 0;
  int      vibLow       = 0;
  int      vibIntensity = 0;      // last completed window intensity
  bool     vibReady     = false;  // emit intensity on the next row

// ------------------------------------------------------------ prototypes
  void     onBeatDetected();
  void     measureVibration();
  void     sensorTask(void *param);
  void     drainQueueToSerial();
  void     serialReport();

  bool     mpuWriteReg(uint8_t reg, uint8_t val);
  bool     mpuReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);
  bool     initIMU();
  bool     readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &tempC);

// =====================================================================
// helpers
// =====================================================================
    void onBeatDetected() {               // keep tiny - runs inside pox.update()
      // Serial.println("Beat Detected!");   // uncomment for beat-level debug
    }

    // Vibration: SW-18010P, non-blocking 100 ms window.
    // Counts HIGH/LOW transitions; total = vibration intensity.
    void measureVibration() {
      int state = digitalRead(VIBRATION_PIN);

      if (!vibMeasuring && state != prevVibState) {     // movement starts a window
        vibMeasuring = true;
        vibStart = millis();
        vibHigh  = 0;
        vibLow   = 0;
      }

      if (vibMeasuring) {
        if (state != prevVibState) {
          if (state == HIGH) vibHigh++; else vibLow++;
          prevVibState = state;
        }
        if (millis() - vibStart >= VIBRATION_WINDOW_MS) {   // window closed
          vibMeasuring = false;
          vibIntensity = vibHigh + vibLow;
          vibReady     = true;
        }
      } else {
        prevVibState = state;
      }
    }

    // ---- MPU-6050 (GY-521), bus 1 - from the bring-up sketch ----

    bool mpuWriteReg(uint8_t reg, uint8_t val) {
      GYWire.beginTransmission(MPU_ADDR);
      GYWire.write(reg);
      GYWire.write(val);
      return GYWire.endTransmission(true) == 0;
    }

    bool mpuReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
      GYWire.beginTransmission(MPU_ADDR);
      GYWire.write(reg);
      if (GYWire.endTransmission(false) != 0) return false;      // repeated start
      if (GYWire.requestFrom((uint8_t)MPU_ADDR, len, (uint8_t)true) != len) return false;
      for (uint8_t i = 0; i < len; i++) buf[i] = GYWire.read();
      return true;
    }

    // Bytes read into a buffer and combined afterwards - Wire.read()<<8 |
    // Wire.read() leaves evaluation order up to the compiler.
    inline int16_t mpuBe16(const uint8_t *b) {
      return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    }

    bool initIMU() {
      uint8_t who = 0;
      if (!mpuReadRegs(REG_WHO_AM_I, &who, 1)) {
        Serial.println("MPU-6050: no reply on 0x68");
        return false;
      }
      Serial.printf("MPU-6050: WHO_AM_I = 0x%02X\n", who);   // 0x68 typical,
                                                              // 0x70/0x71/0x73 = compatible clone

      if (!mpuWriteReg(REG_PWR_MGMT_1, 0x80)) return false;   // device reset
      delay(100);
      if (!mpuWriteReg(REG_PWR_MGMT_1, 0x01)) return false;   // wake, clock = gyro X PLL
      delay(10);

      if (!mpuWriteReg(REG_CONFIG,       DLPF_CFG))          return false;
      if (!mpuWriteReg(REG_SMPLRT_DIV,   SMPLRT_DIV))        return false;
      if (!mpuWriteReg(REG_GYRO_CONFIG,  GYRO_FS_SEL  << 3)) return false;
      if (!mpuWriteReg(REG_ACCEL_CONFIG, ACCEL_FS_SEL << 3)) return false;
      return true;
    }

    // One 14-byte burst read (accel + temp + gyro), scaled to g / dps / C.
    // Called at SAMPLE_INTERVAL_MS cadence from sensorTask, not every
    // loop iteration - unlike pox.update(), the MPU doesn't need polling
    // faster than the rate we actually log at.
    bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &tempC) {
      uint8_t b[14];
      if (!mpuReadRegs(REG_ACCEL_XOUT_H, b, 14)) return false;

      int16_t rax = mpuBe16(&b[0]),  ray = mpuBe16(&b[2]),  raz = mpuBe16(&b[4]);
      int16_t rt  = mpuBe16(&b[6]);                          // TEMP_OUT
      int16_t rgx = mpuBe16(&b[8]),  rgy = mpuBe16(&b[10]), rgz = mpuBe16(&b[12]);

      ax = rax / ACCEL_LSB_PER_G;  ay = ray / ACCEL_LSB_PER_G;  az = raz / ACCEL_LSB_PER_G;
      gx = rgx / GYRO_LSB_PER_DPS; gy = rgy / GYRO_LSB_PER_DPS; gz = rgz / GYRO_LSB_PER_DPS;
      tempC = rt / 340.0f + 36.53f;    // datasheet formula - die temp, not body temp,
                                        // runs a few degrees above ambient
      return true;
    }

// =====================================================================
// CORE 1 : sensor task - fast loop, ALWAYS sampling (no scan gate)
// =====================================================================
  void sensorTask(void *param) {
    uint32_t lastSample = millis();

    for (;;) {
      pox.update();          // MUST be called as fast as possible (MAX30100)
      measureVibration();    // non-blocking

      uint32_t now = millis();

      if (now - lastSample >= SAMPLE_INTERVAL_MS) {
        lastSample += SAMPLE_INTERVAL_MS;

        heartRate = pox.getHeartRate();
        spo2      = pox.getSpO2();

        SensorData d;
        d.scanTime = now - senseStartTime;

        if (!imuOk) imuOk = initIMU();          // keep retrying so hot-fixed wiring recovers
        if (!imuOk || !readIMU(d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.tempC)) {
          d.ax = d.ay = d.az = 0.0f;            // read failed this sample - log zeros, don't stall
          d.gx = d.gy = d.gz = 0.0f;
          d.tempC = 0.0f;
          imuOk = false;                        // force re-init attempt next sample
        }
        imuTempC = d.tempC;

        d.vibration = vibReady ? vibIntensity : 0;
        d.heartRate = heartRate;
        d.spo2      = spo2;

        xQueueSend(sensorQueue, &d, 0);  // non-blocking: never stall the sensor
        vibReady = false;
      }

      // pdMS_TO_TICKS(1) instead of a bare "1" - correct regardless of
      // configTICK_RATE_HZ (bare vTaskDelay(1) can be ~10ms on a 100Hz tick).
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

// =====================================================================
// SETUP  (runs on core 1)
// =====================================================================
void setup() {
  Serial.begin(921600);
  delay(100);
  Serial.println("\n=== Fetal Kick Logger - Stage 1: sensing only (serial) ===");

  // ---- MAX30100 ----
  Wire.begin(MAX_SDA, MAX_SCL);
  Wire.setClock(400000);
  Serial.print("MAX30100... ");
  if (!pox.begin()) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);   // 7.6 mA works best
    pox.setOnBeatDetectedCallback(onBeatDetected);
  }

  // ---- vibration ----
  pinMode(VIBRATION_PIN, INPUT);
  prevVibState = digitalRead(VIBRATION_PIN);

  // ---- GY-521 (MPU-6050), bus 1 - separate peripheral from the MAX30100 ----
  GYWire.begin(GY_SDA, GY_SCL, 400000);    // 400 kHz is the MPU-6050 max
  Serial.print("MPU-6050... ");
  imuOk = initIMU();
  Serial.println(imuOk ? "OK (+/-2g, +/-250dps, DLPF ~21Hz, ~200Hz internal ODR)"
                        : "FAILED (will keep retrying during sensing)");

  // ---- BLUETOOTH DISABLED (Stage 3) ----
  // if (!SerialBT.begin("ESP32_SENSOR_LOGGER"))
  //   Serial.println("Bluetooth FAILED");
  // else
  //   Serial.println("Bluetooth: ESP32_SENSOR_LOGGER");

  // ---- SD WRITE DISABLED (Stage 2) ----
  // SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  // if (!SD.begin(SD_CS, SD_SPI, 20000000)) Serial.println("SD INIT FAILED");

  // ---- queue + core-1 sensor task ----
  sensorQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorData));
  if (!sensorQueue) Serial.println("Queue alloc FAILED");

  senseStartTime = millis();

  // Pinned to core 1: keeps this off the Bluetooth controller's core (0),
  // even though BT itself is off right now - this is the permanent home.
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, NULL, 3, NULL, 1);

  Serial.println("Sensing started. Printing to Serial only, nothing written.");
  Serial.println("scan_time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,vibration,heart_rate,spo2");
}

// =====================================================================
// LOOP  (core 1) : drain queue to Serial + periodic human-readable report
// =====================================================================
void loop() {
  drainQueueToSerial();
  serialReport();
  delay(1);
}

// ------------------------------------------------- queue -> Serial (CSV)
  void drainQueueToSerial() {
    SensorData d;
    char line[128];
    while (xQueueReceive(sensorQueue, &d, 0) == pdTRUE) {
      int n = snprintf(line, sizeof(line),
                      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%.1f,%.1f",
                      (unsigned long)d.scanTime,
                      d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.tempC,
                      d.vibration, d.heartRate, d.spo2);
      if (n < 0) continue;
      Serial.println(line);

      // ---- SD WRITE DISABLED (Stage 2) ----
      // Same line, ready to go straight to the file when we re-enable this:
      //   int n = snprintf(line, sizeof(line), "%lu,...\n", ...);
      //   scanFile.write((const uint8_t *)line, n);
      // See serviceSD() in the previous version for the flush/close logic.
    }
  }

// ------------------------------------------------- USB status line
  void serialReport() {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < SERIAL_INTERVAL_MS) return;
    last = now;
    Serial.printf(">> t=%lus  HR=%.0f  SpO2=%.0f  vib=%d  IMU_T=%.1fC  queued=%u\n",
                  (unsigned long)((now - senseStartTime) / 1000),
                  heartRate, spo2, vibIntensity, imuTempC,
                  (unsigned)uxQueueMessagesWaiting(sensorQueue));
  }

// =====================================================================
// ---- SD WRITE DISABLED (Stage 2) ----
// Everything below is the write-mechanism scaffolding from the full
// version, kept intact but inert, so re-enabling is just: uncomment,
// restore the SD/SPI includes and pins above, and call these from
// setup()/loop() where noted.
// =====================================================================
/*
  bool     sdReady     = false;
  bool     fileOpen    = false;
  File     scanFile;
  String   currentFile = "";
  uint32_t scanNumber  = 0;

  String fileNameFor(uint32_t n) {
    char name[24];
    snprintf(name, sizeof(name), "/SCAN_%03lu.csv", (unsigned long)n);
    return String(name);
  }

  uint32_t nextScanNumber() {
    uint32_t n = 1;
    while (SD.exists(fileNameFor(n))) n++;
    return n;
  }

  void cmdSdInit() {
    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SD_SPI, 20000000)) { Serial.println("SD INIT FAILED"); return; }
    sdReady = true;
    Serial.println("SD OK");
  }

  void cmdStart() {
    if (!sdReady) { Serial.println("run SD_INIT first"); return; }
    if (fileOpen) { Serial.println("previous file still closing, wait"); return; }
    if (SD.totalBytes() - SD.usedBytes() < MIN_FREE_SPACE) { Serial.println("SD FULL"); return; }

    scanNumber  = nextScanNumber();
    currentFile = fileNameFor(scanNumber);
    scanFile    = SD.open(currentFile, FILE_WRITE);
    if (!scanFile) { Serial.println("cannot create " + currentFile); return; }

    scanFile.println("scan_time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,vibration,heart_rate,spo2");
    scanFile.flush();
    fileOpen = true;
    Serial.println("SCAN STARTED -> " + currentFile);
  }

  void serviceSD() {
    static uint32_t lastFlush = 0;
    if (!fileOpen) return;

    SensorData d;
    char line[128];
    while (xQueueReceive(sensorQueue, &d, 0) == pdTRUE) {
      int n = snprintf(line, sizeof(line),
                      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%.1f,%.1f\n",
                      (unsigned long)d.scanTime,
                      d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.tempC,
                      d.vibration, d.heartRate, d.spo2);
      if (n < 0) continue;
      if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
      scanFile.write((const uint8_t *)line, n);
    }

    uint32_t now = millis();
    if (now - lastFlush >= FLUSH_INTERVAL_MS) {
      lastFlush = now;
      scanFile.flush();
    }
  }

  void cmdStop() {
    scanFile.flush();
    scanFile.close();
    fileOpen = false;
    Serial.println("SCAN STOPPED. FILE: " + currentFile);
  }
*/
