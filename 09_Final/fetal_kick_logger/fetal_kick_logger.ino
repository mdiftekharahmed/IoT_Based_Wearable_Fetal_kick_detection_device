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
 *   - MAX30100 heart rate / SpO2  (I2C, SDA 25 / SCL 26)
 *   - SW-18010P vibration          (GPIO 27)
 *   - sensorTask() on CORE 1, always sampling, always printing to Serial
 *
 *  WHAT'S DISABLED (but left in place, commented, for later)
 *  -----------------------------------------------------------
 *   - Bluetooth (BluetoothSerial, SerialBT, all BT command handling)
 *   - SD card writing (SD.h, file open/write/flush, queue -> file)
 *   - GY-87 accel/gyro (still disabled from before - hardware damaged)
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
 *   Stage 4: bring GY-87 back online
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ---- BLUETOOTH DISABLED (Stage 3) ----
// #include <BluetoothSerial.h>

// ---- SD WRITE DISABLED (Stage 2) ----
// #include <SPI.h>
// #include <SD.h>

// ---------------------------------------------------------------- pins
  #define MAX_SDA 25
  #define MAX_SCL 26
  #define VIBRATION_PIN 27

  // ---- SD WRITE DISABLED (Stage 2) ----
  // #define SD_CS   5
  // #define SD_SCK  18
  // #define SD_MISO 19
  // #define SD_MOSI 23

  // GY-87 (add later - hardware damaged)
  // #define GY_SDA 21
  // #define GY_SCL 22

// -------------------------------------------------------------- config
  const uint32_t SAMPLE_INTERVAL_MS  = 100;            // 100 ms = 10 Hz
  const uint32_t SERIAL_INTERVAL_MS  = 1000;           // USB report cadence
  const uint32_t VIBRATION_WINDOW_MS = 100;            // vibration hardness window
  const int      QUEUE_LENGTH        = 400;            // cross-core buffer

  // ---- SD WRITE DISABLED (Stage 2) ----
  // const uint32_t FLUSH_INTERVAL_MS = 1000;
  // const uint64_t MIN_FREE_SPACE    = 16ULL * 1024;

// -------------------------------------------------------------- objects
  PulseOximeter   pox;
  QueueHandle_t   sensorQueue;

  // ---- BLUETOOTH DISABLED (Stage 3) ----
  // BluetoothSerial SerialBT;

  // ---- SD WRITE DISABLED (Stage 2) ----
  // SPIClass SD_SPI(VSPI);

// --------------------------------------------------------- sample record
  struct SensorData {
    uint32_t scanTime;      // ms since boot-of-sensing
    float    ax, ay, az;    // accel g    (GY-87 - 0 for now)
    float    gx, gy, gz;    // gyro dps   (GY-87 - 0 for now)
    int      vibration;     // window transition count (intensity) or 0
    float    heartRate;
    float    spo2;
  };

// ------------------------------------------------- shared / sense state
  volatile uint32_t senseStartTime = 0;   // set once in setup()

// live values (written on core 1 sensorTask, read on core 1 loop() for report)
  float heartRate = 0.0f;
  float spo2      = 0.0f;

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

        // --- GY-87 (add later): readMPU6050(); d.ax=accelX; ... d.gz=gyroZ; ---
        d.ax = d.ay = d.az = 0.0f;     // GY-87 disabled
        d.gx = d.gy = d.gz = 0.0f;     // GY-87 disabled

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

  // ---- GY-87 (add later - hardware damaged) ----
  // GYWire.begin(GY_SDA, GY_SCL);  initMPU6050();  initBMP180();

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
  Serial.println("scan_time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,vibration,heart_rate,spo2");
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
                      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.1f,%.1f",
                      (unsigned long)d.scanTime,
                      d.ax, d.ay, d.az, d.gx, d.gy, d.gz,
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
    Serial.printf(">> t=%lus  HR=%.0f  SpO2=%.0f  vib=%d  queued=%u\n",
                  (unsigned long)((now - senseStartTime) / 1000),
                  heartRate, spo2, vibIntensity,
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

    scanFile.println("scan_time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,vibration,heart_rate,spo2");
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
                      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.1f,%.1f\n",
                      (unsigned long)d.scanTime,
                      d.ax, d.ay, d.az, d.gx, d.gy, d.gz,
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
