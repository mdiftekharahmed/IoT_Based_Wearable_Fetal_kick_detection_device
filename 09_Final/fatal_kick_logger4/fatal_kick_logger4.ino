/*
 * =====================================================================
 *  Fetal Kick Detection Device  -  STAGE 2: Sensing + SD Card Logging
 * =====================================================================
 *
 *  WHAT'S ACTIVE
 *  --------------
 *   - MAX30100 heart rate / SpO2  (I2C bus 0, SDA 25 / SCL 26)
 *   - SW-18010P vibration          (GPIO 27)
 *   - GY-521 (MPU-6050) accel/gyro (I2C bus 1, SDA 21 / SCL 22 - Wire1)
 *   - SPI SD Card Module          (VSPI: CS 5, SCK 18, MISO 19, MOSI 23)
 *   - sensorTask() on CORE 1, always sampling into sensorQueue
 *   - loop() on CORE 1, draining sensorQueue to both Serial & SD Card
 *
 *  WHAT'S REMOVED
 *  --------------
 *   - Bluetooth completely stripped out.
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "MAX30100_PulseOximeter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ---------------------------------------------------------------- pins
  #define MAX_SDA       25
  #define MAX_SCL       26
  #define VIBRATION_PIN 27

  // SD Card Pins (VSPI Hardware Bus)
  #define SD_CS         5
  #define SD_SCK        18
  #define SD_MISO       19
  #define SD_MOSI       23

  // GY-521 (MPU-6050) - bus 1
  #define GY_SDA        21
  #define GY_SCL        22

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

  const uint32_t FLUSH_INTERVAL_MS   = 1000;           // Flush SD file every 1 sec
  const uint64_t MIN_FREE_SPACE      = 16ULL * 1024;   // Minimum 16 KB free space

  const uint8_t  ACCEL_FS_SEL     = 0;      // 0=+/-2g 1=+/-4g 2=+/-8g 3=+/-16g
  const uint8_t  GYRO_FS_SEL      = 0;      // 0=250 1=500 2=1000 3=2000 dps
  const float    ACCEL_LSB_PER_G  = 16384.0f;
  const float    GYRO_LSB_PER_DPS = 131.0f;

  const uint8_t  DLPF_CFG   = 4;
  const uint8_t  SMPLRT_DIV = 4;

// -------------------------------------------------------------- objects
  PulseOximeter   pox;
  QueueHandle_t   sensorQueue;
  TwoWire        &GYWire = Wire1;    // second hardware I2C peripheral for MPU-6050
  bool            imuOk  = false;

  SPIClass        SD_SPI(VSPI);
  File            scanFile;
  bool            sdReady     = false;
  bool            fileOpen    = false;
  String          currentFile = "";
  uint32_t        scanNumber  = 0;

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
  volatile uint32_t senseStartTime = 0;

// live values
  float heartRate = 0.0f;
  float spo2      = 0.0f;
  float imuTempC  = 0.0f;

// vibration state (sensorTask only)
  int      prevVibState = LOW;
  bool     vibMeasuring = false;
  uint32_t vibStart     = 0;
  int      vibHigh      = 0;
  int      vibLow       = 0;
  int      vibIntensity = 0;
  bool     vibReady     = false;

// ------------------------------------------------------------ prototypes
  void     onBeatDetected();
  void     measureVibration();
  void     sensorTask(void *param);
  void     drainQueueAndWrite();
  void     serialReport();

  bool     initSD();
  bool     startNewScanFile();
  String   fileNameFor(uint32_t n);
  uint32_t nextScanNumber();

  bool     mpuWriteReg(uint8_t reg, uint8_t val);
  bool     mpuReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);
  bool     initIMU();
  bool     readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &tempC);

// =====================================================================
// HELPERS & SD CARD SCALLFOLDING
// =====================================================================
  void onBeatDetected() {
    // Beat callback
  }

  void measureVibration() {
    int state = digitalRead(VIBRATION_PIN);

    if (!vibMeasuring && state != prevVibState) {
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
      if (millis() - vibStart >= VIBRATION_WINDOW_MS) {
        vibMeasuring = false;
        vibIntensity = vibHigh + vibLow;
        vibReady     = true;
      }
    } else {
      prevVibState = state;
    }
  }

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

  bool initSD() {
    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SD_SPI, 20000000)) {
      Serial.println("SD Card: INIT FAILED!");
      return false;
    }
    sdReady = true;
    Serial.println("SD Card: OK");
    return true;
  }

  bool startNewScanFile() {
    if (!sdReady) return false;
    
    uint64_t freeBytes = SD.totalBytes() - SD.usedBytes();
    if (freeBytes < MIN_FREE_SPACE) {
      Serial.println("SD Card: FULL (Not enough space)");
      return false;
    }

    scanNumber  = nextScanNumber();
    currentFile = fileNameFor(scanNumber);
    scanFile    = SD.open(currentFile, FILE_WRITE);
    
    if (!scanFile) {
      Serial.println("SD Card: Cannot create " + currentFile);
      return false;
    }

    // Write CSV Header
    scanFile.println("scan_time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,vibration,heart_rate,spo2");
    scanFile.flush();
    fileOpen = true;
    
    Serial.println("SD Card: Recording started -> " + currentFile);
    return true;
  }

  // ---- MPU-6050 (GY-521) ----
  bool mpuWriteReg(uint8_t reg, uint8_t val) {
    GYWire.beginTransmission(MPU_ADDR);
    GYWire.write(reg);
    GYWire.write(val);
    return GYWire.endTransmission(true) == 0;
  }

  bool mpuReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    GYWire.beginTransmission(MPU_ADDR);
    GYWire.write(reg);
    if (GYWire.endTransmission(false) != 0) return false;
    if (GYWire.requestFrom((uint8_t)MPU_ADDR, len, (uint8_t)true) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = GYWire.read();
    return true;
  }

  inline int16_t mpuBe16(const uint8_t *b) {
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  }

  bool initIMU() {
    uint8_t who = 0;
    if (!mpuReadRegs(REG_WHO_AM_I, &who, 1)) return false;
    
    if (!mpuWriteReg(REG_PWR_MGMT_1, 0x80)) return false;
    delay(100);
    if (!mpuWriteReg(REG_PWR_MGMT_1, 0x01)) return false;
    delay(10);

    if (!mpuWriteReg(REG_CONFIG,       DLPF_CFG))          return false;
    if (!mpuWriteReg(REG_SMPLRT_DIV,   SMPLRT_DIV))        return false;
    if (!mpuWriteReg(REG_GYRO_CONFIG,  GYRO_FS_SEL  << 3)) return false;
    if (!mpuWriteReg(REG_ACCEL_CONFIG, ACCEL_FS_SEL << 3)) return false;
    return true;
  }

  bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &tempC) {
    uint8_t b[14];
    if (!mpuReadRegs(REG_ACCEL_XOUT_H, b, 14)) return false;

    int16_t rax = mpuBe16(&b[0]),  ray = mpuBe16(&b[2]),  raz = mpuBe16(&b[4]);
    int16_t rt  = mpuBe16(&b[6]);
    int16_t rgx = mpuBe16(&b[8]),  rgy = mpuBe16(&b[10]), rgz = mpuBe16(&b[12]);

    ax = rax / ACCEL_LSB_PER_G;  ay = ray / ACCEL_LSB_PER_G;  az = raz / ACCEL_LSB_PER_G;
    gx = rgx / GYRO_LSB_PER_DPS; gy = rgy / GYRO_LSB_PER_DPS; gz = rgz / GYRO_LSB_PER_DPS;
    tempC = rt / 340.0f + 36.53f;
    return true;
  }

// =====================================================================
// CORE 1 : sensor task - fast loop, ALWAYS sampling
// =====================================================================
  void sensorTask(void *param) {
    uint32_t lastSample = millis();

    for (;;) {
      pox.update();
      measureVibration();

      uint32_t now = millis();

      if (now - lastSample >= SAMPLE_INTERVAL_MS) {
        lastSample += SAMPLE_INTERVAL_MS;

        heartRate = pox.getHeartRate();
        spo2      = pox.getSpO2();

        SensorData d;
        d.scanTime = now - senseStartTime;

        if (!imuOk) imuOk = initIMU();
        if (!imuOk || !readIMU(d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.tempC)) {
          d.ax = d.ay = d.az = 0.0f;
          d.gx = d.gy = d.gz = 0.0f;
          d.tempC = 0.0f;
          imuOk = false;
        }
        imuTempC = d.tempC;

        d.vibration = vibReady ? vibIntensity : 0;
        d.heartRate = heartRate;
        d.spo2      = spo2;

        xQueueSend(sensorQueue, &d, 0);
        vibReady = false;
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

// =====================================================================
// SETUP  (runs on core 1)
// =====================================================================
void setup() {
  Serial.begin(921600);
  delay(100);
  Serial.println("\n=== Fetal Kick Logger - Stage 2: Sensing + SD Logging ===");

  // ---- MAX30100 ----
  Wire.begin(MAX_SDA, MAX_SCL);
  Wire.setClock(400000);
  Serial.print("MAX30100... ");
  if (!pox.begin()) {
    Serial.println("FAILED");
  } else {
    Serial.println("OK");
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    pox.setOnBeatDetectedCallback(onBeatDetected);
  }

  // ---- Vibration ----
  pinMode(VIBRATION_PIN, INPUT);
  prevVibState = digitalRead(VIBRATION_PIN);

  // ---- MPU-6050 ----
  GYWire.begin(GY_SDA, GY_SCL, 400000);
  Serial.print("MPU-6050... ");
  imuOk = initIMU();
  Serial.println(imuOk ? "OK" : "FAILED (Will keep retrying)");

  // ---- SD Card Initialization & File Creation ----
  if (initSD()) {
    startNewScanFile();
  }

  // ---- Queue & Task Setup ----
  sensorQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorData));
  if (!sensorQueue) Serial.println("Queue alloc FAILED");

  senseStartTime = millis();

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, NULL, 3, NULL, 1);

  Serial.println("Sensing & Logging active.");
  Serial.println("scan_time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,vibration,heart_rate,spo2");
}

// =====================================================================
// LOOP  (core 1) : drain queue to Serial & SD file
// =====================================================================
void loop() {
  drainQueueAndWrite();
  serialReport();
  delay(1);
}

// ------------------------------------------------- Queue -> Serial & SD
  void drainQueueAndWrite() {
    static uint32_t lastFlush = 0;
    SensorData d;
    char line[128];

    while (xQueueReceive(sensorQueue, &d, 0) == pdTRUE) {
      int n = snprintf(line, sizeof(line),
                      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%.1f,%.1f",
                      (unsigned long)d.scanTime,
                      d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.tempC,
                      d.vibration, d.heartRate, d.spo2);
      if (n < 0) continue;

      // 1. Output to Serial Monitor
      Serial.println(line);

      // 2. Write to SD Card file (if open)
      if (fileOpen) {
        scanFile.println(line);
      }
    }

    // Flush to SD card every 1 second to prevent data loss on power drop
    uint32_t now = millis();
    if (fileOpen && (now - lastFlush >= FLUSH_INTERVAL_MS)) {
      lastFlush = now;
      scanFile.flush();
    }
  }

// ------------------------------------------------- USB status line
  void serialReport() {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < SERIAL_INTERVAL_MS) return;
    last = now;
    Serial.printf(">> t=%lus  HR=%.0f  SpO2=%.0f  vib=%d  IMU_T=%.1fC  queued=%u  SD=%s\n",
                  (unsigned long)((now - senseStartTime) / 1000),
                  heartRate, spo2, vibIntensity, imuTempC,
                  (unsigned)uxQueueMessagesWaiting(sensorQueue),
                  fileOpen ? currentFile.c_str() : "DISABLED/FAIL");
  }