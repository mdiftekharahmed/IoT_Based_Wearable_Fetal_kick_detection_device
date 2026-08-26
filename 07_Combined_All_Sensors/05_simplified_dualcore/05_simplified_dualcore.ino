/*
 * =====================================================================
 *  Fetal Kick Detection Device  -  simplified dual-core logger
 * =====================================================================
 *
 *  CORE 0 : sensorTask()  -> reads sensors as fast as possible and,
 *                            while scanning, pushes samples to a queue.
 *  CORE 1 : loop()        -> handles Bluetooth commands, drains the queue
 *                            onto the SD card, and (later) uploads to a server.
 *
 *  Sensors
 *  -------
 *   MAX30100  heart rate / SpO2   I2C  (SDA 25, SCL 26)   like 03_Heart_Rate_Sensor/sensor_testing
 *   SW-18010P vibration           GPIO 27                 like 04_.../02_Sense_vibration_hardness
 *   GY-87     accel + gyro        I2C  (SDA 21, SCL 22)   DISABLED (damaged) - add later
 *
 *  Logging  (one uniform time grid so every signal lines up)
 *  ---------------------------------------------------------
 *   accel_x/y/z, gyro_x/y/z : GY-87, full rate   (0 for now - add later)
 *   vibration               : transition count of the last 100 ms window
 *                             (= intensity); 0 when there is no vibration
 *   heart_rate, spo2        : latest MAX30100 value on every row
 *
 *   Master rate = SAMPLE_INTERVAL_MS. 10 Hz now (GY-87 off); set it to
 *   10 (ms) for 100 Hz once the accelerometer is back.
 *
 *  Bluetooth (device "ESP32_SENSOR_LOGGER"):
 *   SD_INIT | START | STOP | LIST | READ <n> | SD_RELEASE | STATUS
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <BluetoothSerial.h>
#include "MAX30100_PulseOximeter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ---------------------------------------------------------------- pins
  #define MAX_SDA 25
  #define MAX_SCL 26
  #define VIBRATION_PIN 27

  #define SD_CS   5
  #define SD_SCK  18
  #define SD_MISO 19
  #define SD_MOSI 23

  // GY-87 (add later - hardware damaged)
  // #define GY_SDA 21
  // #define GY_SCL 22

// -------------------------------------------------------------- config
  // Master log rate. GY-87 disabled -> 10 Hz is plenty.
  // When the accelerometer is back, set this to 10  (= 100 Hz).
  const uint32_t SAMPLE_INTERVAL_MS  = 100;            // 100 ms = 10 Hz

  const uint32_t FLUSH_INTERVAL_MS   = 1000;           // SD flush cadence
  const uint32_t SERIAL_INTERVAL_MS  = 1000;           // USB report while scanning
  const uint32_t VIBRATION_WINDOW_MS = 100;            // vibration hardness window
  const uint64_t MIN_FREE_SPACE      = 16ULL * 1024;   // stop before the card fills
  const int      QUEUE_LENGTH        = 400;            // generous cross-core buffer

// -------------------------------------------------------------- objects
  BluetoothSerial SerialBT;
  PulseOximeter   pox;
  SPIClass        SD_SPI(VSPI);
  QueueHandle_t   sensorQueue;

// --------------------------------------------------------- sample record
  struct SensorData {
    uint32_t scanTime;      // ms since START
    float    ax, ay, az;    // accel g    (GY-87 - 0 for now)
    float    gx, gy, gz;    // gyro dps   (GY-87 - 0 for now)
    int      vibration;     // window transition count (intensity) or 0
    float    heartRate;
    float    spo2;
  };

// ------------------------------------------------- shared / scan state
  volatile bool     scanning      = false;
  volatile uint32_t scanStartTime = 0;

// live values (written on core 0, read on core 1 for the status line)
  float heartRate = 0.0f;
  float spo2      = 0.0f;

// vibration state (core 0 only)
  int      prevVibState = LOW;
  bool     vibMeasuring = false;
  uint32_t vibStart     = 0;
  int      vibHigh      = 0;
  int      vibLow       = 0;
  int      vibIntensity = 0;      // last completed window intensity (for display/log)
  bool     vibReady     = false;  // emit intensity on the next logged row

// SD state (core 1 only)
  bool     sdReady     = false;
  bool     fileOpen    = false;
  File     scanFile;
  String   currentFile = "";
  uint32_t scanNumber  = 0;

// ------------------------------------------------------------ prototypes
  void     say(const String &m);
  void     onBeatDetected();
  void     measureVibration();
  void     sensorTask(void *param);
  void     handleCommands();
  void     readStream(Stream &s, String &buf);
  void     dispatch(String cmd);
  void     serviceSD();
  void     serialReport();
  void     cmdSdInit();
  void     cmdStart();
  void     cmdStop();
  void     cmdList();
  void     cmdRead(String arg);
  void     cmdRelease();
  void     cmdStatus();
  String   fileNameFor(uint32_t n);
  uint32_t nextScanNumber();

// =====================================================================
// helpers
// =====================================================================
    void say(const String &m) {           // send to phone + USB
      Serial.println(m);
      SerialBT.println(m);
    }

    void onBeatDetected() {               // keep tiny - runs inside pox.update()
    }

    // Vibration: SW-18010P, non-blocking 100 ms window (from 02_Sense_vibration_hardness).
    // Counts HIGH/LOW transitions; total = vibration intensity / hardness.
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
          vibIntensity = vibHigh + vibLow;                  // sensitivity value
          vibReady     = true;                              // emit on next row
        }
      } else {
        prevVibState = state;
      }
    }

// =====================================================================
// CORE 0 : sensor task - fast loop, queues samples while scanning
// =====================================================================
  void sensorTask(void *param) {
    uint32_t lastSample = millis();

    for (;;) {
      pox.update();          // MUST be called as fast as possible (MAX30100)
      measureVibration();    // non-blocking

      uint32_t now = millis();

      if (!scanning) {
        lastSample = now;    // stay current so START does not burst
      } else if (now - lastSample >= SAMPLE_INTERVAL_MS) {
        lastSample += SAMPLE_INTERVAL_MS;

        heartRate = pox.getHeartRate();
        spo2      = pox.getSpO2();

        SensorData d;
        d.scanTime = now - scanStartTime;

        // --- GY-87 (add later): readMPU6050(); d.ax=accelX; ... d.gz=gyroZ; ---
        d.ax = d.ay = d.az = 0.0f;     // GY-87 disabled
        d.gx = d.gy = d.gz = 0.0f;     // GY-87 disabled

        d.vibration = vibReady ? vibIntensity : 0;
        d.heartRate = heartRate;
        d.spo2      = spo2;

        xQueueSend(sensorQueue, &d, 0);  // non-blocking: never stall the sensor
        vibReady = false;                // intensity emitted on exactly one row
      }

      // Yield ~1 ms: keeps the Bluetooth stack + watchdog alive on core 0.
      // pox.update() still runs ~1000x/s (>= 10x the sample rate) - no data lost.
      vTaskDelay(1);
    }
  }

// =====================================================================
// SETUP  (runs on core 1)
// =====================================================================
void setup() {
  Serial.begin(921600);
  delay(100);
  Serial.println("\n=== Fetal Kick Logger (simplified dual-core) ===");

  // ---- MAX30100 (same settings as 03_Heart_Rate_Sensor/sensor_testing) ----
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

  // ---- Bluetooth ----
  if (!SerialBT.begin("ESP32_SENSOR_LOGGER"))
    Serial.println("Bluetooth FAILED");
  else
    Serial.println("Bluetooth: ESP32_SENSOR_LOGGER");

  // ---- queue + core-0 sensor task ----
  sensorQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorData));
  if (!sensorQueue) Serial.println("Queue alloc FAILED");

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, NULL, 3, NULL, 0);

  say("READY. Commands: SD_INIT | START | STOP | LIST | READ <n> | SD_RELEASE | STATUS");
}

// =====================================================================
// LOOP  (core 1) : Bluetooth commands + SD writing
// =====================================================================
void loop() {
  handleCommands();
  serviceSD();
  serialReport();
  delay(1);
}

// ------------------------------------------------- command input
    void readStream(Stream &s, String &buf) {
      while (s.available()) {
        char c = s.read();
        if (c == '\n' || c == '\r') {
          buf.trim();
          if (buf.length()) dispatch(buf);
          buf = "";
        } else if (buf.length() < 40) {
          buf += c;
        }
      }
    }

    void handleCommands() {
      static String btBuf  = "";
      static String usbBuf = "";
      readStream(SerialBT, btBuf);
      readStream(Serial,   usbBuf);
    }

    void dispatch(String cmd) {
      if      (cmd.equalsIgnoreCase("SD_INIT"))    cmdSdInit();
      else if (cmd.equalsIgnoreCase("START"))      cmdStart();
      else if (cmd.equalsIgnoreCase("STOP"))       cmdStop();
      else if (cmd.equalsIgnoreCase("LIST"))       cmdList();
      else if (cmd.equalsIgnoreCase("SD_RELEASE")) cmdRelease();
      else if (cmd.equalsIgnoreCase("STATUS"))     cmdStatus();
      else if (cmd.substring(0, 5).equalsIgnoreCase("READ ")) cmdRead(cmd.substring(5));
      else say("? unknown: " + cmd);
    }

// ------------------------------------------------- SD writing / draining
  void serviceSD() {
    static uint32_t lastFlush = 0;
    if (!fileOpen) return;

    // One formatted CSV line per sample. Building it once (not field-by-field)
    // keeps writes fast and lets the SAME line be pushed to a server later.
    SensorData d;
    char line[128];
    while (xQueueReceive(sensorQueue, &d, 0) == pdTRUE) {
      int n = snprintf(line, sizeof(line),
                      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.1f,%.1f\n",
                      (unsigned long)d.scanTime,
                      d.ax, d.ay, d.az, d.gx, d.gy, d.gz,
                      d.vibration, d.heartRate, d.spo2);
      if (n < 0) continue;
      if (n >= (int)sizeof(line)) n = sizeof(line) - 1;   // clamp (shouldn't happen)
      scanFile.write((const uint8_t *)line, n);

      // LATER (core 1 server upload): forward this same 'line' to the server,
      // e.g. buffer it and POST in batches over WiFi -> uploadToServer(line, n);
    }

    uint32_t now = millis();
    if (now - lastFlush >= FLUSH_INTERVAL_MS) {
      lastFlush = now;
      scanFile.flush();
    }

    // STOP finished: not scanning and queue drained -> close the file
    if (!scanning && uxQueueMessagesWaiting(sensorQueue) == 0) {
      scanFile.flush();
      scanFile.close();
      fileOpen = false;
      say("SCAN STOPPED. FILE: " + currentFile);
    }
  }

// ------------------------------------------- USB status line (scanning only)
  void serialReport() {
    static uint32_t last = 0;
    if (!scanning) return;                       // no serial spam when idle
    uint32_t now = millis();
    if (now - last < SERIAL_INTERVAL_MS) return;
    last = now;
    Serial.printf("t=%lus  HR=%.0f  SpO2=%.0f  vib=%d  queued=%u\n",
                  (unsigned long)((now - scanStartTime) / 1000),
                  heartRate, spo2, vibIntensity,
                  (unsigned)uxQueueMessagesWaiting(sensorQueue));
  }

// ------------------------------------------------- commands
  void cmdSdInit() {
    if (scanning) { say("busy: scanning"); return; }
    if (sdReady)  { say("SD already initialized"); return; }

    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SD_SPI, 20000000)) { say("SD INIT FAILED"); return; }
    sdReady = true;

    uint64_t total = SD.totalBytes();
    uint64_t used  = SD.usedBytes();
    say("SD OK");
    say("  total " + String((unsigned long)(total / 1024)) + " KB");
    say("  used  " + String((unsigned long)(used  / 1024)) + " KB");
    say("  free  " + String((unsigned long)((total - used) / 1024)) + " KB");
  }

  void cmdStart() {
    if (!sdReady) { say("run SD_INIT first"); return; }
    if (scanning) { say("already scanning"); return; }
    if (fileOpen) { say("previous file still closing, wait"); return; }

    if (SD.totalBytes() - SD.usedBytes() < MIN_FREE_SPACE) { say("SD FULL"); return; }

    scanNumber  = nextScanNumber();
    currentFile = fileNameFor(scanNumber);
    scanFile    = SD.open(currentFile, FILE_WRITE);
    if (!scanFile) { say("cannot create " + currentFile); return; }

    scanFile.println("scan_time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,vibration,heart_rate,spo2");
    scanFile.flush();

    fileOpen      = true;
    scanStartTime = millis();
    scanning      = true;          // set last: core 0 starts queuing now

    int hz = 1000 / SAMPLE_INTERVAL_MS;
    say("SCAN STARTED -> " + currentFile + "  (" + String(hz) + " Hz)");
  }

  void cmdStop() {
    if (!scanning) { say("no scan in progress"); return; }
    scanning = false;              // core 0 stops; serviceSD() drains + closes
    say("stopping... flushing remaining data");
  }

  void cmdList() {
    if (!sdReady) { say("run SD_INIT first"); return; }
    if (scanning) { say("stop scan first"); return; }

    File root = SD.open("/");
    if (!root) { say("cannot open root"); return; }

    int idx = 0;
    say("--- files ---");
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      String n = f.name();
      if (!f.isDirectory() && n.endsWith(".csv"))
        say(String(idx++) + ": " + n + "  (" + String((unsigned long)f.size()) + " B)");
    }
    if (idx == 0) say("(none)");
    root.close();
  }

  void cmdRead(String arg) {
    if (!sdReady) { say("run SD_INIT first"); return; }
    if (scanning) { say("stop scan first"); return; }
    int target = arg.toInt();

    File root = SD.open("/");
    if (!root) { say("cannot open root"); return; }

    int  idx  = 0;
    bool sent = false;
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      String n = f.name();
      if (!f.isDirectory() && n.endsWith(".csv")) {
        if (idx == target) {
          say("--- " + n + " ---");
          uint8_t buf[64];
          while (f.available()) {
            int k = f.read(buf, sizeof(buf));
            SerialBT.write(buf, k);
          }
          SerialBT.println();
          say("--- end ---");
          sent = true;
          break;
        }
        idx++;
      }
    }
    root.close();
    if (!sent) say("no file at index " + String(target));
  }

  void cmdRelease() {
    if (scanning) { say("stop scan first"); return; }
    if (fileOpen) { say("still writing, try again in a moment"); return; }
    if (!sdReady) { say("SD not initialized"); return; }
    SD.end();
    sdReady = false;
    say("SD RELEASED - safe to remove card");
  }

  void cmdStatus() {
    say("--- status ---");
    say(String("sd_ready : ") + (sdReady  ? "yes" : "no"));
    say(String("scanning : ") + (scanning ? "yes" : "no"));
    say(String("file     : ") + (fileOpen ? currentFile : String("-")));
    say(String("queued   : ") + String((unsigned)uxQueueMessagesWaiting(sensorQueue)));
    if (sdReady)
      say(String("free     : ") +
          String((unsigned long)((SD.totalBytes() - SD.usedBytes()) / 1024)) + " KB");
  }

// ------------------------------------------------- filename helpers
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
