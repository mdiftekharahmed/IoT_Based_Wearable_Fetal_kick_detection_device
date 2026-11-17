#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <SD.h>
#include "MAX30100_PulseOximeter.h"

// ============================================================
// I2C PINS
// ============================================================

  #define GY_SDA 21
  #define GY_SCL 22

  #define MAX_SDA 25
  #define MAX_SCL 26

// ============================================================
// VIBRATION
// ============================================================

#define VIBRATION_PIN 27

// ============================================================
// SD CARD
// Standard VSPI pins on ESP32
// ============================================================

  #define SD_CS   19
  #define SD_SCK  21
  #define SD_MISO 5
  #define SD_MOSI 18

// ============================================================
// WIFI
// ============================================================

  const char* WIFI_SSID = "YOUR_WIFI";
  const char* WIFI_PASSWORD = "YOUR_PASSWORD";

  // Change this to your actual server endpoint
  const char* SERVER_URL = "http://YOUR_SERVER_IP/api/sensor";

// ============================================================
// TIMING
// ============================================================

  // MPU6050 = 100 Hz
  const uint32_t MPU_INTERVAL_MS = 10;

  // BMP180 = 10 Hz
  const uint32_t BMP_INTERVAL_MS = 100;

  // Heart-rate reporting
  const uint32_t HEART_REPORT_INTERVAL_MS = 1000;

  // Serial output
  const uint32_t SERIAL_INTERVAL_MS = 1000;

  // SD write interval
  const uint32_t SD_WRITE_INTERVAL_MS = 1000;

  // WiFi upload attempt
  const uint32_t WIFI_INTERVAL_MS = 5000;

// ============================================================
// MAX30100
// ============================================================

  PulseOximeter pox;

  float heartRate = 0.0;
  float spo2 = 0.0;

  uint32_t lastHeartReport = 0;

// ============================================================
// GY-87 I2C
// ============================================================

TwoWire GYWire = TwoWire(1);

// ============================================================
// MPU6050 DATA
// ============================================================

  int16_t rawAX = 0;
  int16_t rawAY = 0;
  int16_t rawAZ = 0;

  int16_t rawGX = 0;
  int16_t rawGY = 0;
  int16_t rawGZ = 0;

  int16_t rawMPUTemp = 0;

  float accelX = 0;
  float accelY = 0;
  float accelZ = 0;

  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;

  float mpuTemperature = 0;

// ============================================================
// BMP180 CALIBRATION
// ============================================================

  int16_t AC1;
  int16_t AC2;
  int16_t AC3;

  uint16_t AC4;
  uint16_t AC5;
  uint16_t AC6;

  int16_t BMP_B1;
  int16_t B2;
  int16_t MB;
  int16_t MC;
  int16_t MD;

// ============================================================
// BMP180 DATA
// ============================================================

  float bmpTemperature = 0;
  float bmpPressure = 0;
  float bmpAltitude = 0;

// ============================================================
// BMP180 STATE MACHINE
// ============================================================

  enum BMPState {
      BMP_IDLE,
      BMP_TEMP_WAIT,
      BMP_PRESSURE_WAIT
  };

  BMPState bmpState = BMP_IDLE;

  uint32_t bmpConversionStart = 0;

  int32_t bmpUT = 0;
  int32_t bmpUP = 0;

// ============================================================
// VIBRATION
// ============================================================

  int prevVibrationState = LOW;

  bool vibrationMeasuring = false;

  uint32_t vibrationStartTime = 0;

  const uint32_t VIBRATION_WINDOW_MS = 100;

  int vibrationOnes = 0;
  int vibrationZeros = 0;

  volatile bool vibrationEvent = false;

// ============================================================
// TIMERS
// ============================================================

  uint32_t lastMPU = 0;
  uint32_t lastBMP = 0;
  uint32_t lastSerial = 0;
  uint32_t lastSDWrite = 0;
  uint32_t lastWiFiAttempt = 0;

// ============================================================
// SD BUFFER
// ============================================================

#define SD_BUFFER_SIZE 32

struct SensorRecord {

    uint32_t timestamp;

    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t mpuTemp;

    int16_t bmpTemp;
    int32_t pressure;

    int16_t heartRate;
    int16_t spo2;

    int16_t vibrationOnes;
    int16_t vibrationZeros;
};

SensorRecord buffer[SD_BUFFER_SIZE];

uint16_t bufferIndex = 0;

// ============================================================
// FILE
// ============================================================

const char* DATA_FILE = "/sensor_data.csv";

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

  void initMPU6050();
  void readMPU6050();

  void initBMP180();
  void startBMPTemperature();
  void processBMP180();

  void measureVibration();

  void updateHeartRate();

  void addRecordToBuffer();
  void writeBufferToSD();

  void printSensorData();

  void initSDCard();

  void connectWiFi();
  void uploadData();

  void onBeatDetected();

// ============================================================
// HEARTBEAT CALLBACK
// ============================================================

void onBeatDetected()
{
    // DON'T PRINT HERE.
    // Serial output inside the callback can slow MAX30100 processing.
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(921600);

    delay(100);

    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32 SENSOR SYSTEM");
    Serial.println("Optimized Data Logger");
    Serial.println("========================================");

    // ========================================================
    // MAX30100 BUS
    // ========================================================

    Wire.begin(MAX_SDA, MAX_SCL);
    Wire.setClock(400000);

    Serial.println("Initializing MAX30100...");

    if (!pox.begin())
    {
        Serial.println("MAX30100 FAILED");
    }
    else
    {
        Serial.println("MAX30100 OK");

        pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
        pox.setOnBeatDetectedCallback(onBeatDetected);
    }

    // ========================================================
    // GY-87 BUS
    // ========================================================

    GYWire.begin(GY_SDA, GY_SCL);
    GYWire.setClock(400000);

    Serial.println("GY-87 I2C initialized at 400 kHz");

    // ========================================================
    // MPU6050
    // ========================================================

    initMPU6050();

    // ========================================================
    // BMP180
    // ========================================================

    Serial.println("Initializing BMP180...");

    GYWire.beginTransmission(0x77);

    if (GYWire.endTransmission() == 0)
    {
        Serial.println("BMP180 detected");

        initBMP180();
    }
    else
    {
        Serial.println("BMP180 NOT FOUND");
    }

    // ========================================================
    // VIBRATION
    // ========================================================

    pinMode(VIBRATION_PIN, INPUT);

    prevVibrationState = digitalRead(VIBRATION_PIN);

    Serial.println("Vibration sensor ready");

    // ========================================================
    // SD
    // ========================================================

    initSDCard();

    // ========================================================
    // WIFI
    // ========================================================

    WiFi.mode(WIFI_STA);

    connectWiFi();

    // ========================================================
    // TIMERS
    // ========================================================

    uint32_t now = millis();

    lastMPU = now;
    lastBMP = now;
    lastSerial = now;
    lastSDWrite = now;
    lastWiFiAttempt = now;

    Serial.println();
    Serial.println("SYSTEM READY");
    Serial.println("========================================");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    uint32_t now = millis();

    // ========================================================
    // MAX30100
    //
    // This must run as frequently as possible.
    // ========================================================

    pox.update();

    // ========================================================
    // VIBRATION
    // ========================================================

    measureVibration();

    // ========================================================
    // MPU6050
    //
    // 100 Hz
    // ========================================================

    if (now - lastMPU >= MPU_INTERVAL_MS)
    {
        lastMPU += MPU_INTERVAL_MS;

        readMPU6050();
    }

    // ========================================================
    // BMP180
    //
    // Non-blocking state machine
    // ========================================================

    processBMP180();

    if (now - lastBMP >= BMP_INTERVAL_MS)
    {
        lastBMP += BMP_INTERVAL_MS;

        if (bmpState == BMP_IDLE)
        {
            startBMPTemperature();
        }
    }

    // ========================================================
    // HEART RATE REPORTING
    // ========================================================

    updateHeartRate();

    // ========================================================
    // ADD DATA TO RAM BUFFER
    //
    // We don't write to SD every sensor read.
    // ========================================================

    static uint32_t lastRecord = 0;

    if (now - lastRecord >= 100)
    {
        lastRecord += 100;

        addRecordToBuffer();
    }

    // ========================================================
    // SD WRITE
    // ========================================================

    if (now - lastSDWrite >= SD_WRITE_INTERVAL_MS)
    {
        lastSDWrite = now;

        writeBufferToSD();
    }

    // ========================================================
    // SERIAL OUTPUT
    // ========================================================

    if (now - lastSerial >= SERIAL_INTERVAL_MS)
    {
        lastSerial = now;

        printSensorData();
    }

    // ========================================================
    // WIFI
    // ========================================================

    if (WiFi.status() != WL_CONNECTED)
    {
        if (now - lastWiFiAttempt >= WIFI_INTERVAL_MS)
        {
            lastWiFiAttempt = now;

            connectWiFi();
        }
    }
    else
    {
        if (now - lastWiFiAttempt >= WIFI_INTERVAL_MS)
        {
            lastWiFiAttempt = now;

            uploadData();
        }
    }
}

// ============================================================
// MPU6050 INITIALIZATION
// ============================================================

void initMPU6050()
{
    Serial.print("Initializing MPU6050... ");

    GYWire.beginTransmission(0x68);

    if (GYWire.endTransmission() != 0)
    {
        Serial.println("FAILED");
        return;
    }

    Serial.println("OK");

    // --------------------------------------------------------
    // Wake MPU6050
    // --------------------------------------------------------

    GYWire.beginTransmission(0x68);

    GYWire.write(0x6B);
    GYWire.write(0x00);

    GYWire.endTransmission();

    // --------------------------------------------------------
    // Accelerometer ±8G
    // --------------------------------------------------------

    GYWire.beginTransmission(0x68);

    GYWire.write(0x1C);
    GYWire.write(0x10);

    GYWire.endTransmission();

    // --------------------------------------------------------
    // Gyroscope ±500 deg/s
    // --------------------------------------------------------

    GYWire.beginTransmission(0x68);

    GYWire.write(0x1B);
    GYWire.write(0x08);

    GYWire.endTransmission();

    // --------------------------------------------------------
    // Digital Low Pass Filter
    // --------------------------------------------------------

    GYWire.beginTransmission(0x68);

    GYWire.write(0x1A);
    GYWire.write(0x03);

    GYWire.endTransmission();

    Serial.println("MPU6050 configured");
}

// ============================================================
// MPU6050 READ
// ============================================================

void readMPU6050()
{
    GYWire.beginTransmission(0x68);

    GYWire.write(0x3B);

    if (GYWire.endTransmission(false) != 0)
        return;

    uint8_t received = GYWire.requestFrom(0x68, 14);

    if (received != 14)
        return;

    rawAX = ((int16_t)GYWire.read() << 8) | GYWire.read();
    rawAY = ((int16_t)GYWire.read() << 8) | GYWire.read();
    rawAZ = ((int16_t)GYWire.read() << 8) | GYWire.read();

    rawMPUTemp = ((int16_t)GYWire.read() << 8) | GYWire.read();

    rawGX = ((int16_t)GYWire.read() << 8) | GYWire.read();
    rawGY = ((int16_t)GYWire.read() << 8) | GYWire.read();
    rawGZ = ((int16_t)GYWire.read() << 8) | GYWire.read();

    // ±8G
    accelX = rawAX / 4096.0f;
    accelY = rawAY / 4096.0f;
    accelZ = rawAZ / 4096.0f;

    // ±500 deg/s
    gyroX = rawGX / 65.5f;
    gyroY = rawGY / 65.5f;
    gyroZ = rawGZ / 65.5f;

    mpuTemperature = (rawMPUTemp / 340.0f) + 36.53f;
}

// ============================================================
// BMP180 CALIBRATION
// ============================================================

void initBMP180()
{
    GYWire.beginTransmission(0x77);

    GYWire.write(0xAA);

    if (GYWire.endTransmission() != 0)
        return;

    if (GYWire.requestFrom(0x77, 22) != 22)
        return;

    AC1 = ((int16_t)GYWire.read() << 8) | GYWire.read();
    AC2 = ((int16_t)GYWire.read() << 8) | GYWire.read();
    AC3 = ((int16_t)GYWire.read() << 8) | GYWire.read();

    AC4 = ((uint16_t)GYWire.read() << 8) | GYWire.read();
    AC5 = ((uint16_t)GYWire.read() << 8) | GYWire.read();
    AC6 = ((uint16_t)GYWire.read() << 8) | GYWire.read();

    BMP_B1 = ((int16_t)GYWire.read() << 8) | GYWire.read();
    B2 = ((int16_t)GYWire.read() << 8) | GYWire.read();

    MB = ((int16_t)GYWire.read() << 8) | GYWire.read();
    MC = ((int16_t)GYWire.read() << 8) | GYWire.read();
    MD = ((int16_t)GYWire.read() << 8) | GYWire.read();

    Serial.println("BMP180 calibration loaded");
}

// ============================================================
// START BMP TEMPERATURE
// ============================================================

void startBMPTemperature()
{
    GYWire.beginTransmission(0x77);

    GYWire.write(0xF4);
    GYWire.write(0x2E);

    if (GYWire.endTransmission() != 0)
        return;

    bmpConversionStart = millis();

    bmpState = BMP_TEMP_WAIT;
}

// ============================================================
// PROCESS BMP180
// ============================================================

void processBMP180()
{
    uint32_t now = millis();

    // ========================================================
    // TEMPERATURE CONVERSION
    // ========================================================

    if (bmpState == BMP_TEMP_WAIT)
    {
        if (now - bmpConversionStart < 5)
            return;

        GYWire.beginTransmission(0x77);

        GYWire.write(0xF6);

        if (GYWire.endTransmission(false) != 0)
        {
            bmpState = BMP_IDLE;
            return;
        }

        if (GYWire.requestFrom(0x77, 2) != 2)
        {
            bmpState = BMP_IDLE;
            return;
        }

        bmpUT = ((int32_t)GYWire.read() << 8) | GYWire.read();

        // ----------------------------------------------------
        // Temperature calculation
        // ----------------------------------------------------

        int32_t X1 =
            ((bmpUT - (int32_t)AC6) * (int32_t)AC5) >> 15;

        int32_t X2 =
            ((int32_t)MC << 11) / (X1 + MD);

        int32_t B5 =
            X1 + X2;

        bmpTemperature =
            ((B5 + 8) >> 4) / 10.0f;

        // ----------------------------------------------------
        // Start pressure conversion
        // OSS = 0
        // ----------------------------------------------------

        GYWire.beginTransmission(0x77);

        GYWire.write(0xF4);
        GYWire.write(0x34);

        if (GYWire.endTransmission() != 0)
        {
            bmpState = BMP_IDLE;
            return;
        }

        bmpConversionStart = now;

        bmpState = BMP_PRESSURE_WAIT;
    }

    // ========================================================
    // PRESSURE CONVERSION
    // ========================================================

    else if (bmpState == BMP_PRESSURE_WAIT)
    {
        if (now - bmpConversionStart < 5)
            return;

        GYWire.beginTransmission(0x77);

        GYWire.write(0xF6);

        if (GYWire.endTransmission(false) != 0)
        {
            bmpState = BMP_IDLE;
            return;
        }

        if (GYWire.requestFrom(0x77, 3) != 3)
        {
            bmpState = BMP_IDLE;
            return;
        }

        int32_t MSB = GYWire.read();
        int32_t LSB = GYWire.read();
        int32_t XLSB = GYWire.read();

        bmpUP =
            ((MSB << 16) |
             (LSB << 8) |
             XLSB) >> 8;

        // ----------------------------------------------------
        // Pressure calculation
        // ----------------------------------------------------

        int32_t X1 =
            ((int32_t)((bmpTemperature * 10.0f)));

        // Recalculate B5 from raw temperature
        int32_t UT = bmpUT;

        X1 =
            ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;

        int32_t X2 =
            ((int32_t)MC << 11) /
            (X1 + MD);

        int32_t B5 =
            X1 + X2;

        X1 =
            ((B5 - 4000) * (int32_t)AC2) >> 11;

        X2 =
            ((int32_t)BMP_B1 *
             ((X1 * X1) >> 12)) >> 16;

        int32_t X3 =
            X1 + X2;

        int32_t B3 =
            ((((int32_t)AC1 * 4 + X3) + 2) >> 2);

        X1 =
            ((int32_t)AC3 * B5) >> 13;

        X2 =
            ((int32_t)B2 *
             ((X1 * X1) >> 12)) >> 16;

        X3 =
            ((X1 + X2) + 2) >> 2;

        uint32_t B4 =
            ((uint32_t)AC4 *
             (uint32_t)(X3 + 32768)) >> 15;

        uint32_t B7 =
            ((uint32_t)bmpUP - B3) * 50000;

        int32_t pressure;

        if (B7 < 0x80000000)
            pressure = (B7 * 2) / B4;
        else
            pressure = (B7 / B4) * 2;

        X1 =
            (pressure >> 8) *
            (pressure >> 8);

        X1 =
            (X1 * 3038) >> 16;

        X2 =
            (-7357 * pressure) >> 16;

        pressure =
            pressure +
            ((X1 + X2 + 3791) >> 4);

        bmpPressure = pressure;

        // ----------------------------------------------------
        // Altitude
        // ----------------------------------------------------

        bmpAltitude =
            44330.0f *
            (1.0f -
             powf(
                 bmpPressure / 101325.0f,
                 0.1903f
             ));

        bmpState = BMP_IDLE;
    }
}

// ============================================================
// VIBRATION
// ============================================================

void measureVibration()
{
    int state = digitalRead(VIBRATION_PIN);

    if (!vibrationMeasuring &&
        state != prevVibrationState)
    {
        vibrationMeasuring = true;

        vibrationOnes = 0;
        vibrationZeros = 0;

        vibrationStartTime = millis();
    }

    if (vibrationMeasuring)
    {
        if (state != prevVibrationState)
        {
            if (state == HIGH)
                vibrationOnes++;
            else
                vibrationZeros++;

            prevVibrationState = state;
        }

        if (millis() - vibrationStartTime >=
            VIBRATION_WINDOW_MS)
        {
            vibrationMeasuring = false;

            vibrationEvent = true;
        }
    }
    else
    {
        prevVibrationState = state;
    }
}

// ============================================================
// HEART RATE
// ============================================================

void updateHeartRate()
{
    uint32_t now = millis();

    if (now - lastHeartReport >= HEART_REPORT_INTERVAL_MS)
    {
        lastHeartReport = now;

        heartRate = pox.getHeartRate();
        spo2 = pox.getSpO2();
    }
}

// ============================================================
// CREATE SENSOR RECORD
// ============================================================

void addRecordToBuffer()
{
    if (bufferIndex >= SD_BUFFER_SIZE)
        return;

    SensorRecord &r = buffer[bufferIndex];

    r.timestamp = millis();

    r.ax = rawAX;
    r.ay = rawAY;
    r.az = rawAZ;

    r.gx = rawGX;
    r.gy = rawGY;
    r.gz = rawGZ;

    r.mpuTemp =
        (int16_t)(mpuTemperature * 100);

    r.bmpTemp =
        (int16_t)(bmpTemperature * 100);

    r.pressure =
        (int32_t)bmpPressure;

    r.heartRate =
        (int16_t)(heartRate * 10);

    r.spo2 =
        (int16_t)(spo2 * 10);

    r.vibrationOnes =
        vibrationOnes;

    r.vibrationZeros =
        vibrationZeros;

    bufferIndex++;
}

// ============================================================
// SD INITIALIZATION
// ============================================================

void initSDCard()
{
    Serial.println("Initializing SD card...");

    SPI.begin(
        SD_SCK,
        SD_MISO,
        SD_MOSI,
        SD_CS
    );

    if (!SD.begin(SD_CS, SPI, 20000000))
    {
        Serial.println("SD CARD FAILED");
        return;
    }

    Serial.println("SD CARD OK");

    // --------------------------------------------------------
    // Create CSV header if file doesn't exist
    // --------------------------------------------------------

    if (!SD.exists(DATA_FILE))
    {
        File file = SD.open(
            DATA_FILE,
            FILE_WRITE
        );

        if (file)
        {
            file.println(
                "timestamp,"
                "ax,ay,az,"
                "gx,gy,gz,"
                "mpuTemp,"
                "bmpTemp,"
                "pressure,"
                "heartRate,"
                "spo2,"
                "vibrationOnes,"
                "vibrationZeros"
            );

            file.close();
        }
    }
}

// ============================================================
// WRITE RAM BUFFER TO SD
// ============================================================

void writeBufferToSD()
{
    if (bufferIndex == 0)
        return;

    File file = SD.open(
        DATA_FILE,
        FILE_APPEND
    );

    if (!file)
    {
        Serial.println("SD WRITE OPEN FAILED");
        return;
    }

    for (uint16_t i = 0;
         i < bufferIndex;
         i++)
    {
        SensorRecord &r = buffer[i];

        file.print(r.timestamp);
        file.print(',');

        file.print(r.ax);
        file.print(',');
        file.print(r.ay);
        file.print(',');
        file.print(r.az);
        file.print(',');

        file.print(r.gx);
        file.print(',');
        file.print(r.gy);
        file.print(',');
        file.print(r.gz);
        file.print(',');

        file.print(r.mpuTemp);
        file.print(',');

        file.print(r.bmpTemp);
        file.print(',');

        file.print(r.pressure);
        file.print(',');

        file.print(r.heartRate);
        file.print(',');

        file.print(r.spo2);
        file.print(',');

        file.print(r.vibrationOnes);
        file.print(',');

        file.println(r.vibrationZeros);
    }

    file.close();

    bufferIndex = 0;
}

// ============================================================
// SERIAL OUTPUT
// ============================================================

void printSensorData()
{
    Serial.println();
    Serial.println("================================================");

    Serial.print("ACCEL: ");
    Serial.print(accelX);
    Serial.print(", ");
    Serial.print(accelY);
    Serial.print(", ");
    Serial.println(accelZ);

    Serial.print("GYRO: ");
    Serial.print(gyroX);
    Serial.print(", ");
    Serial.print(gyroY);
    Serial.print(", ");
    Serial.println(gyroZ);

    Serial.print("MPU TEMP: ");
    Serial.print(mpuTemperature);
    Serial.println(" C");

    Serial.print("BMP TEMP: ");
    Serial.print(bmpTemperature);
    Serial.println(" C");

    Serial.print("PRESSURE: ");
    Serial.print(bmpPressure);
    Serial.println(" Pa");

    Serial.print("ALTITUDE: ");
    Serial.print(bmpAltitude);
    Serial.println(" m");

    Serial.print("HEART RATE: ");
    Serial.print(heartRate);
    Serial.println(" BPM");

    Serial.print("SPO2: ");
    Serial.print(spo2);
    Serial.println(" %");

    Serial.print("VIBRATION: ");
    Serial.print(vibrationOnes);
    Serial.print(":");
    Serial.println(vibrationZeros);

    Serial.print("RAM BUFFER: ");
    Serial.print(bufferIndex);
    Serial.print("/");
    Serial.println(SD_BUFFER_SIZE);

    Serial.print("WIFI: ");

    if (WiFi.status() == WL_CONNECTED)
        Serial.println("CONNECTED");
    else
        Serial.println("DISCONNECTED");

    Serial.println("================================================");
}

// ============================================================
// WIFI CONNECTION
// ============================================================

void connectWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return;

    Serial.println("Connecting to WiFi...");

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    // IMPORTANT:
    // Don't wait here.
    // The main sensor loop continues running.
}

// ============================================================
// WIFI UPLOAD
// ============================================================

void uploadData()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    // --------------------------------------------------------
    // IMPORTANT
    //
    // This is only a placeholder.
    // Your actual server API needs to be implemented here.
    //
    // The SD file remains the source of truth.
    // --------------------------------------------------------

    Serial.println("WiFi available - upload task");

    /*
    HTTPClient http;

    http.begin(SERVER_URL);

    http.addHeader(
        "Content-Type",
        "application/json"
    );

    String json = "{";
    json += "\"timestamp\":" + String(millis()) + ",";
    json += "\"ax\":" + String(accelX, 4) + ",";
    json += "\"ay\":" + String(accelY, 4) + ",";
    json += "\"az\":" + String(accelZ, 4) + ",";
    json += "\"gx\":" + String(gyroX, 4) + ",";
    json += "\"gy\":" + String(gyroY, 4) + ",";
    json += "\"gz\":" + String(gyroZ, 4) + ",";
    json += "\"temperature\":" + String(mpuTemperature, 2) + ",";
    json += "\"pressure\":" + String(bmpPressure, 2) + ",";
    json += "\"heartRate\":" + String(heartRate, 2) + ",";
    json += "\"spo2\":" + String(spo2, 2);
    json += "}";

    int response =
        http.POST(json);

    Serial.print("HTTP response: ");
    Serial.println(response);

    http.end();
    */
}