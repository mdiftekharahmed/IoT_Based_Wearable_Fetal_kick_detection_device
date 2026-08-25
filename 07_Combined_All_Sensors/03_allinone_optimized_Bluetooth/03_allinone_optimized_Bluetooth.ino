#include <Wire.h>
#include <math.h>
#include "BluetoothSerial.h"
#include "MAX30100_PulseOximeter.h"

// ============================================================
// BLUETOOTH
// ============================================================

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth SPP is not available on this chip or is not enabled.
#endif

BluetoothSerial SerialBT;


// ============================================================
// I2C PINS
// ============================================================

// GY-87
#define GY_SDA 21
#define GY_SCL 22

// MAX30100
#define MAX_SDA 25
#define MAX_SCL 26


// ============================================================
// VIBRATION
// ============================================================

#define VIBRATION_PIN 27


// ============================================================
// TIMING
// ============================================================

// MPU6050 sampling
// 100 Hz
const uint32_t MPU_INTERVAL_MS = 10;

// BMP180 sampling
// 10 Hz
const uint32_t BMP_INTERVAL_MS = 100;

// Heart-rate value update
const uint32_t HEART_REPORT_INTERVAL_MS = 1000;

// Bluetooth sensor data
// Every 5 seconds
const uint32_t BT_REPORT_INTERVAL_MS = 5000;

// Serial Monitor report
const uint32_t SERIAL_INTERVAL_MS = 1000;

// Measurement interval
const uint32_t MEASUREMENT_INTERVAL_MS = 1000;


// ============================================================
// DEVICE MODES
// ============================================================

enum DeviceMode
{
    MODE_AUTO,
    MODE_MANUAL
};

DeviceMode currentMode = MODE_MANUAL;


// ============================================================
// SYSTEM STATE
// ============================================================

bool sampling = false;

// Whether measurements should be uploaded to server.
// Actual WiFi/GSM upload code will be connected later.
bool serverUpload = false;


// ============================================================
// MEASUREMENT TIMERS
// ============================================================

uint32_t lastMPU = 0;
uint32_t lastBMP = 0;
uint32_t lastSerial = 0;
uint32_t lastBTReport = 0;
uint32_t lastHeartReport = 0;
uint32_t lastMeasurement = 0;


// ============================================================
// BLUETOOTH COMMAND
// ============================================================

String bluetoothCommand = "";


// ============================================================
// SENSOR STATUS
// ============================================================

bool mpuOK = false;
bool bmpOK = false;
bool maxOK = false;
bool vibrationOK = false;


// ============================================================
// MAX30100
// ============================================================

PulseOximeter pox;

float heartRate = 0.0;
float spo2 = 0.0;


// ============================================================
// GY-87 I2C BUS
// ============================================================

TwoWire GYWire = TwoWire(1);


// ============================================================
// MPU6050 RAW DATA
// ============================================================

int16_t rawAX = 0;
int16_t rawAY = 0;
int16_t rawAZ = 0;

int16_t rawGX = 0;
int16_t rawGY = 0;
int16_t rawGZ = 0;

int16_t rawMPUTemp = 0;


// ============================================================
// MPU6050 CALCULATED DATA
// ============================================================

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

enum BMPState
{
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

bool vibrationEvent = false;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

// Sensor initialization
void initMPU6050();
void initBMP180();

// Sensor reading
void readMPU6050();
void startBMPTemperature();
void processBMP180();
void measureVibration();
void updateHeartRate();

// Measurement system
void takeMeasurement();
void uploadData();

// Bluetooth
void processBluetooth();
void processCommand(String command);
void sendBluetoothStatus();
void sendBluetoothSensorData();
void sendBluetoothSensorStatus();

// Modes
void setAutoMode();
void setManualMode();

void startSampling();
void stopSampling();

// Serial
void printSensorData();

void onBeatDetected();


// ============================================================
// MAX30100 CALLBACK
// ============================================================

void onBeatDetected()
{
    // Keep callback extremely fast.
    // Do not print anything here.
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
    Serial.println("Bluetooth Control Enabled");
    Serial.println("========================================");


    // ========================================================
    // BLUETOOTH
    // ========================================================

    SerialBT.begin("ESP32-BME");

    Serial.println();
    Serial.println("Bluetooth started");
    Serial.println("Device name: ESP32-BME");


    // ========================================================
    // MAX30100
    // ========================================================

    Serial.println();
    Serial.println("Initializing MAX30100...");

    Wire.begin(MAX_SDA, MAX_SCL);

    // Fast I2C
    Wire.setClock(400000);

    if (!pox.begin())
    {
        Serial.println("MAX30100 FAILED");

        maxOK = false;
    }
    else
    {
        Serial.println("MAX30100 OK");

        maxOK = true;

        pox.setIRLedCurrent(
            MAX30100_LED_CURR_7_6MA
        );

        pox.setOnBeatDetectedCallback(
            onBeatDetected
        );
    }


    // ========================================================
    // GY-87
    // ========================================================

    Serial.println();
    Serial.println("Initializing GY-87...");

    GYWire.begin(GY_SDA, GY_SCL);

    GYWire.setClock(400000);

    Serial.println("GY-87 I2C initialized");


    // ========================================================
    // MPU6050
    // ========================================================

    initMPU6050();


    // ========================================================
    // BMP180
    // ========================================================

    Serial.println();
    Serial.println("Initializing BMP180...");

    GYWire.beginTransmission(0x77);

    if (GYWire.endTransmission() == 0)
    {
        Serial.println("BMP180 detected");

        initBMP180();

        bmpOK = true;
    }
    else
    {
        Serial.println("BMP180 NOT FOUND");

        bmpOK = false;
    }


    // ========================================================
    // VIBRATION
    // ========================================================

    pinMode(VIBRATION_PIN, INPUT);

    prevVibrationState =
        digitalRead(VIBRATION_PIN);

    vibrationOK = true;

    Serial.println();
    Serial.println("Vibration sensor ready");


    // ========================================================
    // TIMERS
    // ========================================================

    uint32_t now = millis();

    lastMPU = now;
    lastBMP = now;
    lastSerial = now;
    lastBTReport = now;
    lastHeartReport = now;
    lastMeasurement = now;


    // ========================================================
    // DEFAULT MODE
    // ========================================================

    setManualMode();


    // ========================================================
    // SEND SENSOR STATUS ONCE
    // ========================================================

    sendBluetoothSensorStatus();


    Serial.println();
    Serial.println("========================================");
    Serial.println("SYSTEM READY");
    Serial.println("========================================");

    Serial.println();
    Serial.println("Bluetooth commands:");
    Serial.println("MODE_AUTO");
    Serial.println("MODE_MANUAL");
    Serial.println("START");
    Serial.println("STOP");
    Serial.println("UPLOAD");
    Serial.println("STATUS");
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
    // IMPORTANT:
    //
    // pox.update() MUST run continuously.
    //
    // It is intentionally outside the sampling condition.
    // ========================================================

    if (maxOK)
    {
        pox.update();
    }


    // ========================================================
    // BLUETOOTH COMMANDS
    // ========================================================

    processBluetooth();


    // ========================================================
    // SENSOR PROCESSING
    //
    // Other sensors only operate while sampling is ON.
    // ========================================================

    if (sampling)
    {
        // ----------------------------------------------------
        // VIBRATION
        // ----------------------------------------------------

        measureVibration();


        // ----------------------------------------------------
        // MPU6050
        //
        // 100 Hz
        // ----------------------------------------------------

        if (now - lastMPU >= MPU_INTERVAL_MS)
        {
            lastMPU += MPU_INTERVAL_MS;

            if (mpuOK)
            {
                readMPU6050();
            }
        }


        // ----------------------------------------------------
        // BMP180
        // ----------------------------------------------------

        processBMP180();

        if (now - lastBMP >= BMP_INTERVAL_MS)
        {
            lastBMP += BMP_INTERVAL_MS;

            if (bmpState == BMP_IDLE && bmpOK)
            {
                startBMPTemperature();
            }
        }


        // ----------------------------------------------------
        // HEART RATE / SpO2
        //
        // pox.update() runs continuously above.
        // Here we only read the latest calculated values.
        // ----------------------------------------------------

        if (maxOK &&
            now - lastHeartReport >= HEART_REPORT_INTERVAL_MS)
        {
            lastHeartReport = now;

            updateHeartRate();
        }


        // ----------------------------------------------------
        // MEASUREMENT
        // ----------------------------------------------------

        if (now - lastMeasurement >=
            MEASUREMENT_INTERVAL_MS)
        {
            lastMeasurement = now;

            takeMeasurement();
        }
    }


    // ========================================================
    // BLUETOOTH SENSOR REPORT
    //
    // Every 5 seconds.
    //
    // Only when sampling is ON.
    // ========================================================

    if (sampling &&
        now - lastBTReport >= BT_REPORT_INTERVAL_MS)
    {
        lastBTReport = now;

        sendBluetoothSensorData();
    }


    // ========================================================
    // SERIAL MONITOR
    // ========================================================

    if (now - lastSerial >= SERIAL_INTERVAL_MS)
    {
        lastSerial += SERIAL_INTERVAL_MS;

        printSensorData();
    }


    // ========================================================
    // NO delay()
    //
    // MAX30100 needs the loop to run as quickly as possible.
    // ========================================================
}


// ============================================================
// BLUETOOTH PROCESSING
// ============================================================

void processBluetooth()
{
    if (!SerialBT.available())
        return;


    bluetoothCommand =
        SerialBT.readStringUntil('\n');

    bluetoothCommand.trim();


    if (bluetoothCommand.length() == 0)
        return;


    Serial.print("BT COMMAND: ");
    Serial.println(bluetoothCommand);


    processCommand(bluetoothCommand);
}


// ============================================================
// COMMAND PROCESSOR
// ============================================================

void processCommand(String command)
{
    command.trim();


    // ========================================================
    // AUTO MODE
    // ========================================================

    if (command == "MODE_AUTO")
    {
        setAutoMode();
    }


    // ========================================================
    // MANUAL MODE
    // ========================================================

    else if (command == "MODE_MANUAL")
    {
        setManualMode();
    }


    // ========================================================
    // START
    // ========================================================

    else if (command == "START")
    {
        startSampling();
    }


    // ========================================================
    // STOP
    // ========================================================

    else if (command == "STOP")
    {
        stopSampling();
    }


    // ========================================================
    // UPLOAD
    // ========================================================

    else if (command == "UPLOAD")
    {
        if (currentMode == MODE_MANUAL)
        {
            uploadData();

            SerialBT.println("UPLOAD_DONE");
        }
        else
        {
            SerialBT.println("UPLOAD_AUTO_MODE");
        }
    }


    // ========================================================
    // STATUS
    // ========================================================

    else if (command == "STATUS")
    {
        sendBluetoothStatus();
    }


    // ========================================================
    // UNKNOWN COMMAND
    // ========================================================

    else
    {
        SerialBT.println("ERROR_UNKNOWN_COMMAND");
    }
}


// ============================================================
// AUTO MODE
// ============================================================

void setAutoMode()
{
    currentMode = MODE_AUTO;

    // Auto mode always samples.
    sampling = true;

    // Auto mode always uploads.
    serverUpload = true;

    lastMeasurement = millis();

    Serial.println();
    Serial.println("MODE_AUTO");

    SerialBT.println("MODE_AUTO");
}


// ============================================================
// MANUAL MODE
// ============================================================

void setManualMode()
{
    currentMode = MODE_MANUAL;

    // Manual mode starts stopped.
    sampling = false;

    // Manual mode never automatically uploads.
    serverUpload = false;

    Serial.println();
    Serial.println("MODE_MANUAL");

    SerialBT.println("MODE_MANUAL");
}


// ============================================================
// START SAMPLING
// ============================================================

void startSampling()
{
    // Auto mode cannot be manually controlled.
    if (currentMode == MODE_AUTO)
    {
        sampling = true;
        serverUpload = true;

        SerialBT.println("MODE_AUTO");
        return;
    }


    // Manual mode
    sampling = true;

    // Manual mode does NOT automatically upload.
    serverUpload = false;

    lastMeasurement = millis();

    Serial.println("SAMPLING_ON");

    SerialBT.println("SAMPLING_ON");
}


// ============================================================
// STOP SAMPLING
// ============================================================

void stopSampling()
{
    // Auto mode must always remain ON.
    if (currentMode == MODE_AUTO)
    {
        sampling = true;
        serverUpload = true;

        SerialBT.println("MODE_AUTO");
        return;
    }


    // Manual mode
    sampling = false;

    serverUpload = false;

    Serial.println("SAMPLING_OFF");

    SerialBT.println("SAMPLING_OFF");
}


// ============================================================
// TAKE MEASUREMENT
// ============================================================

void takeMeasurement()
{
    Serial.println("Measurement cycle completed.");


    // ========================================================
    // AUTO MODE
    //
    // Upload automatically.
    // ========================================================

    if (currentMode == MODE_AUTO &&
        serverUpload)
    {
        uploadData();
    }


    // ========================================================
    // MANUAL MODE
    //
    // Do NOT upload here.
    //
    // Upload only happens when UPLOAD command is received.
    // ========================================================
}


// ============================================================
// SERVER UPLOAD
// ============================================================

void uploadData()
{
    // ========================================================
    // PLACEHOLDER
    //
    // Put your WiFi/GSM/HTTP/MQTT upload code here later.
    // ========================================================

    Serial.println("SERVER: Upload requested.");
}


// ============================================================
// BLUETOOTH SENSOR STATUS
//
// Sent ONCE during startup.
// ============================================================

void sendBluetoothSensorStatus()
{
    SerialBT.println("SENSOR_STATUS");

    SerialBT.print("MPU6050=");
    SerialBT.println(mpuOK ? "OK" : "FAILED");

    SerialBT.print("BMP180=");
    SerialBT.println(bmpOK ? "OK" : "FAILED");

    SerialBT.print("MAX30100=");
    SerialBT.println(maxOK ? "OK" : "FAILED");

    SerialBT.print("VIBRATION=");
    SerialBT.println(vibrationOK ? "OK" : "FAILED");
}


// ============================================================
// BLUETOOTH STATUS
//
// Only sent when STATUS command is received.
// ============================================================

void sendBluetoothStatus()
{
    SerialBT.println("STATUS");

    // --------------------------------------------------------
    // MODE
    // --------------------------------------------------------

    if (currentMode == MODE_AUTO)
    {
        SerialBT.println("MODE_AUTO");
    }
    else
    {
        SerialBT.println("MODE_MANUAL");
    }


    // --------------------------------------------------------
    // SAMPLING
    // --------------------------------------------------------

    SerialBT.print("SAMPLING=");

    if (sampling)
        SerialBT.println("ON");
    else
        SerialBT.println("OFF");


    // --------------------------------------------------------
    // SERVER UPLOAD
    // --------------------------------------------------------

    SerialBT.print("SERVER_UPLOAD=");

    if (serverUpload)
        SerialBT.println("ON");
    else
        SerialBT.println("OFF");


    // --------------------------------------------------------
    // SENSOR STATUS
    //
    // Only returned when STATUS is requested.
    // --------------------------------------------------------

    SerialBT.println("MPU6050=");

    SerialBT.print("MPU6050=");
    SerialBT.println(mpuOK ? "OK" : "FAILED");

    SerialBT.print("BMP180=");
    SerialBT.println(bmpOK ? "OK" : "FAILED");

    SerialBT.print("MAX30100=");
    SerialBT.println(maxOK ? "OK" : "FAILED");

    SerialBT.print("VIBRATION=");
    SerialBT.println(vibrationOK ? "OK" : "FAILED");
}


// ============================================================
// BLUETOOTH SENSOR DATA
//
// Sent every 5 seconds while sampling.
// ============================================================

void sendBluetoothSensorData()
{
    SerialBT.println("DATA_START");


    // ========================================================
    // MPU6050
    // ========================================================

    SerialBT.print("MPU,");

    SerialBT.print(accelX, 3);
    SerialBT.print(",");

    SerialBT.print(accelY, 3);
    SerialBT.print(",");

    SerialBT.print(accelZ, 3);
    SerialBT.print(",");

    SerialBT.print(gyroX, 2);
    SerialBT.print(",");

    SerialBT.print(gyroY, 2);
    SerialBT.print(",");

    SerialBT.print(gyroZ, 2);
    SerialBT.print(",");

    SerialBT.println(mpuTemperature, 2);


    // ========================================================
    // BMP180
    // ========================================================

    SerialBT.print("BMP,");

    SerialBT.print(bmpTemperature, 2);
    SerialBT.print(",");

    SerialBT.print(bmpPressure, 0);
    SerialBT.print(",");

    SerialBT.println(bmpAltitude, 2);


    // ========================================================
    // MAX30100
    // ========================================================

    SerialBT.print("MAX,");

    SerialBT.print(heartRate, 1);
    SerialBT.print(",");

    SerialBT.println(spo2, 1);


    // ========================================================
    // VIBRATION
    // ========================================================

    SerialBT.print("VIB,");

    SerialBT.print(vibrationOnes);
    SerialBT.print(",");

    SerialBT.print(vibrationZeros);
    SerialBT.print(",");

    SerialBT.println(vibrationEvent ? "1" : "0");


    SerialBT.println("DATA_END");


    // Event has now been reported.
    vibrationEvent = false;
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

        mpuOK = false;

        return;
    }

    Serial.println("OK");

    mpuOK = true;


    // Wake MPU6050

    GYWire.beginTransmission(0x68);

    GYWire.write(0x6B);
    GYWire.write(0x00);

    GYWire.endTransmission();


    // Accelerometer ±8G

    GYWire.beginTransmission(0x68);

    GYWire.write(0x1C);
    GYWire.write(0x10);

    GYWire.endTransmission();


    // Gyroscope ±500 deg/s

    GYWire.beginTransmission(0x68);

    GYWire.write(0x1B);
    GYWire.write(0x08);

    GYWire.endTransmission();


    // Digital Low Pass Filter

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

    uint8_t received =
        GYWire.requestFrom(0x68, 14);

    if (received != 14)
        return;


    rawAX =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    rawAY =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    rawAZ =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    rawMPUTemp =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    rawGX =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    rawGY =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    rawGZ =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();


    // Accelerometer ±8G

    accelX = rawAX / 4096.0f;
    accelY = rawAY / 4096.0f;
    accelZ = rawAZ / 4096.0f;


    // Gyroscope ±500 deg/s

    gyroX = rawGX / 65.5f;
    gyroY = rawGY / 65.5f;
    gyroZ = rawGZ / 65.5f;


    // Temperature

    mpuTemperature =
        (rawMPUTemp / 340.0f) + 36.53f;
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


    AC1 =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    AC2 =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    AC3 =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    AC4 =
        ((uint16_t)GYWire.read() << 8) |
        GYWire.read();

    AC5 =
        ((uint16_t)GYWire.read() << 8) |
        GYWire.read();

    AC6 =
        ((uint16_t)GYWire.read() << 8) |
        GYWire.read();

    BMP_B1 =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    B2 =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    MB =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    MC =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();

    MD =
        ((int16_t)GYWire.read() << 8) |
        GYWire.read();


    Serial.println("BMP180 calibration loaded");
}


// ============================================================
// START BMP180 TEMPERATURE
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
// BMP180 PROCESSING
// ============================================================

void processBMP180()
{
    uint32_t now = millis();


    // ========================================================
    // TEMPERATURE
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


        bmpUT =
            ((int32_t)GYWire.read() << 8) |
            GYWire.read();


        int32_t X1 =
            ((bmpUT - (int32_t)AC6) *
             (int32_t)AC5) >> 15;

        int32_t X2 =
            ((int32_t)MC << 11) /
            (X1 + MD);

        int32_t B5 =
            X1 + X2;


        bmpTemperature =
            ((B5 + 8) >> 4) / 10.0f;


        // Start pressure conversion

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
    // PRESSURE
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


        int32_t UT = bmpUT;


        int32_t X1 =
            ((UT - (int32_t)AC6) *
             (int32_t)AC5) >> 15;

        int32_t X2 =
            ((int32_t)MC << 11) /
            (X1 + MD);

        int32_t B5 =
            X1 + X2;


        X1 =
            ((B5 - 4000) *
             (int32_t)AC2) >> 11;

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


        bmpAltitude =
            44330.0f *
            (
                1.0f -
                powf(
                    bmpPressure / 101325.0f,
                    0.1903f
                )
            );


        bmpState = BMP_IDLE;
    }
}


// ============================================================
// VIBRATION
// ============================================================

void measureVibration()
{
    int state =
        digitalRead(VIBRATION_PIN);


    if (!vibrationMeasuring &&
        state != prevVibrationState)
    {
        vibrationMeasuring = true;

        vibrationOnes = 0;
        vibrationZeros = 0;

        vibrationStartTime = millis();

        vibrationEvent = false;
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
    heartRate = pox.getHeartRate();

    spo2 = pox.getSpO2();


    // Sanity checks

    if (isnan(heartRate) ||
        heartRate < 0)
    {
        heartRate = 0;
    }

    if (isnan(spo2) ||
        spo2 < 0)
    {
        spo2 = 0;
    }
}


// ============================================================
// SERIAL MONITOR
// ============================================================

void printSensorData()
{
    Serial.println();
    Serial.println("================================================");
    Serial.println("                 SENSOR DATA");
    Serial.println("================================================");


    // ========================================================
    // SYSTEM
    // ========================================================

    Serial.println();
    Serial.println("[ SYSTEM ]");

    Serial.print("Mode: ");

    if (currentMode == MODE_AUTO)
        Serial.println("MODE_AUTO");
    else
        Serial.println("MODE_MANUAL");


    Serial.print("Sampling: ");
    Serial.println(sampling ? "ON" : "OFF");


    Serial.print("Server Upload: ");
    Serial.println(serverUpload ? "ON" : "OFF");


    // ========================================================
    // SENSOR STATUS
    // ========================================================

    // Serial.println();
    // Serial.println("[ SENSOR STATUS ]");

    // Serial.print("MPU6050: ");
    // Serial.println(mpuOK ? "OK" : "FAILED");

    // Serial.print("BMP180: ");
    // Serial.println(bmpOK ? "OK" : "FAILED");

    // Serial.print("MAX30100: ");
    // Serial.println(maxOK ? "OK" : "FAILED");

    // Serial.print("Vibration: ");
    // Serial.println(vibrationOK ? "OK" : "FAILED");


    if (!sampling)
    {
        Serial.println();
        Serial.println("Sampling is OFF.");
        Serial.println("================================================");

        return;
    }


    // ========================================================
    // MPU6050
    // ========================================================

    Serial.println();
    Serial.println("[ MPU6050 ]");

    Serial.print("Acceleration (g): ");

    Serial.print(accelX, 3);
    Serial.print(", ");

    Serial.print(accelY, 3);
    Serial.print(", ");

    Serial.println(accelZ, 3);


    Serial.print("Gyroscope (deg/s): ");

    Serial.print(gyroX, 2);
    Serial.print(", ");

    Serial.print(gyroY, 2);
    Serial.print(", ");

    Serial.println(gyroZ, 2);


    Serial.print("Temperature: ");
    Serial.print(mpuTemperature, 2);
    Serial.println(" C");


    // ========================================================
    // BMP180
    // ========================================================

    Serial.println();
    Serial.println("[ BMP180 ]");

    Serial.print("Temperature: ");
    Serial.print(bmpTemperature, 2);
    Serial.println(" C");

    Serial.print("Pressure: ");
    Serial.print(bmpPressure, 0);
    Serial.println(" Pa");

    Serial.print("Altitude: ");
    Serial.print(bmpAltitude, 2);
    Serial.println(" m");


    // ========================================================
    // MAX30100
    // ========================================================

    Serial.println();
    Serial.println("[ MAX30100 ]");

    Serial.print("Heart Rate: ");
    Serial.print(heartRate, 1);
    Serial.println(" BPM");

    Serial.print("SpO2: ");
    Serial.print(spo2, 1);
    Serial.println(" %");


    // ========================================================
    // VIBRATION
    // ========================================================

    Serial.println();
    Serial.println("[ VIBRATION ]");

    Serial.print("Transitions HIGH: ");
    Serial.println(vibrationOnes);

    Serial.print("Transitions LOW: ");
    Serial.println(vibrationZeros);

    Serial.print("Event: ");

    if (vibrationEvent)
        Serial.println("YES");
    else
        Serial.println("NO");


    Serial.println();
    Serial.println("================================================");
}