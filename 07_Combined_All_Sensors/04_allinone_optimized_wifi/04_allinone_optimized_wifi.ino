#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
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

bool bluetoothActive = true;


// ============================================================
// WIFI
// ============================================================

Preferences preferences;

String wifiSSID = "";
String wifiPassword = "";

bool wifiConfigured = false;

volatile bool wifiConnected = false;

uint32_t lastWiFiAttempt = 0;

const uint32_t WIFI_RETRY_INTERVAL_MS = 10000;


// ============================================================
// THINGSBOARD
// ============================================================

const char *THINGSBOARD_HOST =
    "https://thingsboard.tail68b90e.ts.net";

const char *THINGSBOARD_ACCESS_TOKEN =
    "YOUR_ACCESS_TOKEN";


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

// MPU6050
const uint32_t MPU_INTERVAL_MS = 10;

// BMP180
const uint32_t BMP_INTERVAL_MS = 100;

// Heart-rate value update
const uint32_t HEART_REPORT_INTERVAL_MS = 1000;

// Serial monitor
const uint32_t SERIAL_INTERVAL_MS = 1000;

// Measurement snapshot
const uint32_t MEASUREMENT_INTERVAL_MS = 1000;

// ThingsBoard upload
const uint32_t UPLOAD_INTERVAL_MS = 5000;


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

volatile bool sampling = false;

volatile bool serverUpload = false;


// ============================================================
// TIMERS
// ============================================================

uint32_t lastMPU = 0;
uint32_t lastBMP = 0;
uint32_t lastSerial = 0;
uint32_t lastHeartReport = 0;
uint32_t lastMeasurement = 0;


// ============================================================
// BLUETOOTH COMMAND BUFFER
//
// Non-blocking.
// We DO NOT use readStringUntil() because it can block.
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
//
// IMPORTANT:
// This runs continuously while sampling.
//
// We maintain a window whenever activity begins.
// The event flag survives until the next measurement snapshot.
// ============================================================

int prevVibrationState = LOW;

bool vibrationMeasuring = false;

uint32_t vibrationStartTime = 0;

const uint32_t VIBRATION_WINDOW_MS = 100;

int vibrationOnes = 0;
int vibrationZeros = 0;

volatile bool vibrationEvent = false;


// ============================================================
// MEASUREMENT STRUCTURE
// ============================================================

struct Measurement
{
    uint32_t timestamp;

    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float mpuTemperature;

    float bmpTemperature;
    float bmpPressure;
    float bmpAltitude;

    float heartRate;
    float spo2;

    int vibrationOnes;
    int vibrationZeros;
    bool vibrationEvent;
};


// ============================================================
// QUEUE
//
// 30 entries = 30 seconds of measurement snapshots.
//
// Sensor loop ONLY writes to queue.
// Upload task ONLY reads from queue.
// ============================================================

const int QUEUE_SIZE = 30;

QueueHandle_t measurementQueue;


// ============================================================
// UPLOAD TASK
// ============================================================

TaskHandle_t uploadTaskHandle = NULL;

volatile bool uploadInProgress = false;

volatile uint32_t uploadSuccessCount = 0;
volatile uint32_t uploadFailCount = 0;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

// Sensors
void initMPU6050();
void initBMP180();

void readMPU6050();

void startBMPTemperature();
void processBMP180();

void measureVibration();
void updateHeartRate();


// Measurement
void takeMeasurement();


// WiFi
void loadWiFiCredentials();
void saveWiFiCredentials(String ssid, String password);
void startWiFiConnection();
void processWiFi();


// ThingsBoard
void uploadTask(void *parameter);
bool uploadMeasurementBatch(
    Measurement *measurements,
    int count
);


// Bluetooth
void processBluetooth();
void processBluetoothCommand(String command);

void sendBluetoothSensorStatus();
void sendBluetoothStatus();

void scanWiFi();


// Modes
void setAutoMode();
void setManualMode();

void startSampling();
void stopSampling();


// Serial
void printSensorData();


// MAX30100
void onBeatDetected();


// ============================================================
// MAX30100 CALLBACK
// ============================================================

void onBeatDetected()
{
    // NEVER print or perform heavy operations here.
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
    Serial.println("========================================");


    // ========================================================
    // BLUETOOTH
    // ========================================================

    SerialBT.begin("ESP32-BME");

    Serial.println();
    Serial.println("Bluetooth started");
    Serial.println("Device: ESP32-BME");


    // ========================================================
    // MAX30100
    // ========================================================

    Serial.println();
    Serial.println("Initializing MAX30100...");

    Wire.begin(MAX_SDA, MAX_SCL);

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
    // LOAD WIFI
    // ========================================================

    preferences.begin("wifi-config", false);

    loadWiFiCredentials();


    // ========================================================
    // QUEUE
    // ========================================================

    measurementQueue =
        xQueueCreate(
            QUEUE_SIZE,
            sizeof(Measurement)
        );

    if (measurementQueue == NULL)
    {
        Serial.println("ERROR: Queue creation failed");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("Measurement queue created");


    // ========================================================
    // START UPLOAD TASK
    //
    // ESP32:
    // Core 0 = Arduino loop
    // Core 1 = upload task
    // ========================================================

    xTaskCreatePinnedToCore(
        uploadTask,
        "UploadTask",
        10000,
        NULL,
        1,
        &uploadTaskHandle,
        1
    );

    Serial.println("Upload task started on Core 1");


    // ========================================================
    // TIMERS
    // ========================================================

    uint32_t now = millis();

    lastMPU = now;
    lastBMP = now;
    lastSerial = now;
    lastHeartReport = now;
    lastMeasurement = now;


    // ========================================================
    // DEFAULT MODE
    // ========================================================

    setManualMode();


    // ========================================================
    // SENSOR STATUS
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
    Serial.println("WIFI_SCAN");
    Serial.println("WIFI_SET|SSID|PASSWORD");
    Serial.println("STATUS");
}


// ============================================================
// MAIN LOOP
//
// THIS LOOP MUST REMAIN FAST.
//
// pox.update() is intentionally first.
// ============================================================

void loop()
{
    uint32_t now = millis();


    // ========================================================
    // MAX30100
    //
    // MUST RUN AS OFTEN AS POSSIBLE.
    // ========================================================

    if (maxOK)
    {
        pox.update();
    }


    // ========================================================
    // BLUETOOTH
    //
    // Only used during configuration/control.
    // No periodic sensor data is sent.
    // ========================================================

    if (bluetoothActive)
    {
        processBluetooth();
    }


    // ========================================================
    // SENSOR PROCESSING
    // ========================================================

    if (sampling)
    {
        // ----------------------------------------------------
        // VIBRATION
        //
        // ALWAYS RUN.
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

            if (bmpState == BMP_IDLE &&
                bmpOK)
            {
                startBMPTemperature();
            }
        }


        // ----------------------------------------------------
        // HEART RATE
        // ----------------------------------------------------

        if (maxOK &&
            now - lastHeartReport >= HEART_REPORT_INTERVAL_MS)
        {
            lastHeartReport = now;

            updateHeartRate();
        }


        // ----------------------------------------------------
        // CREATE MEASUREMENT SNAPSHOT
        //
        // 1 per second.
        // ----------------------------------------------------

        if (now - lastMeasurement >=
            MEASUREMENT_INTERVAL_MS)
        {
            lastMeasurement +=
                MEASUREMENT_INTERVAL_MS;

            takeMeasurement();
        }
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
    // NO DELAY
    // ========================================================
}


// ============================================================
// MEASUREMENT
//
// Creates one 1-second snapshot and places it into the
// FreeRTOS queue.
//
// THIS FUNCTION DOES NOT TOUCH THE NETWORK.
// ============================================================

void takeMeasurement()
{
    Measurement m;

    m.timestamp = millis();

    m.accelX = accelX;
    m.accelY = accelY;
    m.accelZ = accelZ;

    m.gyroX = gyroX;
    m.gyroY = gyroY;
    m.gyroZ = gyroZ;

    m.mpuTemperature = mpuTemperature;

    m.bmpTemperature = bmpTemperature;
    m.bmpPressure = bmpPressure;
    m.bmpAltitude = bmpAltitude;

    m.heartRate = heartRate;
    m.spo2 = spo2;

    m.vibrationOnes = vibrationOnes;
    m.vibrationZeros = vibrationZeros;
    m.vibrationEvent = vibrationEvent;


    // Reset vibration event AFTER copying it.
    vibrationEvent = false;

    vibrationOnes = 0;
    vibrationZeros = 0;


    // ========================================================
    // QUEUE
    // ========================================================

    if (xQueueSend(
            measurementQueue,
            &m,
            0
        ) != pdTRUE)
    {
        // Queue full.
        //
        // IMPORTANT:
        // We DO NOT wait.
        //
        // Dropping a snapshot is preferable to blocking
        // the sensor loop.
        Serial.println(
            "WARNING: Measurement queue full"
        );
    }
}


// ============================================================
// UPLOAD TASK
//
// RUNS ON ESP32 CORE 1.
//
// Uploads up to 5 one-second measurements every 5 seconds.
// ============================================================

void uploadTask(void *parameter)
{
    Measurement batch[5];

    uint32_t lastUpload = millis();


    for (;;)
    {
        // ====================================================
        // WAIT UNTIL 5 SECOND INTERVAL
        // ====================================================

        vTaskDelay(
            pdMS_TO_TICKS(100)
        );


        if (!sampling)
        {
            continue;
        }


        uint32_t now = millis();

        if (now - lastUpload <
            UPLOAD_INTERVAL_MS)
        {
            continue;
        }

        lastUpload = now;


        // ====================================================
        // WIFI
        // ====================================================

        if (WiFi.status() != WL_CONNECTED)
        {
            wifiConnected = false;

            Serial.println(
                "[UPLOAD] WiFi unavailable"
            );

            continue;
        }

        wifiConnected = true;


        // ====================================================
        // GET UP TO 5 MEASUREMENTS
        //
        // Normally this will be exactly 5.
        // ====================================================

        int count = 0;

        while (count < 5)
        {
            if (xQueueReceive(
                    measurementQueue,
                    &batch[count],
                    0
                ) == pdTRUE)
            {
                count++;
            }
            else
            {
                break;
            }
        }


        if (count == 0)
        {
            Serial.println(
                "[UPLOAD] No queued measurements"
            );

            continue;
        }


        // ====================================================
        // UPLOAD
        // ====================================================

        uploadInProgress = true;

        Serial.print(
            "[UPLOAD] Sending "
        );

        Serial.print(count);

        Serial.println(
            " measurement(s)"
        );


        bool success =
            uploadMeasurementBatch(
                batch,
                count
            );


        uploadInProgress = false;


        // ====================================================
        // SUCCESS
        // ====================================================

        if (success)
        {
            uploadSuccessCount += count;

            Serial.println(
                "[UPLOAD] SUCCESS"
            );
        }


        // ====================================================
        // FAILURE
        //
        // Put measurements back into queue.
        // ========================================================

        else
        {
            uploadFailCount += count;

            Serial.println(
                "[UPLOAD] FAILED - returning data to queue"
            );


            // Return in original order.
            //
            // Queue should normally have enough room because
            // we just removed these records.

            for (int i = 0; i < count; i++)
            {
                xQueueSend(
                    measurementQueue,
                    &batch[i],
                    0
                );
            }
        }
    }
}


// ============================================================
// THINGSBOARD UPLOAD
//
// ThingsBoard accepts:
//
// [
//   {
//     "ts": ...,
//     "values": {...}
//   },
//   ...
// ]
//
// This allows multiple timestamped readings in one request.
// ============================================================

bool uploadMeasurementBatch(
    Measurement *measurements,
    int count
)
{
    WiFiClientSecure client;

    // ========================================================
    // PROTOTYPE TLS MODE
    //
    // This skips certificate validation.
    //
    // For production we should install/validate the proper
    // CA certificate instead.
    // ========================================================

    client.setInsecure();


    HTTPClient https;


    String url =
        String(THINGSBOARD_HOST) +
        "/api/v1/" +
        String(THINGSBOARD_ACCESS_TOKEN) +
        "/telemetry";


    if (!https.begin(
            client,
            url
        ))
    {
        Serial.println(
            "[UPLOAD] HTTPS begin failed"
        );

        return false;
    }


    https.addHeader(
        "Content-Type",
        "application/json"
    );


    // ========================================================
    // BUILD JSON ARRAY
    // ========================================================

    String payload;

    payload.reserve(
        count * 450
    );

    payload += "[";


    for (int i = 0; i < count; i++)
    {
        if (i > 0)
            payload += ",";


        // ====================================================
        // Convert millis() to a relative timestamp.
        //
        // IMPORTANT:
        // millis() is NOT Unix time.
        //
        // For now we intentionally don't pretend it is.
        //
        // The server will receive the measurement batch.
        // Later we will add NTP and real Unix timestamps.
        // ====================================================

        payload += "{";

        payload +=
            "\"values\":{";


        payload +=
            "\"temperature\":" +
            String(
                measurements[i].bmpTemperature,
                2
            );

        payload +=
            ",\"pressure\":" +
            String(
                measurements[i].bmpPressure,
                0
            );

        payload +=
            ",\"altitude\":" +
            String(
                measurements[i].bmpAltitude,
                2
            );

        payload +=
            ",\"acceleration_x\":" +
            String(
                measurements[i].accelX,
                3
            );

        payload +=
            ",\"acceleration_y\":" +
            String(
                measurements[i].accelY,
                3
            );

        payload +=
            ",\"acceleration_z\":" +
            String(
                measurements[i].accelZ,
                3
            );

        payload +=
            ",\"gyro_x\":" +
            String(
                measurements[i].gyroX,
                2
            );

        payload +=
            ",\"gyro_y\":" +
            String(
                measurements[i].gyroY,
                2
            );

        payload +=
            ",\"gyro_z\":" +
            String(
                measurements[i].gyroZ,
                2
            );

        payload +=
            ",\"mpu_temperature\":" +
            String(
                measurements[i].mpuTemperature,
                2
            );

        payload +=
            ",\"heart_rate\":" +
            String(
                measurements[i].heartRate,
                1
            );

        payload +=
            ",\"spo2\":" +
            String(
                measurements[i].spo2,
                1
            );

        payload +=
            ",\"vibration_high\":" +
            String(
                measurements[i].vibrationOnes
            );

        payload +=
            ",\"vibration_low\":" +
            String(
                measurements[i].vibrationZeros
            );

        payload +=
            ",\"vibration_event\":" +
            String(
                measurements[i].vibrationEvent
                ? "true"
                : "false"
            );


        payload +=
            "}";

        payload +=
            "}";
    }


    payload += "]";


    // ========================================================
    // SEND
    // ========================================================

    int httpCode =
        https.POST(payload);


    Serial.print(
        "[UPLOAD] HTTP code: "
    );

    Serial.println(httpCode);


    if (httpCode > 0)
    {
        String response =
            https.getString();

        Serial.print(
            "[UPLOAD] Response: "
        );

        Serial.println(response);
    }


    https.end();


    return (
        httpCode >= 200 &&
        httpCode < 300
    );
}


// ============================================================
// WIFI CREDENTIAL STORAGE
// ============================================================

void loadWiFiCredentials()
{
    wifiSSID =
        preferences.getString(
            "ssid",
            ""
        );

    wifiPassword =
        preferences.getString(
            "password",
            ""
        );


    if (wifiSSID.length() > 0)
    {
        wifiConfigured = true;

        Serial.println();
        Serial.println(
            "Saved WiFi credentials found"
        );

        Serial.print(
            "SSID: "
        );

        Serial.println(
            wifiSSID
        );


        startWiFiConnection();
    }
    else
    {
        Serial.println();
        Serial.println(
            "No WiFi credentials configured"
        );
    }
}


// ============================================================
// SAVE WIFI
// ============================================================

void saveWiFiCredentials(
    String ssid,
    String password
)
{
    ssid.trim();
    password.trim();


    if (ssid.length() == 0)
    {
        Serial.println(
            "ERROR: Empty SSID"
        );

        if (bluetoothActive)
            SerialBT.println(
                "WIFI_SET_FAILED"
            );

        return;
    }


    preferences.putString(
        "ssid",
        ssid
    );

    preferences.putString(
        "password",
        password
    );


    wifiSSID = ssid;
    wifiPassword = password;

    wifiConfigured = true;


    Serial.println();
    Serial.println(
        "WiFi credentials saved"
    );

    Serial.print(
        "SSID: "
    );

    Serial.println(
        wifiSSID
    );


    if (bluetoothActive)
    {
        SerialBT.println(
            "WIFI_CONFIG_SAVED"
        );
    }


    startWiFiConnection();
}


// ============================================================
// START WIFI CONNECTION
// ============================================================

void startWiFiConnection()
{
    if (!wifiConfigured)
        return;


    Serial.println();
    Serial.print(
        "Connecting to WiFi: "
    );

    Serial.println(
        wifiSSID
    );


    if (bluetoothActive)
    {
        SerialBT.println(
            "WIFI_CONNECTING"
        );
    }


    WiFi.mode(WIFI_STA);

    WiFi.disconnect();

    delay(20);

    WiFi.begin(
        wifiSSID.c_str(),
        wifiPassword.c_str()
    );


    lastWiFiAttempt = millis();
}


// ============================================================
// WIFI PROCESSING
//
// This runs on Arduino loop.
// It NEVER waits for connection.
// ============================================================

void processWiFi()
{
    if (!wifiConfigured)
        return;


    if (WiFi.status() ==
        WL_CONNECTED)
    {
        if (!wifiConnected)
        {
            wifiConnected = true;

            Serial.println();
            Serial.println(
                "WIFI CONNECTED"
            );

            Serial.print(
                "IP: "
            );

            Serial.println(
                WiFi.localIP()
            );


            if (bluetoothActive)
            {
                SerialBT.println(
                    "WIFI_CONNECTED"
                );

                SerialBT.print(
                    "WIFI_IP="
                );

                SerialBT.println(
                    WiFi.localIP()
                );
            }
        }

        return;
    }


    wifiConnected = false;


    uint32_t now = millis();


    if (now - lastWiFiAttempt >=
        WIFI_RETRY_INTERVAL_MS)
    {
        lastWiFiAttempt = now;

        Serial.println(
            "WiFi reconnect attempt"
        );

        WiFi.disconnect();

        WiFi.begin(
            wifiSSID.c_str(),
            wifiPassword.c_str()
        );
    }
}


// ============================================================
// BLUETOOTH PROCESSING
//
// Completely non-blocking.
// ============================================================

void processBluetooth()
{
    while (SerialBT.available())
    {
        char c =
            SerialBT.read();


        if (c == '\n' ||
            c == '\r')
        {
            if (bluetoothCommand.length() > 0)
            {
                processBluetoothCommand(
                    bluetoothCommand
                );

                bluetoothCommand = "";
            }
        }
        else
        {
            bluetoothCommand += c;


            // Prevent runaway command buffer.
            if (bluetoothCommand.length() > 250)
            {
                bluetoothCommand = "";
            }
        }
    }
}


// ============================================================
// BLUETOOTH COMMAND PROCESSOR
// ============================================================

void processBluetoothCommand(
    String command
)
{
    command.trim();


    Serial.print(
        "BT COMMAND: "
    );

    Serial.println(
        command
    );


    // ========================================================
    // MODE AUTO
    // ========================================================

    if (command == "MODE_AUTO")
    {
        setAutoMode();

        return;
    }


    // ========================================================
    // MODE MANUAL
    // ========================================================

    if (command == "MODE_MANUAL")
    {
        setManualMode();

        return;
    }


    // ========================================================
    // START
    // ========================================================

    if (command == "START")
    {
        startSampling();

        return;
    }


    // ========================================================
    // STOP
    // ========================================================

    if (command == "STOP")
    {
        stopSampling();

        return;
    }


    // ========================================================
    // WIFI SCAN
    // ========================================================

    if (command == "WIFI_SCAN")
    {
        scanWiFi();

        return;
    }


    // ========================================================
    // WIFI SET
    //
    // WIFI_SET|SSID|PASSWORD
    // ========================================================

    if (command.startsWith("WIFI_SET|"))
    {
        int firstSeparator =
            command.indexOf('|');

        int secondSeparator =
            command.indexOf(
                '|',
                firstSeparator + 1
            );


        if (secondSeparator < 0)
        {
            SerialBT.println(
                "WIFI_SET_FAILED"
            );

            return;
        }


        String ssid =
            command.substring(
                firstSeparator + 1,
                secondSeparator
            );


        String password =
            command.substring(
                secondSeparator + 1
            );


        saveWiFiCredentials(
            ssid,
            password
        );

        return;
    }


    // ========================================================
    // STATUS
    // ========================================================

    if (command == "STATUS")
    {
        sendBluetoothStatus();

        return;
    }


    // ========================================================
    // UNKNOWN
    // ========================================================

    SerialBT.println(
        "ERROR_UNKNOWN_COMMAND"
    );
}


// ============================================================
// WIFI SCAN
// ============================================================

void scanWiFi()
{
    if (!bluetoothActive)
        return;


    Serial.println();
    Serial.println(
        "Starting WiFi scan..."
    );


    SerialBT.println(
        "WIFI_SCAN_START"
    );


    int numberOfNetworks =
        WiFi.scanNetworks(
            false,
            false
        );


    Serial.print(
        "Networks found: "
    );

    Serial.println(
        numberOfNetworks
    );


    SerialBT.println(
        "WIFI_LIST"
    );


    for (int i = 0;
         i < numberOfNetworks;
         i++)
    {
        SerialBT.print(
            WiFi.SSID(i)
        );

        SerialBT.print(
            "|RSSI="
        );

        SerialBT.print(
            WiFi.RSSI(i)
        );

        SerialBT.print(
            "|ENC="
        );

        SerialBT.println(
            WiFi.encryptionType(i)
        );
    }


    SerialBT.println(
        "WIFI_LIST_END"
    );


    WiFi.scanDelete();
}


// ============================================================
// AUTO MODE
// ============================================================

void setAutoMode()
{
    currentMode = MODE_AUTO;

    sampling = true;

    serverUpload = true;


    lastMeasurement =
        millis();


    Serial.println();
    Serial.println(
        "MODE_AUTO"
    );


    if (bluetoothActive)
    {
        SerialBT.println(
            "MODE_AUTO"
        );
    }
}


// ============================================================
// MANUAL MODE
// ============================================================

void setManualMode()
{
    currentMode = MODE_MANUAL;

    sampling = false;

    serverUpload = false;


    Serial.println();
    Serial.println(
        "MODE_MANUAL"
    );


    if (bluetoothActive)
    {
        SerialBT.println(
            "MODE_MANUAL"
        );
    }
}


// ============================================================
// START SAMPLING
// ============================================================

void startSampling()
{
    if (currentMode ==
        MODE_AUTO)
    {
        sampling = true;
        serverUpload = true;
    }
    else
    {
        sampling = true;
        serverUpload = false;
    }


    lastMeasurement =
        millis();


    Serial.println();
    Serial.println(
        "SAMPLING_ON"
    );


    if (bluetoothActive)
    {
        SerialBT.println(
            "SAMPLING_ON"
        );


        SerialBT.println(
            "BLUETOOTH_STOPPING"
        );


        delay(50);


        // ====================================================
        // Bluetooth is only needed for configuration/control.
        //
        // Once measurement starts, shut it down so the
        // measurement system has minimal overhead.
        // ====================================================

        SerialBT.disconnect();

        SerialBT.end();

        bluetoothActive = false;


        Serial.println(
            "Bluetooth disabled for measurement"
        );
    }
}


// ============================================================
// STOP SAMPLING
//
// Since Bluetooth is disabled after START, STOP cannot be
// received through Bluetooth during acquisition.
//
// This function is retained for future control methods.
// ============================================================

void stopSampling()
{
    sampling = false;

    serverUpload = false;


    Serial.println();
    Serial.println(
        "SAMPLING_OFF"
    );
}


// ============================================================
// SENSOR STATUS
// ============================================================

void sendBluetoothSensorStatus()
{
    if (!bluetoothActive)
        return;


    SerialBT.println(
        "SENSOR_STATUS"
    );


    SerialBT.print(
        "MPU6050="
    );

    SerialBT.println(
        mpuOK ? "OK" : "FAILED"
    );


    SerialBT.print(
        "BMP180="
    );

    SerialBT.println(
        bmpOK ? "OK" : "FAILED"
    );


    SerialBT.print(
        "MAX30100="
    );

    SerialBT.println(
        maxOK ? "OK" : "FAILED"
    );


    SerialBT.print(
        "VIBRATION="
    );

    SerialBT.println(
        vibrationOK ? "OK" : "FAILED"
    );
}


// ============================================================
// BLUETOOTH STATUS
// ============================================================

void sendBluetoothStatus()
{
    if (!bluetoothActive)
        return;


    SerialBT.println(
        "STATUS"
    );


    // MODE

    SerialBT.print(
        "MODE="
    );

    if (currentMode ==
        MODE_AUTO)
    {
        SerialBT.println(
            "AUTO"
        );
    }
    else
    {
        SerialBT.println(
            "MANUAL"
        );
    }


    // SAMPLING

    SerialBT.print(
        "SAMPLING="
    );

    SerialBT.println(
        sampling
        ? "ON"
        : "OFF"
    );


    // SENSOR STATUS

    SerialBT.print(
        "MPU6050="
    );

    SerialBT.println(
        mpuOK
        ? "OK"
        : "FAILED"
    );


    SerialBT.print(
        "BMP180="
    );

    SerialBT.println(
        bmpOK
        ? "OK"
        : "FAILED"
    );


    SerialBT.print(
        "MAX30100="
    );

    SerialBT.println(
        maxOK
        ? "OK"
        : "FAILED"
    );


    SerialBT.print(
        "VIBRATION="
    );

    SerialBT.println(
        vibrationOK
        ? "OK"
        : "FAILED"
    );


    // WIFI

    SerialBT.print(
        "WIFI="
    );

    SerialBT.println(
        WiFi.status() ==
        WL_CONNECTED
        ? "CONNECTED"
        : "DISCONNECTED"
    );


    if (WiFi.status() ==
        WL_CONNECTED)
    {
        SerialBT.print(
            "WIFI_IP="
        );

        SerialBT.println(
            WiFi.localIP()
        );

        SerialBT.print(
            "WIFI_RSSI="
        );

        SerialBT.println(
            WiFi.RSSI()
        );
    }


    // QUEUE

    SerialBT.print(
        "QUEUE="
    );

    SerialBT.println(
        uxQueueMessagesWaiting(
            measurementQueue
        )
    );


    // UPLOAD

    SerialBT.print(
        "UPLOAD="
    );

    if (uploadInProgress)
    {
        SerialBT.println(
            "IN_PROGRESS"
        );
    }
    else
    {
        SerialBT.println(
            "IDLE"
        );
    }


    SerialBT.print(
        "UPLOAD_SUCCESS="
    );

    SerialBT.println(
        uploadSuccessCount
    );


    SerialBT.print(
        "UPLOAD_FAILED="
    );

    SerialBT.println(
        uploadFailCount
    );
}


// ============================================================
// MPU6050 INITIALIZATION
// ============================================================

void initMPU6050()
{
    Serial.print(
        "Initializing MPU6050... "
    );


    GYWire.beginTransmission(
        0x68
    );


    if (GYWire.endTransmission() != 0)
    {
        Serial.println(
            "FAILED"
        );

        mpuOK = false;

        return;
    }


    Serial.println(
        "OK"
    );

    mpuOK = true;


    // Wake MPU6050

    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(0x6B);
    GYWire.write(0x00);

    GYWire.endTransmission();


    // Accelerometer ±8G

    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(0x1C);
    GYWire.write(0x10);

    GYWire.endTransmission();


    // Gyroscope ±500 deg/s

    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(0x1B);
    GYWire.write(0x08);

    GYWire.endTransmission();


    // Digital Low Pass Filter

    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(0x1A);
    GYWire.write(0x03);

    GYWire.endTransmission();


    Serial.println(
        "MPU6050 configured"
    );
}


// ============================================================
// MPU6050 READ
// ============================================================

void readMPU6050()
{
    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(0x3B);


    if (GYWire.endTransmission(false) != 0)
        return;


    uint8_t received =
        GYWire.requestFrom(
            0x68,
            14
        );


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

    accelX =
        rawAX / 4096.0f;

    accelY =
        rawAY / 4096.0f;

    accelZ =
        rawAZ / 4096.0f;


    // Gyroscope ±500 deg/s

    gyroX =
        rawGX / 65.5f;

    gyroY =
        rawGY / 65.5f;

    gyroZ =
        rawGZ / 65.5f;


    // Temperature

    mpuTemperature =
        (rawMPUTemp / 340.0f)
        + 36.53f;
}


// ============================================================
// BMP180 CALIBRATION
// ============================================================

void initBMP180()
{
    GYWire.beginTransmission(
        0x77
    );

    GYWire.write(0xAA);


    if (GYWire.endTransmission() != 0)
        return;


    if (GYWire.requestFrom(
            0x77,
            22
        ) != 22)
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


    Serial.println(
        "BMP180 calibration loaded"
    );
}


// ============================================================
// START BMP180 TEMPERATURE
// ============================================================

void startBMPTemperature()
{
    GYWire.beginTransmission(
        0x77
    );

    GYWire.write(0xF4);
    GYWire.write(0x2E);


    if (GYWire.endTransmission() != 0)
        return;


    bmpConversionStart =
        millis();

    bmpState =
        BMP_TEMP_WAIT;
}


// ============================================================
// BMP180 PROCESSING
// ============================================================

void processBMP180()
{
    uint32_t now =
        millis();


    // ========================================================
    // TEMPERATURE
    // ========================================================

    if (bmpState ==
        BMP_TEMP_WAIT)
    {
        if (now -
            bmpConversionStart < 5)
            return;


        GYWire.beginTransmission(
            0x77
        );

        GYWire.write(0xF6);


        if (GYWire.endTransmission(false) != 0)
        {
            bmpState =
                BMP_IDLE;

            return;
        }


        if (GYWire.requestFrom(
                0x77,
                2
            ) != 2)
        {
            bmpState =
                BMP_IDLE;

            return;
        }


        bmpUT =
            ((int32_t)GYWire.read() << 8) |
            GYWire.read();


        int32_t X1 =
            ((bmpUT -
              (int32_t)AC6) *
             (int32_t)AC5) >> 15;


        int32_t X2 =
            ((int32_t)MC << 11) /
            (X1 + MD);


        int32_t B5 =
            X1 + X2;


        bmpTemperature =
            ((B5 + 8) >> 4) /
            10.0f;


        // Start pressure conversion

        GYWire.beginTransmission(
            0x77
        );

        GYWire.write(0xF4);
        GYWire.write(0x34);


        if (GYWire.endTransmission() != 0)
        {
            bmpState =
                BMP_IDLE;

            return;
        }


        bmpConversionStart =
            now;

        bmpState =
            BMP_PRESSURE_WAIT;
    }


    // ========================================================
    // PRESSURE
    // ========================================================

    else if (
        bmpState ==
        BMP_PRESSURE_WAIT
    )
    {
        if (now -
            bmpConversionStart < 5)
            return;


        GYWire.beginTransmission(
            0x77
        );

        GYWire.write(0xF6);


        if (GYWire.endTransmission(false) != 0)
        {
            bmpState =
                BMP_IDLE;

            return;
        }


        if (GYWire.requestFrom(
                0x77,
                3
            ) != 3)
        {
            bmpState =
                BMP_IDLE;

            return;
        }


        int32_t MSB =
            GYWire.read();

        int32_t LSB =
            GYWire.read();

        int32_t XLSB =
            GYWire.read();


        bmpUP =
            (
                (MSB << 16) |
                (LSB << 8) |
                XLSB
            ) >> 8;


        int32_t UT =
            bmpUT;


        int32_t X1 =
            ((UT -
              (int32_t)AC6) *
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
            (
                (
                    ((int32_t)AC1 * 4 + X3)
                    + 2
                ) >> 2
            );


        X1 =
            ((int32_t)AC3 * B5) >> 13;


        X2 =
            ((int32_t)B2 *
             ((X1 * X1) >> 12)) >> 16;


        X3 =
            ((X1 + X2) + 2) >> 2;


        uint32_t B4 =
            (
                (uint32_t)AC4 *
                (uint32_t)(X3 + 32768)
            ) >> 15;


        uint32_t B7 =
            ((uint32_t)bmpUP - B3) *
            50000;


        int32_t pressure;


        if (B7 < 0x80000000)
        {
            pressure =
                (B7 * 2) / B4;
        }
        else
        {
            pressure =
                (B7 / B4) * 2;
        }


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


        bmpPressure =
            pressure;


        bmpAltitude =
            44330.0f *
            (
                1.0f -
                powf(
                    bmpPressure /
                    101325.0f,
                    0.1903f
                )
            );


        bmpState =
            BMP_IDLE;
    }
}


// ============================================================
// VIBRATION
//
// Runs continuously.
//
// IMPORTANT:
// The event flag is independent of the 1-second measurement.
// ============================================================

void measureVibration()
{
    int state =
        digitalRead(
            VIBRATION_PIN
        );


    // ========================================================
    // DETECT START OF ACTIVITY
    // ========================================================

    if (!vibrationMeasuring &&
        state != prevVibrationState)
    {
        vibrationMeasuring = true;

        vibrationOnes = 0;
        vibrationZeros = 0;

        vibrationStartTime =
            millis();

        vibrationEvent = true;
    }


    // ========================================================
    // MEASURE ACTIVITY
    // ========================================================

    if (vibrationMeasuring)
    {
        if (state != prevVibrationState)
        {
            if (state == HIGH)
            {
                vibrationOnes++;
            }
            else
            {
                vibrationZeros++;
            }


            prevVibrationState =
                state;
        }


        if (millis() -
            vibrationStartTime >=
            VIBRATION_WINDOW_MS)
        {
            vibrationMeasuring =
                false;
        }
    }
    else
    {
        prevVibrationState =
            state;
    }
}


// ============================================================
// HEART RATE
// ============================================================

void updateHeartRate()
{
    heartRate =
        pox.getHeartRate();

    spo2 =
        pox.getSpO2();


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
    Serial.println(
        "================================================"
    );

    Serial.println(
        "                 SENSOR DATA"
    );

    Serial.println(
        "================================================"
    );


    Serial.println();
    Serial.println(
        "[ SYSTEM ]"
    );


    Serial.print(
        "Mode: "
    );

    if (currentMode ==
        MODE_AUTO)
    {
        Serial.println(
            "MODE_AUTO"
        );
    }
    else
    {
        Serial.println(
            "MODE_MANUAL"
        );
    }


    Serial.print(
        "Sampling: "
    );

    Serial.println(
        sampling
        ? "ON"
        : "OFF"
    );


    Serial.print(
        "WiFi: "
    );

    Serial.println(
        WiFi.status() ==
        WL_CONNECTED
        ? "CONNECTED"
        : "DISCONNECTED"
    );


    Serial.print(
        "Queue: "
    );

    Serial.println(
        measurementQueue
        ? uxQueueMessagesWaiting(
              measurementQueue
          )
        : 0
    );


    if (!sampling)
    {
        Serial.println();
        Serial.println(
            "Sampling is OFF."
        );

        Serial.println(
            "================================================"
        );

        return;
    }


    // ========================================================
    // MPU6050
    // ========================================================

    Serial.println();
    Serial.println(
        "[ MPU6050 ]"
    );


    Serial.print(
        "Acceleration (g): "
    );

    Serial.print(
        accelX,
        3
    );

    Serial.print(", ");

    Serial.print(
        accelY,
        3
    );

    Serial.print(", ");

    Serial.println(
        accelZ,
        3
    );


    Serial.print(
        "Gyroscope (deg/s): "
    );

    Serial.print(
        gyroX,
        2
    );

    Serial.print(", ");

    Serial.print(
        gyroY,
        2
    );

    Serial.print(", ");

    Serial.println(
        gyroZ,
        2
    );


    Serial.print(
        "Temperature: "
    );

    Serial.print(
        mpuTemperature,
        2
    );

    Serial.println(
        " C"
    );


    // ========================================================
    // BMP180
    // ========================================================

    Serial.println();
    Serial.println(
        "[ BMP180 ]"
    );


    Serial.print(
        "Temperature: "
    );

    Serial.print(
        bmpTemperature,
        2
    );

    Serial.println(
        " C"
    );


    Serial.print(
        "Pressure: "
    );

    Serial.print(
        bmpPressure,
        0
    );

    Serial.println(
        " Pa"
    );


    Serial.print(
        "Altitude: "
    );

    Serial.print(
        bmpAltitude,
        2
    );

    Serial.println(
        " m"
    );


    // ========================================================
    // MAX30100
    // ========================================================

    Serial.println();
    Serial.println(
        "[ MAX30100 ]"
    );


    Serial.print(
        "Heart Rate: "
    );

    Serial.print(
        heartRate,
        1
    );

    Serial.println(
        " BPM"
    );


    Serial.print(
        "SpO2: "
    );

    Serial.print(
        spo2,
        1
    );

    Serial.println(
        " %"
    );


    // ========================================================
    // VIBRATION
    // ========================================================

    Serial.println();
    Serial.println(
        "[ VIBRATION ]"
    );


    Serial.print(
        "Transitions HIGH: "
    );

    Serial.println(
        vibrationOnes
    );


    Serial.print(
        "Transitions LOW: "
    );

    Serial.println(
        vibrationZeros
    );


    Serial.print(
        "Event: "
    );

    Serial.println(
        vibrationEvent
        ? "YES"
        : "NO"
    );


    Serial.println();
    Serial.println(
        "================================================"
    );
}