#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <BluetoothSerial.h>
#include "MAX30100_PulseOximeter.h"

// ============================================================
// BLUETOOTH
// ============================================================

BluetoothSerial SerialBT;

// ============================================================
// I2C PINS
// ============================================================

// GY-521 (MPU-6050)
#define GY_SDA 21
#define GY_SCL 22

// MAX30100
#define MAX_SDA 25
#define MAX_SCL 26

// ============================================================
// SD CARD
// ============================================================

#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

SPIClass SD_SPI(VSPI);

const uint64_t MIN_FREE_SPACE = 16ULL * 1024ULL;

// ============================================================
// VIBRATION
// ============================================================

#define VIBRATION_PIN 27

// ============================================================
// TIMING
// ============================================================

// MPU6050 = 100 Hz
const uint32_t MPU_INTERVAL_MS = 10;

// Heart-rate values updated once per second
const uint32_t HEART_REPORT_INTERVAL_MS = 1000;

// Serial sensor display
const uint32_t SERIAL_INTERVAL_MS = 1000;

// SD flush
const uint32_t SD_FLUSH_INTERVAL_MS = 1000;

// ============================================================
// MAX30100
// ============================================================

PulseOximeter pox;

float heartRate = 0.0;
float spo2 = 0.0;

// ============================================================
// GY-521 I2C
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
// BMP180
// ============================================================
//
// BMP180 IS NO LONGER USED.
//
// All BMP180 code has been disabled/removed from the active
// system and from the CSV file.
//
// ============================================================

// ============================================================
// VIBRATION STATE
// ============================================================

int prevVibrationState = LOW;

bool vibrationMeasuring = false;

uint32_t vibrationStartTime = 0;

const uint32_t VIBRATION_WINDOW_MS = 100;

// Live counters during the current measurement
int vibrationOnes = 0;
int vibrationZeros = 0;

// Completed event result waiting to be written
int completedVibrationOnes = 0;
int completedVibrationZeros = 0;

bool vibrationEvent = false;

// ============================================================
// SYSTEM STATE
// ============================================================

volatile bool scanning = false;

volatile bool stopRequested = false;

volatile bool sdInitialized = false;

volatile bool sdFull = false;

volatile bool sdWriterBusy = false;

volatile bool sdReleaseRequested = false;

// ============================================================
// SCAN
// ============================================================

uint32_t scanStartTime = 0;

uint32_t scanNumber = 0;

String currentScanFilename = "";

// ============================================================
// SENSOR DATA
// ============================================================

struct SensorData
{
    uint32_t scanTime;

    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float mpuTemperature;

    float heartRate;
    float spo2;

    int vibrationOnes;
    int vibrationZeros;

    bool vibrationEvent;
};

// ============================================================
// FREE RTOS QUEUE
// ============================================================

QueueHandle_t sensorQueue;

// ============================================================
// SD COMMANDS
// ============================================================

enum SDCommandType
{
    SD_CMD_START,
    SD_CMD_STOP,
    SD_CMD_INIT,
    SD_CMD_LIST,
    SD_CMD_READ
};

struct SDCommand
{
    SDCommandType type;
    int index;
};

QueueHandle_t sdCommandQueue;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void initMPU6050();
void readMPU6050();

void measureVibration();

void printSensorData();

void onBeatDetected();

void sensorTask(void *parameter);
void sdTask(void *parameter);

void processBluetoothCommands();

void sendBluetooth(const String &message);

void initializeSDCard();

void startScan(File &currentFile);

void listScanFiles();

void readScanFile(int index);

void releaseSDCard();

bool writeSensorData(
    File &file,
    const SensorData &data
);

void writeCSVHeader(File &file);

String getScanFilename(
    uint32_t number
);

uint32_t findNextScanNumber();

void printSystemStatus();

// ============================================================
// MAX30100 CALLBACK
// ============================================================

void onBeatDetected()
{
    // Keep callback extremely fast.
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
    Serial.println("ESP32 SENSOR DATA LOGGER");
    Serial.println("DUAL CORE + BLUETOOTH + SD");
    Serial.println("========================================");

    // ========================================================
    // BLUETOOTH
    // ========================================================

    Serial.println();
    Serial.println("Initializing Bluetooth...");

    if (!SerialBT.begin("ESP32_SENSOR_LOGGER"))
    {
        Serial.println("Bluetooth FAILED");
    }
    else
    {
        Serial.println("Bluetooth OK");
        Serial.println("Device: ESP32_SENSOR_LOGGER");
    }

    // ========================================================
    // MAX30100
    // ========================================================

    Serial.println();
    Serial.println("Initializing MAX30100...");

    Wire.begin(
        MAX_SDA,
        MAX_SCL
    );

    Wire.setClock(400000);

    if (!pox.begin())
    {
        Serial.println("MAX30100 FAILED");
    }
    else
    {
        Serial.println("MAX30100 OK");

        pox.setIRLedCurrent(
            MAX30100_LED_CURR_7_6MA
        );

        pox.setOnBeatDetectedCallback(
            onBeatDetected
        );
    }

    // ========================================================
    // GY-521 / MPU6050
    // ========================================================

    Serial.println();
    Serial.println("Initializing GY-521 (MPU-6050)...");

    GYWire.begin(
        GY_SDA,
        GY_SCL,
        400000
    );

    GYWire.setClock(400000);

    Serial.println(
        "GY-521 I2C initialized on bus 1 (SDA 21 / SCL 22)"
    );

    initMPU6050();

    // ========================================================
    // BMP180
    // ========================================================
    //
    // BMP180 IS NO LONGER PART OF THE SYSTEM.
    //
    // No BMP180 initialization is performed.
    // No BMP180 values are written to the CSV.
    //
    // ========================================================

    Serial.println();
    Serial.println(
        "BMP180 DISABLED / REMOVED"
    );

    // ========================================================
    // VIBRATION
    // ========================================================

    pinMode(
        VIBRATION_PIN,
        INPUT
    );

    prevVibrationState =
        digitalRead(
            VIBRATION_PIN
        );

    // Make sure all vibration values start at zero.
    vibrationOnes = 0;
    vibrationZeros = 0;

    completedVibrationOnes = 0;
    completedVibrationZeros = 0;

    vibrationEvent = false;
    vibrationMeasuring = false;

    Serial.println();
    Serial.println(
        "Vibration sensor ready"
    );

    // ========================================================
    // SD SPI
    // ========================================================

    SD_SPI.begin(
        SD_SCK,
        SD_MISO,
        SD_MOSI,
        SD_CS
    );

    // ========================================================
    // QUEUES
    // ========================================================

    sensorQueue =
        xQueueCreate(
            300,
            sizeof(SensorData)
        );

    sdCommandQueue =
        xQueueCreate(
            10,
            sizeof(SDCommand)
        );

    if (sensorQueue == NULL)
    {
        Serial.println(
            "ERROR: Sensor queue failed"
        );
    }

    if (sdCommandQueue == NULL)
    {
        Serial.println(
            "ERROR: SD command queue failed"
        );
    }

    // ========================================================
    // SENSOR TASK
    // CORE 1
    // ========================================================

    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        8192,
        NULL,
        3,
        NULL,
        1
    );

    // ========================================================
    // SD TASK
    // CORE 0
    // ========================================================

    xTaskCreatePinnedToCore(
        sdTask,
        "SDTask",
        8192,
        NULL,
        2,
        NULL,
        0
    );

    // ========================================================
    // READY
    // ========================================================

    Serial.println();
    Serial.println("========================================");
    Serial.println("SYSTEM READY");
    Serial.println("========================================");

    Serial.println();
    Serial.println("Bluetooth commands:");
    Serial.println();

    Serial.println("SD_INIT");
    Serial.println("START");
    Serial.println("STOP");
    Serial.println("LIST");
    Serial.println("READ <index>");
    Serial.println("SD_RELEASE");
    Serial.println("STATUS");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    processBluetoothCommands();

    delay(2);
}

// ============================================================
// BLUETOOTH COMMAND PROCESSOR
// ============================================================

void processBluetoothCommands()
{
    static String command = "";

    while (SerialBT.available())
    {
        char c = SerialBT.read();

        if (
            c == '\n' ||
            c == '\r'
        )
        {
            if (command.length() > 0)
            {
                command.trim();

                Serial.print(
                    "BT COMMAND: "
                );

                Serial.println(
                    command
                );

                // =================================================
                // SD_INIT
                // =================================================

                if (
                    command.equalsIgnoreCase(
                        "SD_INIT"
                    )
                )
                {
                    SDCommand cmd;

                    cmd.type =
                        SD_CMD_INIT;

                    cmd.index = -1;

                    xQueueSend(
                        sdCommandQueue,
                        &cmd,
                        0
                    );
                }

                // =================================================
                // START
                // =================================================

                else if (
                    command.equalsIgnoreCase(
                        "START"
                    )
                )
                {
                    SDCommand cmd;

                    cmd.type =
                        SD_CMD_START;

                    cmd.index = -1;

                    xQueueSend(
                        sdCommandQueue,
                        &cmd,
                        0
                    );
                }

                // =================================================
                // STOP
                // =================================================

                else if (
                    command.equalsIgnoreCase(
                        "STOP"
                    )
                )
                {
                    if (!scanning)
                    {
                        sendBluetooth(
                            "NO SCAN IN PROGRESS"
                        );
                    }
                    else
                    {
                        stopRequested = true;

                        sendBluetooth(
                            "STOP REQUESTED"
                        );
                    }
                }

                // =================================================
                // LIST
                // =================================================

                else if (
                    command.equalsIgnoreCase(
                        "LIST"
                    )
                )
                {
                    SDCommand cmd;

                    cmd.type =
                        SD_CMD_LIST;

                    cmd.index = -1;

                    xQueueSend(
                        sdCommandQueue,
                        &cmd,
                        0
                    );
                }

                // =================================================
                // READ
                // =================================================

                else if (
                    command.startsWith("READ ") ||
                    command.startsWith("read ")
                )
                {
                    String number =
                        command.substring(5);

                    int index =
                        number.toInt();

                    SDCommand cmd;

                    cmd.type =
                        SD_CMD_READ;

                    cmd.index =
                        index;

                    xQueueSend(
                        sdCommandQueue,
                        &cmd,
                        0
                    );
                }

                // =================================================
                // SD_RELEASE
                // =================================================

                else if (
                    command.equalsIgnoreCase(
                        "SD_RELEASE"
                    )
                )
                {
                    if (scanning)
                    {
                        sendBluetooth(
                            "SCAN IN PROGRESS"
                        );

                        sendBluetooth(
                            "STOP SCAN FIRST"
                        );
                    }
                    else
                    {
                        sdReleaseRequested = true;

                        sendBluetooth(
                            "SD RELEASE REQUESTED"
                        );
                    }
                }

                // =================================================
                // STATUS
                // =================================================

                else if (
                    command.equalsIgnoreCase(
                        "STATUS"
                    )
                )
                {
                    printSystemStatus();
                }

                // =================================================
                // UNKNOWN
                // =================================================

                else
                {
                    sendBluetooth(
                        "UNKNOWN COMMAND"
                    );
                }

                command = "";
            }
        }
        else
        {
            if (command.length() < 100)
            {
                command += c;
            }
        }
    }
}

// ============================================================
// BLUETOOTH SEND
// ============================================================

void sendBluetooth(
    const String &message
)
{
    SerialBT.println(message);

    Serial.println(
        "[BT] " + message
    );
}

// ============================================================
// SENSOR TASK
// CORE 1
// ============================================================

void sensorTask(void *parameter)
{
    uint32_t lastMPU =
        millis();

    uint32_t lastHeart =
        millis();

    uint32_t lastSerial =
        millis();

    while (true)
    {
        uint32_t now =
            millis();

        // ========================================================
        // MAX30100
        // ========================================================

        pox.update();

        // ========================================================
        // VIBRATION
        // ========================================================

        measureVibration();

        // ========================================================
        // MPU6050
        // 100 Hz
        // ========================================================

        if (
            now - lastMPU >=
            MPU_INTERVAL_MS
        )
        {
            lastMPU +=
                MPU_INTERVAL_MS;

            readMPU6050();

            // ====================================================
            // CREATE LOG SAMPLE
            // ====================================================

            if (scanning)
            {
                SensorData data;

                data.scanTime =
                    now -
                    scanStartTime;

                data.accelX =
                    accelX;

                data.accelY =
                    accelY;

                data.accelZ =
                    accelZ;

                data.gyroX =
                    gyroX;

                data.gyroY =
                    gyroY;

                data.gyroZ =
                    gyroZ;

                data.mpuTemperature =
                    mpuTemperature;

                // =================================================
                // NO BMP180
                // =================================================
                //
                // BMP180 fields no longer exist in SensorData.
                //
                // =================================================

                data.heartRate =
                    heartRate;

                data.spo2 =
                    spo2;

                // =================================================
                // VIBRATION
                // =================================================

                data.vibrationOnes =
                    completedVibrationOnes;

                data.vibrationZeros =
                    completedVibrationZeros;

                data.vibrationEvent =
                    vibrationEvent;

                // =================================================
                // QUEUE
                // =================================================

                if (
                    xQueueSend(
                        sensorQueue,
                        &data,
                        0
                    ) != pdTRUE
                )
                {
                    Serial.println(
                        "WARNING: SENSOR QUEUE FULL"
                    );
                }

                // =================================================
                // CLEAR COMPLETED VIBRATION RESULT
                // =================================================

                if (vibrationEvent)
                {
                    vibrationEvent =
                        false;

                    completedVibrationOnes =
                        0;

                    completedVibrationZeros =
                        0;
                }
            }
        }

        // ========================================================
        // HEART RATE
        // ========================================================

        if (
            now - lastHeart >=
            HEART_REPORT_INTERVAL_MS
        )
        {
            lastHeart +=
                HEART_REPORT_INTERVAL_MS;

            heartRate =
                pox.getHeartRate();

            spo2 =
                pox.getSpO2();
        }

        // ========================================================
        // STOP REQUEST
        // ========================================================

        if (
            stopRequested &&
            scanning
        )
        {
            scanning = false;

            stopRequested = false;

            sendBluetooth(
                "SCAN STOPPING"
            );
        }

        // ========================================================
        // SERIAL OUTPUT
        // ========================================================

        if (
            now - lastSerial >=
            SERIAL_INTERVAL_MS
        )
        {
            lastSerial +=
                SERIAL_INTERVAL_MS;

            printSensorData();
        }

        // ========================================================
        // YIELD
        // ========================================================

        taskYIELD();
    }
}

// ============================================================
// SD TASK
// CORE 0
// ============================================================

void sdTask(void *parameter)
{
    File currentFile;

    bool scanFileOpen = false;

    uint32_t lastFlush =
        millis();

    bool stopMessageSent =
        false;

    while (true)
    {
        // ========================================================
        // HANDLE COMMANDS
        // ========================================================

        SDCommand cmd;

        if (
            xQueueReceive(
                sdCommandQueue,
                &cmd,
                0
            ) == pdTRUE
        )
        {
            // ====================================================
            // SD INIT
            // ====================================================

            if (
                cmd.type ==
                SD_CMD_INIT
            )
            {
                if (scanning)
                {
                    sendBluetooth(
                        "ERROR: SCAN IN PROGRESS"
                    );
                }
                else if (sdInitialized)
                {
                    sendBluetooth(
                        "SD CARD ALREADY INITIALIZED"
                    );
                }
                else
                {
                    initializeSDCard();
                }
            }

            // ====================================================
            // START
            // ====================================================

            else if (
                cmd.type ==
                SD_CMD_START
            )
            {
                if (!sdInitialized)
                {
                    sendBluetooth(
                        "ERROR: SD NOT INITIALIZED"
                    );
                }
                else if (scanning)
                {
                    sendBluetooth(
                        "ERROR: SCAN ALREADY RUNNING"
                    );
                }
                else if (scanFileOpen)
                {
                    sendBluetooth(
                        "ERROR: PREVIOUS FILE STILL OPEN"
                    );
                }
                else if (sdFull)
                {
                    sendBluetooth(
                        "ERROR: SD CARD FULL"
                    );
                }
                else
                {
                    startScan(
                        currentFile
                    );

                    if (currentFile)
                    {
                        scanFileOpen =
                            true;

                        stopMessageSent =
                            false;

                        lastFlush =
                            millis();
                    }
                }
            }

            // ====================================================
            // LIST
            // ====================================================

            else if (
                cmd.type ==
                SD_CMD_LIST
            )
            {
                if (!sdInitialized)
                {
                    sendBluetooth(
                        "ERROR: SD NOT INITIALIZED"
                    );
                }
                else if (scanning)
                {
                    sendBluetooth(
                        "ERROR: STOP SCAN FIRST"
                    );
                }
                else if (scanFileOpen)
                {
                    sendBluetooth(
                        "ERROR: FILE WRITING IN PROGRESS"
                    );
                }
                else
                {
                    listScanFiles();
                }
            }

            // ====================================================
            // READ
            // ====================================================

            else if (
                cmd.type ==
                SD_CMD_READ
            )
            {
                if (!sdInitialized)
                {
                    sendBluetooth(
                        "ERROR: SD NOT INITIALIZED"
                    );
                }
                else if (scanning)
                {
                    sendBluetooth(
                        "ERROR: STOP SCAN FIRST"
                    );
                }
                else if (scanFileOpen)
                {
                    sendBluetooth(
                        "ERROR: FILE WRITING IN PROGRESS"
                    );
                }
                else
                {
                    readScanFile(
                        cmd.index
                    );
                }
            }
        }

        // ========================================================
        // WRITE SENSOR DATA
        // ========================================================

        if (
            scanFileOpen &&
            currentFile
        )
        {
            SensorData data;

            if (
                xQueueReceive(
                    sensorQueue,
                    &data,
                    0
                ) == pdTRUE
            )
            {
                sdWriterBusy =
                    true;

                if (
                    !writeSensorData(
                        currentFile,
                        data
                    )
                )
                {
                    sdFull =
                        true;

                    scanning =
                        false;

                    stopRequested =
                        false;

                    sendBluetooth(
                        "SD WRITE FAILED"
                    );

                    sendBluetooth(
                        "SCAN STOPPED"
                    );

                    currentFile.flush();

                    currentFile.close();

                    scanFileOpen =
                        false;

                    currentScanFilename =
                        "";

                    sdWriterBusy =
                        false;

                    stopMessageSent =
                        true;
                }
                else
                {
                    sdWriterBusy =
                        false;
                }
            }
        }

        // ========================================================
        // PERIODIC FLUSH
        // ========================================================

        if (
            scanFileOpen &&
            currentFile &&
            millis() - lastFlush >=
            SD_FLUSH_INTERVAL_MS
        )
        {
            currentFile.flush();

            lastFlush =
                millis();
        }

        // ========================================================
        // STOPPED SCAN
        // ========================================================

        if (
            scanFileOpen &&
            !scanning
        )
        {
            if (
                uxQueueMessagesWaiting(
                    sensorQueue
                ) == 0 &&
                !sdWriterBusy
            )
            {
                currentFile.flush();

                currentFile.close();

                scanFileOpen =
                    false;

                String finishedFile =
                    currentScanFilename;

                currentScanFilename =
                    "";

                if (!stopMessageSent)
                {
                    sendBluetooth(
                        "SCAN STOPPED"
                    );

                    sendBluetooth(
                        "FILE: " +
                        finishedFile
                    );

                    stopMessageSent =
                        true;
                }

                Serial.println(
                    "Scan file closed"
                );
            }
            else
            {
                if (!stopMessageSent)
                {
                    sendBluetooth(
                        "DATA WRITING IN PROGRESS"
                    );

                    sendBluetooth(
                        "FINISHING QUEUED DATA..."
                    );
                }

                stopMessageSent =
                    true;
            }
        }

        // ========================================================
        // SD RELEASE
        // ========================================================

        if (
            sdReleaseRequested
        )
        {
            if (scanning)
            {
                sendBluetooth(
                    "SCAN IN PROGRESS"
                );

                sendBluetooth(
                    "STOP SCAN FIRST"
                );

                sdReleaseRequested =
                    false;
            }
            else if (
                scanFileOpen ||
                sdWriterBusy ||
                uxQueueMessagesWaiting(
                    sensorQueue
                ) > 0
            )
            {
                static uint32_t lastWaitMessage = 0;

                if (
                    millis() -
                    lastWaitMessage >=
                    1000
                )
                {
                    sendBluetooth(
                        "DATA WRITING IN PROGRESS"
                    );

                    sendBluetooth(
                        "PLEASE WAIT..."
                    );

                    lastWaitMessage =
                        millis();
                }
            }
            else
            {
                releaseSDCard();

                sdReleaseRequested =
                    false;
            }
        }

        // ========================================================
        // TASK DELAY
        // ========================================================

        vTaskDelay(1);
    }
}

// ============================================================
// SD INITIALIZATION
// ============================================================

void initializeSDCard()
{
    sendBluetooth(
        "INITIALIZING SD CARD..."
    );

    if (
        SD.begin(
            SD_CS,
            SD_SPI,
            20000000
        )
    )
    {
        sdInitialized =
            true;

        sdFull =
            false;

        uint64_t total =
            SD.totalBytes();

        uint64_t used =
            SD.usedBytes();

        uint64_t freeSpace =
            total - used;

        Serial.println();
        Serial.println(
            "SD CARD OK"
        );

        Serial.print(
            "Total: "
        );

        Serial.println(
            total
        );

        Serial.print(
            "Used: "
        );

        Serial.println(
            used
        );

        Serial.print(
            "Free: "
        );

        Serial.println(
            freeSpace
        );

        sendBluetooth(
            "SD CARD INITIALIZED"
        );

        sendBluetooth(
            "FREE SPACE: " +
            String(
                (double)freeSpace /
                1024.0 /
                1024.0,
                2
            ) +
            " MB"
        );
    }
    else
    {
        sdInitialized =
            false;

        Serial.println(
            "SD CARD FAILED"
        );

        sendBluetooth(
            "SD CARD INITIALIZATION FAILED"
        );
    }
}

// ============================================================
// START SCAN
// ============================================================

void startScan(File &currentFile)
{
    uint64_t total =
        SD.totalBytes();

    uint64_t used =
        SD.usedBytes();

    uint64_t freeSpace =
        total - used;

    if (
        freeSpace <
        MIN_FREE_SPACE
    )
    {
        sdFull =
            true;

        sendBluetooth(
            "ERROR: SD CARD FULL"
        );

        return;
    }

    // ========================================================
    // RESET VIBRATION STATE FOR NEW SCAN
    // ========================================================

    vibrationOnes = 0;
    vibrationZeros = 0;

    completedVibrationOnes = 0;
    completedVibrationZeros = 0;

    vibrationEvent = false;

    vibrationMeasuring = false;

    prevVibrationState =
        digitalRead(
            VIBRATION_PIN
        );

    // ========================================================
    // FIND NEXT FILE
    // ========================================================

    scanNumber =
        findNextScanNumber();

    currentScanFilename =
        getScanFilename(
            scanNumber
        );

    // ========================================================
    // OPEN FILE
    // ========================================================

    currentFile =
        SD.open(
            currentScanFilename,
            FILE_WRITE
        );

    if (!currentFile)
    {
        currentScanFilename =
            "";

        sendBluetooth(
            "ERROR: CANNOT CREATE FILE"
        );

        return;
    }

    // ========================================================
    // HEADER
    // ========================================================

    writeCSVHeader(
        currentFile
    );

    currentFile.flush();

    // ========================================================
    // START TIMER
    // ========================================================

    scanStartTime =
        millis();

    scanning =
        true;

    stopRequested =
        false;

    sdFull =
        false;

    sendBluetooth(
        "================================"
    );

    sendBluetooth(
        "SCAN STARTED"
    );

    sendBluetooth(
        "FILE: " +
        currentScanFilename
    );

    sendBluetooth(
        "SAMPLING: 100 Hz"
    );

    sendBluetooth(
        "================================"
    );
}

// ============================================================
// CSV HEADER
// ============================================================
//
// BMP180 IS NOT INCLUDED.
//
// ============================================================

void writeCSVHeader(File &file)
{
    file.println(
        "scan_time_ms,"
        "accel_x,"
        "accel_y,"
        "accel_z,"
        "gyro_x,"
        "gyro_y,"
        "gyro_z,"
        "mpu_temperature,"
        "heart_rate,"
        "spo2,"
        "vibration_event,"
        "vibration_high,"
        "vibration_low"
    );
}

// ============================================================
// WRITE SENSOR DATA
// ============================================================

bool writeSensorData(
    File &file,
    const SensorData &data
)
{
    if (!file)
    {
        return false;
    }

    // ========================================================
    // SCAN TIME
    // ========================================================

    file.print(
        data.scanTime
    );

    file.print(",");

    // ========================================================
    // ACCELEROMETER
    // ========================================================

    file.print(
        data.accelX,
        4
    );

    file.print(",");

    file.print(
        data.accelY,
        4
    );

    file.print(",");

    file.print(
        data.accelZ,
        4
    );

    file.print(",");

    // ========================================================
    // GYROSCOPE
    // ========================================================

    file.print(
        data.gyroX,
        3
    );

    file.print(",");

    file.print(
        data.gyroY,
        3
    );

    file.print(",");

    file.print(
        data.gyroZ,
        3
    );

    file.print(",");

    // ========================================================
    // MPU TEMPERATURE
    // ========================================================

    file.print(
        data.mpuTemperature,
        2
    );

    file.print(",");

    // ========================================================
    // MAX30100
    // ========================================================

    file.print(
        data.heartRate,
        1
    );

    file.print(",");

    file.print(
        data.spo2,
        1
    );

    file.print(",");

    // ========================================================
    // VIBRATION
    // ========================================================

    file.print(
        data.vibrationEvent
        ? 1
        : 0
    );

    file.print(",");

    file.print(
        data.vibrationOnes
    );

    file.print(",");

    file.println(
        data.vibrationZeros
    );

    // ========================================================
    // CHECK CARD SPACE
    // ========================================================

    static uint32_t lastSpaceCheck = 0;

    if (
        millis() -
        lastSpaceCheck >=
        1000
    )
    {
        lastSpaceCheck =
            millis();

        uint64_t total =
            SD.totalBytes();

        uint64_t used =
            SD.usedBytes();

        if (total > used)
        {
            uint64_t freeSpace =
                total - used;

            if (
                freeSpace <
                MIN_FREE_SPACE
            )
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    // ========================================================
    // FILE WRITE ERROR CHECK
    // ========================================================

    if (
        file.getWriteError()
    )
    {
        return false;
    }

    return true;
}

// ============================================================
// RELEASE SD CARD
// ============================================================

void releaseSDCard()
{
    if (scanning)
    {
        sendBluetooth(
            "ERROR: SCAN IN PROGRESS"
        );

        return;
    }

    if (!sdInitialized)
    {
        sendBluetooth(
            "SD CARD NOT INITIALIZED"
        );

        return;
    }

    SD.end();

    sdInitialized =
        false;

    sdFull =
        false;

    sendBluetooth(
        "SD CARD RELEASED"
    );

    sendBluetooth(
        "SAFE TO REMOVE"
    );

    Serial.println(
        "SD card released"
    );
}

// ============================================================
// FILENAME
// ============================================================

String getScanFilename(
    uint32_t number
)
{
    char filename[32];

    snprintf(
        filename,
        sizeof(filename),
        "/SCAN_%03lu.csv",
        (unsigned long)number
    );

    return String(
        filename
    );
}

// ============================================================
// FIND NEXT SCAN NUMBER
// ============================================================

uint32_t findNextScanNumber()
{
    uint32_t number = 1;

    while (true)
    {
        String filename =
            getScanFilename(
                number
            );

        if (
            !SD.exists(filename)
        )
        {
            return number;
        }

        number++;

        if (number > 99999)
        {
            return 99999;
        }
    }
}

// ============================================================
// LIST FILES
// ============================================================

void listScanFiles()
{
    File root =
        SD.open("/");

    if (!root)
    {
        sendBluetooth(
            "ERROR: CANNOT OPEN SD ROOT"
        );

        return;
    }

    sendBluetooth(
        "================================"
    );

    sendBluetooth(
        "AVAILABLE SCANS"
    );

    sendBluetooth(
        "================================"
    );

    int index = 0;

    File file =
        root.openNextFile();

    while (file)
    {
        if (!file.isDirectory())
        {
            String name =
                file.name();

            if (
                name.endsWith(".csv") ||
                name.endsWith(".CSV")
            )
            {
                uint64_t size =
                    file.size();

                SerialBT.print("[");
                SerialBT.print(index);
                SerialBT.print("] ");

                SerialBT.print(name);

                SerialBT.print("  ");

                SerialBT.print(
                    (double)size /
                    1024.0,
                    2
                );

                SerialBT.println(
                    " KB"
                );

                index++;
            }
        }

        file.close();

        file =
            root.openNextFile();
    }

    root.close();

    if (index == 0)
    {
        sendBluetooth(
            "NO SCAN FILES FOUND"
        );
    }
    else
    {
        sendBluetooth(
            "--------------------------------"
        );

        sendBluetooth(
            "Use READ <index>"
        );
    }

    sendBluetooth(
        "================================"
    );
}

// ============================================================
// READ FILE
// ============================================================

void readScanFile(
    int requestedIndex
)
{
    File root =
        SD.open("/");

    if (!root)
    {
        sendBluetooth(
            "ERROR: CANNOT OPEN SD ROOT"
        );

        return;
    }

    int currentIndex = 0;

    String selectedFilename =
        "";

    File file =
        root.openNextFile();

    while (file)
    {
        if (!file.isDirectory())
        {
            String name =
                file.name();

            if (
                name.endsWith(".csv") ||
                name.endsWith(".CSV")
            )
            {
                if (
                    currentIndex ==
                    requestedIndex
                )
                {
                    selectedFilename =
                        name;

                    file.close();

                    break;
                }

                currentIndex++;
            }
        }

        file.close();

        file =
            root.openNextFile();
    }

    root.close();

    // ========================================================
    // INVALID INDEX
    // ========================================================

    if (
        selectedFilename.length() == 0
    )
    {
        sendBluetooth(
            "INVALID FILE INDEX"
        );

        return;
    }

    // ========================================================
    // OPEN
    // ========================================================

    File selected =
        SD.open(
            selectedFilename,
            FILE_READ
        );

    if (!selected)
    {
        sendBluetooth(
            "ERROR: CANNOT OPEN FILE"
        );

        return;
    }

    sendBluetooth(
        "================================"
    );

    sendBluetooth(
        "READING: " +
        selectedFilename
    );

    sendBluetooth(
        "================================"
    );

    // ========================================================
    // STREAM
    // ========================================================

    uint8_t buffer[128];

    while (
        selected.available()
    )
    {
        size_t bytesRead =
            selected.read(
                buffer,
                sizeof(buffer)
            );

        if (
            bytesRead == 0
        )
        {
            break;
        }

        SerialBT.write(
            buffer,
            bytesRead
        );

        delay(2);
    }

    selected.close();

    SerialBT.println();

    sendBluetooth(
        "================================"
    );

    sendBluetooth(
        "END OF FILE"
    );

    sendBluetooth(
        "================================"
    );
}

// ============================================================
// STATUS
// ============================================================

void printSystemStatus()
{
    SerialBT.println();

    SerialBT.println(
        "========== STATUS =========="
    );

    SerialBT.print(
        "SD: "
    );

    SerialBT.println(
        sdInitialized
        ? "INITIALIZED"
        : "NOT INITIALIZED"
    );

    SerialBT.print(
        "Scanning: "
    );

    SerialBT.println(
        scanning
        ? "YES"
        : "NO"
    );

    SerialBT.print(
        "Writer: "
    );

    SerialBT.println(
        sdWriterBusy
        ? "BUSY"
        : "IDLE"
    );

    SerialBT.print(
        "Queue entries: "
    );

    SerialBT.println(
        uxQueueMessagesWaiting(
            sensorQueue
        )
    );

    SerialBT.print(
        "SD full: "
    );

    SerialBT.println(
        sdFull
        ? "YES"
        : "NO"
    );

    // ========================================================
    // VIBRATION STATUS
    // ========================================================

    SerialBT.print(
        "Vibration HIGH: "
    );

    SerialBT.println(
        completedVibrationOnes
    );

    SerialBT.print(
        "Vibration LOW: "
    );

    SerialBT.println(
        completedVibrationZeros
    );

    SerialBT.print(
        "Vibration event: "
    );

    SerialBT.println(
        vibrationEvent
        ? "YES"
        : "NO"
    );

    // ========================================================
    // SD SPACE
    // ========================================================

    if (sdInitialized)
    {
        uint64_t total =
            SD.totalBytes();

        uint64_t used =
            SD.usedBytes();

        uint64_t freeSpace =
            total - used;

        SerialBT.print(
            "Free space: "
        );

        SerialBT.print(
            (double)freeSpace /
            1024.0 /
            1024.0,
            2
        );

        SerialBT.println(
            " MB"
        );
    }

    SerialBT.println(
        "============================"
    );
}

// ============================================================
// GY-521 / MPU6050 CONFIGURATION
// ============================================================

void initMPU6050()
{
    Serial.print(
        "Initializing MPU-6050... "
    );

    GYWire.beginTransmission(
        0x68
    );

    if (
        GYWire.endTransmission() != 0
    )
    {
        Serial.println(
            "FAILED"
        );

        return;
    }

    Serial.println(
        "OK"
    );

    // ========================================================
    // WAKE MPU6050
    // ========================================================

    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(
        0x6B
    );

    GYWire.write(
        0x00
    );

    GYWire.endTransmission();

    // ========================================================
    // ACCELEROMETER +/-8G
    // ========================================================

    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(
        0x1C
    );

    GYWire.write(
        0x10
    );

    GYWire.endTransmission();

    // ========================================================
    // GYROSCOPE +/-500 deg/s
    // ========================================================

    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(
        0x1B
    );

    GYWire.write(
        0x08
    );

    GYWire.endTransmission();

    // ========================================================
    // DIGITAL LOW PASS FILTER 44 Hz
    // ========================================================

    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(
        0x1A
    );

    GYWire.write(
        0x03
    );

    GYWire.endTransmission();

    Serial.println(
        "MPU6050 configured"
    );
}

// ============================================================
// READ MPU6050
// ============================================================

void readMPU6050()
{
    GYWire.beginTransmission(
        0x68
    );

    GYWire.write(
        0x3B
    );

    if (
        GYWire.endTransmission(
            false
        ) != 0
    )
    {
        return;
    }

    uint8_t received =
        GYWire.requestFrom(
            0x68,
            14
        );

    if (
        received != 14
    )
    {
        return;
    }

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

    // ========================================================
    // +/-8G
    // ========================================================

    accelX =
        rawAX / 4096.0f;

    accelY =
        rawAY / 4096.0f;

    accelZ =
        rawAZ / 4096.0f;

    // ========================================================
    // +/-500 deg/s
    // ========================================================

    gyroX =
        rawGX / 65.5f;

    gyroY =
        rawGY / 65.5f;

    gyroZ =
        rawGZ / 65.5f;

    // ========================================================
    // TEMPERATURE
    // ========================================================

    mpuTemperature =
        (rawMPUTemp / 340.0f) +
        36.53f;
}

// ============================================================
// VIBRATION MEASUREMENT
// ============================================================
//
// The SW-18010P is treated as a digital transition sensor.
//
// A transition starts a 100 ms measurement window.
// During that window every HIGH/LOW transition is counted.
//
// When the window finishes:
//
//   completedVibrationOnes
//   completedVibrationZeros
//
// receive the result.
//
// The live counters are then reset.
//
// The completed result is consumed by the next 100 Hz
// SensorData sample.
//
// After that sample is queued, the completed values are
// reset to zero.
//
// ============================================================

void measureVibration()
{
    int state =
        digitalRead(
            VIBRATION_PIN
        );

    // ========================================================
    // START NEW VIBRATION MEASUREMENT
    // ========================================================

    if (
        !vibrationMeasuring &&
        state != prevVibrationState
    )
    {
        vibrationMeasuring =
            true;

        vibrationOnes =
            0;

        vibrationZeros =
            0;

        vibrationStartTime =
            millis();

        vibrationEvent =
            false;
    }

    // ========================================================
    // COUNT TRANSITIONS
    // ========================================================

    if (vibrationMeasuring)
    {
        if (
            state !=
            prevVibrationState
        )
        {
            if (
                state == HIGH
            )
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

        // ====================================================
        // END 100 ms WINDOW
        // ====================================================

        if (
            millis() -
            vibrationStartTime >=
            VIBRATION_WINDOW_MS
        )
        {
            vibrationMeasuring =
                false;

            // Save the completed result.
            completedVibrationOnes =
                vibrationOnes;

            completedVibrationZeros =
                vibrationZeros;

            vibrationEvent =
                true;

            // Clear live counters.
            vibrationOnes =
                0;

            vibrationZeros =
                0;
        }
    }
    else
    {
        // ====================================================
        // NO ACTIVE VIBRATION MEASUREMENT
        // ====================================================

        prevVibrationState =
            state;
    }
}

// ============================================================
// SERIAL SENSOR OUTPUT
// ============================================================

void printSensorData()
{
    Serial.println();

    Serial.println(
        "================================================"
    );

    Serial.println(
        "                SENSOR DATA"
    );

    Serial.println(
        "================================================"
    );

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
    //
    // REMOVED.
    //
    // ========================================================

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
        completedVibrationOnes
    );

    Serial.print(
        "Transitions LOW: "
    );

    Serial.println(
        completedVibrationZeros
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