/*
 * =====================================================================
 *  Fetal Kick Detection Device - FINAL LOGGER (fetal_kick_logger5)
 * =====================================================================
 *  Based on fetal_kick_logger3. Same structure:
 *    sensorTask (core 1) -> sensorQueue -> sdTask (core 0) -> /SCAN_00N.csv
 *
 *  What changed from logger3
 *  -------------------------
 *   1. Vibration: 4 sensors on GPIO 32, 33, 34, 35 (was one on GPIO 27).
 *      One shared 100 ms window, one hardness value per sensor.
 *   2. MAX30100 is optional: if it does not answer at boot, heart_rate
 *      and spo2 are logged as 0 and everything else keeps running.
 *   3. Bluetooth removed. The ESP32 creates its own Wi-Fi network and
 *      serves a control page protected by a username and password.
 *   4. Pins: GY-521 on SDA 22 / SCL 23, SD on CS 19 / SCK 21 / MISO 5 / MOSI 18.
 *
 *  How to use
 *  ----------
 *   1. Connect a phone/PC to Wi-Fi "FetalKickLogger" (password below).
 *   2. Open http://192.168.4.1 and log in (username/password below).
 *   3. SD Init -> Start -> Stop -> download the CSV from the Files list.
 *
 *  CSV columns
 *  -----------
 *   scan_time_ms, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
 *   mpu_temperature, heart_rate, spo2, vibration_event,
 *   vib_s1, vib_s2, vib_s3, vib_s4
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include "MAX30100_PulseOximeter.h"

// ============================================================
// WIFI ACCESS POINT + WEB LOGIN   (CHANGE THESE)
// ============================================================

const char *AP_SSID      = "FetalKickLogger";
const char *AP_PASSWORD  = "kick12345";      // Wi-Fi password, minimum 8 characters

const char *WEB_USER     = "admin";          // web page login
const char *WEB_PASSWORD = "admin123";

WebServer server(80);

// ============================================================
// I2C PINS
// ============================================================

// GY-521 / MPU6050
#define GY_SDA 22
#define GY_SCL 23

// MAX30100
#define MAX_SDA 25
#define MAX_SCL 26

// ============================================================
// SD CARD
// ============================================================

#define SD_CS   19
#define SD_SCK  21
#define SD_MISO 5
#define SD_MOSI 18

SPIClass SD_SPI(VSPI);

const uint64_t MIN_FREE_SPACE = 16ULL * 1024ULL;

// ============================================================
// VIBRATION ARRAY
// ============================================================

#define VIBRATION_COUNT 4

// GPIO 34 and 35 are input-only and have no internal pull resistors.
const uint8_t VIBRATION_PINS[VIBRATION_COUNT] = {32, 33, 34, 35};

// ============================================================
// MPU6050
// ============================================================

#define MPU6050_ADDRESS 0x68

// ============================================================
// TIMING
// ============================================================

const uint32_t MPU_INTERVAL_MS = 10;          // 100 Hz
const uint32_t HEART_REPORT_INTERVAL_MS = 1000;
const uint32_t SERIAL_INTERVAL_MS = 1000;
const uint32_t SD_FLUSH_INTERVAL_MS = 1000;

// Vibration window
const uint32_t VIBRATION_WINDOW_MS = 100;

// ============================================================
// MAX30100 (OPTIONAL)
// ============================================================

PulseOximeter pox;

bool maxReady = false;      // true only if the sensor answered at boot

float heartRate = 0.0;
float spo2 = 0.0;

// ============================================================
// GY-521 I2C
// ============================================================

TwoWire GYWire = TwoWire(1);

// MPU6050 RAW DATA
int16_t rawAX = 0;
int16_t rawAY = 0;
int16_t rawAZ = 0;
int16_t rawGX = 0;
int16_t rawGY = 0;
int16_t rawGZ = 0;
int16_t rawMPUTemp = 0;

// MPU6050 CALCULATED DATA
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
float mpuTemperature = 0.0;

bool mpuReady = false;
uint8_t mpuWhoAmI = 0;

// ============================================================
// VIBRATION STATE
// ============================================================

int prevVibrationState[VIBRATION_COUNT];

bool vibrationMeasuring = false;
uint32_t vibrationStartTime = 0;
uint32_t vibrationCompletedTime = 0;

// Current 100 ms window: transitions counted per sensor (= hardness)
volatile int vibrationHardness[VIBRATION_COUNT] = {0, 0, 0, 0};

volatile bool vibrationEvent = false;

// Used to indicate that a new 100 ms window has completed
volatile bool vibrationWindowComplete = false;

// Last completed window, kept for the web page and serial output
volatile int lastVibration[VIBRATION_COUNT] = {0, 0, 0, 0};
volatile uint32_t lastVibrationTime = 0;

// ============================================================
// SYSTEM STATE
// ============================================================

volatile bool scanning = false;
volatile bool stopRequested = false;
volatile bool sdInitialized = false;
volatile bool sdFull = false;
volatile bool sdWriterBusy = false;
volatile bool sdReleaseRequested = false;
volatile bool scanFileOpen = false;
volatile bool webSdBusy = false;     // web page is listing or downloading files

// ============================================================
// SCAN
// ============================================================

uint32_t scanStartTime = 0;
uint32_t scanNumber = 0;
String currentScanFilename = "";
char scanFilenameShared[32] = "";    // copy that the web task can read safely

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
    int vibration[VIBRATION_COUNT];
    bool vibrationEvent;
};

// ============================================================
// FREE RTOS QUEUES
// ============================================================

QueueHandle_t sensorQueue;

enum SDCommandType
{
    SD_CMD_START,
    SD_CMD_INIT
};

struct SDCommand
{
    SDCommandType type;
};

QueueHandle_t sdCommandQueue;

// ============================================================
// MESSAGE LOG (shown on the web page, replaces Bluetooth replies)
// ============================================================

#define LOG_LINES 40
#define LOG_LINE_LENGTH 96

char logLines[LOG_LINES][LOG_LINE_LENGTH];
uint32_t logCount = 0;
SemaphoreHandle_t logMutex = NULL;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void initMPU6050();
bool readMPU6050();
void measureVibration();
void printSensorData();
void onBeatDetected();

void sensorTask(void *parameter);
void sdTask(void *parameter);
void webTask(void *parameter);

void processCommand(String command);
void logMessage(const String &message);

void initializeSDCard();
void startScan(File &currentFile);
void releaseSDCard();
bool writeSensorData(File &file, const SensorData &data);
void writeCSVHeader(File &file);
String getScanFilename(uint32_t number);
uint32_t findNextScanNumber();
void printSystemStatus();

bool requireLogin();
void handleRoot();
void handleStatus();
void handleCommand();
void handleFiles();
void handleDownload();
String jsonEscape(const char *text);

// ============================================================
// WEB PAGE
// ============================================================

const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Fetal Kick Logger</title>
<style>
body{font-family:sans-serif;margin:0;padding:12px;background:#f2f4f7;color:#1c2430}
h1{font-size:20px;margin:4px 0 12px}
h2{font-size:15px;margin:0 0 8px}
.card{background:#fff;border-radius:8px;padding:12px;margin-bottom:12px;box-shadow:0 1px 3px rgba(0,0,0,.15)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px}
.item{background:#f2f4f7;border-radius:6px;padding:8px}
.item b{display:block;font-size:11px;color:#5b6675;font-weight:normal}
.item span{font-size:15px}
button{font-size:15px;padding:10px 14px;margin:0 6px 6px 0;border:0;border-radius:6px;background:#1f6feb;color:#fff}
button.stop{background:#c62828}
button.plain{background:#5b6675}
table{width:100%;border-collapse:collapse;font-size:14px}
td,th{text-align:left;padding:6px;border-bottom:1px solid #e3e6ea}
pre{background:#10151c;color:#d6e2f0;padding:8px;border-radius:6px;height:180px;overflow:auto;font-size:12px;margin:0}
.ok{color:#1b7f3b}.bad{color:#c62828}
</style>
</head>
<body>
<h1>Fetal Kick Logger <small id="conn"></small></h1>

<div class="card">
<h2>Control</h2>
<button onclick="cmd('SD_INIT')">SD Init</button>
<button onclick="cmd('START')">Start</button>
<button class="stop" onclick="cmd('STOP')">Stop</button>
<button class="plain" onclick="cmd('SD_RELEASE')">SD Release</button>
</div>

<div class="card">
<h2>Status</h2>
<div class="grid">
<div class="item"><b>SD card</b><span id="sd">-</span></div>
<div class="item"><b>Scan</b><span id="scan">-</span></div>
<div class="item"><b>File</b><span id="file">-</span></div>
<div class="item"><b>Scan time</b><span id="t">-</span></div>
<div class="item"><b>Queue</b><span id="queue">-</span></div>
<div class="item"><b>MPU6050</b><span id="mpu">-</span></div>
<div class="item"><b>MAX30100</b><span id="max">-</span></div>
</div>
</div>

<div class="card">
<h2>Live values</h2>
<div class="grid">
<div class="item"><b>Accel (g)</b><span id="a">-</span></div>
<div class="item"><b>Gyro (deg/s)</b><span id="g">-</span></div>
<div class="item"><b>MPU temp (C)</b><span id="temp">-</span></div>
<div class="item"><b>Heart rate / SpO2</b><span id="hr">-</span></div>
<div class="item"><b>Last vibration hardness</b><span id="vib">-</span></div>
</div>
</div>

<div class="card">
<h2>Files</h2>
<button class="plain" onclick="files()">Refresh list</button>
<div id="files"></div>
</div>

<div class="card">
<h2>Messages</h2>
<pre id="log"></pre>
</div>

<script>
var since = 0;
function el(id){ return document.getElementById(id); }
function flag(id, on, yes, no){ var e = el(id); e.textContent = on ? yes : no; e.className = on ? 'ok' : 'bad'; }

function poll(){
  fetch('/status?since=' + since).then(function(r){ return r.json(); }).then(function(j){
    el('conn').textContent = '';
    flag('sd', j.sd, j.sdFull ? 'FULL' : 'Ready', 'Not initialised');
    flag('scan', j.scanning, 'RUNNING', j.fileOpen ? 'Finishing...' : 'Stopped');
    el('file').textContent = j.file || '-';
    el('t').textContent = j.scanning ? (j.t / 1000).toFixed(1) + ' s' : '-';
    el('queue').textContent = j.queue;
    flag('mpu', j.mpu, 'OK', 'Not responding');
    flag('max', j.max, 'OK', 'Not found (logging 0)');
    el('a').textContent = j.a.join('  ');
    el('g').textContent = j.g.join('  ');
    el('temp').textContent = j.temp;
    el('hr').textContent = j.hr + ' bpm / ' + j.spo2 + ' %';
    var v = 's1: ' + j.vib[0] + ', s2: ' + j.vib[1] + ', s3: ' + j.vib[2] + ', s4: ' + j.vib[3];
    el('vib').textContent = j.vibAge < 0 ? 'none yet' : v + ' (' + j.vibAge + ' s ago)';
    if (j.seq < since) { el('log').textContent = ''; }
    var log = el('log');
    for (var i = 0; i < j.log.length; i++) { log.textContent += j.log[i] + '\n'; }
    if (j.log.length) { log.scrollTop = log.scrollHeight; }
    since = j.seq;
  }).catch(function(){ el('conn').textContent = '(connection lost)'; });
}

function cmd(c){
  fetch('/cmd?c=' + c, {method: 'POST'}).then(function(){
    poll();
    if (c == 'STOP' || c == 'SD_INIT') { setTimeout(files, 2000); }
  });
}

function files(){
  fetch('/files').then(function(r){
    if (!r.ok) { return r.text().then(function(t){ el('files').textContent = t; }); }
    return r.json().then(function(list){
      if (!list.length) { el('files').textContent = 'No scan files found'; return; }
      var h = '<table><tr><th>File</th><th>Size</th><th></th></tr>';
      for (var i = 0; i < list.length; i++) {
        h += '<tr><td>' + list[i].name + '</td><td>' + (list[i].size / 1024).toFixed(1) + ' KB</td>' +
             '<td><a href="/download?name=' + encodeURIComponent(list[i].name) + '">Download</a></td></tr>';
      }
      el('files').innerHTML = h + '</table>';
    });
  });
}

setInterval(poll, 1000);
poll();
</script>
</body>
</html>
)HTMLPAGE";

// ============================================================
// MAX30100 CALLBACK
// ============================================================

void onBeatDetected()
{
    // Keep callback empty and extremely fast.
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(921600);
    delay(100);

    logMutex = xSemaphoreCreateMutex();

    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32 SENSOR DATA LOGGER");
    Serial.println("DUAL CORE + WIFI WEB UI + SD");
    Serial.println("========================================");

    // --------------------------------------------------------
    // MAX30100 (OPTIONAL)
    // --------------------------------------------------------

    Serial.println();
    Serial.println("Initializing MAX30100...");

    Wire.begin(MAX_SDA, MAX_SCL);
    Wire.setClock(400000);

    if (!pox.begin())
    {
        maxReady = false;
        Serial.println("MAX30100 NOT FOUND - heart_rate and spo2 will be 0");
    }
    else
    {
        maxReady = true;
        Serial.println("MAX30100 OK");
        pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
        pox.setOnBeatDetectedCallback(onBeatDetected);
    }

    // --------------------------------------------------------
    // GY-521
    // --------------------------------------------------------

    Serial.println();
    Serial.println("Initializing GY-521 / MPU6050...");

    GYWire.begin(GY_SDA, GY_SCL, 100000);
    GYWire.setClock(100000);

    Serial.printf("GY-521 I2C: SDA=%d SCL=%d\n", GY_SDA, GY_SCL);

    initMPU6050();

    // --------------------------------------------------------
    // VIBRATION ARRAY
    // --------------------------------------------------------

    for (int i = 0; i < VIBRATION_COUNT; i++)
    {
        pinMode(VIBRATION_PINS[i], INPUT);
        prevVibrationState[i] = digitalRead(VIBRATION_PINS[i]);
        vibrationHardness[i] = 0;
    }

    vibrationMeasuring = false;
    vibrationStartTime = millis();
    vibrationEvent = false;

    Serial.println();
    Serial.println("Vibration array ready (GPIO 32, 33, 34, 35)");

    // --------------------------------------------------------
    // SD SPI
    // --------------------------------------------------------

    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    // --------------------------------------------------------
    // QUEUES
    // --------------------------------------------------------

    sensorQueue = xQueueCreate(300, sizeof(SensorData));
    sdCommandQueue = xQueueCreate(10, sizeof(SDCommand));

    if (sensorQueue == NULL)
    {
        Serial.println("ERROR: Sensor queue failed");
    }

    if (sdCommandQueue == NULL)
    {
        Serial.println("ERROR: SD command queue failed");
    }

    // --------------------------------------------------------
    // WIFI ACCESS POINT + WEB SERVER
    // --------------------------------------------------------

    Serial.println();
    Serial.println("Starting Wi-Fi access point...");

    WiFi.mode(WIFI_AP);

    if (!WiFi.softAP(AP_SSID, AP_PASSWORD))
    {
        Serial.println("Wi-Fi AP FAILED");
    }
    else
    {
        Serial.print("Wi-Fi AP OK: ");
        Serial.println(AP_SSID);
        Serial.print("Open http://");
        Serial.println(WiFi.softAPIP());
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/cmd", HTTP_POST, handleCommand);
    server.on("/files", HTTP_GET, handleFiles);
    server.on("/download", HTTP_GET, handleDownload);
    server.begin();

    // --------------------------------------------------------
    // SENSOR TASK - CORE 1
    // --------------------------------------------------------

    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 3, NULL, 1);

    // --------------------------------------------------------
    // SD TASK - CORE 0
    // --------------------------------------------------------

    xTaskCreatePinnedToCore(sdTask, "SDTask", 8192, NULL, 2, NULL, 0);

    // --------------------------------------------------------
    // WEB TASK - CORE 0 (kept off the sensor core)
    // --------------------------------------------------------

    xTaskCreatePinnedToCore(webTask, "WebTask", 8192, NULL, 1, NULL, 0);

    // --------------------------------------------------------
    // READY
    // --------------------------------------------------------

    Serial.println();
    Serial.println("========================================");
    Serial.println("SYSTEM READY");
    Serial.println("========================================");

    logMessage("SYSTEM READY");
    logMessage(maxReady ? "MAX30100: OK" : "MAX30100: NOT FOUND (logging 0)");
    logMessage(mpuReady ? "MPU6050: OK" : "MPU6050: NOT RESPONDING");
}

// ============================================================
// MAIN LOOP
// ============================================================

// Nothing runs here: the web server has its own task on core 0 so it
// never competes with sensorTask on core 1.
void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ============================================================
// WEB TASK - CORE 0
// ============================================================

void webTask(void *parameter)
{
    while (true)
    {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ============================================================
// COMMAND PROCESSOR (same commands logger3 took over Bluetooth)
// ============================================================

void processCommand(String command)
{
    command.trim();

    Serial.print("WEB COMMAND: ");
    Serial.println(command);

    // SD_INIT
    if (command.equalsIgnoreCase("SD_INIT"))
    {
        SDCommand cmd;
        cmd.type = SD_CMD_INIT;
        xQueueSend(sdCommandQueue, &cmd, 0);
    }

    // START
    else if (command.equalsIgnoreCase("START"))
    {
        SDCommand cmd;
        cmd.type = SD_CMD_START;
        xQueueSend(sdCommandQueue, &cmd, 0);
    }

    // STOP
    else if (command.equalsIgnoreCase("STOP"))
    {
        if (!scanning)
        {
            logMessage("NO SCAN IN PROGRESS");
        }
        else
        {
            stopRequested = true;
            logMessage("STOP REQUESTED");
        }
    }

    // SD_RELEASE
    else if (command.equalsIgnoreCase("SD_RELEASE"))
    {
        if (scanning)
        {
            logMessage("SCAN IN PROGRESS");
            logMessage("STOP SCAN FIRST");
        }
        else
        {
            sdReleaseRequested = true;
            logMessage("SD RELEASE REQUESTED");
        }
    }

    // STATUS
    else if (command.equalsIgnoreCase("STATUS"))
    {
        printSystemStatus();
    }

    // UNKNOWN
    else
    {
        logMessage("UNKNOWN COMMAND");
    }
}

// ============================================================
// MESSAGE LOG (Serial + web page)
// ============================================================

void logMessage(const String &message)
{
    Serial.println(message);

    if (logMutex == NULL)
    {
        return;
    }

    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        char *line = logLines[logCount % LOG_LINES];
        strncpy(line, message.c_str(), LOG_LINE_LENGTH - 1);
        line[LOG_LINE_LENGTH - 1] = '\0';
        logCount++;
        xSemaphoreGive(logMutex);
    }
}

// ============================================================
// WEB HANDLERS
// ============================================================

bool requireLogin()
{
    if (!server.authenticate(WEB_USER, WEB_PASSWORD))
    {
        server.requestAuthentication();
        return false;
    }
    return true;
}

void handleRoot()
{
    if (!requireLogin()) return;
    server.send_P(200, "text/html", INDEX_HTML);
}

String jsonEscape(const char *text)
{
    String out = "";
    for (const char *p = text; *p; p++)
    {
        if (*p == '"' || *p == '\\')
        {
            out += '\\';
            out += *p;
        }
        else if ((uint8_t)*p >= 0x20)
        {
            out += *p;
        }
    }
    return out;
}

void handleStatus()
{
    if (!requireLogin()) return;

    uint32_t since = 0;
    if (server.hasArg("since"))
    {
        since = strtoul(server.arg("since").c_str(), NULL, 10);
    }

    uint32_t now = millis();
    long vibAge = (lastVibrationTime == 0) ? -1 : (long)((now - lastVibrationTime) / 1000);

    char buf[640];
    snprintf(
        buf, sizeof(buf),
        "{\"sd\":%d,\"scanning\":%d,\"fileOpen\":%d,\"sdFull\":%d,\"mpu\":%d,\"max\":%d,"
        "\"file\":\"%s\",\"queue\":%u,\"t\":%lu,"
        "\"a\":[%.3f,%.3f,%.3f],\"g\":[%.2f,%.2f,%.2f],\"temp\":%.1f,"
        "\"hr\":%.1f,\"spo2\":%.1f,\"vib\":[%d,%d,%d,%d],\"vibAge\":%ld,",
        sdInitialized ? 1 : 0, scanning ? 1 : 0, scanFileOpen ? 1 : 0, sdFull ? 1 : 0,
        mpuReady ? 1 : 0, maxReady ? 1 : 0,
        scanFilenameShared,
        (unsigned)uxQueueMessagesWaiting(sensorQueue),
        (unsigned long)(scanning ? (now - scanStartTime) : 0),
        accelX, accelY, accelZ, gyroX, gyroY, gyroZ, mpuTemperature,
        heartRate, spo2,
        (int)lastVibration[0], (int)lastVibration[1], (int)lastVibration[2], (int)lastVibration[3],
        vibAge
    );

    String json = buf;
    json += "\"log\":[";

    uint32_t total = 0;

    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        total = logCount;

        uint32_t first = since;
        if (first > total) first = 0;                       // device was restarted
        if (total - first > LOG_LINES) first = total - LOG_LINES;

        for (uint32_t i = first; i < total; i++)
        {
            if (i != first) json += ",";
            json += "\"";
            json += jsonEscape(logLines[i % LOG_LINES]);
            json += "\"";
        }

        xSemaphoreGive(logMutex);
    }
    else
    {
        total = since;
    }

    json += "],\"seq\":";
    json += String(total);
    json += "}";

    server.send(200, "application/json", json);
}

void handleCommand()
{
    if (!requireLogin()) return;

    if (!server.hasArg("c"))
    {
        server.send(400, "text/plain", "missing command");
        return;
    }

    processCommand(server.arg("c"));
    server.send(200, "text/plain", "OK");
}

// File list for the web page (replaces LIST)
void handleFiles()
{
    if (!requireLogin()) return;

    webSdBusy = true;

    if (!sdInitialized || scanning || scanFileOpen)
    {
        webSdBusy = false;
        server.send(409, "text/plain",
                    !sdInitialized ? "SD not initialised - press SD Init"
                                   : "Stop the scan to see files");
        return;
    }

    File root = SD.open("/");

    if (!root)
    {
        webSdBusy = false;
        server.send(500, "text/plain", "ERROR: CANNOT OPEN SD ROOT");
        return;
    }

    String json = "[";
    bool firstEntry = true;

    File file = root.openNextFile();

    while (file)
    {
        if (!file.isDirectory())
        {
            String name = file.name();

            if (name.endsWith(".csv") || name.endsWith(".CSV"))
            {
                if (!name.startsWith("/")) name = "/" + name;

                if (!firstEntry) json += ",";
                firstEntry = false;

                json += "{\"name\":\"";
                json += jsonEscape(name.c_str());
                json += "\",\"size\":";
                json += String((unsigned long)file.size());
                json += "}";
            }
        }

        file.close();
        file = root.openNextFile();
    }

    root.close();
    webSdBusy = false;

    json += "]";
    server.send(200, "application/json", json);
}

// File download for the web page (replaces READ <index>)
void handleDownload()
{
    if (!requireLogin()) return;

    String name = server.arg("name");
    if (!name.startsWith("/")) name = "/" + name;

    // Only CSV files in the card's root folder
    if (name.indexOf("..") >= 0 ||
        name.indexOf('/', 1) >= 0 ||
        !(name.endsWith(".csv") || name.endsWith(".CSV")))
    {
        server.send(400, "text/plain", "INVALID FILE NAME");
        return;
    }

    webSdBusy = true;

    if (!sdInitialized || scanning || scanFileOpen)
    {
        webSdBusy = false;
        server.send(409, "text/plain", "Stop the scan first (and run SD Init)");
        return;
    }

    File file = SD.open(name, FILE_READ);

    if (!file || file.isDirectory())
    {
        if (file) file.close();
        webSdBusy = false;
        server.send(404, "text/plain", "ERROR: CANNOT OPEN FILE");
        return;
    }

    server.sendHeader("Content-Disposition",
                      "attachment; filename=\"" + name.substring(1) + "\"");
    server.streamFile(file, "text/csv");

    file.close();
    webSdBusy = false;
}

// ============================================================
// SENSOR TASK - CORE 1
// ============================================================

void sensorTask(void *parameter)
{
    uint32_t lastMPU = millis();
    uint32_t lastHeart = millis();
    uint32_t lastSerial = millis();

    while (true)
    {
        uint32_t now = millis();

        // ----------------------------------------------------
        // MAX30100 - MUST RUN CONTINUOUSLY (only if present)
        // ----------------------------------------------------

        if (maxReady)
        {
            pox.update();
        }

        // ----------------------------------------------------
        // VIBRATION ARRAY - NON-BLOCKING
        // ----------------------------------------------------

        measureVibration();

        // ----------------------------------------------------
        // MPU6050 - 100 Hz
        // ----------------------------------------------------

        if (now - lastMPU >= MPU_INTERVAL_MS)
        {
            lastMPU += MPU_INTERVAL_MS;

            readMPU6050();

            // CREATE LOG SAMPLE
            if (scanning)
            {
                SensorData data;

                data.scanTime = now - scanStartTime;

                data.accelX = accelX;
                data.accelY = accelY;
                data.accelZ = accelZ;

                data.gyroX = gyroX;
                data.gyroY = gyroY;
                data.gyroZ = gyroZ;

                data.mpuTemperature = mpuTemperature;

                data.heartRate = heartRate;
                data.spo2 = spo2;

                for (int i = 0; i < VIBRATION_COUNT; i++)
                {
                    data.vibration[i] = vibrationHardness[i];
                }

                data.vibrationEvent = vibrationEvent;

                if (xQueueSend(sensorQueue, &data, 0) != pdTRUE)
                {
                    Serial.println("WARNING: SENSOR QUEUE FULL");
                }

                // Event is only for this sample.
                vibrationEvent = false;
            }
        }

        // ----------------------------------------------------
        // HEART RATE (0 when the MAX30100 is not present)
        // ----------------------------------------------------

        if (now - lastHeart >= HEART_REPORT_INTERVAL_MS)
        {
            lastHeart += HEART_REPORT_INTERVAL_MS;

            if (maxReady)
            {
                heartRate = pox.getHeartRate();
                spo2 = pox.getSpO2();
            }
            else
            {
                heartRate = 0.0;
                spo2 = 0.0;
            }
        }

        // ----------------------------------------------------
        // STOP REQUEST
        // ----------------------------------------------------

        if (stopRequested && scanning)
        {
            scanning = false;
            stopRequested = false;
            logMessage("SCAN STOPPING");
        }

        // ----------------------------------------------------
        // SERIAL SENSOR OUTPUT - ONLY DURING SCAN
        // ----------------------------------------------------

        if (scanning && now - lastSerial >= SERIAL_INTERVAL_MS)
        {
            lastSerial += SERIAL_INTERVAL_MS;
            printSensorData();
        }

        // YIELD
        taskYIELD();
    }
}

// ============================================================
// SD TASK - CORE 0
// ============================================================

void sdTask(void *parameter)
{
    File currentFile;

    uint32_t lastFlush = millis();
    bool stopMessageSent = false;

    while (true)
    {
        // ----------------------------------------------------
        // HANDLE COMMANDS
        // ----------------------------------------------------

        SDCommand cmd;

        if (xQueueReceive(sdCommandQueue, &cmd, 0) == pdTRUE)
        {
            // SD INIT
            if (cmd.type == SD_CMD_INIT)
            {
                if (scanning)
                {
                    logMessage("ERROR: SCAN IN PROGRESS");
                }
                else if (sdInitialized)
                {
                    logMessage("SD CARD ALREADY INITIALIZED");
                }
                else
                {
                    initializeSDCard();
                }
            }

            // START
            else if (cmd.type == SD_CMD_START)
            {
                if (!sdInitialized)
                {
                    logMessage("ERROR: SD NOT INITIALIZED");
                }
                else if (scanning)
                {
                    logMessage("ERROR: SCAN ALREADY RUNNING");
                }
                else if (scanFileOpen)
                {
                    logMessage("ERROR: PREVIOUS FILE STILL OPEN");
                }
                else if (webSdBusy)
                {
                    logMessage("ERROR: FILE DOWNLOAD IN PROGRESS");
                }
                else if (sdFull)
                {
                    logMessage("ERROR: SD CARD FULL");
                }
                else
                {
                    startScan(currentFile);

                    if (currentFile)
                    {
                        scanFileOpen = true;
                        stopMessageSent = false;
                        lastFlush = millis();
                    }
                }
            }
        }

        // ----------------------------------------------------
        // WRITE SENSOR DATA
        // ----------------------------------------------------

        if (scanFileOpen && currentFile)
        {
            SensorData data;

            if (xQueueReceive(sensorQueue, &data, 0) == pdTRUE)
            {
                sdWriterBusy = true;

                if (!writeSensorData(currentFile, data))
                {
                    sdFull = true;
                    scanning = false;
                    stopRequested = false;

                    logMessage("SD WRITE FAILED");
                    logMessage("SCAN STOPPED");

                    currentFile.flush();
                    currentFile.close();

                    scanFileOpen = false;
                    currentScanFilename = "";
                    scanFilenameShared[0] = '\0';

                    sdWriterBusy = false;
                    stopMessageSent = true;
                }
                else
                {
                    sdWriterBusy = false;
                }
            }
        }

        // ----------------------------------------------------
        // PERIODIC FLUSH
        // ----------------------------------------------------

        if (scanFileOpen && currentFile &&
            millis() - lastFlush >= SD_FLUSH_INTERVAL_MS)
        {
            currentFile.flush();
            lastFlush = millis();
        }

        // ----------------------------------------------------
        // STOPPED SCAN - drain remaining samples first
        // ----------------------------------------------------

        if (scanFileOpen && !scanning)
        {
            if (uxQueueMessagesWaiting(sensorQueue) == 0 && !sdWriterBusy)
            {
                currentFile.flush();
                currentFile.close();

                scanFileOpen = false;

                String finishedFile = currentScanFilename;
                currentScanFilename = "";
                scanFilenameShared[0] = '\0';

                logMessage("SCAN STOPPED");
                logMessage("FILE: " + finishedFile);
                stopMessageSent = true;
            }
            else
            {
                if (!stopMessageSent)
                {
                    logMessage("DATA WRITING IN PROGRESS");
                    logMessage("FINISHING QUEUED DATA...");
                    stopMessageSent = true;
                }
            }
        }

        // ----------------------------------------------------
        // SD RELEASE
        // ----------------------------------------------------

        if (sdReleaseRequested)
        {
            if (scanning)
            {
                logMessage("SCAN IN PROGRESS");
                logMessage("STOP SCAN FIRST");
                sdReleaseRequested = false;
            }
            else if (scanFileOpen || sdWriterBusy || webSdBusy ||
                     uxQueueMessagesWaiting(sensorQueue) > 0)
            {
                static uint32_t lastWaitMessage = 0;

                if (millis() - lastWaitMessage >= 1000)
                {
                    logMessage("DATA WRITING IN PROGRESS");
                    logMessage("PLEASE WAIT...");
                    lastWaitMessage = millis();
                }
            }
            else
            {
                releaseSDCard();
                sdReleaseRequested = false;
            }
        }

        // TASK DELAY
        vTaskDelay(1);
    }
}

// ============================================================
// SD INITIALIZATION
// ============================================================

void initializeSDCard()
{
    logMessage("INITIALIZING SD CARD...");

    if (SD.begin(SD_CS, SD_SPI, 20000000))
    {
        sdInitialized = true;
        sdFull = false;

        uint64_t total = SD.totalBytes();
        uint64_t used = SD.usedBytes();
        uint64_t freeSpace = total - used;

        logMessage("SD CARD INITIALIZED");
        logMessage("FREE SPACE: " + String((double)freeSpace / 1024.0 / 1024.0, 2) + " MB");
    }
    else
    {
        sdInitialized = false;
        logMessage("SD CARD INITIALIZATION FAILED");
    }
}

// ============================================================
// START SCAN
// ============================================================

void startScan(File &currentFile)
{
    uint64_t total = SD.totalBytes();
    uint64_t used = SD.usedBytes();
    uint64_t freeSpace = total - used;

    if (freeSpace < MIN_FREE_SPACE)
    {
        sdFull = true;
        logMessage("ERROR: SD CARD FULL");
        return;
    }

    // CLEAR OLD VIBRATION WINDOW
    for (int i = 0; i < VIBRATION_COUNT; i++)
    {
        vibrationHardness[i] = 0;
    }
    vibrationEvent = false;
    vibrationWindowComplete = false;

    // Drop any samples left over from a previous scan
    xQueueReset(sensorQueue);

    // FIND NEXT FILE
    scanNumber = findNextScanNumber();
    currentScanFilename = getScanFilename(scanNumber);

    // OPEN FILE
    currentFile = SD.open(currentScanFilename, FILE_WRITE);

    if (!currentFile)
    {
        currentScanFilename = "";
        logMessage("ERROR: CANNOT CREATE FILE");
        return;
    }

    strncpy(scanFilenameShared, currentScanFilename.c_str(), sizeof(scanFilenameShared) - 1);
    scanFilenameShared[sizeof(scanFilenameShared) - 1] = '\0';

    // HEADER
    writeCSVHeader(currentFile);
    currentFile.flush();

    // START TIMER
    scanStartTime = millis();
    scanning = true;
    stopRequested = false;
    sdFull = false;

    logMessage("SCAN STARTED");
    logMessage("FILE: " + currentScanFilename);
    logMessage("SAMPLING: 100 Hz");
}

// ============================================================
// CSV HEADER
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
        "vib_s1,"
        "vib_s2,"
        "vib_s3,"
        "vib_s4"
    );
}

// ============================================================
// WRITE SENSOR DATA
// ============================================================

bool writeSensorData(File &file, const SensorData &data)
{
    if (!file)
    {
        return false;
    }

    file.print(data.scanTime);
    file.print(",");
    file.print(data.accelX, 4);
    file.print(",");
    file.print(data.accelY, 4);
    file.print(",");
    file.print(data.accelZ, 4);
    file.print(",");
    file.print(data.gyroX, 3);
    file.print(",");
    file.print(data.gyroY, 3);
    file.print(",");
    file.print(data.gyroZ, 3);
    file.print(",");
    file.print(data.mpuTemperature, 2);
    file.print(",");
    file.print(data.heartRate, 1);
    file.print(",");
    file.print(data.spo2, 1);
    file.print(",");
    file.print(data.vibrationEvent ? 1 : 0);

    for (int i = 0; i < VIBRATION_COUNT; i++)
    {
        file.print(",");
        file.print(data.vibration[i]);
    }

    file.println();

    // CHECK CARD SPACE
    static uint32_t lastSpaceCheck = 0;

    if (millis() - lastSpaceCheck >= 1000)
    {
        lastSpaceCheck = millis();

        uint64_t total = SD.totalBytes();
        uint64_t used = SD.usedBytes();

        if (total > used)
        {
            uint64_t freeSpace = total - used;

            if (freeSpace < MIN_FREE_SPACE)
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    // FILE WRITE ERROR
    if (file.getWriteError())
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
        logMessage("ERROR: SCAN IN PROGRESS");
        return;
    }

    if (!sdInitialized)
    {
        logMessage("SD CARD NOT INITIALIZED");
        return;
    }

    SD.end();

    sdInitialized = false;
    sdFull = false;

    logMessage("SD CARD RELEASED");
    logMessage("SAFE TO REMOVE");
}

// ============================================================
// FILENAME
// ============================================================

String getScanFilename(uint32_t number)
{
    char filename[32];

    snprintf(filename, sizeof(filename), "/SCAN_%03lu.csv", (unsigned long)number);

    return String(filename);
}

// ============================================================
// FIND NEXT SCAN NUMBER
// ============================================================

uint32_t findNextScanNumber()
{
    uint32_t number = 1;

    while (true)
    {
        String filename = getScanFilename(number);

        if (!SD.exists(filename))
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
// STATUS (to Serial and the web message log)
// ============================================================

void printSystemStatus()
{
    logMessage("========== STATUS ==========");
    logMessage(String("SD: ") + (sdInitialized ? "INITIALIZED" : "NOT INITIALIZED"));
    logMessage(String("Scanning: ") + (scanning ? "YES" : "NO"));
    logMessage(String("MPU6050: ") + (mpuReady ? "READY" : "NOT READY"));
    logMessage(String("MAX30100: ") + (maxReady ? "READY" : "NOT FOUND"));
    logMessage(String("Writer: ") + (sdWriterBusy ? "BUSY" : "IDLE"));
    logMessage("Queue entries: " + String((unsigned)uxQueueMessagesWaiting(sensorQueue)));
    logMessage("Last vibration s1: " + String((int)lastVibration[0]) +
               ", s2: " + String((int)lastVibration[1]) +
               ", s3: " + String((int)lastVibration[2]) +
               ", s4: " + String((int)lastVibration[3]));
    logMessage(String("SD full: ") + (sdFull ? "YES" : "NO"));
    logMessage("============================");
}

// ============================================================
// MPU6050 INITIALIZATION
// ============================================================

void initMPU6050()
{
    Serial.print("Checking MPU6050 at 0x68... ");

    GYWire.beginTransmission(MPU6050_ADDRESS);
    uint8_t error = GYWire.endTransmission(true);

    if (error != 0)
    {
        Serial.print("FAILED, I2C error = ");
        Serial.println(error);
        Serial.printf("Check GY-521 VCC, GND, SDA=%d, SCL=%d\n", GY_SDA, GY_SCL);
        mpuReady = false;
        return;
    }

    Serial.println("DEVICE FOUND");

    // WHO_AM_I
    GYWire.beginTransmission(MPU6050_ADDRESS);
    GYWire.write(0x75);

    if (GYWire.endTransmission(false) != 0)
    {
        Serial.println("MPU6050 WHO_AM_I REQUEST FAILED");
        mpuReady = false;
        return;
    }

    uint8_t received = GYWire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)1, (uint8_t)true);

    if (received != 1)
    {
        Serial.println("MPU6050 WHO_AM_I READ FAILED");
        mpuReady = false;
        return;
    }

    uint8_t whoAmI = GYWire.read();
    mpuWhoAmI = whoAmI;

    Serial.print("MPU6050 WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI != 0x68 && whoAmI != 0x69)
    {
        Serial.println("WARNING: UNEXPECTED MPU6050 WHO_AM_I");
    }

    // RESET MPU6050
    GYWire.beginTransmission(MPU6050_ADDRESS);
    GYWire.write(0x6B);
    GYWire.write(0x80);
    GYWire.endTransmission(true);

    delay(100);

    // WAKE MPU6050
    GYWire.beginTransmission(MPU6050_ADDRESS);
    GYWire.write(0x6B);
    GYWire.write(0x00);
    GYWire.endTransmission(true);

    // ACCELEROMETER +/-8G   (register 0x1C, 0x10)
    GYWire.beginTransmission(MPU6050_ADDRESS);
    GYWire.write(0x1C);
    GYWire.write(0x10);
    GYWire.endTransmission(true);

    // GYROSCOPE +/-500 DEG/S   (register 0x1B, 0x08)
    GYWire.beginTransmission(MPU6050_ADDRESS);
    GYWire.write(0x1B);
    GYWire.write(0x08);
    GYWire.endTransmission(true);

    // DIGITAL LOW PASS FILTER   (0x03 = approximately 44 Hz)
    GYWire.beginTransmission(MPU6050_ADDRESS);
    GYWire.write(0x1A);
    GYWire.write(0x03);
    GYWire.endTransmission(true);

    delay(50);

    mpuReady = true;

    Serial.println("MPU6050 configured successfully");
}

// ============================================================
// MPU6050 READ
// ============================================================

bool readMPU6050()
{
    // REQUEST ACCELEROMETER REGISTER
    GYWire.beginTransmission(MPU6050_ADDRESS);
    GYWire.write(0x3B);

    if (GYWire.endTransmission(false) != 0)
    {
        static uint32_t lastErrorPrint = 0;

        if (millis() - lastErrorPrint >= 1000)
        {
            Serial.println("MPU READ FAILED: register request");
            lastErrorPrint = millis();
        }

        mpuReady = false;
        return false;
    }

    // READ 14 BYTES: ACCEL X/Y/Z, TEMP, GYRO X/Y/Z
    uint8_t received = GYWire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14, (uint8_t)true);

    if (received != 14)
    {
        static uint32_t lastErrorPrint = 0;

        if (millis() - lastErrorPrint >= 1000)
        {
            Serial.print("MPU READ FAILED: expected 14 bytes, got ");
            Serial.println(received);
            lastErrorPrint = millis();
        }

        mpuReady = false;
        return false;
    }

    uint8_t b[14];
    for (int i = 0; i < 14; i++)
    {
        b[i] = GYWire.read();
    }

    // RAW ACCELEROMETER
    rawAX = (int16_t)((b[0] << 8) | b[1]);
    rawAY = (int16_t)((b[2] << 8) | b[3]);
    rawAZ = (int16_t)((b[4] << 8) | b[5]);

    // RAW TEMPERATURE
    rawMPUTemp = (int16_t)((b[6] << 8) | b[7]);

    // RAW GYROSCOPE
    rawGX = (int16_t)((b[8] << 8) | b[9]);
    rawGY = (int16_t)((b[10] << 8) | b[11]);
    rawGZ = (int16_t)((b[12] << 8) | b[13]);

    // CONVERT ACCELEROMETER   (+/-8G = 4096 LSB/G)
    accelX = rawAX / 4096.0f;
    accelY = rawAY / 4096.0f;
    accelZ = rawAZ / 4096.0f;

    // CONVERT GYROSCOPE   (+/-500 deg/s = 65.5 LSB/(deg/s))
    gyroX = rawGX / 65.5f;
    gyroY = rawGY / 65.5f;
    gyroZ = rawGZ / 65.5f;

    // TEMPERATURE
    // MPU-6500 / MPU-9250 clones (WHO_AM_I 0x70 / 0x71 / 0x73) use a
    // different formula from the genuine MPU-6050.
    if (mpuWhoAmI == 0x70 || mpuWhoAmI == 0x71 || mpuWhoAmI == 0x73)
    {
        mpuTemperature = (rawMPUTemp / 333.87f) + 21.0f;
    }
    else
    {
        mpuTemperature = (rawMPUTemp / 340.0f) + 36.53f;
    }

    mpuReady = true;

    return true;
}

// ============================================================
// VIBRATION ARRAY - NON-BLOCKING 100 ms WINDOW
// ============================================================
//
// Same behaviour as logger3, for 4 sensors:
//  - a state change on ANY sensor opens one shared 100 ms window
//  - during the window, transitions are counted per sensor (hardness)
//  - when the window closes the counts are held so the next logged
//    sample carries them, then they return to zero

void measureVibration()
{
    uint32_t now = millis();

    int state[VIBRATION_COUNT];
    bool anyChange = false;

    for (int i = 0; i < VIBRATION_COUNT; i++)
    {
        state[i] = digitalRead(VIBRATION_PINS[i]);

        if (state[i] != prevVibrationState[i])
        {
            anyChange = true;
        }
    }

    // START NEW WINDOW WHEN ANY SENSOR CHANGES
    if (!vibrationMeasuring && anyChange)
    {
        vibrationMeasuring = true;
        vibrationStartTime = now;
        vibrationCompletedTime = 0;

        for (int i = 0; i < VIBRATION_COUNT; i++)
        {
            vibrationHardness[i] = 0;
        }

        vibrationEvent = false;
        vibrationWindowComplete = false;
    }

    // CURRENT WINDOW
    if (vibrationMeasuring)
    {
        // Count transitions per sensor
        for (int i = 0; i < VIBRATION_COUNT; i++)
        {
            if (state[i] != prevVibrationState[i])
            {
                vibrationHardness[i]++;
                prevVibrationState[i] = state[i];
            }
        }

        // END 100 ms WINDOW
        if (now - vibrationStartTime >= VIBRATION_WINDOW_MS)
        {
            vibrationMeasuring = false;
            vibrationEvent = true;
            vibrationWindowComplete = true;
            vibrationCompletedTime = now;

            for (int i = 0; i < VIBRATION_COUNT; i++)
            {
                lastVibration[i] = vibrationHardness[i];
            }
            lastVibrationTime = now;

            // Counters are NOT reset here: the next logged sample needs them.
        }
    }
    else
    {
        // Keep tracking the current physical state.
        for (int i = 0; i < VIBRATION_COUNT; i++)
        {
            prevVibrationState[i] = state[i];
        }
    }

    // RESET AFTER THE COMPLETED WINDOW HAS BEEN LOGGED
    // Give the 100 Hz sampler one period to pick the values up, then
    // return them to zero.
    if (!vibrationMeasuring && vibrationWindowComplete &&
        now - vibrationCompletedTime >= 10)
    {
        for (int i = 0; i < VIBRATION_COUNT; i++)
        {
            vibrationHardness[i] = 0;
        }

        vibrationEvent = false;
        vibrationWindowComplete = false;
    }
}

// ============================================================
// SERIAL SENSOR OUTPUT - ONLY CALLED WHILE SCANNING (1 Hz)
// ============================================================

void printSensorData()
{
    Serial.printf(
        "t=%lu ms | A %.3f %.3f %.3f g | G %.2f %.2f %.2f dps | T %.1f C | "
        "HR %.1f SpO2 %.1f | hardness s1: %d, s2: %d, s3: %d, s4: %d | queue %u\n",
        (unsigned long)(millis() - scanStartTime),
        accelX, accelY, accelZ,
        gyroX, gyroY, gyroZ,
        mpuTemperature,
        heartRate, spo2,
        (int)lastVibration[0], (int)lastVibration[1],
        (int)lastVibration[2], (int)lastVibration[3],
        (unsigned)uxQueueMessagesWaiting(sensorQueue)
    );
}
