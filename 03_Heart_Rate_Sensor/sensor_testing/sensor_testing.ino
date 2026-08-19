#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define SDA_PIN 25
#define SCL_PIN 26

#define REPORTING_PERIOD_MS 1000

PulseOximeter pox;
uint32_t tsLastReport = 0;

// Called whenever a heartbeat is detected
void onBeatDetected()
    {
        Serial.println("Beat Detected!");
    }

void setup()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.begin(115200);

    Serial.print("Initializing MAX30100... ");

    if (!pox.begin()) {
        Serial.println("FAILED");
        while (1);
    }

    Serial.println("System loading SUCCESS");

    // IMPORTANT:
    //module worked better at 7.6mA, so keep it there.
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    pox.setOnBeatDetectedCallback(onBeatDetected);

    Serial.println();
    Serial.println("Place your finger on the sensor.");
}

void loop()

{
    // Must be called as fast as possible
    pox.update();

    if (millis() - tsLastReport >= REPORTING_PERIOD_MS) {

        Serial.print("Heart Rate: ");
        Serial.print(pox.getHeartRate());
        Serial.print(" BPM");

        Serial.print("   SpO2: ");
        Serial.print(pox.getSpO2());
        Serial.println(" %");

        tsLastReport = millis();
    }
}