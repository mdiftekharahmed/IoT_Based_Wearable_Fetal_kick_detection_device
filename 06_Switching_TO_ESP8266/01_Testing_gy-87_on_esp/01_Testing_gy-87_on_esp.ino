#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS 1000

PulseOximeter pox;
uint32_t tsLastReport = 0;

// Called whenever a heartbeat is detected
void onBeatDetected()
{
    Serial.println("Beat!");
}

void setup()
{
    Serial.begin(9600);

    Serial.print("Initializing MAX30100... ");

    if (!pox.begin()) {
        Serial.println("FAILED");
        while (1);
    }

    Serial.println("SUCCESS");

    // IMPORTANT:
    // Your module worked better at 7.6mA, so keep it there.
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