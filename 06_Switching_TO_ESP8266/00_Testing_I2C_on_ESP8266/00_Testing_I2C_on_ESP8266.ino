#include <Wire.h>

void setup() {
  Serial.begin(9600);

  Wire.begin(D2, D1);   // SDA, SCL

  Serial.println("Scanning...");
}

void loop() {

  byte count = 0;

  for (byte addr = 1; addr < 127; addr++) {

    Wire.beginTransmission(addr);

    if (Wire.endTransmission() == 0) {
      Serial.print("Found: 0x");

      if (addr < 16)
        Serial.print("0");

      Serial.println(addr, HEX);

      count++;
    }
  }

  if (!count)
    Serial.println("No devices");

  Serial.println("----------------");

  delay(3000);
}