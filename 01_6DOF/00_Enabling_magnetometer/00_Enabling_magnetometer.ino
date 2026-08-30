#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define MPU6050_ADDR 0x68

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Enabling MPU6050 I2C bypass...");

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Enable I2C bypass
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();

  delay(100);

  Serial.println("Scanning I2C bus...");
  Serial.println("------------------");

  for (byte address = 1; address < 127; address++) {

    Wire.beginTransmission(address);

    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
    }
  }

  Serial.println("------------------");
  Serial.println("Scan complete.");
}

void loop() {
}