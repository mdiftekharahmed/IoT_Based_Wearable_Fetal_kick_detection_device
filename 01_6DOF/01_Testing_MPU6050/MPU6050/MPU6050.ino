
// #include <Wire.h>
// #include <MPU6050_tockn.h>

// MPU6050 mpu(Wire);

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();

//   mpu.begin();
//   mpu.calcGyroOffsets(true);

//   Serial.println("Ready");
// }

// void loop() {
//   mpu.update();

//   Serial.print("Accel X: ");
//   Serial.print(mpu.getAccX());
//   Serial.print("  Y: ");
//   Serial.print(mpu.getAccY());
//   Serial.print("  Z: ");
//   Serial.println(mpu.getAccZ());

//   delay(100);
// }


// Arduino UNO I2C Scanner (SDA = A4, SCL = A5)
#include <Wire.h>

void setup() {
  Wire.begin(); // Standard AVR Wire library automatically uses A4 (SDA) and A5 (SCL)
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor connection
  Serial.println("\n--- Arduino UNO I2C Scanner ---");
  Serial.println("Connections: SDA -> A4, SCL -> A5\n");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning I2C bus...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Known standard addresses
      if (address == 0x68 || address == 0x69) {
        Serial.print("  <-- MPU6050 (Accel/Gyro)");
      } else if (address == 0x76 || address == 0x77) {
        Serial.print("  <-- BME280 / BMP280 / BME680 (Temp/Humidity/Press)");
      } else if (address == 0x40) {
        Serial.print("  <-- HTU21D / Si7021 (Temp/Humidity)");
      } else if (address == 0x44 || address == 0x45) {
        Serial.print("  <-- SHT30 / SHT31 (Temp/Humidity)");
      } else if (address == 0x38) {
        Serial.print("  <-- AHT10 / AHT20 (Temp/Humidity)");
      }
      
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found. Check 5V/3.3V, GND, A4, and A5 wiring!\n");
  } else {
    Serial.print("Scan complete. Found ");
    Serial.print(nDevices);
    Serial.println(" device(s).\n");
  }

  delay(5000); // Re-scan every 5 seconds
}