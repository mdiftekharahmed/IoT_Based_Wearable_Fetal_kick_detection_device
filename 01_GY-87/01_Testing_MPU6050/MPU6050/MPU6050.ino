
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mpu.begin();
  mpu.calcGyroOffsets(true);

  Serial.println("Ready");
}

void loop() {
  mpu.update();

  Serial.print("Accel X: ");
  Serial.print(mpu.getAccX());
  Serial.print("  Y: ");
  Serial.print(mpu.getAccY());
  Serial.print("  Z: ");
  Serial.println(mpu.getAccZ());

  delay(100);
}