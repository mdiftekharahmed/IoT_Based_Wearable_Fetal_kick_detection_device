#include<Wire.h>
#include<Adafruit_MPU6050.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BMP085.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.print("Sensor reading: ");
  Serial.println("=====================");

  //mpu6050
  if(!mpu.begin(0x68, &Wire))Serial.println("MPU6050 Not Found!");
  else {
    Serial.println("MPU6050 OK...");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  //BMP180
  if(!bmp.begin()) Serial.println("BMP180 not found!");
  else Serial.println("BMP180 ok...");

}

void loop() {
  // put your main code here, to run repeatedly:

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(&accel, &gyro, &temp);
  Serial.print("Acceleration (m/s^2): ");
  Serial.print(accel.acceleration.x);
  Serial.print(", ");
  Serial.print(accel.acceleration.y);
  Serial.print(", ");
  Serial.print(accel.acceleration.z);
  Serial.println("");

  Serial.print("Gyeroscope (rad/s): ");
  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.print(gyro.gyro.z);
  Serial.println("");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" C");

  Serial.print("GMP 180: ");
  Serial.print("Temperature: ");
  Serial.print(bmp.readTemperature());
  Serial.println(" pa");

  Serial.print("Altitde: ");
  Serial.print(bmp.readAltitude());
  Serial.println(" m");

  delay(500);
}
