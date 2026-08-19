#include<Wire.h>
#include<Adafruit_MPU6050.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BMP085.h>
#include "MAX30100_PulseOximeter.h"

// I2C Communication
  #define GY_SDA 21
  #define GY_SCL 22

  #define MAX_SDA 25
  #define MAX_SCL 26

// Sensor PIN Definition
  #define Vibration 27

// HeartRate sensor vars
  #define REPORTING_PERIOD_MS 1000

// Vibration Sensor vars
  unsigned long startTime;
  int prevState;
  bool measuring = false;
  int ones=0, zeros = 0;
  const unsigned long measuringWindow=100; //measure for 100ms after first change

// Central time system
  // unsigned long startTime;
  unsigned long gy87Time;
  uint32_t tsLastReport = 0;

// Function Definition
  void measureVibration();
  void measure6DOF();
  void measureHeart();

// GY087 Sensor Objects
  Adafruit_MPU6050 mpu;
  Adafruit_BMP085 bmp;

//Heartrate
  PulseOximeter pox;

// Separate I2C bus for GY-87
  TwoWire GYWire = TwoWire(1);


// Custom function for GMP180
  // BMP180 calibration data
  int16_t AC1, AC2, AC3, BMP_B1, B2, MB, MC, MD;
  uint16_t AC4, AC5, AC6;

  float bmpTemperature = 0;
  float bmpPressure = 0;
  float bmpAltitude = 0;

  // Calibration function
  void initBMP180(){
      uint8_t data[2];

      // Read calibration coefficients
      GYWire.beginTransmission(0x77);
      GYWire.write(0xAA);
      GYWire.endTransmission();

      GYWire.requestFrom(0x77, 22);

      AC1 = (GYWire.read() << 8) | GYWire.read();
      AC2 = (GYWire.read() << 8) | GYWire.read();
      AC3 = (GYWire.read() << 8) | GYWire.read();

      AC4 = (GYWire.read() << 8) | GYWire.read();
      AC5 = (GYWire.read() << 8) | GYWire.read();
      AC6 = (GYWire.read() << 8) | GYWire.read();

      BMP_B1 = (GYWire.read() << 8) | GYWire.read();
      B2 = (GYWire.read() << 8) | GYWire.read();

      MB = (GYWire.read() << 8) | GYWire.read();
      MC = (GYWire.read() << 8) | GYWire.read();
      MD = (GYWire.read() << 8) | GYWire.read();

      Serial.println("BMP180 calibration loaded");
  }


void onBeatDetected()
    {
        Serial.println("Beat Detected!");
    }


void setup() {
  // global
  Serial.begin(115200);
  Serial.println("Starting up the system...");

  // =========================
  // MAX30100 I2C
  // =========================
  Wire.begin(MAX_SDA, MAX_SCL);
  Wire.setClock(100000);

  // HeartRate
  Serial.print("Initializing MAX30100... ");

  if (!pox.begin()) {
      Serial.println("FAILED");
      while (1);
  }

  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  Serial.println();
  Serial.println("You can now place your finger on the sensor.");


  // =========================
  // GY-87 I2C
  // =========================
  GYWire.begin(GY_SDA, GY_SCL);
  GYWire.setClock(100000);

  Serial.println();
  Serial.println("Initializing GY-87...");


  // =========================
  // MPU6050
  // =========================

  Serial.print("Initializing MPU6050... ");

  GYWire.beginTransmission(0x68);
  uint8_t error = GYWire.endTransmission();

  if (error != 0)
  {
      Serial.println("FAILED");
  }
  else
  {
      Serial.println("SUCCESS");

      // Wake MPU6050
      GYWire.beginTransmission(0x68);
      GYWire.write(0x1C);
      GYWire.write(0x10);
      GYWire.endTransmission();

      Serial.println("MPU6050 initialized manually");
  }


  // =========================
  // BMP180
  // =========================

  Serial.print("Initializing BMP180... ");

  GYWire.beginTransmission(0x77);

  if (GYWire.endTransmission() == 0)
  {
      Serial.println("SUCCESS");
      initBMP180();
  }
  else
  {
      Serial.println("FAILED");
  }


  // Pins
  pinMode(Vibration, INPUT);
  Serial.println("Vibration pin ready!.....");


  gy87Time = millis();
  tsLastReport = millis();

}




void loop() {
  pox.update();

  measureVibration();
  measure6DOF();
  measureHeart();

}


void measureVibration(){
  int state = digitalRead(Vibration);

  if(!measuring && state!=prevState){
    measuring = true;
    ones = 0;
    zeros = 0;
    startTime = millis();
  }

  if(measuring){
    if(state!=prevState){
      if(state==HIGH)
        ones++;
      else
        zeros++;

      prevState = state;
    }

    if(millis()-startTime >= measuringWindow){
      measuring = false;

      Serial.print("Vibration:: ");
      Serial.print(ones);
      Serial.print(":");
      Serial.println(zeros);

      prevState = digitalRead(Vibration);
    }
  }
  else
    prevState = state;

}

void measureHeart(){
    // Must be called as fast as possible
    // pox.update();

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

void measure6DOF(){
    if (millis() - gy87Time < 500)
        return;


    // =====================================================
    // MPU6050
    // =====================================================

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mpuTemp;


    // Start reading from register 0x3B
    GYWire.beginTransmission(0x68);
    GYWire.write(0x3B);
    GYWire.endTransmission(false);

    GYWire.requestFrom(0x68, 14);


    if (GYWire.available() >= 14)
    {
        ax = (GYWire.read() << 8) | GYWire.read();
        ay = (GYWire.read() << 8) | GYWire.read();
        az = (GYWire.read() << 8) | GYWire.read();

        mpuTemp = (GYWire.read() << 8) | GYWire.read();

        gx = (GYWire.read() << 8) | GYWire.read();
        gy = (GYWire.read() << 8) | GYWire.read();
        gz = (GYWire.read() << 8) | GYWire.read();


        // MPU6050 configured for ±8G
        float accelX = ax / 4096.0;
        float accelY = ay / 4096.0;
        float accelZ = az / 4096.0;


        // MPU6050 configured for ±500°/s
        float gyroX = gx / 65.5;
        float gyroY = gy / 65.5;
        float gyroZ = gz / 65.5;


        // MPU6050 temperature
        float temperature = (mpuTemp / 340.0) + 36.53;


        Serial.println("=====================================================");


        Serial.print("Acceleration (g): ");
        Serial.print(accelX);
        Serial.print(", ");
        Serial.print(accelY);
        Serial.print(", ");
        Serial.println(accelZ);


        Serial.print("Gyroscope (deg/s): ");
        Serial.print(gyroX);
        Serial.print(", ");
        Serial.print(gyroY);
        Serial.print(", ");
        Serial.println(gyroZ);


        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" C");
    }


    // =====================================================
    // BMP180
    // =====================================================

    // -------- Read temperature --------

    GYWire.beginTransmission(0x77);
    GYWire.write(0xF4);
    GYWire.write(0x2E);
    GYWire.endTransmission();

    delay(5);

    GYWire.beginTransmission(0x77);
    GYWire.write(0xF6);
    GYWire.endTransmission(false);

    GYWire.requestFrom(0x77, 2);

    int32_t UT = (GYWire.read() << 8) | GYWire.read();


    // -------- Calculate temperature --------

    int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;

    int32_t X2 = ((int32_t)MC << 11) /
                (X1 + MD);

    int32_t B5 = X1 + X2;

    float bmpTemp = ((B5 + 8) >> 4) / 10.0;


    // -------- Start pressure conversion --------

    GYWire.beginTransmission(0x77);
    GYWire.write(0xF4);
    GYWire.write(0x34);   // OSS = 0
    GYWire.endTransmission();

    delay(5);


    // -------- Read pressure --------

    GYWire.beginTransmission(0x77);
    GYWire.write(0xF6);
    GYWire.endTransmission(false);

    GYWire.requestFrom(0x77, 3);

    int32_t MSB = GYWire.read();
    int32_t LSB = GYWire.read();
    int32_t XLSB = GYWire.read();

    int32_t UP = ((MSB << 16) + (LSB << 8) + XLSB) >> 8;


    // -------- Calculate pressure --------

    X1 = ((B5 - 4000) * (int32_t)AC2) >> 11;

    X2 = ((int32_t)BMP_B1 * ((X1 * X1) >> 12)) >> 16;

    int32_t X3 = X1 + X2;

    int32_t B3 = ((((int32_t)AC1 * 4 + X3) + 2) >> 2);

    X1 = ((int32_t)AC3 * B5) >> 13;

    X2 = ((int32_t)B2 * ((X1 * X1) >> 12)) >> 16;

    X3 = ((X1 + X2) + 2) >> 2;

    uint32_t B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;

    uint32_t B7 = ((uint32_t)UP - B3) * 50000;

    int32_t pressure;

    if (B7 < 0x80000000)
        pressure = (B7 * 2) / B4;
    else
        pressure = (B7 / B4) * 2;

    X1 = (pressure >> 8) * (pressure >> 8);

    X1 = (X1 * 3038) >> 16;

    X2 = (-7357 * pressure) >> 16;

    pressure = pressure +
              ((X1 + X2 + 3791) >> 4);


    // =====================================================
    // BMP180 Output
    // =====================================================

    float altitude =
        44330.0 *
        (1.0 - pow(pressure / 101325.0, 0.1903));


    Serial.print("BMP180 Temperature: ");
    Serial.print(bmpTemp);
    Serial.println(" C");


    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" Pa");


    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");


    Serial.println("=====================================================");


    gy87Time = millis();
}



// void measure6DOF(){
//     if(millis()-gy87Time <500)return;

//     sensors_event_t accel;
//     sensors_event_t gyro;
//     sensors_event_t temp;

//     Serial.println("=====================================================");

//     mpu.getEvent(&accel, &gyro, &temp);

//     Serial.print("Acceleration (m/s^2): ");
//     Serial.print(accel.acceleration.x);
//     Serial.print(", ");
//     Serial.print(accel.acceleration.y);
//     Serial.print(", ");
//     Serial.print(accel.acceleration.z);
//     Serial.println("");

//     Serial.print("Gyeroscope (rad/s): ");
//     Serial.print(gyro.gyro.x);
//     Serial.print(", ");
//     Serial.print(gyro.gyro.y);
//     Serial.print(", ");
//     Serial.print(gyro.gyro.z);
//     Serial.println("");

//     Serial.print("Temperature: ");
//     Serial.print(temp.temperature);
//     Serial.println(" C");

//     Serial.print("GMP 180: ");
//     Serial.print("Temperature: ");
//     Serial.print(bmp.readTemperature());
//     Serial.println(" pa");

//     Serial.print("Altitde: ");
//     Serial.print(bmp.readAltitude());
//     Serial.println(" m");

//     Serial.println("=====================================================");


//     gy87Time = millis();
// }


