/*
 * GY-521 (MPU-6050) sanity check - confirms the replacement module works.
 * Ran on the Arduino; ESP32 version is in 05_GY-521_on_ESP32.
 *
 * NOTE on the byte count: the MPU-6050 register map is
 *   0x3B..0x40  ACCEL_XOUT/YOUT/ZOUT   (6 bytes)
 *   0x41..0x42  TEMP_OUT               (2 bytes)   <-- sits in the middle
 *   0x43..0x48  GYRO_XOUT/YOUT/ZOUT    (6 bytes)
 * so a burst read from 0x3B needs 14 bytes, not 12. Requesting 12 and
 * skipping Tmp made the temperature land in GyX, GyX in GyY, GyY in GyZ,
 * and GyZ was never read at all. Accel was unaffected, which is why the
 * output still looked believable.
 */

#include<Wire.h>
const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void  setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void  loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();    // TEMP_OUT - must be read to stay aligned
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();

  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = ");  Serial.println(AcZ);

  Serial.print("Gyroscope: ");
  Serial.print("X  = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);

  Serial.print("Temp: "); Serial.print(Tmp/340.0 + 36.53); Serial.println(" C");
  Serial.println(" ");
  delay(333);
}