/*
 * =====================================================================
 *  GY-521 (MPU-6050) bring-up on the ESP32  -  I2C bus 1
 * =====================================================================
 *  Wiring
 *    GY-521 VCC -> 3V3
 *    GY-521 GND -> GND
 *    GY-521 SDA -> GY_SDA (below)
 *    GY-521 SCL -> GY_SCL (below)
 *    GY-521 AD0 -> unconnected / GND  => address 0x68
 *
 *  What "good" looks like
 *    Lying flat and still: one accel axis near +/-16384 (1 g), the other
 *    two near 0, gyro axes near their bias. Temperature is the die
 *    temperature (a few degrees above room), not body temperature.
 * =====================================================================
 */

#include <Wire.h>

// ---------------------------------------------------------------- pins
  #define GY_SDA 22
  #define GY_SCL 23

// ------------------------------------------------------- MPU-6050 regs
  #define MPU_ADDR         0x68
  #define REG_SMPLRT_DIV   0x19
  #define REG_CONFIG       0x1A
  #define REG_GYRO_CONFIG  0x1B
  #define REG_ACCEL_CONFIG 0x1C
  #define REG_ACCEL_XOUT_H 0x3B
  #define REG_PWR_MGMT_1   0x6B
  #define REG_WHO_AM_I     0x75

// ------------------------------------------------------------- config
  //   ACCEL  +/-2 g     -> 16384 LSB/g
  //   GYRO   +/-250 dps -> 131   LSB/(dps)
  const uint8_t  ACCEL_FS_SEL      = 0;     // 0=+/-2g 1=+/-4g 2=+/-8g 3=+/-16g
  const uint8_t  GYRO_FS_SEL       = 0;     // 0=250  1=500  2=1000 3=2000 dps
  const float    ACCEL_LSB_PER_G   = 16384.0f;
  const float    GYRO_LSB_PER_DPS  = 131.0f;

  const uint8_t  DLPF_CFG          = 4;     // ~21 Hz bandwidth
  const uint8_t  SMPLRT_DIV        = 4;     // 1 kHz / (1 + 4) = 200 Hz internal rate

  const uint32_t PRINT_INTERVAL_MS = 200;   // 5 Hz to the serial monitor

// ------------------------------------------------------------- state
  TwoWire &GYWire = Wire1;                  // second hardware I2C peripheral
  bool     imuOk  = false;
  uint8_t  whoAmI = 0;                      // chip ID, selects the temperature formula

// ------------------------------------------------------------ helpers
  bool writeReg(uint8_t reg, uint8_t val) {
    GYWire.beginTransmission(MPU_ADDR);
    GYWire.write(reg);
    GYWire.write(val);
    return GYWire.endTransmission(true) == 0;
  }

  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    GYWire.beginTransmission(MPU_ADDR);
    GYWire.write(reg);
    if (GYWire.endTransmission(false) != 0) return false;      // repeated start
    if (GYWire.requestFrom((uint8_t)MPU_ADDR, len, (uint8_t)true) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = GYWire.read();
    return true;
  }

  inline int16_t be16(const uint8_t *b) {
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  }

  void scanBus() {
    Serial.printf("I2C scan on bus 1 (SDA %d / SCL %d):\n", GY_SDA, GY_SCL);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      GYWire.beginTransmission(addr);
      if (GYWire.endTransmission(true) == 0) {
        Serial.printf("  found 0x%02X\n", addr);
        found++;
      }
    }
    if (!found) Serial.println("  nothing responded - check VCC, GND, SDA, SCL");
  }

  bool initIMU() {
    uint8_t who = 0;
    if (!readRegs(REG_WHO_AM_I, &who, 1)) {
      Serial.println("MPU-6050: no reply on 0x68");
      return false;
    }
    whoAmI = who;

    Serial.printf("MPU-6050: WHO_AM_I = 0x%02X ", who);
    if (who != 0x68) {
      // 0x70 -> MPU-6500, 0x71/0x73 -> MPU-9250 family. Accel/gyro registers
      // are compatible, but the temperature formula is different.
      Serial.println("(not 0x68 - clone chip, using MPU-6500 temperature formula)");
    } else {
      Serial.println("OK");
    }

    if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;   // device reset
    delay(100);
    if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;   // wake, clock = gyro X PLL,
    delay(10);                                           // temperature sensor enabled

    if (!writeReg(REG_CONFIG,       DLPF_CFG))            return false;
    if (!writeReg(REG_SMPLRT_DIV,   SMPLRT_DIV))          return false;
    if (!writeReg(REG_GYRO_CONFIG,  GYRO_FS_SEL  << 3))   return false;
    if (!writeReg(REG_ACCEL_CONFIG, ACCEL_FS_SEL << 3))   return false;
    return true;
  }

  // MPU-6050 (0x68):                raw / 340    + 36.53
  // MPU-6500 / 9250 (0x70/71/73):   raw / 333.87 + 21.0
  float rawToTempC(int16_t raw) {
    if (whoAmI == 0x68) return raw / 340.0f + 36.53f;
    return raw / 333.87f + 21.0f;
  }

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== GY-521 (MPU-6050) on ESP32 - bus 1, SDA %d / SCL %d ===\n",
                GY_SDA, GY_SCL);

  GYWire.begin(GY_SDA, GY_SCL, 400000);    // 400 kHz is the MPU-6050 maximum
  scanBus();

  imuOk = initIMU();
  Serial.println(imuOk ? "IMU configured: +/-2 g, +/-250 dps, DLPF ~21 Hz, 200 Hz ODR"
                       : "IMU INIT FAILED");
  Serial.println("raw counts | scaled  (accel /16384 = g, gyro /131 = dps) | temp raw = C\n");
}

// =====================================================================
void loop() {
  static uint32_t last = 0;

  if (!imuOk) {                            // keep retrying so you can hot-fix wiring
    delay(1000);
    imuOk = initIMU();
    return;
  }

  if (millis() - last < PRINT_INTERVAL_MS) return;
  last = millis();

  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) {
    Serial.println("read failed");
    imuOk = false;
    return;
  }

  int16_t ax = be16(&b[0]),  ay = be16(&b[2]),  az = be16(&b[4]);
  int16_t tRaw = be16(&b[6]);                        // TEMP_OUT sits here
  int16_t gx = be16(&b[8]),  gy = be16(&b[10]), gz = be16(&b[12]);

  Serial.printf("A %6d %6d %6d | G %6d %6d %6d  ", ax, ay, az, gx, gy, gz);
  Serial.printf("=> %6.3f %6.3f %6.3f g | %7.2f %7.2f %7.2f dps | T raw %6d = %.1f C\n",
                ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G,
                gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS,
                tRaw, rawToTempC(tRaw));
}
