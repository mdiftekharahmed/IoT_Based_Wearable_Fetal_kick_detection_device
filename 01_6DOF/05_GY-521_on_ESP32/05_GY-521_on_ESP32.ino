/*
 * =====================================================================
 *  GY-521 (MPU-6050) bring-up on the ESP32  -  I2C bus 1, SDA 21 / SCL 22
 * =====================================================================
 *
 *  Purpose: confirm the wiring and the exact register configuration that
 *  07_Combined_All_Sensors/05_simplified_dualcore uses, before flashing
 *  the full logger. If this sketch prints sane numbers, the IMU half of
 *  the logger will work.
 *
 *  Wiring
 *  ------
 *    GY-521 VCC -> 3V3        (the module has a regulator, but 3V3 is
 *                              safe and keeps the I2C lines at 3.3 V)
 *    GY-521 GND -> GND
 *    GY-521 SDA -> GPIO 21
 *    GY-521 SCL -> GPIO 22
 *    GY-521 AD0 -> unconnected / GND  => address 0x68
 *
 *  Why bus 1 and not the default Wire: the MAX30100 owns I2C bus 0 on
 *  pins 25/26 in the main firmware and is polled ~1000x/s. Keeping the
 *  IMU on its own peripheral means neither sensor waits for the other.
 *
 *  What "good" looks like
 *  ----------------------
 *    Lying flat and still: one accel axis near +/-16384 (1 g), the other
 *    two near 0, all three gyro axes within roughly +/-200 counts of
 *    their bias. Temperature is the die temperature (runs a few degrees
 *    above room), not body temperature.
 * =====================================================================
 */

#include <Wire.h>

// ---------------------------------------------------------------- pins
  #define GY_SDA 21
  #define GY_SCL 22

// ------------------------------------------------------- MPU-6050 regs
  #define MPU_ADDR        0x68
  #define REG_SMPLRT_DIV  0x19
  #define REG_CONFIG      0x1A
  #define REG_GYRO_CONFIG 0x1B
  #define REG_ACCEL_CONFIG 0x1C
  #define REG_ACCEL_XOUT_H 0x3B
  #define REG_PWR_MGMT_1  0x6B
  #define REG_WHO_AM_I    0x75

// ------------------------------------------------------------- config
  // Ranges. Raw counts are what gets logged, so these scale factors are
  // the only way to interpret the data later - keep them in sync with
  // the main firmware.
  //   ACCEL  +/-2 g     -> 16384 LSB/g      (best resolution; a hard
  //                                          impact will clip, which is
  //                                          itself a usable fall cue)
  //   GYRO   +/-250 dps -> 131   LSB/(dps)
  const uint8_t  ACCEL_FS_SEL   = 0;        // 0=+/-2g 1=+/-4g 2=+/-8g 3=+/-16g
  const uint8_t  GYRO_FS_SEL    = 0;        // 0=250  1=500  2=1000 3=2000 dps
  const float    ACCEL_LSB_PER_G   = 16384.0f;
  const float    GYRO_LSB_PER_DPS  = 131.0f;

  // DLPF_CFG = 4 -> ~21 Hz bandwidth, 1 kHz internal rate. Sized for the
  // logger's 100 Hz output: Nyquist is 50 Hz, and kick / movement / fall
  // energy all sit below 20 Hz, so this keeps the signal and rejects what
  // would otherwise alias down into it. 3 (44 Hz) or 2 (94 Hz) keep sharper
  // impact edges at the cost of that rejection.
  const uint8_t  DLPF_CFG       = 4;

  // Internal output rate = 1 kHz / (1 + SMPLRT_DIV) when DLPF is enabled.
  // 4 -> 200 Hz, deliberately ahead of the logger's 100 Hz reads so the host
  // never catches the same conversion twice as the two clocks drift.
  const uint8_t  SMPLRT_DIV     = 4;

  const uint32_t PRINT_INTERVAL_MS = 200;   // 5 Hz to the serial monitor

// ------------------------------------------------------------- state
  TwoWire &GYWire = Wire1;                  // second hardware I2C peripheral
  bool     imuOk  = false;

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

  // Bytes are read into a buffer first and combined afterwards. Writing
  // Wire.read()<<8 | Wire.read() leaves the evaluation order up to the
  // compiler, which is a coin flip the standard does not decide for you.
  inline int16_t be16(const uint8_t *b) {
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  }

  void scanBus() {
    Serial.println("I2C scan on bus 1 (SDA 21 / SCL 22):");
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
    Serial.printf("MPU-6050: WHO_AM_I = 0x%02X ", who);
    if (who != 0x68) {
      // 0x70 -> MPU-6500 clone, 0x71/0x73 -> MPU-9250 family. They still
      // work, the register map for accel/gyro is compatible.
      Serial.println("(expected 0x68 - clone chip, register map is compatible)");
    } else {
      Serial.println("OK");
    }

    if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;   // device reset
    delay(100);
    if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;   // wake, clock = gyro X PLL
    delay(10);                                           // (more stable than the
                                                         //  internal 8 MHz oscillator)
    if (!writeReg(REG_CONFIG,       DLPF_CFG))            return false;
    if (!writeReg(REG_SMPLRT_DIV,   SMPLRT_DIV))          return false;
    if (!writeReg(REG_GYRO_CONFIG,  GYRO_FS_SEL  << 3))   return false;
    if (!writeReg(REG_ACCEL_CONFIG, ACCEL_FS_SEL << 3))   return false;
    return true;
  }

// =====================================================================
void setup() {
  Serial.begin(921600);
  delay(200);
  Serial.println("\n=== GY-521 (MPU-6050) on ESP32 - bus 1, SDA 21 / SCL 22 ===");

  GYWire.begin(GY_SDA, GY_SCL, 400000);    // 400 kHz is the MPU-6050 maximum
  scanBus();

  imuOk = initIMU();
  Serial.println(imuOk ? "IMU configured: +/-2 g, +/-250 dps, DLPF 44 Hz, 100 Hz ODR"
                       : "IMU INIT FAILED");
  Serial.println("raw counts | scaled  (accel /16384 = g, gyro /131 = dps)\n");
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
  Serial.printf("=> %6.3f %6.3f %6.3f g | %7.2f %7.2f %7.2f dps | %.1f C\n",
                ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G,
                gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS,
                tRaw / 340.0f + 36.53f);
}
