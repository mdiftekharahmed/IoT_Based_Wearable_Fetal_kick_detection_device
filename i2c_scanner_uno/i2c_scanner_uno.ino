/*
 * I2C Scanner for Arduino Uno
 * ---------------------------
 * Scans the I2C bus and reports every device that responds.
 * Also guesses the likely chip for common sensor addresses.
 *
 * Wiring (Arduino Uno R3) - use the dedicated I2C pins on the digital
 * header, the two just past AREF:
 *
 *      ... 12  13  GND  AREF  SDA  SCL
 *                              |    |
 *                            SDA   SCL
 *
 *   SDA -> SDA pin (next to AREF)
 *   SCL -> SCL pin (end of the header)
 *   VCC -> 5V or 3.3V (match your module!)
 *   GND -> GND  (common ground is mandatory)
 *
 * IMPORTANT: those SDA/SCL pins are the SAME electrical nets as A4/A5 -
 * they are bonded together on the PCB (ATmega328P pins PC4/PC5). So there
 * is nothing to configure: Wire.begin() drives them automatically, and you
 * must NOT treat SDA/SCL and A4/A5 as two separate buses. Also avoid using
 * A4/A5 as analog inputs while I2C is running.
 *
 * Serial Monitor: set baud to 9600.
 *
 * Note: the Uno's internal pull-ups are weak. If nothing is detected, add
 * an external 4.7k resistor from SDA to VCC and another from SCL to VCC
 * (most breakout modules already include these).
 */

#include <Wire.h>

// Rescan interval in milliseconds
const unsigned long SCAN_INTERVAL = 5000;

// Returns a likely chip name for well-known I2C addresses, or NULL.
const char* guessDevice(uint8_t addr) {
  switch (addr) {
    case 0x0D: return "HMC5883L / QMC5883L magnetometer";
    case 0x1E: return "HMC5883L magnetometer";
    case 0x27: return "PCF8574 (16x2 LCD backpack)";
    case 0x3C: return "SSD1306 OLED (128x64)";
    case 0x3D: return "SSD1306 OLED (alt address)";
    case 0x40: return "INA219 current sensor / HTU21D";
    case 0x44: return "SHT31 temp/humidity";
    case 0x48: return "ADS1115 ADC";
    case 0x50: return "AT24Cxx EEPROM";
    case 0x53: return "ADXL345 accelerometer";
    case 0x57: return "MAX30100/MAX30102 pulse oximeter";
    case 0x5A: return "MLX90614 IR thermometer / CCS811";
    case 0x68: return "MPU6050 / MPU9250 / DS3231 RTC";
    case 0x69: return "MPU6050 / MPU9250 (AD0 pulled HIGH)";
    case 0x6B: return "LSM6DS3 / LSM6DSO IMU";
    case 0x76: return "BMP280 / BME280 (SDO LOW)";
    case 0x77: return "BMP280 / BME280 / BMP180 (SDO HIGH)";
    default:   return NULL;
  }
}

// Prints an address as 0xNN with a leading zero when needed.
void printHexAddress(uint8_t addr) {
  Serial.print("0x");
  if (addr < 16) Serial.print("0");
  Serial.print(addr, HEX);
}

void scanBus() {
  Serial.println(F("Scanning I2C bus (0x01 - 0x7E)..."));

  uint8_t found = 0;
  uint8_t errors = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    // A device ACKs its address, which makes endTransmission() return 0.
    Wire.beginTransmission(addr);
    uint8_t result = Wire.endTransmission();

    if (result == 0) {
      found++;
      Serial.print(F("  FOUND  device at "));
      printHexAddress(addr);
      Serial.print(F("  (decimal "));
      Serial.print(addr);
      Serial.print(F(")"));

      const char* name = guessDevice(addr);
      if (name != NULL) {
        Serial.print(F("  -> likely: "));
        Serial.print(name);
      }
      Serial.println();

    } else if (result == 4) {
      errors++;
      Serial.print(F("  ERROR  unknown fault at "));
      printHexAddress(addr);
      Serial.println();
    }
    // result 1/2/3 simply mean "no device acknowledged" - stay quiet.
  }

  Serial.println();
  if (found == 0) {
    Serial.println(F("No I2C devices found."));
    Serial.println(F("Check: SDA/SCL on the header past AREF (= A4/A5),"));
    Serial.println(F("common GND, VCC present, SDA/SCL not swapped or"));
    Serial.println(F("bridged, and 4.7k pull-ups to VCC."));
  } else {
    Serial.print(F("Scan complete: "));
    Serial.print(found);
    Serial.println(found == 1 ? F(" device found.") : F(" devices found."));
  }

  if (errors > 0) {
    Serial.print(F("Note: "));
    Serial.print(errors);
    Serial.println(F(" address(es) reported a bus fault - suspect a short or noise."));
  }

  Serial.println(F("--------------------------------------------------"));
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for the serial port (harmless on the Uno)
  }

  // On the Uno the I2C pins are fixed in hardware. Wire.begin() takes no
  // pin arguments on AVR - it always uses the SDA/SCL header pins past
  // AREF (the same nets as A4/A5). Unlike the ESP32, you cannot remap them.
  Wire.begin();
  Wire.setClock(100000); // 100 kHz standard mode is the most forgiving

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("            Arduino Uno I2C Scanner"));
  Serial.println(F("=================================================="));
  Serial.println();
}

void loop() {
  scanBus();
  delay(SCAN_INTERVAL);
}
