// #include <Wire.h>

// void setup() {
//   Wire.begin();
//   Serial.begin(9600);

//   Serial.println("I2C Scanner");
// }
// int i=0;


// void loop() {
//   Serial.println(i);

//   byte error, address;
//   int count = 0;

//   for(address = 1; address < 127; address++) { 
//     Serial.print("Testing addr: ");
//     Serial.println(address);

//     Wire.beginTransmission(address);
//     // Serial.println("Passed wire transmission1");
//     error = Wire.endTransmission();
//     // Serial.println("Passed wire transmission2");
//     if(error == 0){
//       Serial.print("Found device at 0x");

//       if(address < 16)
//         Serial.print("0");

//       Serial.println(address, HEX);

//       count++;
//     }
//   }

//   if(count == 0)
//     Serial.println("No I2C devices found.");

//   delay(3000);
//   i++;
//   while(1);
// }






// #include <Wire.h>

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();

//   Serial.println("Scanning...");
// }

// void loop() {

//   for (byte address = 1; address < 127; address++) {

//     Wire.beginTransmission(address);
//     byte error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("Found: 0x");
//       if (address < 16) Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }

//   Serial.println("Done.");
//   while (1);
// }




// #include <Wire.h>

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();

//   // Enable bypass mode on MPU6050
//   Wire.beginTransmission(0x68);
//   Wire.write(0x37);      // INT_PIN_CFG register
//   Wire.write(0x02);      // BYPASS_EN = 1
//   Wire.endTransmission();

//   delay(100);

//   Serial.println("Scanning...");

//   for (byte addr = 1; addr < 127; addr++) {
//     Wire.beginTransmission(addr);
//     if (Wire.endTransmission() == 0) {
//       Serial.print("Found: 0x");
//       if (addr < 16) Serial.print("0");
//       Serial.println(addr, HEX);
//     }
//   }
// }

// void loop() {}





#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Scanning...");
}

void loop() {
  byte count = 0;

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("Found: 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }

  if (count == 0)
    Serial.println("No devices found");

  while (1);
}