#define SENSOR_PIN 27

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.println(digitalRead(SENSOR_PIN));
  delay(50);
}





// /*
// Arduino-MAX30100 oximetry / heart rate integrated sensor library
// Copyright (C) 2017  OXullo Intersecans <x@brainrapers.org>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// */

// // This example can be used to test connectivity and operation of the sensor
// // check the README file on the repo's root for a troubleshooting guide in case
// // any of the tests running fail

// #include <Wire.h>
// #include "MAX30100.h"

// MAX30100 sensor;

// void setup()
// {
//     Serial.begin(9600);

//     Serial.print("Initializing MAX30100..");

//     if (!sensor.begin()) {
//         Serial.print("FAILED: ");

//         uint8_t partId = sensor.getPartId();
//         if (partId == 0xff) {
//             Serial.println("I2C error");
//         } else {
//             Serial.print("wrong part ID 0x");
//             Serial.print(partId, HEX);
//             Serial.print(" (expected: 0x");
//             Serial.println(EXPECTED_PART_ID, HEX);
//         }
//         // Stop here
//         for(;;);
//     } else {
//         Serial.println("Success");
//     }

//     Serial.print("Enabling HR/SPO2 mode..");
//     sensor.setMode(MAX30100_MODE_SPO2_HR);
//     Serial.println("done.");

//     Serial.print("Configuring LEDs biases to 50mA..");
//     sensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_50MA);
//     Serial.println("done.");

//     delay(1000);

//     Serial.print("Lowering the current to 7.6mA..");
//     sensor.setLedsCurrent(MAX30100_LED_CURR_7_6MA, MAX30100_LED_CURR_7_6MA);
//     Serial.println("done.");

//     delay(1000);

//     Serial.print("Shutting down..");
//     sensor.shutdown();
//     Serial.println("done.");

//     delay(1000);

//     Serial.print("Resuming normal operation..");
//     sensor.resume();
//     delay(500);
//     Serial.println("done.");

//     uint32_t tsTempSampStart = millis();
//     Serial.print("Sampling die temperature..");
//     sensor.startTemperatureSampling();
//     while(!sensor.isTemperatureReady()) {
//         if (millis() - tsTempSampStart > 1000) {
//             Serial.println("ERROR: timeout");
//             // Stop here
//             for(;;);
//         }
//     }

//     float temperature = sensor.retrieveTemperature();
//     Serial.print("done, temp=");
//     Serial.print(temperature);
//     Serial.println("C");

//     if (temperature < 5) {
//         Serial.println("WARNING: Temperature probe reported an odd value");
//     } else {
//         Serial.println("All test pass.");
//     }

//     Serial.println();
//     Serial.println("Press any key to go into sampling loop mode");
//     while (!Serial.available());

//     sensor.resetFifo();
// }

// void loop()
// {
//     uint16_t ir, red;

//     sensor.update();

//     while (sensor.getRawValues(&ir, &red)) {
//         Serial.print("IR=");
//         Serial.print(ir);
//         Serial.print(" RED=");
//         Serial.println(red);
//     }
// }




// /*
// Arduino-MAX30100 oximetry / heart rate integrated sensor library
// Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// */

// // The example shows how to retrieve raw values from the sensor
// // experimenting with the most relevant configuration parameters.
// // Use the "Serial Plotter" app from arduino IDE 1.6.7+ to plot the output

// #include <Wire.h>
// #include "MAX30100.h"

// // Sampling is tightly related to the dynamic range of the ADC.
// // refer to the datasheet for further info
// #define SAMPLING_RATE                       MAX30100_SAMPRATE_100HZ

// // The LEDs currents must be set to a level that avoids clipping and maximises the
// // dynamic range

// // #define IR_LED_CURRENT                      MAX30100_LED_CURR_50MA
// // #define RED_LED_CURRENT                     MAX30100_LED_CURR_27_1MA

// #define IR_LED_CURRENT   MAX30100_LED_CURR_7_6MA
// #define RED_LED_CURRENT  MAX30100_LED_CURR_7_6MA

// // The pulse width of the LEDs driving determines the resolution of
// // the ADC (which is a Sigma-Delta).
// // set HIGHRES_MODE to true only when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US_16BITS
// #define PULSE_WIDTH                         MAX30100_SPC_PW_1600US_16BITS
// #define HIGHRES_MODE                        true


// // Instantiate a MAX30100 sensor class
// MAX30100 sensor;

// void setup()
// {
//     Serial.begin(9600);

//     Serial.print("Initializing MAX30100..");

//     // Initialize the sensor
//     // Failures are generally due to an improper I2C wiring, missing power supply
//     // or wrong target chip
//     if (!sensor.begin()) {
//         Serial.println("FAILED");
//         for(;;);
//     } else {
//         Serial.println("SUCCESS");
//     }

//     // Set up the wanted parameters
//     sensor.setMode(MAX30100_MODE_SPO2_HR);
//     sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
//     sensor.setLedsPulseWidth(PULSE_WIDTH);
//     sensor.setSamplingRate(SAMPLING_RATE);
//     sensor.setHighresModeEnabled(HIGHRES_MODE);
// }

// // void loop()
// // {
// //     uint16_t ir, red;

// //     sensor.update();

// //     while (sensor.getRawValues(&ir, &red)) {
// //         Serial.print(ir);
// //         Serial.print('\t');
// //         Serial.println(red);
// //     }
// // }
// void loop()
// {
//     uint16_t ir, red;

//     sensor.update();

//     if (sensor.getRawValues(&ir, &red)) {
//         Serial.print(ir);
//         Serial.print('\t');
//         Serial.println(red);
//     } else {
//         Serial.println("No sample");
//     }

//     delay(10);
// }