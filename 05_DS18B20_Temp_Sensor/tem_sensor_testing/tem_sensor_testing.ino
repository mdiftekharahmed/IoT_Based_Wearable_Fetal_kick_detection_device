
#include <OneWire.h>
#include <DallasTemperature.h>

const int ONE_WIRE_BUS =26;
OneWire oneWireLocal(ONE_WIRE_BUS);
DallasTemperature sensorsLocal(&oneWireLocal);

float tempLocal = 0.0;

void setup() {
  Serial.begin(115200); // Higher baud rate for ESP32
  delay(1000);
  
  sensorsLocal.begin();
  
  // Check if sensor is actually detected on the bus
  int deviceCount = sensorsLocal.getDeviceCount();
  Serial.print("DS18B20 Devices Found: ");
  Serial.println(deviceCount);

  if (deviceCount == 0) {
    Serial.println("ERROR: No DS18B20 found! Check 4.7k resistor & pin wiring.");
  }
}

void loop() {
  CheckTemps();
  
  if (tempLocal == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data (-127)");
  } else {
    Serial.print("Current Temp: ");
    Serial.print(tempLocal);
    Serial.println(" °C");
  }

  delay(1000);
}

void CheckTemps() {
  sensorsLocal.requestTemperatures(); 
  // Standard conversion time at 12-bit is up to 750ms
  delay(750); 
  tempLocal = sensorsLocal.getTempCByIndex(0);
}