#define SENSOR_PIN 8
int prevState;
bool measuring = false;

unsigned long startTime;

int ones = 0;
int zeros = 0;

const unsigned long WINDOW = 500;   // Measure for 100 ms after first change

void setup() {
  Serial.begin(115200);
  Serial.println("inside setup ");
  pinMode(SENSOR_PIN, INPUT);

  prevState = digitalRead(SENSOR_PIN);
  Serial.println("Setup Success, system ready!");
}

void loop() {

  int state = digitalRead(SENSOR_PIN);

  // First transition starts a measurement
  if (!measuring && state != prevState) {
    measuring = true;
    startTime = millis();
    ones = 0;
    zeros = 0;
  }

  if (measuring) {

    if (state != prevState) {

      if (state == HIGH)
        ones++;
      else
        zeros++;

      prevState = state;
    }

    if (millis() - startTime >= WINDOW) {
      measuring = false;

      Serial.print("Hardness = ");
      Serial.print(ones);
      Serial.print(":");
      Serial.println(zeros);

      prevState = digitalRead(SENSOR_PIN);
    }
  }
  else {
    prevState = state;
  }
}



// void setup() {
//   Serial.begin(9600);
//   delay(2000);
//   Serial.println();
//   Serial.println("===== ESP8266 TEST =====");
// }

// void loop() {
//   Serial.println("ALIVE");
//   delay(1000);
// }