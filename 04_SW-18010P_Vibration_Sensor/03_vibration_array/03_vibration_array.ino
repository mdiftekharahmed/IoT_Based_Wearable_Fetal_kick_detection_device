const uint8_t SENSOR_PINS[4] = {32, 33, 34, 35};
const unsigned long WINDOW = 500;   // ms measured after the first change on any sensor

int  prevState[4];
int  hardness[4];                   // transitions counted per sensor in the window
bool measuring = false;
unsigned long startTime;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    prevState[i] = digitalRead(SENSOR_PINS[i]);
  }

  Serial.println("Setup Success, system ready!");
}

void loop() {
  int state[4];
  bool anyChange = false;

  for (int i = 0; i < 4; i++) {
    state[i] = digitalRead(SENSOR_PINS[i]);
    if (state[i] != prevState[i]) anyChange = true;
  }

  // First transition on ANY sensor starts one shared window
  if (!measuring && anyChange) {
    measuring = true;
    startTime = millis();
    for (int i = 0; i < 4; i++) hardness[i] = 0;
  }

  if (measuring) {
    for (int i = 0; i < 4; i++) {
      if (state[i] != prevState[i]) {
        hardness[i]++;
        prevState[i] = state[i];
      }
    }

    if (millis() - startTime >= WINDOW) {
      measuring = false;

      Serial.printf("s1: %d, s2: %d, s3: %d, s4: %d\n",
                    hardness[0], hardness[1], hardness[2], hardness[3]);

      for (int i = 0; i < 4; i++) prevState[i] = digitalRead(SENSOR_PINS[i]);
    }
  } else {
    for (int i = 0; i < 4; i++) prevState[i] = state[i];
  }
}