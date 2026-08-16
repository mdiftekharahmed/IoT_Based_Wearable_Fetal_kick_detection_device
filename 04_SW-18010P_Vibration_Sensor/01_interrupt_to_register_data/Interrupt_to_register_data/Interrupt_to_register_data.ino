void setup() {
  pinMode(27, INPUT);
    Serial.begin(115200);
    Serial.println("start");
}

int val=0,prevval = 0;

void loop() {
    prevval = val;
    val = digitalRead(3);
  if(val!=prevval)Serial.print(val);
//   delay(100);
  
}