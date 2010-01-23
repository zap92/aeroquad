#define OFF 0
#define ON 1
byte state = OFF;

void setup() {
  Serial.begin(115200);
  pinMode(24, OUTPUT);
}

void loop() {
  if (state == ON) {
    state = OFF;
    Serial.println("LED off");
  }
  else {
    state = ON;
    Serial.println("LED on");
  }
  
  digitalWrite(24, state);
  delay(500);
}
  
