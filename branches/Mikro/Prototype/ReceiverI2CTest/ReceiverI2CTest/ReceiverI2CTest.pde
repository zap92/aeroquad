// Receiver I2C Test

#include <Wire.h>
#define LEDPIN 13
#define THROTTLE 0
#define ROLL 1
#define PITCH 2
#define YAW 3
#define MODE 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8
#define AUX5 9
#define LASTCHANNEL 10
#define AERORECEIVER_ADR 29
#define AERORECEIVER_MSG 4

byte ledOutput = HIGH;
byte channel = THROTTLE;
byte i, dataByte[AERORECEIVER_MSG];
unsigned int receiverData[LASTCHANNEL];
unsigned int currentTime = 0;
unsigned int previousTime = 0;

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
}

void loop()
{
  // Read I2C Receiver Data
  previousTime = micros();
  Wire.requestFrom(AERORECEIVER_ADR, AERORECEIVER_MSG);
  if (Wire.available() == AERORECEIVER_MSG) {
    for (i = 0; i < AERORECEIVER_MSG; i++)
      dataByte[i] = Wire.receive();
  }
  currentTime = micros() - previousTime;
  
  // Decode data into channel number and channel value
  channel = word(dataByte[1], dataByte[0]);
  receiverData[channel] = word(dataByte[3], dataByte[2]);
  
  // Print data for debug
  for (channel = THROTTLE; channel < LASTCHANNEL; channel++) {
    Serial.print(receiverData[channel]);Serial.print(',');
  }
  Serial.print(currentTime);
  Serial.println();
  
  // Blink LED
  (ledOutput == HIGH) ? ledOutput = LOW : ledOutput = HIGH;
  digitalWrite(LEDPIN, ledOutput);
  delay(10);
}


