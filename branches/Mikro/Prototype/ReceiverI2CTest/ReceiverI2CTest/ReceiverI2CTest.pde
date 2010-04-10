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

uint8_t byteDataHigh, byteDataLow, i;
uint8_t ledOutput = HIGH;
uint8_t channel = THROTTLE;
uint8_t dataByte[20];
unsigned int receiverData[20];
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
  previousTime = millis();
  Wire.requestFrom(29, 20);
  if (Wire.available() == 20) {
    for (i = 0; i < 20; i++)
      dataByte[i] = Wire.receive();
  }
  currentTime = millis() - previousTime;
  
  for (i = 0; i < 20; i+=2) {
    receiverData[i] = word(dataByte[i+1], dataByte[i]);
    Serial.print(receiverData[i]);Serial.print(',');
  }
  Serial.print(currentTime);
  Serial.println();
  
  
  ledOutput = (ledOutput == HIGH) ? ledOutput = LOW : ledOutput = HIGH;
  digitalWrite(LEDPIN, ledOutput);
  delay(100);
}


