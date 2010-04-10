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

#include "WProgram.h"
void setup();
void loop();
uint8_t byteDataHigh, byteDataLow, i;
uint8_t ledOutput = HIGH;
uint8_t channel = THROTTLE;
unsigned int receiverData[10];

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
}

void loop()
{
  Wire.beginTransmission(29);
  Wire.send(channel++);
  Wire.endTransmission();
  channel = (channel == LASTCHANNEL) ? channel = THROTTLE : channel = channel;

  Wire.requestFrom(29, 2);

  if(2 <= Wire.available()) {
    receiverData[channel] = Wire.receive();
    receiverData[channel] = receiverData[channel] << 8;
    receiverData[channel] |= Wire.receive();
    Serial.println(receiverData[channel]);
  }
  
  ledOutput = (ledOutput == HIGH) ? ledOutput = LOW : ledOutput = HIGH;
  digitalWrite(LEDPIN, ledOutput);
  delay(100);
}



int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

