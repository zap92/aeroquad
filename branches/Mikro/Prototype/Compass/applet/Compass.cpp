// Arduino Mega / MicroMag 3-Axis v1.0 Compass Example
// Uses native SPI support in Atmgea 1280
// Adapted from: http://forum.sparkfun.com/viewtopic.php?p=27072&sid=17c72d5a264f383836578d728cb60881#27072
// Ted Carancho - www.AeroQuad.info

#include "Spi.h"

// Additional pin assignments for the MicroMag 3
#define RESET 49
#define DRDY 48
#define LEDPIN 13
// Pin assignments for accelerometers
#define XACCPIN 0
#define YACCPIN 1
#define ZACCPIN 2

#define XGYROPIN 5
#define YGYROPIN 4
#define ZGYROPIN 8

#define WAIT 3
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#include "WProgram.h"
int readAxis(byte axis);
void setup();
void loop();
int x = 0;        // magnetic field x axis
int y = 0;        // magnetic field y axis
int z = 0;        // magnetic field z axis
int xaccel = 0;
int yaccel = 0;
int zaccel = 0;
float heading = 0;  // magnetic field heading
float roll = 0;
float pitch = 0;
float g = 0;
float CMx = 0;
float CMy = 0;

int readAxis(byte axis){
  int measurement;
  // Send reset
  digitalWrite(RESET, HIGH);
  delayMicroseconds(WAIT);
  digitalWrite(RESET, LOW);

  // Send command byte
  // Description found on page 9 of MicroMag3 data sheet
  // First nibble defines speed/accuracy of measurement
  // Use 0x70 for the slowest/best accuracy, 0x10 for fastest/least accuracy
  // Last nibble defines axis (X = 0x01, Y = 0x02, Z - 0x03)
  switch (axis) {
    case XAXIS:
      Spi.transfer(0x61);
      break;
    case YAXIS:
      Spi.transfer(0x62);
      break;
    case ZAXIS:
      Spi.transfer(0x63);
      break;
  }

  // Wait for data to be ready, then read two bytes
  while(digitalRead(DRDY) == LOW);  
  return (Spi.transfer(0xFF) << 8) | Spi.transfer(0xFF);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT); 
  digitalWrite(LEDPIN, HIGH);
  pinMode(DRDY, INPUT);
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  // Auto zero pins for IDG 500 gyros
  digitalWrite(22, LOW);
  digitalWrite(23, LOW);
} 

void loop() {
  x = readAxis(XAXIS);  // read the x-axis magnetic field value
  y = readAxis(YAXIS);  // read the y-axis magnetic field value
  z = readAxis(ZAXIS);  // read the z-axis magnetic field value
  
  // Estimated level position by rotating accels on each axis and found midpoint
  // Real implementation will use properly calibrated accelerometer values
  xaccel = 355-analogRead(XACCPIN);
  yaccel = 355-analogRead(YACCPIN);
  zaccel = analogRead(ZACCPIN)-355;
  /*Serial.print(xaccel);
  Serial.print(" ");
  Serial.print(yaccel);
  Serial.print(" ");
  Serial.println(zaccel);*/
  
  roll = atan2(xaccel, zaccel);
  pitch = atan2(yaccel, zaccel);
  /*Serial.print("roll = ");
  Serial.print(degrees(roll));
  Serial.print(", pitch = ");
  Serial.println(degrees(pitch));*/
  
  CMx = (x * cos(pitch)) + (y *sin(roll) * sin(pitch)) - (z * cos(roll) * sin(pitch));
  CMy = (y * cos(roll)) + (z * sin(roll));
  heading = abs(degrees(atan(CMy/CMx)));
  if (CMx >= 0 && CMy >= 0) {heading = 180 - heading;}
  if (CMx >= 0 && CMy < 0) {heading = heading + 180;}
  if (CMx < 0 && CMy < 0) {heading = 360 - heading;}
  /*Serial.print(xaccel);
  Serial.print(", ");
  Serial.print(yaccel);
  Serial.print(", ");
  Serial.print(zaccel);
  Serial.print(" | ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print(" = ");*/
  Serial.println(heading);
}


int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

