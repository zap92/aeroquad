// Arduino Mega / MicroMag 3-Axis v1.0 Compass Example
// Uses native SPI support in Atmgea 1280
// Adapted from: http://forum.sparkfun.com/viewtopic.php?p=27072&sid=17c72d5a264f383836578d728cb60881#27072
// Ted Carancho - www.AeroQuad.info

#include "Spi.h"

void configureCompass() {
  pinMode(MAG_DRDY, INPUT);
  pinMode(MAG_RESET, OUTPUT);
  digitalWrite(MAG_RESET, LOW);
} 

int readCompass(byte axis){
  int measurement;
  // Send reset
  digitalWrite(MAG_RESET, HIGH);
  delay(MAG_WAIT);
  digitalWrite(MAG_RESET, LOW);

  // Send command byte
  // Description found on page 9 of MicroMag3 data sheet
  // First nibble defines speed/accuracy of measurement
  // Use 0x70 for the slowest/best accuracy, 0x10 for fastest/least accuracy
  // Last nibble defines axis (X = 0x01, Y = 0x02, Z - 0x03)
  switch (axis) {
    case MAG_XAXIS:
      Spi.transfer(0x61);
      break;
    case MAG_YAXIS:
      Spi.transfer(0x62);
      break;
    case MAG_ZAXIS:
      Spi.transfer(0x63);
      break;
  }
  // Wait for data to be ready, then read two bytes
  while(digitalRead(MAG_DRDY) == LOW);
  return (Spi.transfer(0xFF) << 8) | Spi.transfer(0xFF);
}

