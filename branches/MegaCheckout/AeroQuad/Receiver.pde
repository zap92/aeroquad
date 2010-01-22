/*
  AeroQuad v2.0 - January 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
  
  Interrupt based method inspired by Dror Caspi
  http://www.rcgroups.com/forums/showpost.php?p=12356667&postcount=1639
  
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "Receiver.h"

void initializeMegaPcInt() {
  pinMode(THROTTLEPIN, INPUT);
  pinMode(ROLLPIN, INPUT);
  pinMode(PITCHPIN, INPUT);
  pinMode(YAWPIN, INPUT);
  pinMode(MODEPIN, INPUT);
  pinMode(AUXPIN, INPUT);
  PCMSK0 |= 0x70;
  PCICR |= 0x01 << 0;
  
  PCMSK2 |= 0xE0;
  PCICR |= 0x01 << 2;
}

static void MegaPcIntISR(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  curr = *portInputRegister(port);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;  

  //Serial.println(curr,DEC);

  // mask is pins that have changed. screen out non pcint pins.
  switch (port) {
    case 0:
      if ((mask &= PCMSK0) == 0) return;
      break;
    case 11:
      if ((mask &= PCMSK2) == 0) return;
      break;
    default:
      break;
  }

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = i;
      if (port == 11) pin += 16;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
       time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge == FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        } 
      }
    }
  }
}

SIGNAL(PCINT0_vect) {
  MegaPcIntISR(2);
}

SIGNAL(PCINT2_vect) {
  MegaPcIntISR(11);
}

// Configure each receiver pin for PCINT
void configureReceiver() {
  initializeMegaPcInt();
  for (channel = ROLL; channel < LASTCHANNEL; channel++)
    pinData[receiverChannel[channel]].edge == FALLING_EDGE;
}

// Calculate PWM pulse width of receiver data
// If invalid PWM measured, use last known good time
unsigned int readReceiver(byte receiverPin) {
  uint16_t data;
  uint8_t oldSREG;
    
  oldSREG = SREG;
  cli();
  data = pinData[receiverPin].lastGoodWidth;
  SREG = oldSREG;  
  return data;
}
