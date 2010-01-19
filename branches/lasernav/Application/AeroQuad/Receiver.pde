/*
  AeroQuad v1.5 - Novmeber 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
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
#include <ServoDecode.h>
char * stateStrings[] = {
  "NOT_SYNCHED", "ACQUIRING", "READY", "in Failsafe"};

#ifndef Mega_AQ1x
void init_servodecode()
{
  //Serial.begin(38400);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  ServoDecode.begin();
  ServoDecode.setFailsafe(3,1234); // set channel 3 failsafe pulse  width
}
// INIT Arduino Standard
#endif

#ifdef Mega_AQ1x
// INIT MEGA_AQ1x
#endif

// Configure each receiver pin for PCINT
void configureReceiver() {
  #ifndef Mega_AQ1x
  // pinMode(THROTTLEPIN, INPUT);
  // pinMode(ROLLPIN, INPUT);
  // pinMode(PITCHPIN, INPUT);
  // pinMode(YAWPIN, INPUT);
  // pinMode(MODEPIN, INPUT);
  // pinMode(AUXPIN, INPUT);
  init_servodecode();
  for (channel = ROLL; channel < LASTCHANNEL; channel++)
      pinData[receiverChannel[channel]].edge == FALLING_EDGE;

  // for (channel = ROLL; channel < LASTCHANNEL; channel++) {
  //   attachPinChangeInterrupt(receiverChannel[channel]);
  //   pinData[receiverChannel[channel]].edge == FALLING_EDGE;
  //}
  #endif
  
  #ifdef Mega_AQ1x
  initializeMegaPcInt2();
  for (channel = ROLL; channel < LASTCHANNEL; channel++)
    pinData[receiverChannel[channel]].edge == FALLING_EDGE;
  #endif
}

// Calculate PWM pulse width of receiver data
// If invalid PWM measured, use last known good time
unsigned int readReceiver(byte receiverPin) {
  
int pulsewidth;
uint16_t data;

  // print the decoder state
  if( ServoDecode.getState()!= READY_state) {
    //		Serial.print("The decoder is ");
    //		Serial.println(stateStrings[ServoDecode.getState()]);
    //for ( int i =0; i <=LASTCHANNEL; i++ )
    //		{ // print the status of the first four channels
    //	Serial.print("Cx"); // if you see this, the decoder does not have a valid signal
    //	Serial.print(i);
    //	Serial.print("= ");
    //	pulsewidth = ServoDecode.GetChannelPulseWidth(i);
    //	Serial.print(pulsewidth);
    //	Serial.print("  ");
    //	}
    //Serial.println("");
  }
  else {
    // decoder is ready, print the channel pulse widths
    // for ( int i =1; i <=LASTCHANNEL; i++ ){ // print the status of the first four channels
    //	Serial.print("Ch");
    //	Serial.print(i);
    //	Serial.print("= ");
    //	pulsewidth = ServoDecode.GetChannelPulseWidth(i);
    //	Serial.print(pulsewidth);
    //	Serial.print("  ");
    //}
    // Serial.println("");
   
    data=ServoDecode.GetChannelPulseWidth((int)receiverPin);
    pinData[receiverPin].lastGoodWidth=data; 
    pinData[receiverPin].edge=RISING_EDGE;
  
    //Serial.print("Ch");
    //Serial.print((int)receiverPin);
    //Serial.print("=");
    //Serial.print(data);
    //Serial.println("");
  //uint8_t oldSREG;
    
  //oldSREG = SREG;
  //cli();
  //data = pinData[receiverPin].lastGoodWidth;
  //SREG = oldSREG;  
   
  digitalWrite(12,LOW);
  digitalWrite(13,LOW);
  }

     return data;
}
