/*
  AeroReceiver 0.1 - April 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
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

#include <Wire.h>
#include "Receiver.h"
#define LEDPIN 13

uint8_t ledOutput = HIGH;
unsigned int receiver[10];

void setup()
{
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, ledOutput);
  
  configureReceiver();

  Wire.begin(29);
  Wire.onRequest(sendReceiverData);
}

void loop()
{
  ledOutput = (ledOutput == HIGH) ? ledOutput = LOW : ledOutput = HIGH;
  digitalWrite(LEDPIN, ledOutput);
  delay(100);
  /*for (channel = THROTTLE; channel < AUX5; channel++) {
    Serial.print(readReceiver(receiverPin[channel]));
    Serial.print(',');
  }
  Serial.println(readReceiver(receiverPin[AUX5]));*/
}

void sendReceiverData() {
  for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
    receiver[channel] = readReceiver(receiverPin[channel]);
  Wire.send((byte*)receiver, sizeof(receiver));
}
