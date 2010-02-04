/*
 AeroQuad v1.6 - March 2010
 www.AeroQuad.info
 Copyright (c) 2010 Ted Carancho.  All rights reserved.
 An Open Source Arduino based quadrocopter.
 
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

#include "SubSystem.h"
#include "pins_arduino.h"
#define byte uint8_t 

class Receiver:
public SubSystem {
private:
  // Receiver pin definitions
  #define THROTTLEPIN 10
  #define ROLLPIN 11
  #define PITCHPIN 12
  #define YAWPIN 67
  #define MODEPIN 68
  #define AUXPIN 69
  #define AUX2PIN 69 // need to fix in shield
  int receiverPin[7];
  int receiverChannel[7];

  // Receiver variables
  #define TIMEOUT 25000
  #define MINCOMMAND 1000
  #define MIDCOMMAND 1500
  #define MAXCOMMAND 2000
  #define MINDELTA 200
  #define MINCHECK MINCOMMAND + 100
  #define MAXCHECK MAXCOMMAND - 100
  #define MINTHROTTLE MINCOMMAND + 100
  #define LEVELOFF 100
  #define ROLL 0
  #define PITCH 1
  #define YAW 2
  #define THROTTLE 3
  #define MODE 4
  #define AUX 5
  #define LASTCHANNEL 6
  #define RISING_EDGE 1
  #define FALLING_EDGE 0
  #define MINONWIDTH 950
  #define MAXONWIDTH 2075
  #define MINOFFWIDTH 12000
  #define MAXOFFWIDTH 24000

  volatile uint8_t *port_to_pcmask[];
  volatile static uint8_t PCintLast[12];

  // Channel data 
  typedef struct {
    byte edge;
    unsigned long riseTime;    
    unsigned long fallTime; 
    unsigned long lastGoodWidth;
  } pinTimingData;  
  volatile static pinTimingData pinData[24]; 
  
  int receiverData[6];
  int transmitterCommand[6];
  int transmitterCommandSmooth[6];
  int transmitterZero[3];
  int transmitterCenter[3];
  byte channel;
  
  // Arm motor safety check 
  byte armed = 0;
  byte safetyCheck = 0;
  byte update = 0;

  static void PcintISR(uint8_t port) {
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
    
  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  unsigned int read(byte receiverPin) {
    uint16_t data;
    uint8_t oldSREG;
    
    oldSREG = SREG;
    cli();
    data = pinData[receiverPin].lastGoodWidth;
    SREG = oldSREG;  
    return data;
  }

public:
  //Required methods to impliment for a SubSystem
  Receiver():
  SubSystem(){
    //Perform any initalization of variables you need in the constructor of this SubSystem
    receiverPin[7] = {5,6,21,4,22,23,24}; // Corresponds to port number on ATmega (not Arduino pin number)
    receiverChannel[7] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, MODEPIN, AUXPIN, AUX2PIN};
    port_to_pcmask[13] = {&PCMSK0, &PCMSK1, 0, 0, 0, 0, 0, 0, 0, 0, 0, &PCMSK2, 0};
    transmitterCommand[6] = {1500,1500,1500,1000,1000,1000};
    transmitterCommandSmooth[6] = {0,0,0,0,0,0};
    transmitterZero[3] = {1500,1500,1500};
    transmitterCenter[3] = {1500,1500,1500};
  }

  void initialize(unsigned int frequency, unsigned int offset = 0)
  {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);

    //Perform any custom initalization you need for this SubSystem
    pinMode(THROTTLEPIN, INPUT);
    pinMode(ROLLPIN, INPUT);
    pinMode(PITCHPIN, INPUT);
    pinMode(YAWPIN, INPUT);
    pinMode(MODEPIN, INPUT);
    pinMode(AUXPIN, INPUT);
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      pinData[receiverChannel[channel]].edge == FALLING_EDGE;
    
    // Configure PORTB
    PCMSK0 |= 0x70; // Select which pins to use for PCINT
    PCICR |= 0x01 << 0; // Enable PCINT for PORTB
  
    // Configure PORTK
    PCMSK2 |= 0xE0; // Select which pins to use for PCINT
    PCICR |= 0x01 << 2; // Enable PCINT for PORTK

    SIGNAL(PCINT0_vect) {
      PcintISR(2);
    }
    SIGNAL(PCINT2_vect) {
      PcintISR(11);
      
      mTransmitter[THROTTLE] = eeprom.read(THROTTLESCALE_ADR);
      bTransmitter[THROTTLE] = eeprom.read(THROTTLEOFFSET_ADR);
      mTransmitter[ROLL] = eeprom.read(ROLLSCALE_ADR);
      bTransmitter[ROLL] = eeprom.read(ROLLOFFSET_ADR);
      mTransmitter[PITCH] = eeprom.read(PITCHSCALE_ADR);
      bTransmitter[PITCH] = eeprom.read(PITCHOFFSET_ADR);
      mTransmitter[YAW] = eeprom.read(YAWSCALE_ADR);
      bTransmitter[YAW] = eeprom.read(YAWOFFSET_ADR);
      mTransmitter[MODE] = eeprom.read(MODESCALE_ADR);
      bTransmitter[MODE] = eeprom.read(MODEOFFSET_ADR);
      mTransmitter[AUX] = eeprom.read(AUXSCALE_ADR);
      bTransmitter[AUX] = eeprom.read(AUXOFFSET_ADR);
      smoothTransmitter[THROTTLE] = eeprom.read(THROTTLESMOOTH_ADR);
      smoothTransmitter[ROLL] = eeprom.read(ROLLSMOOTH_ADR);
      smoothTransmitter[PITCH] = eeprom.read(PITCHSMOOTH_ADR);
      smoothTransmitter[YAW] = eeprom.read(YAWSMOOTH_ADR);
      smoothTransmitter[MODE] = eeprom.read(MODESMOOTH_ADR);
      smoothTransmitter[AUX] = eeprom.read(AUXSMOOTH_ADR);
      xmitFactor = eeprom.read(XMITFACTOR_ADR);

      safetyCheck = 0;
    }
  }

  void process(unsigned long currentTime)
  {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime))
    {
      //If the code reaches this point the SubSystem is allowed to run.
      for (channel = ROLL; channel < LASTCHANNEL; channel++)
        receiverData[channel] = (mTransmitter[channel] * read(receiverPin[channel])) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs (roll, pitch, yaw, throttle)
      for (channel = ROLL; channel < LASTCHANNEL; channel++)
        transmitterCommandSmooth[channel] = smooth(receiverData[channel], transmitterCommandSmooth[channel], smoothTransmitter[channel]);
      // Reduce transmitter commands using xmitFactor and center around 1500
      for (channel = ROLL; channel < LASTAXIS; channel++)
        transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
      // No xmitFactor reduction applied for throttle, mode and AUX
      for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
        transmitterCommand[channel] = transmitterCommandSmooth[channel];
      // Read quad configuration commands from transmitter when throttle down
      if (receiverData[THROTTLE] < MINCHECK) {
        zeroIntegralError();
        // Disarm motors (left stick lower left corner)
        if (receiverData[YAW] < MINCHECK && armed == 1) {
          armed = 0;
          commandAllMotors(MINCOMMAND);
        }    
        // Zero sensors (left stick lower left, right stick lower right corner)
        if ((receiverData[YAW] < MINCHECK) && (receiverData[ROLL] > MAXCHECK) && (receiverData[PITCH] < MINCHECK)) {
          autoZeroGyros();
          zeroGyros();
          zeroAccelerometers();
          zeroIntegralError();
          pulseMotors(3);
        }   
        // Arm motors (left stick lower right corner)
        if (receiverData[YAW] > MAXCHECK && armed == 0 && safetyCheck == 1) {
          armed = 1;
          zeroIntegralError();
          minCommand = MINTHROTTLE;
          transmitterCenter[PITCH] = receiverData[PITCH];
          transmitterCenter[ROLL] = receiverData[ROLL];
        }
        // Prevents accidental arming of motor output if no transmitter command received
        if (receiverData[YAW] > MINCHECK) safetyCheck = 1; 
      }
      // Prevents too little power applied to motors during hard manuevers
      if (receiverData[THROTTLE] > (MIDCOMMAND - MINDELTA)) minCommand = receiverData[THROTTLE] - MINDELTA;
      if (receiverData[THROTTLE] < MINTHROTTLE) minCommand = MINTHROTTLE;
      // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
      //if ((receiverData[ROLL] < MINCHECK) || (receiverData[ROLL] > MAXCHECK) || (receiverData[PITCH] < MINCHECK) || (receiverData[PITCH] > MAXCHECK))
        //minCommand = MINTHROTTLE;
    }
  }
};




