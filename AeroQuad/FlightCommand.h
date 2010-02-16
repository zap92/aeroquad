/*
 AeroQuad v1.6 - March 2010
 www.AeroQuad.info
 Copyright (c) 2010 Ted Carancho, Chris Whiteford.  All rights reserved.
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
 
// This class is responsible for reading external pilot commands

#include "SubSystem.h"
#include "pins_arduino.h"

// Receiver pin definitions
#define ROLLPIN 2
#define PITCHPIN 5
#define YAWPIN 6
#define THROTTLEPIN 4
#define MODEPIN 7
#define AUXPIN 8

// Remote Command Variables
#define LEVELOFF 100
#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100


class FlightCommand_Duemilanove: public SubSystem {
private:
  int receiverPin[6];
  int receiverChannel[6];
  volatile uint8_t *port_to_pcmask[3];
  volatile static uint8_t PCintLast[3];
  Filter filter[6];

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
  float smoothTransmitter[6];
  byte channel;
  
  // Controls the strength of the commands sent from the transmitter
  // xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
  float xmitFactor; // Read in from EEPROM
  float mTransmitter[6];
  float bTransmitter[6];
  
  // Attaches PCINT to Arduino Pin
  void attachPinChangeInterrupt(uint8_t pin) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    uint8_t slot;
    volatile uint8_t *pcmask;
  
    // map pin to PCIR register
    if (port == NOT_A_PORT) {
      return;
    } 
    else {
      port -= 2;
      pcmask = port_to_pcmask[port];
    }
    // set the mask
    *pcmask |= bit;
    // enable the interrupt
    PCICR |= 0x01 << port;
  }
  
  // ISR which records time of rising or falling edge of signal
  void measurePulseWidthISR(uint8_t port) {
    uint8_t bit;
    uint8_t curr;
    uint8_t mask;
    uint8_t pin;
    uint32_t currentTime;
    uint32_t time;
  
    // get the pin states for the indicated port.
    curr = *portInputRegister(port+2);
    mask = curr ^ PCintLast[port];
    PCintLast[port] = curr;
    // mask is pins that have changed. screen out non pcint pins.
    if ((mask &= *port_to_pcmask[port]) == 0) {
      return;
    }
    currentTime = micros();
    // mask is pcint pins that have changed.
    for (uint8_t i=0; i < 8; i++) {
      bit = 0x01 << i;
      if (bit & mask) {
        pin = port * 8 + i;
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
  unsigned int readPWM(byte receiverPin) {
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
  FlightCommand_Duemilanove(): SubSystem() {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    receiverPin[ROLL] = 18;
    receiverPin[PITCH] = 21;
    receiverPin[YAW] = 22;
    receiverPin[THROTTLE] = 20;
    receiverPin[MODE] = 23;
    receiverPin[AUX] = 0;
    receiverChannel[ROLL] = ROLLPIN;
    receiverChannel[PITCH] = PITCHPIN;
    receiverChannel[YAW] = YAWPIN;
    receiverChannel[THROTTLE] = THROTTLEPIN;
    receiverChannel[MODE] = MODEPIN;
    receiverChannel[AUX] = AUXPIN;
    transmitterCommand[ROLL] = 1500;
    transmitterCommand[PITCH] = 1500;
    transmitterCommand[YAW] = 1500;
    transmitterCommand[THROTTLE] = 1000;
    transmitterCommand[MODE] = 1000;
    transmitterCommand[AUX] = 1000;
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      transmitterCommandSmooth[channel] = 0;
    for (channel = ROLL; channel < LASTAXIS; channel++) {
      transmitterZero[channel] = 1500;
      transmitterCenter[channel] = 1500;
    }
    port_to_pcmask[0] = &PCMSK0;
    port_to_pcmask[1] = &PCMSK1;
    port_to_pcmask[2] = &PCMSK2;
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      pinData[receiverChannel[channel]].edge == FALLING_EDGE;

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
    filter[THROTTLE].setSmoothFactor(eeprom.read(THROTTLESMOOTH_ADR));
    filter[ROLL].setSmoothFactor(eeprom.read(ROLLSMOOTH_ADR));
    filter[PITCH].setSmoothFactor(eeprom.read(PITCHSMOOTH_ADR));
    filter[YAW].setSmoothFactor(eeprom.read(YAWSMOOTH_ADR));
    filter[MODE].setSmoothFactor(eeprom.read(MODESMOOTH_ADR));
    filter[AUX].setSmoothFactor(eeprom.read(AUXSMOOTH_ADR));
    xmitFactor = eeprom.read(XMITFACTOR_ADR);
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
  }

  void process(unsigned long currentTime) {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime)) {
      //If the code reaches this point the SubSystem is allowed to run.
      for (channel = ROLL; channel < LASTCHANNEL; channel++)
        receiverData[channel] = (mTransmitter[channel] * readPWM(receiverPin[channel])) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs (roll, pitch, yaw, throttle)
      for (channel = ROLL; channel < LASTCHANNEL; channel++)
        transmitterCommandSmooth[channel] = filter[channel].smooth(receiverData[channel]);
      // Reduce transmitter commands using xmitFactor and center around 1500
      for (channel = ROLL; channel < LASTAXIS; channel++)
        transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
      // No xmitFactor reduction applied for throttle, mode and AUX
      for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
        transmitterCommand[channel] = transmitterCommandSmooth[channel];
    }
  }
  
  int read(byte axis) {return transmitterCommand[axis];}
  void setXmitFactor(float value) {xmitFactor = value;}
  float getXmitFactor(void) {return xmitFactor;}
  void setTransmitterSmoothing(byte axis, float value) {filter[axis].setSmoothFactor(value);}
  float getTransmitterSmoothing(byte axis) {return filter[axis].getSmoothFactor();}
  void setTransmitterSlope(byte axis, float value) {mTransmitter[axis] = value;}
  float getTransmitterSlope(byte axis) {return mTransmitter[axis];}
  void setTransmitterOffset(byte axis, float value) {bTransmitter[axis] = value;}
  float getTransmitterOffset(byte axis) {return bTransmitter[axis];}
  int getScaledReceiverData(byte axis) {return receiverData[axis];}
};

