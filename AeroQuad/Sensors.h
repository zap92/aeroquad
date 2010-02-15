/*
  AeroQuad v1.6 - March 2010
  www.AeroQuad.com
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

// This class is responsible for reading sensor data and applying smoothing

#include "SubSystem.h"
#include "Filter.h"

// Sensor pin assignments
#define PITCHACCELPIN 0
#define ROLLACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 4
#define ROLLRATEPIN 3
#define YAWRATEPIN 5
#define AZPIN 22
#define AZYAWPIN 23

// Need to write a calibration routine for accelerometers (including Z axis)
// These A/D values depend on how well the sensors are mounted
// change these values to your unique configuration
#define ZMIN 454
#define ZMAX 687
#define ZAXIS 2
#define FINDZERO 50

class Sensors: public SubSystem {
private:
  float aref; // Analog Reference Value measured with a DMM
  float gyroScaleFactor; // Gyro scale factor from datasheet
  int findZero[FINDZERO];
  int axis; // Use as an index

  // Accelerometer setup
  Filter gyroFilter[3];
  int accelChannel[3];
  int accelData[3];
  int accelZero[3];
  int accelADC[3];
  int accelInvert[3];
  // Gyro setup
  Filter accelFilter[3];
  int gyroChannel[3];
  int gyroData[3];
  int gyroZero[3];
  int gyroADC[3];
  int gyroInvert[3];

  int findMode(int *data, int arraySize) {
    // The mode of a set of numbers is the value that occurs most frequently
    byte i;
    int temp, maxData;
    boolean done = 0;
    int frequency = 0;
    int maxFrequency = 0;
  
    // Sorts numbers from lowest to highest
    while (done != 1) {        
      done = 1;
      for (i=0; i<(arraySize-1); i++) {
        if (data[i] > data[i+1]) {     // numbers are out of order - swap
          temp = data[i+1];
          data[i+1] = data[i];
          data[i] = temp;
          done = 0;
        }
      }
    }
  
    // Count number of times a value occurs in sorted array
    for (i=0; i<arraySize; i++) {
      if (data[i] > temp) {
        frequency = 0;
        temp = data[i];
        frequency++;
      } else if (data[i] == temp) frequency++;
      if (frequency > maxFrequency) {
        maxFrequency = frequency;
        maxData = data[i];
      }
    }
    return maxData;
  }

  float arctan2(float y, float x) {
    // Taken from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
    float coeff_1 = PI/4;
    float coeff_2 = 3*coeff_1;
    float abs_y = abs(y) + 0.0000000001;      // kludge to prevent 0/0 condition
    float r, angle;

    if (x >= 0) {
      r = (x - abs_y) / (x + abs_y);
      angle = coeff_1 - coeff_1 * r;
    }
    else {
      r = (x + abs_y) / (abs_y - x);
      angle = coeff_2 - coeff_1 * r;
    }
    if (y < 0)
      return(-angle);     // negate if in quad III or IV
    else
      return(angle);
  }

public:
  // Required methods to implement a SubSystem
  Sensors(): SubSystem() {  
    const float aref = 2.725; // Analog Reference Value measured with a DMM
    const float gyroScaleFactor = 0.002; // Gyro scale factor from datasheet
  
    // Perform any initalization of variables you need in the constructor of this SubSystem
    gyroChannel[ROLL] = ROLLRATEPIN;
    gyroChannel[PITCH] = PITCHRATEPIN;
    gyroChannel[YAW] = YAWRATEPIN;
    accelChannel[ROLL] = ROLLACCELPIN;
    accelChannel[PITCH] = PITCHACCELPIN;
    accelChannel[YAW] = ZACCELPIN;
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      // Accelerometer setup
      accelData[axis] = 0;
      accelZero[axis] = 0;
      accelADC[axis] = 0;
      accelInvert[axis] = 1;
      // Gyro setup
      gyroData[axis] = 0;
      gyroZero[axis] = 0;
      gyroADC[axis] = 0;
      gyroInvert[axis] = 1;
    }
  }

  void initialize(unsigned int frequency, unsigned int offset = 0) {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);

    //Perform any custom initalization you need for this SubSystem
    // Configure gyro auto zero pins
    analogReference(EXTERNAL); // Current external ref is connected to 3.3V
    pinMode (AZPIN, OUTPUT);
    pinMode (AZYAWPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    digitalWrite(AZYAWPIN, LOW);
    
    accelZero[ROLL] = eeprom.read(LEVELROLLCAL_ADR);
    accelZero[PITCH] = eeprom.read(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = eeprom.read(LEVELZCAL_ADR);
    gyroZero[ROLL] = eeprom.read(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = eeprom.read(GYRO_PITCH_ZERO_ADR);
    gyroZero[YAW] = eeprom.read(GYRO_YAW_ZERO_ADR);
    for (axis = ROLL; axis < LASTAXIS; axis++) gyroFilter[axis].initialize(eeprom.read(GYROSMOOTH_ADR));
    for (int axis = ROLL; axis < LASTAXIS; axis++) accelFilter[axis].initialize(eeprom.read(ACCSMOOTH_ADR));
  }

  void process(unsigned long currentTime) {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime)) {
      //If the code reaches this point the SubSystem is allowed to run.

      // *********************** Read Sensors **********************
      // Apply low pass filter to sensor values and center around zero
      // Did not convert to engineering units, since will experiment to find P gain anyway
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        gyroADC[axis] =  (analogRead(gyroChannel[axis]) - gyroZero[axis]) * gyroInvert[axis];
        accelADC[axis] = (analogRead(accelChannel[axis]) - accelZero[axis]) * accelInvert[axis];
      }
    
      // Compiler seems to like calculating this in separate loop better
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        gyroData[axis] = gyroFilter[axis].smooth(gyroADC[axis]);
        accelData[axis] = accelFilter[axis].smooth(accelADC[axis]);
      }
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void zeroGyros() {
    // Pulse AZ pins on gyros to perform autozero
    digitalWrite(AZPIN, HIGH);
    digitalWrite(AZYAWPIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(AZPIN, LOW);
    digitalWrite(AZYAWPIN, LOW);
    delay(8);
    // Due to gyro drift, finds ADC value which represents zero rate
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      for (int i=0; i<FINDZERO; i++) findZero[i] = analogRead(gyroChannel[axis]);
      gyroZero[axis] = findMode(findZero, FINDZERO);
    }
    eeprom.write(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    eeprom.write(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    eeprom.write(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }

  void zeroAccelerometers() { // Finds ADC value which represents zero acceleration
    for (axis = ROLL; axis < YAW; axis++) {
      for (int i=0; i<FINDZERO; i++) findZero[i] = analogRead(accelChannel[axis]);
      accelZero[axis] = findMode(findZero, FINDZERO);
    }
    accelZero[ZAXIS] = ZMAX - ((ZMAX - ZMIN)/2);
    eeprom.write(accelZero[ROLL], LEVELROLLCAL_ADR);
    eeprom.write(accelZero[PITCH], LEVELPITCHCAL_ADR);
    eeprom.write(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
  
  int getRawGyro(byte axis) {return gyroADC[axis];}
  int getRawAccel(byte axis) {return accelADC[axis];}
  int getGyro(byte axis) {return gyroData[axis];}
  int getAccel(byte axis) {return accelData[axis];}
  float getGyroScaleFactor(void) {return gyroScaleFactor;}
  float getRateDegPerSec(byte axis) {return (gyroADC[axis] / 1024) * aref / gyroScaleFactor;}
  float getRateRadPerSec(byte axis) {return radians(getRateRadPerSec(axis));}
  float getAngleDegrees(byte axis) {return degrees(getAngleRadians(axis));} 
  float getAngleRadians(byte axis) {
    if (axis == PITCH) return arctan2(accelADC[PITCH], sqrt((accelADC[ROLL] * accelADC[ROLL]) + (accelADC[ZAXIS] * accelADC[ZAXIS])));
    if (axis == ROLL) return arctan2(accelADC[ROLL], sqrt((accelADC[PITCH] * accelADC[PITCH]) + (accelADC[ZAXIS] * accelADC[ZAXIS])));
  }
  void setGyroSmoothFactor(byte axis, float value) {gyroFilter[axis].setSmoothFactor(value);}
  float getGyroSmoothFactor(byte axis) {return gyroFilter[axis].getSmoothFactor();}
  void setAccelSmoothFactor(byte axis, float value) {accelFilter[axis].setSmoothFactor(value);}
  float getAccelSmoothFactor(byte axis) {return accelFilter[axis].getSmoothFactor();}
  float getAnalogReference(void) {return aref;}
};

