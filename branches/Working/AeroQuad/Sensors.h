/*
  AeroQuad v2.0 - January 2010
  www.AeroQuad.com
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

class Sensors:
public SubSystem
{
private:
  // Need to write a calibration routine for accelerometers (including Z axis)
  // These A/D values depend on how well the sensors are mounted
  // change these values to your unique configuration
  #define ZMIN 454
  #define ZMAX 687
  #define ZAXIS 2

  int _axis;
  // Accelerometer setup
  int _accelChannel[3];
  int _accelData[3];
  int _accelZero[3];
  int _accelADC[3];
  int _accelInvert[3];
  // Gyro setup
  int _gyroChannel[3];
  int _gyroData[3];
  int _gyroZero[3];
  int _gyroADC[3];
  int _gyroInvert[3];
  // Calibration parameters
  #define FINDZERO 50
  int _findZero[FINDZERO];

  int _findMode(int *data, int arraySize) {
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

public:
  // Required methods to implement a SubSystem
  Sensors():
  SubSystem()
  {
    // Perform any initalization of variables you need in the constructor of this SubSystem
    _gyroChannel[3] = {ROLLRATEPIN, PITCHRATEPIN, YAWRATEPIN};
    _accelChannel[3] = {ROLLACCELPIN, PITCHACCELPIN, ZACCELPIN};
    // Accelerometer setup
    _accelData[3] = {0,0,0};
    _accelZero[3] = {0,0,0};
    _accelADC[3] = {0,0,0};
    _accelInvert[3] = {1,1,0};
    // Gyro setup
    _gyroData[3] = {0,0,0};
    _gyroZero[3] = {0,0,0};
    _gyroADC[3] = {0,0,0};
    _gyroInvert[3] = {0,0,1};
  }

  void initialize(unsigned int frequency, unsigned int offset = 0)
  {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);

    //Perform any custom initalization you need for this SubSystem

    // Configure gyro auto zero pins
    pinMode (AZPIN, OUTPUT);
    pinMode (AZYAWPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    digitalWrite(AZYAWPIN, LOW);
  }

  void process(unsigned long currentTime)
  {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime))
    {
      //If the code reaches this point the SubSystem is allowed to run.

    // *********************** Read Sensors **********************
    // Apply low pass filter to sensor values and center around zero
    // Did not convert to engineering units, since will experiment to find P gain anyway
    for (_axis = ROLL; _axis < LASTAXIS; _axis++) {
      _gyroADC[_axis] = _gyroZero[_axis] - analogRead(_gyroChannel[_axis]);
      _accelADC[_axis] = analogRead(_accelChannel[_axis]) - _accelZero[_axis];
    }
    
    //gyroADC[YAW] = -gyroADC[YAW];
    //accelADC[ZAXIS] = -accelADC[ZAXIS];

    // Compiler seems to like calculating this in separate loop better
    for (_axis = ROLL; _axis < LASTAXIS; _axis++) {
      _gyroData[_axis] = gyro.smooth[_axis](_gyroADC[_axis]);
      _accelData[_axis] = accel.smooth[_axis](_accelADC[_axis];
    }

    // ****************** Calculate Absolute Angle *****************
    
    // Fix for calculating unfiltered flight angle per RoyLB
    // http://carancho.com/AeroQuad/forum/index.php?action=profile;u=77;sa=showPosts
    // perpendicular = sqrt((analogRead(accelChannel[ZAXIS]) - accelZero[ZAXIS])) ^2 + (analogRead(accelChannel[PITCH]) - accelZero[PITCH]) ^2)
    // flightAngle[ROLL] = atan2(analogRead(accelChannel[ROLL]) - accelZero[ROLL], perpendicular) * 57.2957795;    

    rawPitchAngle = atan2(_accelADC[PITCH], sqrt((_accelADC[ROLL] * _accelADC[ROLL]) + (_accelADC[ZAXIS] * _accelADC[ZAXIS])));
    rawRollAngle = atan2(_accelADC[ROLL], sqrt((_accelADC[PITCH] * _accelADC[PITCH]) + (_accelADC[ZAXIS] * _accelADC[ZAXIS])));
    
    #ifndef KalmanFilter
      //filterData(previousAngle, gyroADC, angle, *filterTerm, dt)
      flightAngle[ROLL] = filterData(flightAngle[ROLL], gyroADC[ROLL], rawRollAngle, filterTermRoll, AIdT);
      flightAngle[PITCH] = filterData(flightAngle[PITCH], gyroADC[PITCH], rawPitchAngle, filterTermPitch, AIdT);
      //flightAngle[ROLL] = filterData(flightAngle[ROLL], gyroData[ROLL], arctan2(accelData[ROLL], accelData[ZAXIS]), filterTermRoll, AIdT);
      //flightAngle[PITCH] = filterData(flightAngle[PITCH], gyroData[PITCH], arctan2(accelData[PITCH], accelData[ZAXIS]), filterTermPitch, AIdT);
    #endif
      
    #ifdef KalmanFilter
      predictKalman(&rollFilter, (gyroADC[ROLL]/1024) * aref * 8.72, AIdT);
      flightAngle[ROLL] = updateKalman(&rollFilter, atan2(accelADC[ROLL], accelADC[ZAXIS])) * 57.2957795;
      predictKalman(&pitchFilter, (gyroADC[PITCH]/1024) * aref * 8.72, AIdT);
      flightAngle[PITCH] = updateKalman(&pitchFilter, atan2(accelADC[PITCH], accelADC[ZAXIS])) * 57.2957795;
    #endif
    
    analogInputTime = currentTime;
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
    for (_axis = ROLL; _axis < LASTAXIS; _axis++) {
      for (int i=0; i<FINDZERO; i++) _findZero[i] = analogRead(_gyroChannel[axis]);
      _gyroZero[axis] = _findMode(_findZero, FINDZERO);
    }
    eeprom.write(_gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    eeprom.write(_gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    eeprom.write(_gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }

  void zeroAccelerometers() { // Finds ADC value which represents zero acceleration
    for (_axis = ROLL; _axis < YAW; _axis++) {
      for (int i=0; i<FINDZERO; i++) _findZero[i] = analogRead(_accelChannel[axis]);
      _accelZero[axis] = _findMode(_findZero, FINDZERO);
    }
    _accelZero[ZAXIS] = ZMAX - ((ZMAX - ZMIN)/2);
    eeprom.write(_accelZero[ROLL], LEVELROLLCAL_ADR);
    eeprom.write(_accelZero[PITCH], LEVELPITCHCAL_ADR);
    eeprom.write(_accelZero[ZAXIS], LEVELZCAL_ADR);
  }
  
  float arctan2(float y, float x) {
    // Taken from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
     float coeff_1 = PI/4;
     float coeff_2 = 3*coeff_1;
     float abs_y = abs(y)+1e-10;      // kludge to prevent 0/0 condition
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
 };

