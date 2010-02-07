/*
  AeroQuad v1.6 - March 2010
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

// This class is responsible for calculating vehicle attitude

#include "SubSystem.h"
#include "Filter.h"
#include <math.h>

class Attitude_CompFilter: public SubSystem {
private:
  float dt;
  
  // Sensor Filter
  float flightAngle[2] = {0,0};
  float timeConstant; // Read in from EEPROM

  // Complementary roll/pitch angle
  float filterTermRoll[4] = {0,0,0,0};
  float filterTermPitch[4] = {0,0,0,0};
  
  // Heading
  float heading;
  float headingScaleFactor;
  Filter filterHeading;

  float CompFilter(float previousAngle, float newAngle, float newRate, float filterTerm, float dt) {
    // Written by RoyLB at http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286    
    filterTerm[0] = (newAngle - previousAngle) * timeConstant * timeConstant;
    filterTerm[2] += filterTerm[0] * dt;
    filterTerm[1] = filterTerm[2] + (newAngle - previousAngle) * 2 * timeConstant + newRate;
    return (filterTerm[1] * dt) + previousAngle;
  }

public:
  //Required methods to impliment for a SubSystem
  Attitude_CompFilter():SubSystem() {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    timeConstant = eeprom.read(FILTERTERM_ADR);
    filterHeading.initialize(eeprom.read(HEADINGSMOOTH_ADR));
  }

  void initialize(unsigned int frequency, unsigned int offset = 0) {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);

    //Perform any custom initalization you need for this SubSystem
    dt = frequency / 1000.0; 
    flightAngle[ROLL] = sensors.getAngleDegrees(ROLL);
    flightAngle[PITCH] = sensors.getAngleDegrees(PITCH);
    filterTermRoll[2] = -sensors.getRateDegPerSec(ROLL);
    filterTermPitch[2] = -sensors.getRateDegPerSec(PITCH);
    headingScaleFactor = sensors.getGyroScaleFactor();
  }
  
  void process(unsigned long currentTime) {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime)) {
      // Calculate absolute angle of vehicle
      flightAngle[ROLL] = CompFilter(flightAngle[ROLL], sensors.getAngleDegrees(ROLL), sensors.getRateDegPerSec(ROLL), filterTermRoll, dt);
      flightAngle[PITCH] = CompFilter(flightAngle[PITCH], sensors.getAngelDegrees(PITCH), sensors.getRateDegPerSec(PITCH), filterTermPitch, dt);
      // Calculate heading from yaw gyro
      heading = filterHeading.smooth(sensors.getGyro(YAW) * headingScaleFactor * dT);
    }
  }
  
  float getFlightAngle(byte axis) return flightAngle[axis];
  float getHeading() return heading;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Attitude_KalmanFilter:
public SubSystem
{
private:
  // Low pass filter parameters
  #define GYRO 0
  #define ACCEL 1
  float smoothFactor[2]; // Read in from EEPROM
  float smoothTransmitter[6]; // Read in from EEPROM
  float smoothHeading; // Read in from EEPROM

  // Kalman filter
  struct Gyro1DKalman {
    float x_angle, x_bias;
    float P_00, P_01, P_10, P_11;	
    float Q_angle, Q_gyro;
    float R_angle;
  };
  struct Gyro1DKalman rollFilter;
  struct Gyro1DKalman pitchFilter;

public:

  //Required methods to impliment for a SubSystem
  Attitude_KalmanFilter():
  SubSystem()
  {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    _blinkState = false;
    _blinkPin = 13;
  }

  void initialize(unsigned int frequency, unsigned int offset = 0)
  {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);

    //Perform any custom initalization you need for this SubSystem
    initGyro1DKalman(&rollFilter, 0.001, 0.003, 0.03);
    initGyro1DKalman(&pitchFilter, 0.001, 0.003, 0.03);
  }

  void process(unsigned long currentTime)
  {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime))
    {
      //If the code reaches this point the SubSystem is allowed to run.

      if (_blinkState)
      {
        digitalWrite(_blinkPin, HIGH);
      }
      else
      {
        digitalWrite(_blinkPin, LOW);
      }

      _blinkState = !_blinkState;
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionaly externally
  // The Kalman filter implementation is directly taken from the work
  // of Tom Pycke at: http://tom.pycke.be/mav/90/sparkfuns-5dof
  void initGyro1DKalman(struct Gyro1DKalman *filterdata, float Q_angle, float Q_gyro, float R_angle) {
  	filterdata->Q_angle = Q_angle;
  	filterdata->Q_gyro  = Q_gyro;
  	filterdata->R_angle = R_angle;
  }
  
  void predictKalman(struct Gyro1DKalman *filterdata, const float dotAngle, const float dt) {
  	filterdata->x_angle += dt * (dotAngle - filterdata->x_bias);
  	filterdata->P_00 +=  - dt * (filterdata->P_10 + filterdata->P_01) + filterdata->Q_angle * dt;
  	filterdata->P_01 +=  - dt * filterdata->P_11;
  	filterdata->P_10 +=  - dt * filterdata->P_11;
  	filterdata->P_11 +=  + filterdata->Q_gyro * dt;
  }
  
  float updateKalman(struct Gyro1DKalman *filterdata, const float angle_m) {
  	const float y = angle_m - filterdata->x_angle;
  	const float S = filterdata->P_00 + filterdata->R_angle;
  	const float K_0 = filterdata->P_00 / S;
  	const float K_1 = filterdata->P_10 / S;
  	
  	filterdata->x_angle +=  K_0 * y;
  	filterdata->x_bias  +=  K_1 * y;
  	filterdata->P_00 -= K_0 * filterdata->P_00;
  	filterdata->P_01 -= K_0 * filterdata->P_01;
  	filterdata->P_10 -= K_1 * filterdata->P_00;
  	filterdata->P_11 -= K_1 * filterdata->P_01;
  
  	return filterdata->x_angle;
  }
};
