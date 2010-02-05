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

// This class implements the PID control algorithm

#include "SubSystem.h"

class FlightControl_PID:
public SubSystem {
private:
  #define ROLL 0
  #define PITCH 1
  #define YAW 2
  #define LASTAXIS 3

  // PID Values
  #define LASTAXIS 3
  #define LEVELROLL 3
  #define LEVELPITCH 4
  #define LASTLEVELAXIS 5
  #define HEADING 5 // other axes defined in Receiver.h

  // Auto level setup
  int levelAdjust[2] = {0,0};
  int levelLimit; // Read in from EEPROM
  int levelOff; // Read in from EEPROM
  float rawRollAngle;
  float rawPitchAngle;

  // Heading hold
  // aref / 1024 = voltage per A/D bit
  // 0.002 = V / deg/sec (from gyro data sheet)
  float headingScaleFactor = (aref / 1024.0) / 0.002 * (PI/2.0);
  float heading = 0; // measured heading from yaw gyro (process variable)
  float headingHold = 0; // calculated adjustment for quad to go to heading (PID output)
  float currentHeading = 0; // current heading the quad is set to (set point)

  float AIdT = AILOOPTIME / 1000.0;
  float controldT = CONTROLLOOPTIME / 1000.0;
  
  struct PIDdata {
    float P, I, D;
    float lastPosition;
    float integratedError;
  } PID[6];
  float windupGuard; // Read in from EEPROM

  // Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
  float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters)
  {
    float error;
    float dTerm;

    error = targetPosition - currentPosition;
  
    PIDparameters->integratedError += error;
    if (PIDparameters->integratedError < -windupGuard) PIDparameters->integratedError = -windupGuard;
    else if (PIDparameters->integratedError > windupGuard) PIDparameters->integratedError = windupGuard;
  
    dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition);
    PIDparameters->lastPosition = currentPosition;
    return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
  }

  void zeroIntegralError() {
    for (axis = ROLL; axis < LASTLEVELAXIS; axis++)
      PID[axis].integratedError = 0;
  }
  
public:

  //Required methods to impliment for a SubSystem
  FlightControl_PID():
  SubSystem()
  {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    levelAdjust[ROLL] = 0;
    levelAdjust[PITCH] = 0;
  }

  void initialize(unsigned int frequency, unsigned int offset = 0)
  {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);

    //Perform any custom initalization you need for this SubSystem
      PID[ROLL].P = eeprom.read(PGAIN_ADR);
      PID[ROLL].I = eeprom.read(IGAIN_ADR);
      PID[ROLL].D = eeprom.read(DGAIN_ADR);
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
  
      PID[PITCH].P = eeprom.read(PITCH_PGAIN_ADR);
      PID[PITCH].I = eeprom.read(PITCH_IGAIN_ADR);
      PID[PITCH].D = eeprom.read(PITCH_DGAIN_ADR);
      PID[PITCH].lastPosition = 0;
      PID[PITCH].integratedError = 0;
  
      PID[YAW].P = eeprom.read(YAW_PGAIN_ADR);
      PID[YAW].I = eeprom.read(YAW_IGAIN_ADR);
      PID[YAW].D = eeprom.read(YAW_DGAIN_ADR);
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
  
      PID[LEVELROLL].P = eeprom.read(LEVEL_PGAIN_ADR);
      PID[LEVELROLL].I = eeprom.read(LEVEL_IGAIN_ADR);
      PID[LEVELROLL].D = eeprom.read(LEVEL_DGAIN_ADR);
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;  
  
      PID[LEVELPITCH].P = eeprom.read(LEVEL_PITCH_PGAIN_ADR);
      PID[LEVELPITCH].I = eeprom.read(LEVEL_PITCH_IGAIN_ADR);
      PID[LEVELPITCH].D = eeprom.read(LEVEL_PITCH_DGAIN_ADR);
      PID[LEVELPITCH].lastPosition = 0;
      PID[LEVELPITCH].integratedError = 0;
  
      PID[HEADING].P = eeprom.read(HEADING_PGAIN_ADR);
      PID[HEADING].I = eeprom.read(HEADING_IGAIN_ADR);
      PID[HEADING].D = eeprom.read(HEADING_DGAIN_ADR);
      PID[HEADING].lastPosition = 0;
      PID[HEADING].integratedError = 0;  
            
      smoothHeading = eeprom.read(HEADINGSMOOTH_ADR);
      windupGuard = eeprom.read(WINDUPGUARD_ADR);
      levelLimit = eeprom.read(LEVELLIMIT_ADR);
      levelOff = eeprom.read(LEVELOFF_ADR);
  }

  void process(unsigned long currentTime)
  {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime))
    {
      //If the code reaches this point the SubSystem is allowed to run.

      // ****************** Calculate Absolute Angle *****************
    
      // Fix for calculating unfiltered flight angle per RoyLB
      // http://carancho.com/AeroQuad/forum/index.php?action=profile;u=77;sa=showPosts
      // perpendicular = sqrt((analogRead(accelChannel[ZAXIS]) - accelZero[ZAXIS])) ^2 + (analogRead(accelChannel[ZAXIS]) - accelZero[ZAXIS]) ^2)
      // flightAngle[ROLL] = atan2(analogRead(accelChannel[ROLL]) - accelZero[ROLL], perpendicular) * 57.2957795;    

      rawPitchAngle = arctan2(accelADC[PITCH], sqrt((accelADC[ROLL] * accelADC[ROLL]) + (accelADC[ZAXIS] * accelADC[ZAXIS])));
      rawRollAngle = arctan2(accelADC[ROLL], sqrt((accelADC[PITCH] * accelADC[PITCH]) + (accelADC[ZAXIS] * accelADC[ZAXIS])));
    
      flightAngle[ROLL] = filterData(flightAngle[ROLL], gyroADC[ROLL], rawRollAngle, filterTermRoll, AIdT);
      flightAngle[PITCH] = filterData(flightAngle[PITCH], gyroADC[PITCH], rawPitchAngle, filterTermPitch, AIdT);

  // ********************* Check Flight Mode *********************
    #ifdef AutoLevel
      if (transmitterCommandSmooth[MODE] < 1500) {
        // Acrobatic Mode
        levelAdjust[ROLL] = 0;
        levelAdjust[PITCH] = 0;
      }
      else {
        // Stable Mode
        for (axis = ROLL; axis < YAW; axis++)
          levelAdjust[axis] = limitRange(updatePID(0, flightAngle[axis], &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
        // Turn off Stable Mode if transmitter stick applied
        if ((abs(receiverData[ROLL] - transmitterCenter[ROLL]) > levelOff)) {
          levelAdjust[ROLL] = 0;
          PID[axis].integratedError = 0;
        }
        if ((abs(receiverData[PITCH] - transmitterCenter[PITCH]) > levelOff)) {
          levelAdjust[PITCH] = 0;
          PID[PITCH].integratedError = 0;
        }
      }
    #endif
    
    // ************************** Update Roll/Pitch ***********************
    // updatedPID(target, measured, PIDsettings);
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
    motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL] + levelAdjust[ROLL], (gyroData[ROLL] * mMotorRate) + bMotorRate, &PID[ROLL]);
    motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH] - levelAdjust[PITCH], (gyroData[PITCH] * mMotorRate) + bMotorRate, &PID[PITCH]);

    // ***************************** Update Yaw ***************************
    // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
    // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
    #ifdef HeadingHold
      currentHeading += gyroData[YAW] * headingScaleFactor * controldT;
      if (transmitterCommand[THROTTLE] > MINCHECK ) { // apply heading hold only when throttle high enough to start flight
        if ((transmitterCommand[YAW] > (MIDCOMMAND + 25)) || (transmitterCommand[YAW] < (MIDCOMMAND - 25))) { // if commanding yaw, turn off heading hold
          headingHold = 0;
          heading = currentHeading;
        }
        else // no yaw input, calculate current heading vs. desired heading heading hold
          headingHold = updatePID(heading, currentHeading, &PID[HEADING]);
      }
      else {
        heading = 0;
        currentHeading = 0;
        headingHold = 0;
        PID[HEADING].integratedError = 0;
      }
      motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW] + headingHold, (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    #endif
    
    #ifndef HeadingHold
      motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW], (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    #endif
////////////////////    
    controlLoopTime = currentTime;
  } 
/////////////////////////
// End of control loop //
/////////////////////////
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionaly externally
  bool getBlinkState()
  {
    return _blinkState;
  }

  void setBlinkState(bool newBlinkState)
  {
    _blinkState = newBlinkState;
  }
};




