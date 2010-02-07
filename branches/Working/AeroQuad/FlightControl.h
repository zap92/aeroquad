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

// This class is responsible for generating motor commands using PID
// It takes data from the Sensor, Attitude and Receiver classes

#include "SubSystem.h"

class FlightControl_PID: public SubSystem {
private:
  // Auto level setup
  byte autoLevel;
  int levelAdjust[2] = {0,0};
  int levelLimit; // Read in from EEPROM
  int levelOff; // Read in from EEPROM
  
  // Heading hold
  byte headingHold;
  float controldT;
  float headingScaleFactor = (aref / 1024.0) / sensors.getGyroScaleFactor() * (PI/2.0);
  float heading = 0; // measured heading from yaw gyro (process variable)
  float headingCommand = 0; // calculated adjustment for quad to go to heading (PID output)
  float currentHeading = 0; // current heading the quad is set to (set point)

  int motorCommand[3];
  int axis;

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
  
public:
  //Required methods to impliment for a SubSystem
  FlightControl_PID(): SubSystem() {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    autoLevel = OFF
    headingHold = OFF
    levelAdjust[ROLL] = 0;
    levelAdjust[PITCH] = 0;
    for (axis = ROLL; axis < LASTAXIS; axis++) motorCommand[axis] = 0;
    headingCommand = 0;
    windupGuard = 0;
  }

  void initialize(unsigned int frequency, unsigned int offset = 0) {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);

    //Perform any custom initalization you need for this SubSystem
    controldT = frequency / 1000.0;
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
          
    windupGuard = eeprom.read(WINDUPGUARD_ADR);
    levelLimit = eeprom.read(LEVELLIMIT_ADR);
    levelOff = eeprom.read(LEVELOFF_ADR);
  }

  void process(unsigned long currentTime) {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime)) {
      //If the code reaches this point the SubSystem is allowed to run.
      if (autoLevel == ON) {
        if (receiver.getPilotCommand(MODE) < 1500) {
          // Acrobatic Mode
          levelAdjust[ROLL] = 0;
          levelAdjust[PITCH] = 0;
        }
        else {
          // Stable Mode
          for (axis = ROLL; axis < YAW; axis++)
            levelAdjust[axis] = filter.limitRange(updatePID(0, attitude.getFlightAngle[axis], &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
          // Turn off Stable Mode if transmitter stick applied
          if (receiver.getPilotCommand(ROLL) > levelOff)) {
            levelAdjust[ROLL] = 0;
            PID[axis].integratedError = 0;
          }
          if (receiver.getPilotCommand(PITCH) > levelOff) {
            levelAdjust[PITCH] = 0;
            PID[PITCH].integratedError = 0;
          }
        }
      }
      // ************************** Update Roll/Pitch ***********************
      // updatedPID(target, measured, PIDsettings);
      // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
      motorCommand[ROLL] = updatePID(receiver.getPilotCommand(ROLL) + levelAdjust[ROLL], (sensors.getGyro(ROLL) * mMotorRate) + bMotorRate, &PID[ROLL]);
      motorCommand[PITCH] = updatePID(receiver.getPilotCommand(PITCH) - levelAdjust[PITCH], (sensors.getGyro(PITCH) * mMotorRate) + bMotorRate, &PID[PITCH]);
  
      // ***************************** Update Yaw ***************************
      // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
      // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
      if (headingHold == ON) {
        if (receiver.getPilotCommand(THROTTLE) > MINCHECK ) { // apply heading hold only when throttle high enough to start flight
          if ((receiver.getPilotCommand(YAW) > (MIDCOMMAND + 25)) || (receiver.getPilotCommand(YAW) < (MIDCOMMAND - 25))) { // if commanding yaw, turn off heading hold
            headingCommand = 0;
            heading = attitude.getHeading();
          }
          else // no yaw input, calculate current heading vs. desired heading heading hold
            headingCommand = updatePID(heading, currentHeading, &PID[HEADING]);
        }
        else {
          heading = 0;
          currentHeading = 0;
          headingCommand = 0;
          PID[HEADING].integratedError = 0;
        }
      }
      motorCommand[YAW] = updatePID(receiver.getPilotCommand(YAW) + headingCommand, (attitude.getHeading() * mMotorRate) + bMotorRate, &PID[YAW]);
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionaly externally
  int getMotorCommand(byte axis) return motorCommand[axis];
  void enableAutoLevel(void) autoLevel = ON;
  void disableAutoLevel(void) autoLevel = OFF;
  void enableHeadingHold(void) headingHold = ON;
  void disableHeadingHold(void) headingHold = OFF;
};




