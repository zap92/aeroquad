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

// This class is responsible for generating motor commands using the defined control algorithm
// This class will take data from Sensor, Attitude and FlightCommand
// FlightCommand data is used to reduce ControlLaw calculations (ie. auto level)

#include "SubSystem.h"

#include "Attitude.h"
FlightAngle_CompFilter flightAngle[2];

class ControlLaw_PID {
private:
  // Auto level setup
  byte autoLevel;
  int levelAdjust[2];
  int levelLimit; // Read in from EEPROM
  int levelOff; // Read in from EEPROM
  
  // Heading hold
  byte headingHold;
  float controldT;
  float headingScaleFactor;
  float heading; // measured heading from yaw gyro (process variable)
  float headingCommand; // calculated adjustment for quad to go to heading (PID output)
  float currentHeading; // current heading the quad is set to (set point)

  int motorCommand[3];
  float command[3];
  float measured[3];
  int axis;
  float mMotorCommand;		
  float bMotorCommand;

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
  ControlLaw_PID() {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    const float headingScaleFactor = (sensors.getAnalogReference() / 1024.0) / sensors.getGyroScaleFactor() * (PI/2.0);
    // Scale motor commands to analogWrite		
    // m = (250-126)/(2000-1000) = 0.124		
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2		
    // mMotorCommand = 0.124;		
    // bMotorCommand = 2;
    // mMotorCommand = flightControl.getMotorSlope();
    // bMotorCommand = flightControl.getMotorOffset();
    
    heading = 0; // measured heading from yaw gyro (process variable)
    headingCommand = 0; // calculated adjustment for quad to go to heading (PID output)
    currentHeading = 0; // current heading the quad is set to (set point)
    autoLevel = OFF;
    headingHold = OFF;
    levelAdjust[ROLL] = 0;
    levelAdjust[PITCH] = 0;
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      motorCommand[axis] = 0;
      command[axis] = 0;
      measured[axis] = 0;
    }
    headingCommand = 0;
    windupGuard = 0;
    controldT = sensors.getUpdateRate() / 1000.0;
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
  
  void intialize (void) {
    flightAngle[ROLL].initialize(sensors.getAngleDegrees(ROLL), sensors.getRateDegPerSec(ROLL), sensors.getUpdateRate());
    flightAngle[PITCH].initialize(sensors.getAngleDegrees(PITCH), sensors.getRateDegPerSec(PITCH), sensors.getUpdateRate());
  }

  void process(void) {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.

    attitude.process();
    
    // ************************** Update Auto Level ***********************
    if (autoLevel == ON) {
      if (flightCommand.read(MODE) < 1500) {
      // Acrobatic Mode
      levelAdjust[ROLL] = 0;
      levelAdjust[PITCH] = 0;
      }
      else {
        // Stable Mode
        for (axis = ROLL; axis < YAW; axis++)
          levelAdjust[axis] = constrain(updatePID(0, flightAngle[axis].read(sensors.getAngleDegrees(axis), sensors.getRateDegPerSec(axis)), &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
        // Turn off Stable Mode if transmitter stick applied
        if (flightCommand.read(ROLL) > levelOff) {
          levelAdjust[ROLL] = 0;
          PID[axis].integratedError = 0;
        }
        if (flightCommand.read(PITCH) > levelOff) {
          levelAdjust[PITCH] = 0;
          PID[PITCH].integratedError = 0;
        }
      }
    }

    // ***************************** Update Heading Hold ***************************
    // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
    // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
    if (headingHold == ON) {
      if (flightCommand.read(THROTTLE) > MINCHECK ) { // apply heading hold only when throttle high enough to start flight
        if ((flightCommand.read(YAW) > (MIDCOMMAND + 25)) || (flightCommand.read(YAW) < (MIDCOMMAND - 25))) { // if commanding yaw, turn off heading hold
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
    
    // ************************ Update Roll, Pitch and Yaw *************************
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      command[axis] = flightCommand.read(axis) + levelAdjust[ROLL];
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
      measured[axis] = (sensors.getGyro(ROLL) * mMotorCommand) + bMotorCommand;
    // updatePID(command, measured, PIDsettings);
      motorCommand[axis] = updatePID(command[axis], measured[axis], &PID[axis]);
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionaly externally
  int read(byte axis) {return motorCommand[axis];}
  void setAutoLevel(byte value) {autoLevel = value;}
  byte getAutoLevel(void) {return autoLevel;}
  void setHeadingHold(byte value) {headingHold = value;}
  byte getHeadingHold(void) {return headingHold;}
  void setP(byte axis, float value) {PID[axis].P = value;}
  float getP(byte axis) {return PID[axis].P;}
  void setI(byte axis, float value) {PID[axis].I = value;}
  float getI(byte axis) {return PID[axis].I;}
  void setD(byte axis, float value) {PID[axis].D = value;}
  float getD(byte axis) {return PID[axis].D;}
  void setInitPosError(byte axis) {
    PID[axis].lastPosition = 0;
    PID[axis].integratedError = 0;
  }
  float zeroIntegralError(void) {
    for (axis = ROLL; axis < HEADING + 1; axis++)
     PID[axis].integratedError = 0;
  }
  void setLevelLimit(float value) {levelLimit = value;}
  float getLevelLimit(void) {return levelLimit;}
  void setLevelOff(float value) {levelOff = value;}
  float getLevelOff(void) {return levelOff;}
  void setWindupGuard(float value) {windupGuard = value;}
  float getWindupGuard(void) {return windupGuard;}
  float getLevelAdjust(byte axis) {return levelAdjust[axis];}
  float getHeadingCommand(void) {return headingCommand;}
  float getHeading(void) {return heading;}
  float getCurrentHeading(void) {return currentHeading;}
};
