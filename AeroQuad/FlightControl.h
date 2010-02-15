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

// This class is responsible for generating motor commands using PID
// It takes data from the Sensor, Attitude and Receiver classes

#include "SubSystem.h"

#include "ControlLaw.h"
ControlLaw_PID controlLaw;

class FlightControl: public SubSystem {
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
  int axis;
  float mMotorCommand;		
  float bMotorCommand;
  
public:
  //Required methods to impliment for a SubSystem
  FlightControl(): SubSystem() {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    // Scale motor commands to analogWrite		
    // m = (250-126)/(2000-1000) = 0.124		
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2		
    // mMotorCommand = 0.124;		
    // bMotorCommand = 2;
    
    heading = 0; // measured heading from yaw gyro (process variable)
    headingCommand = 0; // calculated adjustment for quad to go to heading (PID output)
    currentHeading = 0; // current heading the quad is set to (set point)
    for (axis = ROLL; axis < LASTAXIS; axis++) motorCommand[axis] = 0;
    headingCommand = 0;
  }

  void initialize(unsigned int frequency, unsigned int offset = 0) {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);
      mMotorCommand = motors.getMotorSlope();
      bMotorCommand = motors.getMotorOffset();
  }

  void process(unsigned long currentTime) {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime)) {
      //If the code reaches this point the SubSystem is allowed to run.
      controlLaw.process();

      // Set motor commands based on receiver input
      // If throttle in minimum position, don't apply yaw
      if (flightCommand.getCommand[THROTTLE] < MINCHECK) {
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motors.setMotorCommand(motor, minCommand);
      }
      // If motor output disarmed, force motor output to minimum
      if (armed == ON) {
        switch (calibrateESC) { // used for calibrating ESC's
        case 1:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motors.setMotorCommand(motor, MAXCOMMAND);
          break;
        case 3:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motors.setMotorCommand(motor, constrain(testCommand, 1000, 1200));
          break;
        case 5:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motors.setMotorCommand(motor, constrain(remoteCommand[motor], 1000, 1200));
          safetyCheck = ON;
          break;
        default:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motors.setMotorCommand(motor, MINCOMMAND);
        }
      }

      // Read quad configuration commands from transmitter when throttle down
      if (flightCommand.getCommand[THROTTLE] < MINCHECK) {
        controlLaw.zeroIntegralError();
        // Disarm motors (left stick lower left corner)
        if (receiverData[YAW] < MINCHECK && armed == ON) {
          armed = OFF;
          motors.commandAllMotors(MINCOMMAND);
        }    
        // Zero sensors (left stick lower left, right stick lower right corner)
        if ((receiverData[YAW] < MINCHECK) && (receiverData[ROLL] > MAXCHECK) && (receiverData[PITCH] < MINCHECK)) {
          flight.zeroGyros();
          sensors.zeroAccelerometers();
          controlLaw.zeroIntegralError();
          motors.pulseMotors(3);
        }   
        // Arm motors (left stick lower right corner)
        if (receiverData[YAW] > MAXCHECK && armed == OFF && safetyCheck == ON) {
          armed = ON;
          controlLaw.zeroIntegralError();
          minCommand = MINTHROTTLE;
          transmitterCenter[PITCH] = receiverData[PITCH];
          transmitterCenter[ROLL] = receiverData[ROLL];
        }
        // Prevents accidental arming of motor output if no transmitter command received
        if (receiverData[YAW] > MINCHECK) safetyCheck = ON; 
      }
      // Prevents too little power applied to motors during hard manuevers
      if (receiverData[THROTTLE] > (MIDCOMMAND - MINDELTA)) minCommand = receiverData[THROTTLE] - MINDELTA;
      if (receiverData[THROTTLE] < MINTHROTTLE) minCommand = MINTHROTTLE;
      // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
      //if ((receiverData[ROLL] < MINCHECK) || (receiverData[ROLL] > MAXCHECK) || (receiverData[PITCH] < MINCHECK) || (receiverData[PITCH] > MAXCHECK))
	  //minCommand = MINTHROTTLE;
    }	
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionaly externally
  int getMotorCommand(byte axis) {return motorCommand[axis];}
  void enableAutoLevel(void) {autoLevel = ON;}
  void disableAutoLevel(void) {autoLevel = OFF;}
  byte getAutoLevelState(void) {return autoLevel;}
  void enableHeadingHold(void) {headingHold = ON;}
  void disableHeadingHold(void) {headingHold = OFF;}
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




