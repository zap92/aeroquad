/*
  AeroQuad v1.7 - March 2010
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

// This class is responsible interpreting pilot commands from the FlightCommand class
// and to send motor commands to the Motor class as calculated from the ControlLaw class

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

  int motorCommand[4];
  int remoteCommand[4];
  byte axis;
  byte motor;
  float mMotorCommand;		
  float bMotorCommand;
  int minCommand;
  int maxCommand;
  
  // ESC Calibration
  byte calibrateESC;
  int testCommand;

  // Arm motor safety check 
  byte armed;
  byte safetyCheck;
 
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
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = 0;
      remoteCommand[motor] = 0;
    }
    headingCommand = 0;
    
    safetyCheck = OFF;
    armed = OFF;
    minCommand = MINCOMMAND;
    calibrateESC = 0;
  }

  void initialize(unsigned int frequency, unsigned int offset = 0) {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);
    mMotorCommand = motors.getMotorSlope();
    bMotorCommand = motors.getMotorOffset();
    controlLaw.initialize();
  }

  void process(unsigned long currentTime) {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime)) {
      //If the code reaches this point the SubSystem is allowed to run.
      controlLaw.process();

      // Read quad configuration commands from transmitter when throttle down
      if (flightCommand.read(THROTTLE) < MINCHECK) {
        controlLaw.zeroIntegralError();
        // Disarm motors (left stick lower left corner)
        if (flightCommand.read(YAW) < MINCHECK && armed == ON) {
          armed = OFF;
          motors.commandAllMotors(MINCOMMAND);
        }    
        // Zero/Calibrate sensors (left stick lower left, right stick lower right corner)
        if ((flightCommand.read(YAW) < MINCHECK) && (flightCommand.read(ROLL) > MAXCHECK) && (flightCommand.read(PITCH) < MINCHECK)) {
          sensors.zeroGyros();
          sensors.zeroAccelerometers();
          controlLaw.zeroIntegralError();
          motors.pulseMotors(3);
        }   
        // Arm motors (left stick lower right corner)
        if (flightCommand.read(YAW) > MAXCHECK && armed == OFF && safetyCheck == ON) {
          armed = ON;
          controlLaw.zeroIntegralError();
          minCommand = MINTHROTTLE;
          //transmitterCenter[PITCH] = flightCommand.read(PITCH);
          //transmitterCenter[ROLL] = flightCommand.read(ROLL);
        }
        // Prevents accidental arming of motor output if no transmitter command received
        if (flightCommand.read(YAW) > MINCHECK) safetyCheck = ON; 
      }
      
      // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
      //if ((receiverData[ROLL] < MINCHECK) || (receiverData[ROLL] > MAXCHECK) || (receiverData[PITCH] < MINCHECK) || (receiverData[PITCH] > MAXCHECK))
      //minCommand = MINTHROTTLE;
    
      // If throttle in minimum position, don't apply yaw
      if (flightCommand.read(THROTTLE) < MINCHECK) {
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motorCommand[motor] = MINTHROTTLE;
      }
      
      // If motor output disarmed, force motor output to minimum
      if (armed == ON) {
        switch (calibrateESC) { // used for calibrating ESC's
        case 1:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motors.write(motor, MAXCOMMAND);
          break;
        case 3:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motors.write(motor, constrain(testCommand, 1000, 1200));
          break;
        case 5:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motors.write(motor, constrain(remoteCommand[motor], 1000, 1200));
          safetyCheck = ON;
          break;
        default:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motors.write(motor, MINCOMMAND);
        }
      }

      // ****************** Calculate Motor Commands *****************
      if (armed == ON && safetyCheck == ON) {
        motorCommand[FRONT] = flightCommand.read(THROTTLE) - controlLaw.read(PITCH) - controlLaw.read(YAW);
        motorCommand[REAR] = flightCommand.read(THROTTLE) + controlLaw.read(PITCH) - controlLaw.read(YAW);
        motorCommand[RIGHT] = flightCommand.read(THROTTLE) - controlLaw.read(ROLL) + controlLaw.read(YAW);
        motorCommand[LEFT] = flightCommand.read(THROTTLE) + controlLaw.read(ROLL) + controlLaw.read(YAW);
        
        // Prevents too little power applied to motors during hard manuevers
        // Also even motor power on high side if low side is limited
        if (motorCommand[FRONT] < MINTHROTTLE) {
          motorCommand[FRONT] = MINTHROTTLE;
          motorCommand[REAR] -= MINTHROTTLE - motorCommand[FRONT];
        }
        if (motorCommand[REAR] < MINTHROTTLE) {
          motorCommand[REAR] = MINTHROTTLE;
          motorCommand[FRONT] -= MINTHROTTLE - motorCommand[REAR];
        }
        if (motorCommand[RIGHT] < MINTHROTTLE) {
          motorCommand[RIGHT] = MINTHROTTLE;
          motorCommand[LEFT] -= MINTHROTTLE - motorCommand[RIGHT];
        }
        if (motorCommand[LEFT] < MINTHROTTLE) {
          motorCommand[LEFT] = MINTHROTTLE;
          motorCommand[RIGHT] -= MINTHROTTLE - motorCommand[LEFT];
        }
        
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motors.write(motor, constrain(motorCommand[motor], MINCOMMAND, MAXCOMMAND));
      }
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionaly externally
  int read(byte axis) {return motorCommand[axis];}
  void setRemoteCommand(byte motor, int value) {remoteCommand[motor] = value;}
  void setAutoLevel(byte value) {controlLaw.setAutoLevel(value);}
  byte getAutoLevel(void) {return controlLaw.getAutoLevel();}

  void setArmStatus(byte value) {armed = value;}
  byte getArmStatus(void) {return armed;}
  void setCalibrationESC(byte value) {calibrateESC = value;}
  byte getCalibrationESC(void) {return calibrateESC;}

  float getLevelAdjust(byte axis) {return levelAdjust[axis];}

  void setHeadingHold(byte value) {controlLaw.setHeadingHold(value);}
  byte getHeadingHold(void) {return controlLaw.getHeadingHold();}
  float getHeadingCommand(void) {return headingCommand;}
  float getHeading(void) {return heading;}
  float getCurrentHeading(void) {return currentHeading;}
};




