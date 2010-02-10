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
 
#include "SubSystem.h"

#define FRONTMOTORPIN 6
#define RIGHTMOTORPIN 7
#define LEFTMOTORPIN 8
#define REARMOTORPIN 9
#define LASTMOTORPIN 10
#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define LASTMOTOR 4

class Motors: public SubSystem {
private:
  float mMotorCommand;		
  float bMotorCommand;
  int motorCommand[4];
  int motorAxisCommand[3];
  int motor;
  float mMotorRate;
  float bMotorRate;
  
  // ESC Calibration
  byte calibrateESC = 0;
  int testCommand = 1000;

  // Ground station control (experimental)
  int remoteCommand[4] = {1000,1000,1000,1000};

public:
  //Required methods to impliment for a SubSystem
  Motors():
  SubSystem()
  {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    // Scale motor commands to analogWrite		
    // m = (250-126)/(2000-1000) = 0.124		
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2		
    mMotorCommand = 0.124;		
    bMotorCommand = 2;

    motorCommand[4] = {1000,1000,1000,1000};
    motorAxisCommand[3] = {0,0,0};
    motor = 0;
    // If AREF = 3.3V, then A/D is 931 at 3V and 465 = 1.5V 
    // Scale gyro output (-465 to +465) to motor commands (1000 to 2000) 
    // use y = mx + b 
    mMotorRate = 1.0753; // m = (y2 - y1) / (x2 - x1) = (2000 - 1000) / (465 - (-465)) 
    bMotorRate = 1500;   // b = y1 - m * x1
  }

  void initialize(unsigned int frequency, unsigned int offset = 0)
  {
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);

    //Perform any custom initalization you need for this SubSystem
    analogWrite(FRONTMOTORPIN, 124);		
    analogWrite(REARMOTORPIN, 124);		
    analogWrite(RIGHTMOTORPIN, 124);		
    analogWrite(LEFTMOTORPIN, 124);		
  }

  void process(unsigned long currentTime)
  {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime)) {
      //If the code reaches this point the SubSystem is allowed to run.

      // ****************** Calculate Motor Commands *****************
      if (armed && safetyCheck) {
        motorCommand[FRONT] = constrain(receiver.getPilotCommand(THROTTLE) - flightcontrol.motorCommand(PITCH) - flightcontrol.motorCommand(YAW), minCommand, MAXCOMMAND);
        motorCommand[REAR] = constrain(receiver.getPilotCommand(THROTTLE) + flightcontrol.motorCommand(PITCH) - flightcontrol.motorCommand(YAW), minCommand, MAXCOMMAND);
        motorCommand[RIGHT] = constrain(receiver.getPilotCommand(THROTTLE) - flightcontrol.motorCommand(ROLL) + flightcontrol.motorCommand(YAW), minCommand, MAXCOMMAND);
        motorCommand[LEFT] = constrain(receiver.getPilotCommand(THROTTLE) + flightcontrol.motorCommand(ROLL) + flightcontrol.motorCommand(YAW), minCommand, MAXCOMMAND);
      }
  
      // If throttle in minimum position, don't apply yaw
      if receiver.getPilotCommand(THROTTLE) < MINCHECK) {
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motorCommand[motor] = minCommand;
      }
      // If motor output disarmed, force motor output to minimum
      if (receiver.getArmStatus == ON) {
        switch (calibrateESC) { // used for calibrating ESC's
        case 1:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motorCommand[motor] = MAXCOMMAND;
          break;
        case 3:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motorCommand[motor] = limitRange(testCommand, 1000, 1200);
          break;
        case 5:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motorCommand[motor] = limitRange(remoteCommand[motor], 1000, 1200);
          safetyCheck = 1;
          break;
        default:
          for (motor = FRONT; motor < LASTMOTOR; motor++)
            motorCommand[motor] = MINCOMMAND;
        }
      }
      analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * mMotorCommand) + bMotorCommand);		
      analogWrite(REARMOTORPIN, (motorCommand[REAR] * mMotorCommand) + bMotorCommand);		
      analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);		
      analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * mMotorCommand) + bMotorCommand);		
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void commandAllMotors(int motorCommand) {   // Sends commands to all motors
    analogWrite(FRONTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    analogWrite(REARMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    analogWrite(RIGHTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    analogWrite(LEFTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
  }

  void pulseMotors(byte quantity) {
    for (byte i = 0; i < quantity; i++) {      
      commandAllMotors(MINCOMMAND + 100);
      delay(250);
      commandAllMotors(MINCOMMAND);
      delay(250);
    }
  }
  
  void setCalibrationESC(byte value) {calibrateESC = value;}
  void remoteMotorCommand(byte motor, int value) {remoteCommand[motor] = value;}
  int getRemoteMotorCommand(byte motor) {return remoteCommand[motor];}
  void setTestCommand(int value) {testCommand = value;}
  int getMotorCommand(byte motor) {return motorCommand[motor];}
};




