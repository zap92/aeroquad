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
#define MINCOMMAND 1000
#define MAXCOMMAND 2000

class Motors_PWM: public SubSystem {
private:
  int motorCommand[4];
  int motorAxisCommand[3];
  int motor;
  float mMotorRate;
  float bMotorRate;
  int minCommand;
  byte axis;
  
  // Ground station control (experimental)
  int remoteCommand[4];

public:
  //Required methods to impliment for a SubSystem
  Motors_PWM(): SubSystem()
  {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = 1000;
      remoteCommand[motor] = 1000;
    }
    for (axis = ROLL; axis < LASTAXIS; axis++)
      motorAxisCommand[axis] = 0;
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
      analogWrite(FRONTMOTORPIN, motorCommand[FRONT]);		
      analogWrite(REARMOTORPIN, motorCommand[REAR]);		
      analogWrite(RIGHTMOTORPIN, motorCommand[RIGHT]);		
      analogWrite(LEFTMOTORPIN, motorCommand[LEFT]);		
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void commandAllMotors(int motorCommand) {   // Sends commands to all motors
    analogWrite(FRONTMOTORPIN, motorCommand);		
    analogWrite(REARMOTORPIN, motorCommand);		
    analogWrite(RIGHTMOTORPIN, motorCommand);		
    analogWrite(LEFTMOTORPIN, motorCommand);		
  }

  void pulseMotors(byte quantity) {
    for (byte i = 0; i < quantity; i++) {      
      commandAllMotors(MINCOMMAND + 100);
      delay(250);
      commandAllMotors(MINCOMMAND);
      delay(250);
    }
  }
  
  void write(byte motor, int value) {remoteCommand[motor] = value;}
  int getRemoteMotorCommand(byte motor) {return remoteCommand[motor];}
  int getMotorCommand(byte motor) {return motorCommand[motor];}
  float getMotorSlope(void) {return mMotorRate;}
  float getMotorOffset(void) {return bMotorRate;}
};




