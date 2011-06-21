/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_

#define FRONT_LEFT  MOTOR1
#define REAR_1      MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR_2      MOTOR4
#define LASTMOTOR   MOTOR4+1

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1


void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  motors->setMotorCommand(FRONT_LEFT,  throttle - motorAxisCommandPitch + motorAxisCommandRoll);
  motors->setMotorCommand(FRONT_RIGHT, throttle - motorAxisCommandPitch - motorAxisCommandRoll);
  motors->setMotorCommand(REAR_2,      throttle + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw));
  motors->setMotorCommand(REAR_1,      throttle + motorAxisCommandPitch + (YAW_DIRECTION * motorAxisCommandYaw));
  if (flightMode == ACRO) {
    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
      motors->setMotorCommand(FRONT_RIGHT, MAXCOMMAND);
      motors->setMotorCommand(REAR_1, MAXCOMMAND);
      motors->setMotorCommand(FRONT_LEFT, minAcro);
      motors->setMotorCommand(REAR_2, minAcro);
    }
    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
      motors->setMotorCommand(FRONT_LEFT, MAXCOMMAND);
      motors->setMotorCommand(REAR_2, MAXCOMMAND);
      motors->setMotorCommand(FRONT_RIGHT, minAcro);
      motors->setMotorCommand(REAR_1, minAcro);
    }
    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motors->setMotorCommand(FRONT_LEFT, MAXCOMMAND);
      motors->setMotorCommand(FRONT_RIGHT, MAXCOMMAND);
      motors->setMotorCommand(REAR_2, minAcro);
      motors->setMotorCommand(REAR_1, minAcro);
    }
    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motors->setMotorCommand(REAR_2, MAXCOMMAND);
      motors->setMotorCommand(REAR_1, MAXCOMMAND);
      motors->setMotorCommand(FRONT_LEFT, minAcro);
      motors->setMotorCommand(FRONT_RIGHT, minAcro);
    }
  }
  
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
