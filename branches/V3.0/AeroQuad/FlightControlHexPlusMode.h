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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_

#define FRONT_LEFT  MOTOR1
#define REAR_RIGHT  MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR_LEFT   MOTOR4
#define FRONT       MOTOR5
#define REAR        MOTOR6
#define LASTMOTOR   MOTOR6+1


void applyMotorCommand() {
  motors->setMotorCommand(FRONT_LEFT,  throttle - motorAxisCommandPitch + motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(REAR_RIGHT,  throttle + motorAxisCommandPitch - motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(FRONT_RIGHT, throttle - motorAxisCommandPitch - motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(REAR_LEFT,   throttle + motorAxisCommandPitch + motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(FRONT,       throttle - motorAxisCommandPitch                        + motorAxisCommandYaw);
  motors->setMotorCommand(REAR,        throttle + motorAxisCommandPitch                        - motorAxisCommandYaw);
  if (flightMode == ACRO) {
    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
      motors->setMotorCommand(FRONT_RIGHT, MAXCOMMAND);
      motors->setMotorCommand(REAR_RIGHT,  MAXCOMMAND);
      motors->setMotorCommand(FRONT_LEFT,  minAcro);
      motors->setMotorCommand(REAR_LEFT,   minAcro);
    }
    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
      motors->setMotorCommand(FRONT_LEFT,  MAXCOMMAND);
      motors->setMotorCommand(REAR_LEFT,   MAXCOMMAND);
      motors->setMotorCommand(FRONT_RIGHT, minAcro);
      motors->setMotorCommand(REAR_RIGHT,  minAcro);
    }
    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motors->setMotorCommand(FRONT,       MAXCOMMAND);
      motors->setMotorCommand(FRONT_LEFT,  MAXCOMMAND);
      motors->setMotorCommand(FRONT_RIGHT, MAXCOMMAND);
      motors->setMotorCommand(REAR,        minAcro);
      motors->setMotorCommand(REAR_LEFT,   minAcro);
      motors->setMotorCommand(REAR_RIGHT,  minAcro);
    }
    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motors->setMotorCommand(REAR,        MAXCOMMAND);
      motors->setMotorCommand(REAR_LEFT,   MAXCOMMAND);
      motors->setMotorCommand(REAR_RIGHT,  MAXCOMMAND);
      motors->setMotorCommand(FRONT,       minAcro);
      motors->setMotorCommand(FRONT_LEFT,  minAcro);
      motors->setMotorCommand(FRONT_RIGHT, minAcro);
    }
  }
}

#endif  // #define _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_

