/*
  AeroQuad v1.4 - September 2009
 www.AeroQuad.info
 Copyright (c) 2009 Ted Carancho.  All rights reserved.
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

void sendSerialTelemetry(HardwareSerial *serialPort, char *queryType) {
  update = 0;
  switch (*queryType) {
  case 'B': // Send roll and pitch gyro PID values
    serialPort->print(PID[ROLL].P);
    comma(serialPort);
    serialPort->print(PID[ROLL].I);
    comma(serialPort);
    serialPort->print(PID[ROLL].D);
    comma(serialPort);
    serialPort->print(PID[PITCH].P);
    comma(serialPort);
    serialPort->print(PID[PITCH].I);
    comma(serialPort);
    serialPort->println(PID[PITCH].D);            

    *queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    serialPort->print(PID[YAW].P);
    comma(serialPort);
    serialPort->print(PID[YAW].I);
    comma(serialPort);
    serialPort->println(PID[YAW].D);

    *queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    serialPort->print(PID[LEVELROLL].P);
    comma(serialPort);
    serialPort->print(PID[LEVELROLL].I);
    comma(serialPort);
    serialPort->print(PID[LEVELROLL].D);
    comma(serialPort);
    serialPort->print(PID[LEVELPITCH].P);
    comma(serialPort);
    serialPort->print(PID[LEVELPITCH].I);
    comma(serialPort);
    serialPort->println(PID[LEVELPITCH].D);
    *queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    serialPort->print(levelLimit);
    comma(serialPort);
    serialPort->println(levelOff);
    *queryType = 'X';
    break;
  case 'J': // Send flight control configuration values
    serialPort->print(windupGuard);
    comma(serialPort);
    serialPort->println(xmitFactor);
    *queryType = 'X';
    break;
  case 'L': // Send data filtering values
    serialPort->print(smoothFactor[GYRO]);
    comma(serialPort);
    serialPort->print(smoothFactor[ACCEL]);
    comma(serialPort);
    serialPort->println(timeConstant);
    *queryType = 'X';
    break;
  case 'N': // Send motor smoothing values
    for (axis= ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(smoothTransmitter[axis]);
      comma(serialPort);
    }
    serialPort->println(smoothTransmitter[THROTTLE]);
    *queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (axis = ROLL; axis < AUX; axis++) {
      serialPort->print(mTransmitter[axis]);
      comma(serialPort);
      serialPort->print(bTransmitter[axis]);
      comma(serialPort);
    }
    serialPort->print(mTransmitter[AUX]);
    comma(serialPort);
    serialPort->println(bTransmitter[AUX]);
    *queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(gyroADC[axis]);
      comma(serialPort);
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(accelADC[axis]);
      comma(serialPort);
    }
    for (axis = ROLL; axis < YAW; axis++) {
      serialPort->print(levelAdjust[axis]);
      comma(serialPort);
    }
    serialPort->print(flightAngle[ROLL]);
    comma(serialPort);
    serialPort->print(flightAngle[PITCH]);
    serialPort->println();
    break;
  case 'R': // Send raw sensor data
    serialPort->print(analogRead(ROLLRATEPIN));
    comma(serialPort);
    serialPort->print(analogRead(PITCHRATEPIN));
    comma(serialPort);
    serialPort->print(analogRead(YAWRATEPIN));
    comma(serialPort);
    serialPort->print(analogRead(ROLLACCELPIN));
    comma(serialPort);
    serialPort->print(analogRead(PITCHACCELPIN));
    comma(serialPort);
    serialPort->println(analogRead(ZACCELPIN));
    break;
  case 'S': // Send all flight data
    serialPort->print(deltaTime);
    comma(serialPort);
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(gyroData[axis]);
      comma(serialPort);
    }
    serialPort->print(transmitterCommand[THROTTLE]);
    comma(serialPort);
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(motorAxisCommand[axis]);
      comma(serialPort);
    }
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      serialPort->print(motorCommand[motor]);
      comma(serialPort);
    }
    serialPort->print(armed, BIN);
    comma(serialPort);
    serialPort->println(flightHelpersEnabled);
    break;
  case 'T': // Send processed transmitter values
    serialPort->print(xmitFactor);
    comma(serialPort);
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(transmitterCommand[axis]);
      comma(serialPort);
    }
    for (axis = ROLL; axis < YAW; axis++) {
      serialPort->print(levelAdjust[axis]);
      comma(serialPort);
    }
    serialPort->print(motorAxisCommand[ROLL]);
    comma(serialPort);
    serialPort->print(motorAxisCommand[PITCH]);
    comma(serialPort);
    serialPort->println(motorAxisCommand[YAW]);
    break;
  case 'U': // Send receiver values
    for (channel = ROLL; channel < AUX; channel++) {
      serialPort->print(transmitterCommandSmooth[channel]);
      comma(serialPort);
    }
    serialPort->println(transmitterCommandSmooth[AUX]);
    break;
  case 'V': // Send receiver status
    for (channel = ROLL; channel < AUX; channel++) {
      serialPort->print(receiverData[channel]);
      comma(serialPort);
    }
    serialPort->println(receiverData[AUX]);
    break;
  case 'X': // Stop sending messages
    break;
  #ifdef HeadingHold
  case 'Z': // Send heading
    serialPort->print(transmitterCommand[YAW]);
    comma(serialPort);
    serialPort->print(transmitterCenter[YAW]);
    comma(serialPort);
    serialPort->print(heading);
    comma(serialPort);
    serialPort->println(commandedYaw);
    break;
  #endif
  case '6': // Report remote commands
    for (motor = FRONT; motor < LEFT; motor++) {
      serialPort->print(remoteCommand[motor]);
      comma(serialPort);
    }
    serialPort->println(remoteCommand[LEFT]);
    break;
  case '!': // Send flight software version
    serialPort->println("1.4");
    *queryType = 'X';
    break;
  case 'e': // Send Heading smooth value
    serialPort->println(smoothHeading);
    break;
  }
}

void comma(HardwareSerial *serialPort) 
{
  serialPort->print(',');
}

void printInt(int data, HardwareSerial *serialPort) 
{
  byte msb, lsb;
  
  msb = data >> 8;
  lsb = data & 0xff;
  
  serialPort->print(msb, BYTE);
  serialPort->print(lsb, BYTE);
}

