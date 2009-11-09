/*
  AeroQuad v1.5 - Novmeber 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho, Chris Whiteford.  All rights reserved.
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

#define BAUD 115200
#define MAXASSIGNEDSERIALPORTS 4

unsigned int _assignedSerialPortCount = 0;
HardwareSerial *_assignedSerialPorts[MAXASSIGNEDSERIALPORTS];
char _lastTelemetryType[MAXASSIGNEDSERIALPORTS];
byte _fastTransferMode[MAXASSIGNEDSERIALPORTS];
byte _SerialComs_isActive = ON;

unsigned long _telemetryTime = 50; // make telemetry output 50ms offset from receiver check
unsigned long _fastTelemetryTime = 0;



//This registers a Hardware serial port to be used for use with the SerialComs subsystem
void SerialComs_assignSerialPort(HardwareSerial *serialPort)
{
  //Check to make sure we don't already have  many serial ports assigned to the SerialComs subsystem
  if (_assignedSerialPortCount <= MAXASSIGNEDSERIALPORTS)
  {
    _assignedSerialPorts[_assignedSerialPortCount++] = serialPort;
  }
}

void SerialComs_initialize()
{
  //For each serial port we have assigned lets initialize it
  for (unsigned int i = 0; i < _assignedSerialPortCount; i++)
  {
    _assignedSerialPorts[i]->begin(BAUD);
    _lastTelemetryType[i] = 'X';
    _fastTransferMode[i] = OFF;
  }
}

void SerialComs_activate()
{
  _SerialComs_isActive = ON;
}

void SerialComs_deactivate()
{
  _SerialComs_isActive = OFF;
}

const byte SerialComs_isActive()
{
  return _SerialComs_isActive;
}

void SerialComs_process(unsigned long currentTime)
{
  if (_SerialComs_isActive == ON)
  {
    if (currentTime > (_telemetryTime + TELEMETRYLOOPTIME)) 
    { 
      // 10Hz  
      //For each serial port we have assigned process any pending commands and send any required telemetry data
      for (unsigned int i = 0; i < _assignedSerialPortCount; i++)
      {
        _SerialComs_readSerialCommand(_assignedSerialPorts[i], &_lastTelemetryType[i], &_fastTransferMode[i]);
        _SerialComs_sendSerialTelemetry(_assignedSerialPorts[i], &_lastTelemetryType[i], &_fastTransferMode[i]);
      }
  
      _telemetryTime = currentTime;
    }
    
    if (currentTime > (_fastTelemetryTime + FASTTELEMETRYTIME)) 
    { 
      // 200Hz means up to 100Hz signal can be detected by FFT
      for (unsigned int i = 0 ; i < _assignedSerialPortCount; i++)
      {
        if (_fastTransferMode[i] == ON)
        {
          _SerialComs_printInt(21845, _assignedSerialPorts[i]); // Start word of 0x5555
          for (axis = ROLL; axis < LASTAXIS; axis++) 
          {
            _SerialComs_printInt(gyroADC[axis], _assignedSerialPorts[i]);
          }
          for (axis = ROLL; axis < LASTAXIS; axis++) 
          {
            _SerialComs_printInt(accelADC[axis], _assignedSerialPorts[i]);
          }
          _SerialComs_printInt(32767, _assignedSerialPorts[i]); // Stop word of 0x7FFF*/
        }
      }
      _fastTelemetryTime = currentTime;
    }
  }
}

void _SerialComs_readSerialCommand(HardwareSerial *serialPort, char *queryType, byte *fastTransfer) 
{
  // Check for serial message
  if (serialPort->available()) 
  {
    digitalWrite(LEDPIN, LOW);
    *queryType = serialPort->read(); 
    
    switch (*queryType)
    {

    case 'A': // Receive roll and pitch gyro PID
      PID[ROLL].P = _SerialComs_readFloatSerial(serialPort);
      PID[ROLL].I = _SerialComs_readFloatSerial(serialPort);
      PID[ROLL].D = _SerialComs_readFloatSerial(serialPort);
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
      PID[PITCH].P = _SerialComs_readFloatSerial(serialPort);
      PID[PITCH].I = _SerialComs_readFloatSerial(serialPort);
      PID[PITCH].D = _SerialComs_readFloatSerial(serialPort);
      PID[PITCH].lastPosition = 0;
      PID[PITCH].integratedError = 0;
      break;

    case 'C': // Receive yaw PID
      PID[YAW].P = _SerialComs_readFloatSerial(serialPort);
      PID[YAW].I = _SerialComs_readFloatSerial(serialPort);
      PID[YAW].D = _SerialComs_readFloatSerial(serialPort);
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
      PID[HEADING].P = _SerialComs_readFloatSerial(serialPort);
      PID[HEADING].I = _SerialComs_readFloatSerial(serialPort);
      PID[HEADING].D = _SerialComs_readFloatSerial(serialPort);
      PID[HEADING].lastPosition = 0;
      PID[HEADING].integratedError = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      PID[LEVELROLL].P = _SerialComs_readFloatSerial(serialPort);
      PID[LEVELROLL].I = _SerialComs_readFloatSerial(serialPort);
      PID[LEVELROLL].D = _SerialComs_readFloatSerial(serialPort);
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;
      PID[LEVELPITCH].P = _SerialComs_readFloatSerial(serialPort);
      PID[LEVELPITCH].I = _SerialComs_readFloatSerial(serialPort);
      PID[LEVELPITCH].D = _SerialComs_readFloatSerial(serialPort);
      PID[LEVELPITCH].lastPosition = 0;
      PID[LEVELPITCH].integratedError = 0;
      break;
    case 'G': // Receive auto level configuration
      levelLimit = _SerialComs_readFloatSerial(serialPort);
      levelOff = _SerialComs_readFloatSerial(serialPort);
      break;
    case 'I': // Receive flight control configuration
      windupGuard = _SerialComs_readFloatSerial(serialPort);
      xmitFactor = _SerialComs_readFloatSerial(serialPort);
      break;
    case 'K': // Receive data filtering values
      smoothFactor[GYRO] = _SerialComs_readFloatSerial(serialPort);
      smoothFactor[ACCEL] = _SerialComs_readFloatSerial(serialPort);
      timeConstant = _SerialComs_readFloatSerial(serialPort);
      break;
    case 'M': // Receive motor smoothing values
      smoothTransmitter[ROLL] = _SerialComs_readFloatSerial(serialPort);
      smoothTransmitter[PITCH] = _SerialComs_readFloatSerial(serialPort);
      smoothTransmitter[YAW] = _SerialComs_readFloatSerial(serialPort);
      smoothTransmitter[THROTTLE] = _SerialComs_readFloatSerial(serialPort);
      smoothTransmitter[MODE] = _SerialComs_readFloatSerial(serialPort);
      smoothTransmitter[AUX] = _SerialComs_readFloatSerial(serialPort);
      break;
    case 'O': // Received transmitter calibrtion values
      mTransmitter[ROLL] = _SerialComs_readFloatSerial(serialPort);
      bTransmitter[ROLL] = _SerialComs_readFloatSerial(serialPort);
      mTransmitter[PITCH] = _SerialComs_readFloatSerial(serialPort);
      bTransmitter[PITCH] = _SerialComs_readFloatSerial(serialPort);
      mTransmitter[YAW] = _SerialComs_readFloatSerial(serialPort);
      bTransmitter[YAW] = _SerialComs_readFloatSerial(serialPort);
      mTransmitter[THROTTLE] = _SerialComs_readFloatSerial(serialPort);
      bTransmitter[THROTTLE] = _SerialComs_readFloatSerial(serialPort);
      mTransmitter[MODE] = _SerialComs_readFloatSerial(serialPort);
      bTransmitter[MODE] = _SerialComs_readFloatSerial(serialPort);
      mTransmitter[AUX] = _SerialComs_readFloatSerial(serialPort);
      bTransmitter[AUX] = _SerialComs_readFloatSerial(serialPort);
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeFloat(PID[ROLL].P, PGAIN_ADR);
      writeFloat(PID[ROLL].I, IGAIN_ADR);
      writeFloat(PID[ROLL].D, DGAIN_ADR);
      writeFloat(PID[PITCH].P, PITCH_PGAIN_ADR);
      writeFloat(PID[PITCH].I, PITCH_IGAIN_ADR);
      writeFloat(PID[PITCH].D, PITCH_DGAIN_ADR);
      writeFloat(PID[LEVELROLL].P, LEVEL_PGAIN_ADR);
      writeFloat(PID[LEVELROLL].I, LEVEL_IGAIN_ADR);
      writeFloat(PID[LEVELROLL].D, LEVEL_DGAIN_ADR);
      writeFloat(PID[LEVELPITCH].P, LEVEL_PITCH_PGAIN_ADR);
      writeFloat(PID[LEVELPITCH].I, LEVEL_PITCH_IGAIN_ADR);
      writeFloat(PID[LEVELPITCH].D, LEVEL_PITCH_DGAIN_ADR);
      writeFloat(PID[YAW].P, YAW_PGAIN_ADR);
      writeFloat(PID[YAW].I, YAW_IGAIN_ADR);
      writeFloat(PID[YAW].D, YAW_DGAIN_ADR);
      writeFloat(PID[HEADING].P, HEADING_PGAIN_ADR);
      writeFloat(PID[HEADING].I, HEADING_IGAIN_ADR);
      writeFloat(PID[HEADING].D, HEADING_DGAIN_ADR);
      writeFloat(windupGuard, WINDUPGUARD_ADR);  
      writeFloat(levelLimit, LEVELLIMIT_ADR);   
      writeFloat(levelOff, LEVELOFF_ADR); 
      writeFloat(xmitFactor, XMITFACTOR_ADR);
      writeFloat(smoothFactor[GYRO], GYROSMOOTH_ADR);
      writeFloat(smoothFactor[ACCEL], ACCSMOOTH_ADR);
      writeFloat(smoothTransmitter[THROTTLE], THROTTLESMOOTH_ADR);
      writeFloat(smoothTransmitter[ROLL], ROLLSMOOTH_ADR);
      writeFloat(smoothTransmitter[PITCH], PITCHSMOOTH_ADR);
      writeFloat(smoothTransmitter[YAW], YAWSMOOTH_ADR);
      writeFloat(smoothTransmitter[MODE], MODESMOOTH_ADR);
      writeFloat(smoothTransmitter[AUX], AUXSMOOTH_ADR);
      writeFloat(timeConstant, FILTERTERM_ADR);
      writeFloat(mTransmitter[THROTTLE], THROTTLESCALE_ADR);
      writeFloat(bTransmitter[THROTTLE], THROTTLEOFFSET_ADR);
      writeFloat(mTransmitter[ROLL], ROLLSCALE_ADR);
      writeFloat(bTransmitter[ROLL], ROLLOFFSET_ADR);
      writeFloat(mTransmitter[PITCH], PITCHSCALE_ADR);
      writeFloat(bTransmitter[PITCH], PITCHOFFSET_ADR);
      writeFloat(mTransmitter[YAW], YAWSCALE_ADR);
      writeFloat(bTransmitter[YAW], YAWOFFSET_ADR);
      writeFloat(mTransmitter[MODE], MODESCALE_ADR);
      writeFloat(bTransmitter[MODE], MODEOFFSET_ADR);
      writeFloat(mTransmitter[AUX], AUXSCALE_ADR);
      writeFloat(bTransmitter[AUX], AUXOFFSET_ADR);
      writeFloat(smoothHeading, HEADINGSMOOTH_ADR);
      zeroIntegralError();
      // Complementary filter setup
      configureFilter(timeConstant);
      break;
    case 'Y': // Initialize EEPROM with default values
      PID[ROLL].P = 3.75;
      PID[ROLL].I = 0;
      PID[ROLL].D = -10;
      PID[PITCH].P = 3.75;
      PID[PITCH].I = 0;
      PID[PITCH].D = -10;
      PID[YAW].P = 12.0;
      PID[YAW].I = 0;
      PID[YAW].D = 0;
      PID[LEVELROLL].P = 2;
      PID[LEVELROLL].I = 0;
      PID[LEVELROLL].D = 0;
      PID[LEVELPITCH].P = 2;
      PID[LEVELPITCH].I = 0;
      PID[LEVELPITCH].D = 0;
      PID[HEADING].P = 3;
      PID[HEADING].I = 0;
      PID[HEADING].D = 0;
      windupGuard = 2000.0;
      xmitFactor = 0.20;  
      levelLimit = 2000.0;
      levelOff = 50;  
      smoothFactor[GYRO] = 0.20;
      smoothFactor[ACCEL] = 0.20;
      timeConstant = 3.0;   
      for (channel = ROLL; channel < LASTCHANNEL; channel++) {
        mTransmitter[channel] = 1.0;
        bTransmitter[channel] = 0.0;
      }
      smoothTransmitter[THROTTLE] = 1.0;
      smoothTransmitter[ROLL] = 1.0;
      smoothTransmitter[PITCH] = 1.0;
      smoothTransmitter[YAW] = 0.5;  
      smoothTransmitter[MODE] = 1.0;
      smoothTransmitter[AUX] = 1.0;
      smoothHeading = 1.0;

      autoZeroGyros();
      zeroGyros();
      zeroAccelerometers();
      zeroIntegralError();
      break;
    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      armed = 0;
      calibrateESC = 1;
      break;
    case '2': // Calibrate ESC's by setting Throttle low on all channels
      armed = 0;
      calibrateESC = 2;
      break;
    case '3': // Test ESC calibration
      armed = 0;
      testCommand = _SerialComs_readFloatSerial(serialPort);
      calibrateESC = 3;
      break;
    case '4': // Turn off ESC calibration
      armed = 0;
      calibrateESC = 0;
      testCommand = 1000;
      break;        
    case '5': // Send individual motor commands (motor, command)
      armed = 0;
      calibrateESC = 5;
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        remoteCommand[motor] = _SerialComs_readFloatSerial(serialPort);
      break;
    case 'a': // Enable/disable fast data transfer of sensor data
      *queryType = 'X'; // Stop any other telemetry streaming
      if (_SerialComs_readFloatSerial(serialPort) == 1)
        *fastTransfer = ON;
      else
        *fastTransfer = OFF;
      break;
    case 'b': // calibrate gyros
      autoZeroGyros();
      zeroGyros();
      break;
    case 'c': // calibrate accels
      zeroAccelerometers();
      break;
    }
    digitalWrite(LEDPIN, HIGH);
  }
}

void _SerialComs_sendSerialTelemetry(HardwareSerial *serialPort, char *queryType, byte *fastTransfer) {
  update = 0;
  switch (*queryType) {
  case 'B': // Send roll and pitch gyro PID values
    serialPort->print(PID[ROLL].P);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[ROLL].I);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[ROLL].D);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[PITCH].P);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[PITCH].I);
    _SerialComs_comma(serialPort);
    serialPort->println(PID[PITCH].D);            

    *queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    serialPort->print(PID[YAW].P);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[YAW].I);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[YAW].D);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[HEADING].P);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[HEADING].I);
    _SerialComs_comma(serialPort);
    serialPort->println(PID[HEADING].D);

    *queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    serialPort->print(PID[LEVELROLL].P);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[LEVELROLL].I);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[LEVELROLL].D);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[LEVELPITCH].P);
    _SerialComs_comma(serialPort);
    serialPort->print(PID[LEVELPITCH].I);
    _SerialComs_comma(serialPort);
    serialPort->println(PID[LEVELPITCH].D);
    *queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    serialPort->print(levelLimit);
    _SerialComs_comma(serialPort);
    serialPort->println(levelOff);
    *queryType = 'X';
    break;
  case 'J': // Send flight control configuration values
    serialPort->print(windupGuard);
    _SerialComs_comma(serialPort);
    serialPort->println(xmitFactor);
    *queryType = 'X';
    break;
  case 'L': // Send data filtering values
    serialPort->print(smoothFactor[GYRO]);
    _SerialComs_comma(serialPort);
    serialPort->print(smoothFactor[ACCEL]);
    _SerialComs_comma(serialPort);
    serialPort->println(timeConstant);
    *queryType = 'X';
    break;
  case 'N': // Send motor smoothing values
    for (axis = ROLL; axis < AUX; axis++) {
      serialPort->print(smoothTransmitter[axis]);
      _SerialComs_comma(serialPort);
    }
    serialPort->println(smoothTransmitter[AUX]);
    *queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (axis = ROLL; axis < AUX; axis++) {
      serialPort->print(mTransmitter[axis]);
      _SerialComs_comma(serialPort);
      serialPort->print(bTransmitter[axis]);
      _SerialComs_comma(serialPort);
    }
    serialPort->print(mTransmitter[AUX]);
    _SerialComs_comma(serialPort);
    serialPort->println(bTransmitter[AUX]);
    *queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(gyroADC[axis]);
      _SerialComs_comma(serialPort);
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(accelADC[axis]);
      _SerialComs_comma(serialPort);
    }
    for (axis = ROLL; axis < YAW; axis++) {
      serialPort->print(levelAdjust[axis]);
      _SerialComs_comma(serialPort);
    }
    serialPort->print(flightAngle[ROLL]);
    _SerialComs_comma(serialPort);
    serialPort->print(flightAngle[PITCH]);
    serialPort->println();
    break;
  case 'R': // Send raw sensor data
    serialPort->print(analogRead(ROLLRATEPIN));
    _SerialComs_comma(serialPort);
    serialPort->print(analogRead(PITCHRATEPIN));
    _SerialComs_comma(serialPort);
    serialPort->print(analogRead(YAWRATEPIN));
    _SerialComs_comma(serialPort);
    serialPort->print(analogRead(ROLLACCELPIN));
    _SerialComs_comma(serialPort);
    serialPort->print(analogRead(PITCHACCELPIN));
    _SerialComs_comma(serialPort);
    serialPort->println(analogRead(ZACCELPIN));
    break;
  case 'S': // Send all flight data
    serialPort->print(deltaTime);
    _SerialComs_comma(serialPort);
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(gyroData[axis]);
      _SerialComs_comma(serialPort);
    }
    serialPort->print(transmitterCommand[THROTTLE]);
    _SerialComs_comma(serialPort);
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(motorAxisCommand[axis]);
      _SerialComs_comma(serialPort);
    }
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      serialPort->print(motorCommand[motor]);
      _SerialComs_comma(serialPort);
    }
    serialPort->print(armed, BIN);
    _SerialComs_comma(serialPort);
#ifdef AutoLevel
    serialPort->println(transmitterCommandSmooth[MODE]);
#endif
#ifndef AutoLevel
    serialPort->println(1000);
#endif
    break;
  case 'T': // Send processed transmitter values
    serialPort->print(xmitFactor);
    _SerialComs_comma(serialPort);
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      serialPort->print(transmitterCommand[axis]);
      _SerialComs_comma(serialPort);
    }
    for (axis = ROLL; axis < YAW; axis++) {
      serialPort->print(levelAdjust[axis]);
      _SerialComs_comma(serialPort);
    }
    serialPort->print(motorAxisCommand[ROLL]);
    _SerialComs_comma(serialPort);
    serialPort->print(motorAxisCommand[PITCH]);
    _SerialComs_comma(serialPort);
    serialPort->println(motorAxisCommand[YAW]);
    break;
  case 'U': // Send smoothed receiver values
    for (channel = ROLL; channel < AUX; channel++) {
      serialPort->print(transmitterCommandSmooth[channel]);
      _SerialComs_comma(serialPort);
    }
    serialPort->println(transmitterCommandSmooth[AUX]);
    break;
  case 'V': // Send receiver status
    for (channel = ROLL; channel < AUX; channel++) {
      serialPort->print(receiverData[channel]);
      _SerialComs_comma(serialPort);
    }
    serialPort->println(receiverData[AUX]);
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    serialPort->print(transmitterCommand[YAW]);
    _SerialComs_comma(serialPort);
    serialPort->print(headingHold);
    _SerialComs_comma(serialPort);
    serialPort->print(heading);
    _SerialComs_comma(serialPort);
    serialPort->println(currentHeading);
    break;
  case '6': // Report remote commands
    for (motor = FRONT; motor < LEFT; motor++) {
      serialPort->print(remoteCommand[motor]);
      _SerialComs_comma(serialPort);
    }
    serialPort->println(remoteCommand[LEFT]);
    break;
  case '!': // Send flight software version
    serialPort->println("1.4");
    *queryType = 'X';
    break;

  case '^':
    serialPort->print(analogRead(PROX1PIN));
    _SerialComs_comma(serialPort);
    serialPort->print(analogRead(VOLTSPIN));
    _SerialComs_comma(serialPort);
    serialPort->println(analogRead(CURRENTPIN));
    break;

  }
}

// Used to read floating point values from the serial port
float _SerialComs_readFloatSerial(HardwareSerial *serialPort) 
{
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (serialPort->available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = serialPort->read();
      timeout = 0;
      index++;
    }
  }  
  while ((data[limitRange(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}

void _SerialComs_comma(HardwareSerial *serialPort) 
{
  serialPort->print(',');
}

void _SerialComs_printInt(int data, HardwareSerial *serialPort) 
{
  byte msb, lsb;
  
  msb = data >> 8;
  lsb = data & 0xff;
  
  serialPort->print(msb, BYTE);
  serialPort->print(lsb, BYTE);
}




