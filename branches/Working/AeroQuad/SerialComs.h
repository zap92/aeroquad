/*
 AeroQuad v1.6 - March 2010
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

#include "SubSystem.h"

#define BAUD 115200
#define MAXASSIGNEDSERIALPORTS 4

class SerialComs: public SubSystem{
private:
  // Communication
  char queryType = 'X';
  byte tlmType = 0;
  char string[32];

  unsigned int _serialPortCount;
  HardwareSerial *_serialPorts[MAXASSIGNEDSERIALPORTS];

  char _lastTelemetryType[MAXASSIGNEDSERIALPORTS];
  byte _fastTransferOn[MAXASSIGNEDSERIALPORTS];

  unsigned long _lastRunTime[MAXASSIGNEDSERIALPORTS];
  unsigned int _normalFrequency[MAXASSIGNEDSERIALPORTS];
  unsigned int _activeFrequency[MAXASSIGNEDSERIALPORTS];

  float _readFloatSerial(HardwareSerial *serialPort) 
  {
    byte index = 0;
    byte timeout = 0;
    char data[128] = "";

    do 
    {
      if (serialPort->available() == 0) 
      {
        delay(10);
        timeout++;
      }
      else 
      {
        data[index] = serialPort->read();
        timeout = 0;
        index++;
      }
    }  
    while ((data[limitRange(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
    return atof(data);
  }

  void _comma(HardwareSerial *serialPort) 
  {
    serialPort->print(',');
  }
  
  void _printInt(int data, HardwareSerial *serialPort) 
  {
    serialPort->print(data >> 8, BYTE); //MSB
    serialPort->print(data & 0xff, BYTE);  //LSB
  }

  void _readCommand(unsigned int i)
  {
    HardwareSerial *serialPort = _serialPorts[i];

    if (serialPort->available()) 
    {
      _lastTelemetryType[i] = serialPort->read(); 

      switch (_lastTelemetryType[i])
      {
      case 'A': // Receive roll and pitch gyro PID
        flightcontrol.setP(ROLL, _readFloatSerial(serialPort));
        flightcontrol.setI(ROLL, _readFloatSerial(serialPort));
        flightcontrol.setD(ROLL, _readFloatSerial(serialPort));
        flightcontrol.setInitPosError(ROLL);
        flightcontrol.setP(PITCH, _readFloatSerial(serialPort));
        flightcontrol.setI(PITCH, _readFloatSerial(serialPort));
        flightcontrol.setD(PITCH, _readFloatSerial(serialPort));
        flightcontrol.setInitPosError(PITCH);
        break;

      case 'C': // Receive yaw PID
        flightcontrol.setP(YAW, _readFloatSerial(serialPort));
        flightcontrol.setI(YAW, _readFloatSerial(serialPort));
        flightcontrol.setD(YAW, _readFloatSerial(serialPort));
        flightcontrol.setInitPosError(YAW);
        flightcontrol.setP(HEADING, _readFloatSerial(serialPort));
        flightcontrol.setI(HEADING, _readFloatSerial(serialPort));
        flightcontrol.setD(HEADING, _readFloatSerial(serialPort));
        flightcontrol.setInitPosError(HEADING);
        break;
      case 'E': // Receive roll and pitch auto level PID
        flightcontrol.setP(LEVELROLL, _readFloatSerial(serialPort));
        flightcontrol.setI(LEVELROLL, _readFloatSerial(serialPort));
        flightcontrol.setD(LEVELROLL, _readFloatSerial(serialPort));
        flightcontrol.setInitPosError(LEVELROLL);
        flightcontrol.setP(LEVELPITCH, _readFloatSerial(serialPort));
        flightcontrol.setI(LEVELPITCH, _readFloatSerial(serialPort));
        flightcontrol.setD(LEVELPITCH, _readFloatSerial(serialPort));
        flightcontrol.setInitPosError(LEVELPITCH);
        break;
      case 'G': // Receive auto level configuration
        flightcontrol.setLevelLimit(_readFloatSerial(serialPort));
        flightcontrol.setLevelOff(_readFloatSerial(serialPort));
        break;
      case 'I': // Receive flight control configuration
        flightcontrol.setWindupGuard(_readFloatSerial(serialPort));
        receiver.setXmitFactor(_readFloatSerial(serialPort));
        break;
      case 'K': // Receive data filtering values
        sensors.setGyroSmoothFactor(_readFloatSerial(serialPort));
        sensors.setAccelSmoothFactor(_readFloatSerial(serialPort));
        attitude.setTimeConstant(_readFloatSerial(serialPort));
        break;
      case 'M': // Receive motor smoothing values
        receiver.setTransmitterSmoothing(ROLL, _readFloatSerial(serialPort));
        receiver.setTransmitterSmoothing(PITCH, _readFloatSerial(serialPort));
        receiver.setTransmitterSmoothing(YAW, _readFloatSerial(serialPort));
        receiver.setTransmitterSmoothing(THROTTLE, _readFloatSerial(serialPort));
        receiver.setTransmitterSmoothing(MODE, _readFloatSerial(serialPort));
        receiver.setTransmitterSmoothing(AUX, _readFloatSerial(serialPort));
        break;
      case 'O': // Received transmitter calibrtion values
        receiver.setTransmitterSlope(ROLL, _readFloatSerial(serialPort));
        receiver.setTransmitterOffset(ROLL, _readFloatSerial(serialPort));
        receiver.setTransmitterSlope(PITCH, _readFloatSerial(serialPort));
        receiver.setTransmitterOffset(PITCH, _readFloatSerial(serialPort));
        receiver.setTransmitterSlope(YAW, _readFloatSerial(serialPort));
        receiver.setTransmitterOffset(YAW, _readFloatSerial(serialPort));
        receiver.setTransmitterSlope(THROTTLE, _readFloatSerial(serialPort));
        receiver.setTransmitterOffset(THROTTLE, _readFloatSerial(serialPort));
        receiver.setTransmitterSlope(MODE, _readFloatSerial(serialPort));
        receiver.setTransmitterOffset(MODE, _readFloatSerial(serialPort));
        receiver.setTransmitterSlope(AUX, _readFloatSerial(serialPort));
        receiver.setTransmitterOffset(AUX, _readFloatSerial(serialPort));
        break;
      case 'W': // Write all user configurable values to EEPROM
        writeFloat(flightcontrol.getP(ROLL), PGAIN_ADR);
        writeFloat(flightcontrol.getI(ROLL), IGAIN_ADR);
        writeFloat(flightcontrol.getD(ROLL), DGAIN_ADR);
        writeFloat(flightcontrol.getP(PITCH), PITCH_PGAIN_ADR);
        writeFloat(flightcontrol.getI(PITCH), PITCH_IGAIN_ADR);
        writeFloat(flightcontrol.getD(PITCH), PITCH_DGAIN_ADR);
        writeFloat(flightcontrol.getP(LEVELROLL), LEVEL_PGAIN_ADR);
        writeFloat(flightcontrol.getI(LEVELROLL), LEVEL_IGAIN_ADR);
        writeFloat(flightcontrol.getD(LEVELROLL), LEVEL_DGAIN_ADR);
        writeFloat(flightcontrol.getP(LEVELPITCH), LEVEL_PITCH_PGAIN_ADR);
        writeFloat(flightcontrol.getI(LEVELPITCH), LEVEL_PITCH_IGAIN_ADR);
        writeFloat(flightcontrol.getD(LEVELPITCH), LEVEL_PITCH_DGAIN_ADR);
        writeFloat(flightcontrol.getP(YAW), YAW_PGAIN_ADR);
        writeFloat(flightcontrol.getI(YAW), YAW_IGAIN_ADR);
        writeFloat(flightcontrol.getD(YAW), YAW_DGAIN_ADR);
        writeFloat(flightcontrol.getP(HEADING), HEADING_PGAIN_ADR);
        writeFloat(flightcontrol.getI(HEADING), HEADING_IGAIN_ADR);
        writeFloat(flightcontrol.getD(HEADING), HEADING_DGAIN_ADR);
        writeFloat(flightcontrol.getWindupGuard(), WINDUPGUARD_ADR);  
        writeFloat(flightcontrol.getLevelLimit(), LEVELLIMIT_ADR);   
        writeFloat(flightcontrol.getLevelOff(), LEVELOFF_ADR); 
        writeFloat(receiver.getXmitFactor(), XMITFACTOR_ADR);
        writeFloat(sensors.getGyroSmoothFactor(), GYROSMOOTH_ADR);
        writeFloat(sensors.getAccelSmoothFactor(), ACCSMOOTH_ADR);
        writeFloat(receiver.getTransmitterSmoothing(THROTTLE), THROTTLESMOOTH_ADR);
        writeFloat(receiver.getTransmitterSmoothing(ROLL), ROLLSMOOTH_ADR);
        writeFloat(receiver.getTransmitterSmoothing(PITCH), PITCHSMOOTH_ADR);
        writeFloat(receiver.getTransmitterSmoothing(YAW), YAWSMOOTH_ADR);
        writeFloat(receiver.getTransmitterSmoothing(MODE), MODESMOOTH_ADR);
        writeFloat(receiver.getTransmitterSmoothing(AUX), AUXSMOOTH_ADR);
        writeFloat(attitude.getTimeConstant(), FILTERTERM_ADR);
        writeFloat(receiver.getTransmitterSlope(THROTTLE), THROTTLESCALE_ADR);
        writeFloat(receiver.getTransmitterOffset(THROTTLE), THROTTLEOFFSET_ADR);
        writeFloat(receiver.getTransmitterSlope(ROLL), ROLLSCALE_ADR);
        writeFloat(receiver.getTransmitterOffset(ROLL), ROLLOFFSET_ADR);
        writeFloat(receiver.getTransmitterSlope(PITCH), PITCHSCALE_ADR);
        writeFloat(receiver.getTransmitterOffset(PITCH), PITCHOFFSET_ADR);
        writeFloat(receiver.getTransmitterSlope(YAW), YAWSCALE_ADR);
        writeFloat(receiver.getTransmitterOffset(YAW), YAWOFFSET_ADR);
        writeFloat(receiver.getTransmitterSlope(MODE), MODESCALE_ADR);
        writeFloat(receiver.getTransmitterOffset(MODE), MODEOFFSET_ADR);
        writeFloat(receiver.getTransmitterSlope(AUX), AUXSCALE_ADR);
        writeFloat(receiver.getTransmitterOffset(AUX), AUXOFFSET_ADR);
        writeFloat(attitude.getHeadingSmoothFactor(), HEADINGSMOOTH_ADR);
        flightcontrol.zeroIntegralError();
        break;
      case 'Y': // Initialize EEPROM with default values
        flightcontrol.setP(ROLL) = 3.75;
        flightcontrol.setI(ROLL) = 0;
        flightcontrol.setD(ROLL) = -10;
        flightcontrol.setP(PITCH) = 3.75;
        flightcontrol.setI(PITCH) = 0;
        flightcontrol.setD(PITCH) = -10;
        flightcontrol.setP(YAW) = 12.0;
        flightcontrol.setI(YAW) = 0;
        flightcontrol.setD(YAW) = 0;
        flightcontrol.setP(LEVELROLL) = 2;
        flightcontrol.setI(LEVELROLL) = 0;
        flightcontrol.setD(LEVELROLL) = 0;
        flightcontrol.setP(LEVELPITCH) = 2;
        flightcontrol.setI(LEVELPITCH) = 0;
        flightcontrol.setD(LEVELPITCH) = 0;
        flightcontrol.setP(HEADING) = 3;
        flightcontrol.setI(HEADING) = 0;
        flightcontrol.setD(HEADING) = 0;
        flightcontro.setWindupGuard() = 2000.0;
        flightcontrol.setLevelLimit() = 2000.0;
        flightcontrol.setLevelOff() = 50;  
        sensors.setsGyroSmoothFactor() = 0.20;
        sensors.setAccelSmoothFactor() = 0.20;
        attitude.setTimeConstant() = 3.0;   
        attitude.setHeadingSmoothFactor() = 1.0;
        for (channel = ROLL; channel < LASTCHANNEL; channel++) {
          receiver.setTransmitterSlope(channel, 1.0);
          receiver.setTransmitterOffset(channel, 0.0):
        }
        receiver.setTransmitterSmoothing(THROTTLE, 1.0);
        receiver.setTransmitterSmoothing(ROLL, 1.0);
        receiver.setTransmitterSmoothing(PITCH, 1.0);
        receiver.setTransmitterSmoothing(YAW, 0.5);  
        receiver.setTransmitterSmoothing(MODE, 1.0);
        receiver.setTransmitterSmoothing(AUX, 1.0);
        receiver.setXmitFactor() = 0.20;  
        sensors.zeroGyros();
        sensors.zeroAccelerometers();
        flightcontrol.zeroIntegralError();
        break;
      case '1': // Calibrate ESCS's by setting Throttle high on all channels
        receiver.setArmStatus(OFF);
        motors.setCalibrationESC(1);
        break;
      case '2': // Calibrate ESC's by setting Throttle low on all channels
        receiver.setArmStatus(OFF);
        motors.setCalibrationESC(2);
        break;
      case '3': // Test ESC calibration
        receiver.setArmStatus(OFF);
        motors.setTestCommand(_readFloatSerial(serialPort));
        motors.setCalibrationESC(3);
        break;
      case '4': // Turn off ESC calibration
        receiver.setArmStatus(OFF);
        motors.setCalibrationESC(0);
        testCommand = 1000;
        break;        
      case '5': // Send individual motor commands (motor, command)
        receiver.setArmStatus(OFF);
        motors.setCalibrationESC();
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motors.setRemoteCommand(motor, _readFloatSerial(serialPort));
        break;

      case 'a': // Enable/disable fast data transfer of sensor data
        _lastTelemetryType[i] = 'X'; // Stop any other telemetry streaming
        if (_readFloatSerial(serialPort) == 1)
        {
          _fastTransferOn[i] = 1;

          //bump the refresh speed for this serial port
          _activeFrequency[i] = _normalFrequency[i] / 10;
        }
        else
        {
          _fastTransferOn[i] = 0;

          //Put the refresh rate back to normal
          _activeFrequency[i] = _normalFrequency[i];
        }
        break;
      case 'b': // calibrate gyros
        sensors.zeroGyros();
        break;
      case 'c': // calibrate accels
        sensors.zeroAccelerometers();
        break;
      }
    }
  }

  void _sendAsciiTelemetry(unsigned int i)
  {
    HardwareSerial *serialPort = _serialPorts[i];

    switch (_lastTelemetryType[i]) 
    {
    case 'B': // Send roll and pitch gyro PID values
      serialPort->print(flightcontrol.getP(ROLL));
      _comma(serialPort);
      serialPort->print(flightcontrol.getI(ROLL));
      _comma(serialPort);
      serialPort->print(flightcontrol.getD(ROLL));
      _comma(serialPort);
      serialPort->print(flightcontrol.getP(PITCH));
      _comma(serialPort);
      serialPort->print(flightcontrol.getI(PITCH));
      _comma(serialPort);
      serialPort->println(flightcontrol.getD(PITCH));            

      _lastTelemetryType[i] = 'X';
      break;
    case 'D': // Send yaw PID values
      serialPort->print(flightcontrol.getP(YAW));
      _comma(serialPort);
      serialPort->print(flightcontrol.getI(YAW));
      _comma(serialPort);
      serialPort->print(flightcontrol.getD(YAW));
      _comma(serialPort);
      serialPort->print(flightcontrol.getP(HEADING));
      _comma(serialPort);
      serialPort->print(flightcontrol.getI(HEADING));
      _comma(serialPort);
      serialPort->println(flightcontrol.getD(HEADING));

      _lastTelemetryType[i] = 'X';
      break;
    case 'F': // Send roll and pitch auto level PID values
      serialPort->print(flightcontrol.getP(LEVELROLL));
      _comma(serialPort);
      serialPort->print(flightcontrol.getI(LEVELROLL));
      _comma(serialPort);
      serialPort->print(flightcontrol.getD(LEVELROLL));
      _comma(serialPort);
      serialPort->print(flightcontrol.getP(LEVELPITCH));
      _comma(serialPort);
      serialPort->print(flightcontrol.getI(LEVELPITCH));
      _comma(serialPort);
      serialPort->println(flightcontrol.getD(LEVELPITCH));
      _lastTelemetryType[i] = 'X';
      break;
    case 'H': // Send auto level configuration values
      serialPort->print(flightcontrol.getLevelLimit());
      _comma(serialPort);
      serialPort->println(flightcontrol.getLevelOff());
      _lastTelemetryType[i] = 'X';
      break;
    case 'J': // Send flight control configuration values
      serialPort->print(flightcontrol.getWindupGuard());
      _comma(serialPort);
      serialPort->println(flightcontrol.getXmitFactor());
      _lastTelemetryType[i] = 'X';
      break;
    case 'L': // Send data filtering values
      serialPort->print(sensors.getGyroSmoothFactor());
      _comma(serialPort);
      serialPort->print(sensors.getAccelSmoothFactor());
      _comma(serialPort);
      serialPort->println(attitude.getTimeConstant());
      _lastTelemetryType[i] = 'X';
      break;
    case 'N': // Send motor smoothing values
      for (axis = ROLL; axis < AUX; axis++) {
        serialPort->print(receiver.getTransmitterSmoothing(axis));
        _comma(serialPort);
      }
      serialPort->println(receiver.getTransmitterSmoothing(AUX));
      _lastTelemetryType[i] = 'X';
      break;
    case 'P': // Send transmitter calibration data
      for (axis = ROLL; axis < AUX; axis++) {
        serialPort->print(receiver.getTransmitterSlope(axis));
        _comma(serialPort);
        serialPort->print(receiver.getTransmitterOffset(axis));
        _comma(serialPort);
      }
      serialPort->print(receiver.getTransmitterSlope(AUX));
      _comma(serialPort);
      serialPort->println(receiver.getTransmitterOffset(AUX));
      _lastTelemetryType[i] = 'X';
      break;
    case 'Q': // Send sensor data
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        serialPort->print(sensors.getRawGyro(axis));
        _comma(serialPort);
      }
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        serialPort->print(sensors.getRawAccel(axis));
        _comma(serialPort);
      }
      for (axis = ROLL; axis < YAW; axis++) {
        serialPort->print(flightcontrol.getLevelAdjust(axis));
        _comma(serialPort);
      }
      serialPort->print(attitude.getFlightAngle(ROLL));
      _comma(serialPort);
      serialPort->print(attitude.getFlightAngle(PITCH));
      serialPort->println();
      break;
    case 'R': // Send raw sensor data
      serialPort->print(analogRead(ROLLRATEPIN));
      _comma(serialPort);
      serialPort->print(analogRead(PITCHRATEPIN));
      _comma(serialPort);
      serialPort->print(analogRead(YAWRATEPIN));
      _comma(serialPort);
      serialPort->print(analogRead(ROLLACCELPIN));
      _comma(serialPort);
      serialPort->print(analogRead(PITCHACCELPIN));
      _comma(serialPort);
      serialPort->println(analogRead(ZACCELPIN));
      break;
    case 'S': // Send all flight data
      serialPort->print(deltaTime);
      _comma(serialPort);
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        serialPort->print(sensors.getGyro(axis));
        _comma(serialPort);
      }
      serialPort->print(receiver.getPilotCommand(THROTTLE));
      _comma(serialPort);
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        serialPort->print(flightcontrol.getMotorCommand(axis));
        _comma(serialPort);
      }
      for (motor = FRONT; motor < LASTMOTOR; motor++) {
        serialPort->print(motors.getMotorCommand(motor));
        _comma(serialPort);
      }
      serialPort->print(armed, BIN);
      _comma(serialPort);
      if (flightcontrol.getAutoLevelState() == ON)
        serialPort->println(receiver.getPilotCommand(MODE));
      else
        serialPort->println(1000);
      break;
////////////////////// continue here
    case 'T': // Send processed transmitter values
      serialPort->print(xmitFactor);
      _comma(serialPort);
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        serialPort->print(transmitterCommand[axis]);
        _comma(serialPort);
      }
      for (axis = ROLL; axis < YAW; axis++) {
        serialPort->print(levelAdjust[axis]);
        _comma(serialPort);
      }
      serialPort->print(motorAxisCommand[ROLL]);
      _comma(serialPort);
      serialPort->print(motorAxisCommand[PITCH]);
      _comma(serialPort);
      serialPort->println(motorAxisCommand[YAW]);
      break;
    case 'U': // Send smoothed receiver values
      for (channel = ROLL; channel < AUX; channel++) {
        serialPort->print(transmitterCommandSmooth[channel]);
        _comma(serialPort);
      }
      serialPort->println(transmitterCommandSmooth[AUX]);
      break;
    case 'V': // Send receiver status
      for (channel = ROLL; channel < AUX; channel++) {
        serialPort->print(receiverData[channel]);
        _comma(serialPort);
      }
      serialPort->println(receiverData[AUX]);
      break;
    case 'X': // Stop sending messages
      break;
    case 'Z': // Send heading
      serialPort->print(transmitterCommand[YAW]);
      _comma(serialPort);
      serialPort->print(headingHold);
      _comma(serialPort);
      serialPort->print(heading);
      _comma(serialPort);
      serialPort->println(currentHeading);
      break;
    case '6': // Report remote commands
      for (motor = FRONT; motor < LEFT; motor++) {
        serialPort->print(remoteCommand[motor]);
        _comma(serialPort);
      }
      serialPort->println(remoteCommand[LEFT]);
      break;
    case '!': // Send flight software version
      serialPort->println("1.5");
      _lastTelemetryType[i] = 'X';
      break;

    case '^':
      serialPort->print(analogRead(PROX1PIN));
      _comma(serialPort);
      serialPort->print(analogRead(VOLTSPIN));
      _comma(serialPort);
      serialPort->println(analogRead(CURRENTPIN));
      break;

    case '~':
      long lat, lng, alt, speed;
      unsigned long course, fix_age;
      
      gps.get_position(&lat, &lng, &alt, &course, &speed, &fix_age);
      serialPort->print(lat);
      _comma(serialPort);
      serialPort->print(lng);
      _comma(serialPort);
      serialPort->print(alt);
      _comma(serialPort);
      serialPort->print(course);
      _comma(serialPort);
      serialPort->print(speed);
      _comma(serialPort);
      serialPort->println(fix_age);
      break;
    }
  }

  void _sendBinaryTelemetry(unsigned int i) 
  {
    _printInt(21845, _serialPorts[i]); // Start word of 0x5555
    for (axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _printInt(gyroADC[axis], _serialPorts[i]);
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _printInt(accelADC[axis], _serialPorts[i]);
    }
    _printInt(32767, _serialPorts[i]); // Stop word of 0x7FFF
  }

public:
  SerialComs() : 
  SubSystem()
  {
    _serialPortCount = 0;
  }

  void assignSerialPort(HardwareSerial *serialPort)
  {
    if (_serialPortCount <= MAXASSIGNEDSERIALPORTS)
    {
      _serialPorts[_serialPortCount++] = serialPort;
    }
  }

  void initialize(unsigned int frequency, unsigned int offset = 0) 
  {
    for(int i = 0; i< MAXASSIGNEDSERIALPORTS; i++)
    {
      _lastTelemetryType[i] = 'X';
      _fastTransferOn[i] = 0;
      _lastRunTime[i] = offset;
      _normalFrequency[i] = frequency;
      _activeFrequency[i] = frequency;    
    }

    if (_serialPortCount > 0)
    {
      for (unsigned int i = 0; i < _serialPortCount; i++)
      {
        _serialPorts[i]->begin(BAUD);
      }
      this->enable();
    }
    else
    {
      //No serial ports.  Disable this subsystem
      this->disable();
    }
  }

  void process(unsigned long currentTime)
  {
    //since we are doing customized timing we need to make sure we are enabled
    if (this->enabled())
    {
      //Check to see if any of the serial ports are running in fast transfer mode
      for (unsigned int i = 0; i < _serialPortCount; i++)
      {
        if (currentTime > (_lastRunTime[i] + _activeFrequency[i]))
        {
          //Check for any incomming command
          _readCommand(i);

          //This serial port has crossed the time threshold and has somthing to do
          if (_fastTransferOn[i])
          {
            //Fast binary telemetry mode
            _sendBinaryTelemetry(i);
          }
          else
          {
            //Normal ascii telemetry mode
            _sendAsciiTelemetry(i);
          }

          _lastRunTime[i] = currentTime;
        }
      }
    }
  }
};

























