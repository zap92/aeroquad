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

class SerialComs: public SubSystem {
private:
  // Communication
  char queryType;
  byte tlmType;
  char string[32];
  byte axis;
  byte channel;
  byte motor;

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
    while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
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
        controlLaw.setP(ROLL, _readFloatSerial(serialPort));
        controlLaw.setI(ROLL, _readFloatSerial(serialPort));
        controlLaw.setD(ROLL, _readFloatSerial(serialPort));
        controlLaw.setInitPosError(ROLL);
        controlLaw.setP(PITCH, _readFloatSerial(serialPort));
        controlLaw.setI(PITCH, _readFloatSerial(serialPort));
        controlLaw.setD(PITCH, _readFloatSerial(serialPort));
        controlLaw.setInitPosError(PITCH);
        break;

      case 'C': // Receive yaw PID
        controlLaw.setP(YAW, _readFloatSerial(serialPort));
        controlLaw.setI(YAW, _readFloatSerial(serialPort));
        controlLaw.setD(YAW, _readFloatSerial(serialPort));
        controlLaw.setInitPosError(YAW);
        controlLaw.setP(HEADING, _readFloatSerial(serialPort));
        controlLaw.setI(HEADING, _readFloatSerial(serialPort));
        controlLaw.setD(HEADING, _readFloatSerial(serialPort));
        controlLaw.setInitPosError(HEADING);
        break;
      case 'E': // Receive roll and pitch auto level PID
        controlLaw.setP(LEVELROLL, _readFloatSerial(serialPort));
        controlLaw.setI(LEVELROLL, _readFloatSerial(serialPort));
        controlLaw.setD(LEVELROLL, _readFloatSerial(serialPort));
        controlLaw.setInitPosError(LEVELROLL);
        controlLaw.setP(LEVELPITCH, _readFloatSerial(serialPort));
        controlLaw.setI(LEVELPITCH, _readFloatSerial(serialPort));
        controlLaw.setD(LEVELPITCH, _readFloatSerial(serialPort));
        controlLaw.setInitPosError(LEVELPITCH);
        break;
      case 'G': // Receive auto level configuration
        controlLaw.setLevelLimit(_readFloatSerial(serialPort));
        controlLaw.setLevelOff(_readFloatSerial(serialPort));
        break;
      case 'I': // Receive flight control configuration
        controlLaw.setWindupGuard(_readFloatSerial(serialPort));
        flightCommand.setXmitFactor(_readFloatSerial(serialPort));
        break;
      case 'K': // Receive data filtering values
        sensors.setGyroSmoothFactor(ROLL, _readFloatSerial(serialPort));
        sensors.setGyroSmoothFactor(PITCH, sensors.getGyroSmoothFactor(ROLL));
        sensors.setGyroSmoothFactor(YAW, sensors.getGyroSmoothFactor(ROLL));
        sensors.setAccelSmoothFactor(ROLL, _readFloatSerial(serialPort));
        sensors.setAccelSmoothFactor(PITCH, sensors.getAccelSmoothFactor(ROLL));
        sensors.setAccelSmoothFactor(YAW, sensors.getAccelSmoothFactor(ROLL));
        //attitude.setTimeConstant(_readFloatSerial(serialPort));
        flightAngle[ROLL].setTimeConstant(_readFloatSerial(serialPort));
        flightAngle[PITCH].setTimeConstant(flightAngle[ROLL].getTimeConstant());
        break;
      case 'M': // Receive motor smoothing values
        flightCommand.setTransmitterSmoothing(ROLL, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSmoothing(PITCH, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSmoothing(YAW, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSmoothing(THROTTLE, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSmoothing(MODE, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSmoothing(AUX, _readFloatSerial(serialPort));
        break;
      case 'O': // Received transmitter calibrtion values
        flightCommand.setTransmitterSlope(ROLL, _readFloatSerial(serialPort));
        flightCommand.setTransmitterOffset(ROLL, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSlope(PITCH, _readFloatSerial(serialPort));
        flightCommand.setTransmitterOffset(PITCH, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSlope(YAW, _readFloatSerial(serialPort));
        flightCommand.setTransmitterOffset(YAW, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSlope(THROTTLE, _readFloatSerial(serialPort));
        flightCommand.setTransmitterOffset(THROTTLE, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSlope(MODE, _readFloatSerial(serialPort));
        flightCommand.setTransmitterOffset(MODE, _readFloatSerial(serialPort));
        flightCommand.setTransmitterSlope(AUX, _readFloatSerial(serialPort));
        flightCommand.setTransmitterOffset(AUX, _readFloatSerial(serialPort));
        break;
      case 'W': // Write all user configurable values to EEPROM
        eeprom.write(controlLaw.getP(ROLL), PGAIN_ADR);
        eeprom.write(controlLaw.getI(ROLL), IGAIN_ADR);
        eeprom.write(controlLaw.getD(ROLL), DGAIN_ADR);
        eeprom.write(controlLaw.getP(PITCH), PITCH_PGAIN_ADR);
        eeprom.write(controlLaw.getI(PITCH), PITCH_IGAIN_ADR);
        eeprom.write(controlLaw.getD(PITCH), PITCH_DGAIN_ADR);
        eeprom.write(controlLaw.getP(LEVELROLL), LEVEL_PGAIN_ADR);
        eeprom.write(controlLaw.getI(LEVELROLL), LEVEL_IGAIN_ADR);
        eeprom.write(controlLaw.getD(LEVELROLL), LEVEL_DGAIN_ADR);
        eeprom.write(controlLaw.getP(LEVELPITCH), LEVEL_PITCH_PGAIN_ADR);
        eeprom.write(controlLaw.getI(LEVELPITCH), LEVEL_PITCH_IGAIN_ADR);
        eeprom.write(controlLaw.getD(LEVELPITCH), LEVEL_PITCH_DGAIN_ADR);
        eeprom.write(controlLaw.getP(YAW), YAW_PGAIN_ADR);
        eeprom.write(controlLaw.getI(YAW), YAW_IGAIN_ADR);
        eeprom.write(controlLaw.getD(YAW), YAW_DGAIN_ADR);
        eeprom.write(controlLaw.getP(HEADING), HEADING_PGAIN_ADR);
        eeprom.write(controlLaw.getI(HEADING), HEADING_IGAIN_ADR);
        eeprom.write(controlLaw.getD(HEADING), HEADING_DGAIN_ADR);
        eeprom.write(controlLaw.getWindupGuard(), WINDUPGUARD_ADR);  
        eeprom.write(controlLaw.getLevelLimit(), LEVELLIMIT_ADR);   
        eeprom.write(controlLaw.getLevelOff(), LEVELOFF_ADR); 
        eeprom.write(flightCommand.getXmitFactor(), XMITFACTOR_ADR);
        for (axis = ROLL; axis < LASTAXIS; axis++) {
          eeprom.write(sensors.getGyroSmoothFactor(axis), GYROSMOOTH_ADR);
          eeprom.write(sensors.getAccelSmoothFactor(axis), ACCSMOOTH_ADR);
        }
        eeprom.write(flightCommand.getTransmitterSmoothing(THROTTLE), THROTTLESMOOTH_ADR);
        eeprom.write(flightCommand.getTransmitterSmoothing(ROLL), ROLLSMOOTH_ADR);
        eeprom.write(flightCommand.getTransmitterSmoothing(PITCH), PITCHSMOOTH_ADR);
        eeprom.write(flightCommand.getTransmitterSmoothing(YAW), YAWSMOOTH_ADR);
        eeprom.write(flightCommand.getTransmitterSmoothing(MODE), MODESMOOTH_ADR);
        eeprom.write(flightCommand.getTransmitterSmoothing(AUX), AUXSMOOTH_ADR);
        //eeprom.write(attitude.getTimeConstant(), FILTERTERM_ADR);
        eeprom.write(flightAngle[ROLL].getTimeConstant(), FILTERTERM_ADR);
        eeprom.write(flightCommand.getTransmitterSlope(THROTTLE), THROTTLESCALE_ADR);
        eeprom.write(flightCommand.getTransmitterOffset(THROTTLE), THROTTLEOFFSET_ADR);
        eeprom.write(flightCommand.getTransmitterSlope(ROLL), ROLLSCALE_ADR);
        eeprom.write(flightCommand.getTransmitterOffset(ROLL), ROLLOFFSET_ADR);
        eeprom.write(flightCommand.getTransmitterSlope(PITCH), PITCHSCALE_ADR);
        eeprom.write(flightCommand.getTransmitterOffset(PITCH), PITCHOFFSET_ADR);
        eeprom.write(flightCommand.getTransmitterSlope(YAW), YAWSCALE_ADR);
        eeprom.write(flightCommand.getTransmitterOffset(YAW), YAWOFFSET_ADR);
        eeprom.write(flightCommand.getTransmitterSlope(MODE), MODESCALE_ADR);
        eeprom.write(flightCommand.getTransmitterOffset(MODE), MODEOFFSET_ADR);
        eeprom.write(flightCommand.getTransmitterSlope(AUX), AUXSCALE_ADR);
        eeprom.write(flightCommand.getTransmitterOffset(AUX), AUXOFFSET_ADR);
        //eeprom.write(attitude.getHeadingSmoothFactor(), HEADINGSMOOTH_ADR);
        controlLaw.zeroIntegralError();
        break;
      case 'Y': // Initialize EEPROM with default values
        controlLaw.setP(ROLL, 3.75);
        controlLaw.setI(ROLL, 0);
        controlLaw.setD(ROLL, -10);
        controlLaw.setP(PITCH, 3.75);
        controlLaw.setI(PITCH, 0);
        controlLaw.setD(PITCH, -10);
        controlLaw.setP(YAW, 12.0);
        controlLaw.setI(YAW, 0);
        controlLaw.setD(YAW, 0);
        controlLaw.setP(LEVELROLL, 2);
        controlLaw.setI(LEVELROLL, 0);
        controlLaw.setD(LEVELROLL, 0);
        controlLaw.setP(LEVELPITCH, 2);
        controlLaw.setI(LEVELPITCH, 0);
        controlLaw.setD(LEVELPITCH, 0);
        controlLaw.setP(HEADING, 3);
        controlLaw.setI(HEADING, 0);
        controlLaw.setD(HEADING, 0);
        controlLaw.setWindupGuard(2000);
        controlLaw.setLevelLimit(2000);
        controlLaw.setLevelOff(50);
        for (axis = ROLL; axis < LASTAXIS; axis++) {  
          sensors.setGyroSmoothFactor(axis, 0.50);
          sensors.setAccelSmoothFactor(axis, 0.50);
        }
        //attitude.setTimeConstant(3.0);
        flightAngle[ROLL].setTimeConstant(3.0);
        flightAngle[PITCH].setTimeConstant(3.0);
        //attitude.setHeadingSmoothFactor(1.0);
        for (channel = ROLL; channel < LASTCHANNEL; channel++) {
          flightCommand.setTransmitterSlope(channel, 1.0);
          flightCommand.setTransmitterOffset(channel, 0.0);
        }
        flightCommand.setTransmitterSmoothing(THROTTLE, 1.0);
        flightCommand.setTransmitterSmoothing(ROLL, 1.0);
        flightCommand.setTransmitterSmoothing(PITCH, 1.0);
        flightCommand.setTransmitterSmoothing(YAW, 0.5);  
        flightCommand.setTransmitterSmoothing(MODE, 1.0);
        flightCommand.setTransmitterSmoothing(AUX, 1.0);
        flightCommand.setXmitFactor(0.20);  
        sensors.zeroGyros();
        sensors.zeroAccelerometers();
        controlLaw.zeroIntegralError();
        break;
      case '1': // Calibrate ESCS's by setting Throttle high on all channels
        flightControl.setArmStatus(OFF);
        flightControl.setCalibrationESC(1);
        break;
      case '2': // Calibrate ESC's by setting Throttle low on all channels
        flightControl.setArmStatus(OFF);
        flightControl.setCalibrationESC(2);
        break;
      case '3': // Test ESC calibration
        flightControl.setArmStatus(OFF);
        flightControl.setCalibrationESC(3);
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          flightControl.setRemoteCommand(motor, _readFloatSerial(serialPort));
        break;
      case '4': // Turn off ESC calibration
        flightControl.setArmStatus(OFF);
        flightControl.setCalibrationESC(0);
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          flightControl.setRemoteCommand(motor, MINCOMMAND);
        break;        
      case '5': // Send individual motor commands (motor, command)
        flightControl.setArmStatus(OFF);
        flightControl.setCalibrationESC(5);
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          flightControl.setRemoteCommand(motor, _readFloatSerial(serialPort));
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
      serialPort->print(controlLaw.getP(ROLL));
      _comma(serialPort);
      serialPort->print(controlLaw.getI(ROLL));
      _comma(serialPort);
      serialPort->print(controlLaw.getD(ROLL));
      _comma(serialPort);
      serialPort->print(controlLaw.getP(PITCH));
      _comma(serialPort);
      serialPort->print(controlLaw.getI(PITCH));
      _comma(serialPort);
      serialPort->println(controlLaw.getD(PITCH));            

      _lastTelemetryType[i] = 'X';
      break;
    case 'D': // Send yaw PID values
      serialPort->print(controlLaw.getP(YAW));
      _comma(serialPort);
      serialPort->print(controlLaw.getI(YAW));
      _comma(serialPort);
      serialPort->print(controlLaw.getD(YAW));
      _comma(serialPort);
      serialPort->print(controlLaw.getP(HEADING));
      _comma(serialPort);
      serialPort->print(controlLaw.getI(HEADING));
      _comma(serialPort);
      serialPort->println(controlLaw.getD(HEADING));

      _lastTelemetryType[i] = 'X';
      break;
    case 'F': // Send roll and pitch auto level PID values
      serialPort->print(controlLaw.getP(LEVELROLL));
      _comma(serialPort);
      serialPort->print(controlLaw.getI(LEVELROLL));
      _comma(serialPort);
      serialPort->print(controlLaw.getD(LEVELROLL));
      _comma(serialPort);
      serialPort->print(controlLaw.getP(LEVELPITCH));
      _comma(serialPort);
      serialPort->print(controlLaw.getI(LEVELPITCH));
      _comma(serialPort);
      serialPort->println(controlLaw.getD(LEVELPITCH));
      _lastTelemetryType[i] = 'X';
      break;
    case 'H': // Send auto level configuration values
      serialPort->print(controlLaw.getLevelLimit());
      _comma(serialPort);
      serialPort->println(controlLaw.getLevelOff());
      _lastTelemetryType[i] = 'X';
      break;
    case 'J': // Send flight control configuration values
      serialPort->print(controlLaw.getWindupGuard());
      _comma(serialPort);
      serialPort->println(flightCommand.getXmitFactor());
      _lastTelemetryType[i] = 'X';
      break;
    case 'L': // Send data filtering values
      serialPort->print(sensors.getGyroSmoothFactor(ROLL));
      _comma(serialPort);
      serialPort->print(sensors.getAccelSmoothFactor(ROLL));
      _comma(serialPort);
      //serialPort->println(attitude.getTimeConstant());
      serialPort->println(flightAngle[ROLL].getTimeConstant());
      _lastTelemetryType[i] = 'X';
      break;
    case 'N': // Send motor smoothing values
      for (axis = ROLL; axis < AUX; axis++) {
        serialPort->print(flightCommand.getTransmitterSmoothing(axis));
        _comma(serialPort);
      }
      serialPort->println(flightCommand.getTransmitterSmoothing(AUX));
      _lastTelemetryType[i] = 'X';
      break;
    case 'P': // Send transmitter calibration data
      for (axis = ROLL; axis < AUX; axis++) {
        serialPort->print(flightCommand.getTransmitterSlope(axis));
        _comma(serialPort);
        serialPort->print(flightCommand.getTransmitterOffset(axis));
        _comma(serialPort);
      }
      serialPort->print(flightCommand.getTransmitterSlope(AUX));
      _comma(serialPort);
      serialPort->println(flightCommand.getTransmitterOffset(AUX));
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
        serialPort->print(controlLaw.getLevelAdjust(axis));
        _comma(serialPort);
      }
      //serialPort->print(attitude.getFlightAngle(ROLL));
      serialPort->print(flightAngle[ROLL].readCurrentAngle());
      _comma(serialPort);
      //serialPort->print(attitude.getFlightAngle(PITCH));
      serialPort->print(flightAngle[PITCH].readCurrentAngle());
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
      serialPort->print(0); // *** remove this in Configurator
      _comma(serialPort);
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        serialPort->print(sensors.getGyro(axis));
        _comma(serialPort);
      }
      serialPort->print(flightCommand.read(THROTTLE));
      _comma(serialPort);
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        serialPort->print(flightControl.read(axis));
        _comma(serialPort);
      }
      for (motor = FRONT; motor < LASTMOTOR; motor++) {
        serialPort->print(motors.getMotorCommand(motor));
        _comma(serialPort);
      }
      serialPort->print(flightControl.getArmStatus(), BIN);
      _comma(serialPort);
      if (flightControl.getAutoLevel() == ON)
        serialPort->println(flightCommand.read(MODE));
      else
        serialPort->println(1000);
      break;
    case 'T': // Send processed transmitter values
      serialPort->print(flightCommand.getXmitFactor());
      _comma(serialPort);
      for (axis = ROLL; axis < LASTAXIS; axis++) {
        serialPort->print(flightCommand.read(axis));
        _comma(serialPort);
      }
      for (axis = ROLL; axis < YAW; axis++) {
        serialPort->print(flightControl.getLevelAdjust(axis));
        _comma(serialPort);
      }
      serialPort->print(flightCommand.read(ROLL));
      _comma(serialPort);
      serialPort->print(flightCommand.read(PITCH));
      _comma(serialPort);
      serialPort->println(flightCommand.read(YAW));
      break;
    case 'U': // Send smoothed receiver values
      for (channel = ROLL; channel < AUX; channel++) {
        serialPort->print(flightCommand.read(channel));
        _comma(serialPort);
      }
      serialPort->println(flightCommand.read(AUX));
      break;
    case 'V': // Send receiver status
      for (channel = ROLL; channel < AUX; channel++) {
        serialPort->print(flightCommand.getScaledReceiverData(channel));
        _comma(serialPort);
      }
      serialPort->println(flightCommand.getScaledReceiverData(AUX));
      break;
    case 'X': // Stop sending messages
      break;
    case 'Z': // Send heading
      serialPort->print(flightCommand.read(YAW));
      _comma(serialPort);
      serialPort->print(flightControl.getHeadingCommand());
      _comma(serialPort);
      serialPort->print(flightControl.getHeading());
      _comma(serialPort);
      serialPort->println(flightControl.getCurrentHeading());
      break;
    case '6': // Report remote commands
      for (motor = FRONT; motor < LEFT; motor++) {
        serialPort->print(motors.getRemoteMotorCommand(motor));
        _comma(serialPort);
      }
      serialPort->println(motors.getRemoteMotorCommand(LEFT));
      break;
    case '!': // Send flight software version
      serialPort->println("1.6");
      _lastTelemetryType[i] = 'X';
      break;
    }
  }

  void _sendBinaryTelemetry(unsigned int i) 
  {
    _printInt(21845, _serialPorts[i]); // Start word of 0x5555
    for (axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _printInt(sensors.getRawGyro(axis), _serialPorts[i]);
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _printInt(sensors.getRawAccel(axis), _serialPorts[i]);
    }
    _printInt(32767, _serialPorts[i]); // Stop word of 0x7FFF
  }

public:
  SerialComs() : 
  SubSystem()
  {
    _serialPortCount = 0;
    queryType = 'X';
    tlmType = 0;
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

  void process(unsigned long currentTime) {
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

























