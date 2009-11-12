// Serial interface
// 
// This serial interface for programming and telemetry is based on
// the AeroQuad 1.3.2 serial interface and aims on beeing compatible
// to allow the use of the AeroQuad Configurator software
// written by Ted Carancho / www.aeroquad.info
//
// Specification: http://aeroquad.info/bin/view/Main/SerialCommands_v11#Q_Read_Sensor_Data
//
char query = 'X';

void updateSerial() {
  // Check for serial message
  if(Serial.available()) {
    query = Serial.read();
    
    // ### INTEPRET COMMANDS ###
    switch(query) {
      case 'A': // Receive roll and pitch gyro PID
        pd[RX_ROLL].P = readIntSerial();
        readIntSerial();
        pd[RX_ROLL].D = readIntSerial();
        pd[RX_ROLL].lastPosition = 0;
        pd[RX_PITCH].P = readIntSerial();
        readIntSerial();
        pd[RX_PITCH].D = readIntSerial();
        pd[RX_PITCH].lastPosition = 0;
        break;
      case 'C': // Receive yaw PID
        pd[RX_YAW].P = readIntSerial();
        readIntSerial();
        pd[RX_YAW].D = readIntSerial();
        pd[RX_YAW].lastPosition = 0;
 /*       pd[LEVEL_HEADING].P = readIntSerial();
        readIntSerial();
        pd[LEVEL_HEADING].D = readIntSerial();
        pd[LEVEL_HEADING].lastPosition = 0;*/
        break;
      case 'E': // Receive roll and pitch auto level PID
        pd[LEVEL_ROLL].P = readIntSerial();
        readIntSerial();
        pd[LEVEL_ROLL].D = readIntSerial();
        pd[LEVEL_ROLL].lastPosition = 0;
        pd[LEVEL_PITCH].P = readIntSerial();
        readIntSerial();
        pd[LEVEL_PITCH].D = readIntSerial();
        pd[LEVEL_PITCH].lastPosition = 0;
        break;
      case 'G': // Receive auto level configuration
        level.limit = readIntSerial();
        level.enableRange = readIntSerial();
        break;
      case 'I': // Receive flight control configuration
        readIntSerial(); // windupguard
        rx.speccyFactor = readIntSerial();
        break;
      case 'K': // Receive data filtering values
        gyro.factor = readIntSerial();
        accel.factor = readIntSerial();
        level.timeConstant = readIntSerial();
        break;
      case 'M': // Receive motor smoothing values
        rx.factor[RX_ROLL] = readIntSerial();
        rx.factor[RX_PITCH] = readIntSerial();
        rx.factor[RX_YAW] = readIntSerial();
        rx.factor[RX_THROTTLE] = readIntSerial();
        rx.factor[RX_GEAR] = readIntSerial();
        rx.factor[RX_AUX] = readIntSerial();
        break;
      case 'O': // Receive transmitter calibration values || IGNORE FOR NOW
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        readIntSerial();
        break;
      case 'W': // Write config to EEPROM
        writeInt(pd[RX_ROLL].P, PGAIN_ADR);
        writeInt(0, IGAIN_ADR);
        writeInt(pd[RX_ROLL].D, DGAIN_ADR);
        writeInt(pd[RX_PITCH].P, PITCH_PGAIN_ADR);
        writeInt(0, PITCH_IGAIN_ADR);
        writeInt(pd[RX_PITCH].D, PITCH_DGAIN_ADR);
        writeInt(pd[LEVEL_ROLL].P, LEVEL_ROLL_PGAIN_ADR);
        writeInt(0, LEVEL_ROLL_IGAIN_ADR);
        writeInt(pd[LEVEL_ROLL].D, LEVEL_ROLL_DGAIN_ADR);
        writeInt(pd[LEVEL_PITCH].P, LEVEL_PITCH_PGAIN_ADR);
        writeInt(0, LEVEL_PITCH_IGAIN_ADR);
        writeInt(pd[LEVEL_PITCH].D, LEVEL_PITCH_DGAIN_ADR);        
        writeInt(pd[RX_YAW].P, YAW_PGAIN_ADR);
        writeInt(0, YAW_IGAIN_ADR);
        writeInt(pd[RX_YAW].D, YAW_DGAIN_ADR);
        writeInt(pd[LEVEL_HEADING].P, HEADING_PGAIN_ADR);
        writeInt(0, HEADING_IGAIN_ADR);
        writeInt(pd[LEVEL_HEADING].D, HEADING_DGAIN_ADR);
        writeInt(0, WINDUPGUARD_ADR);
        writeInt(level.limit, LEVEL_LIMIT_ADR);
        writeInt(level.enableRange, LEVEL_ENABLERANGE_ADR);
        writeInt(rx.speccyFactor, XMITFACTOR_ADR);
        writeInt(gyro.factor, GYROSMOOTH_ADR);
        writeInt(accel.factor, ACCSMOOTH_ADR);
        writeInt(rx.factor[RX_THROTTLE], THROTTLESMOOTH_ADR);
        writeInt(rx.factor[RX_ROLL], ROLLSMOOTH_ADR);
        writeInt(rx.factor[RX_PITCH], PITCHSMOOTH_ADR);
        writeInt(rx.factor[RX_YAW], YAWSMOOTH_ADR);
        writeInt(rx.factor[RX_GEAR], GEARSMOOTH_ADR);
        writeInt(rx.factor[RX_AUX], AUXSMOOTH_ADR);
        writeInt(level.timeConstant, FILTERTERM_ADR);
        break;
      case 'Y': // Initialize config with default values
        pd[RX_ROLL].P = 1000;
        pd[RX_ROLL].D = -100;
        pd[RX_PITCH].P = 1000;
        pd[RX_PITCH].D = -100;
        pd[RX_YAW].P = 1200;
        pd[RX_YAW].D = 0;
        pd[LEVEL_ROLL].P = 80;
        pd[LEVEL_ROLL].D = 0;
        pd[LEVEL_PITCH].P = 80;
        pd[LEVEL_PITCH].D = 0;
        pd[LEVEL_HEADING].P = 80;
        pd[LEVEL_HEADING].D = 0;
        level.limit = 250;
        level.enableRange = 50;
        rx.speccyFactor = 1;
        gyro.factor = 4;
        accel.factor = 3;
        rx.factor[RX_ROLL] = 8;
        rx.factor[RX_PITCH] = 8;
        rx.factor[RX_YAW] = 4;
        rx.factor[RX_THROTTLE] = 16;
        rx.factor[RX_GEAR] = 16;
        rx.factor[RX_AUX] = 16;
        level.timeConstant = 245;
        zeroGyros();
        zeroAccelerometers();
        break;
      case 'a': // Enable/disable fast data transfer
        query = 'X';
        if(readIntSerial() == 1) {
          fastTransfer = true;
        } else {
          fastTransfer = false;
        }
        break;
      case 'b': // Calibrate gyros
        zeroGyros();
        break;
      case 'c': // Calibrate accelerometers
        zeroAccelerometers();
        break;
    }
  }

  // ### OUTPUT TELEMETRY ###
  switch(query) {
    case 'B': // Send roll and pitch gyro PID values
      Serial.print(pd[RX_ROLL].P);
      comma();
      Serial.print(0);
      comma();
      Serial.print(pd[RX_ROLL].D);
      comma();
      Serial.print(pd[RX_PITCH].P);
      comma();
      Serial.print(0);
      comma();
      Serial.println(pd[RX_PITCH].D);
      query = 'X';
      break;
    case 'D': // Send yaw PID values
      Serial.print(pd[RX_YAW].P);
      comma();
      Serial.print(0);
      comma();
      Serial.println(pd[RX_YAW].D);
/*      comma();
      Serial.print(pd[LEVEL_HEADING].P);
      comma();
      Serial.print(0);
      comma();
      Serial.println(pd[LEVEL_HEADING].D);*/
      query = 'X';
      break;
    case 'F': // Send roll and pitch auto level PID values
      Serial.print(pd[LEVEL_ROLL].P);
      comma();
      Serial.print(0);
      comma();
      Serial.print(pd[LEVEL_ROLL].D);
      comma();
      Serial.print(pd[LEVEL_PITCH].P);
      comma();
      Serial.print(0);
      comma();
      Serial.println(pd[LEVEL_PITCH].D);
      query = 'X';
      break;
    case 'H': // Send auto level configuration values
      Serial.print(level.limit);
      comma();
      Serial.println(level.enableRange);
      query = 'X';
      break;
    case 'J': // Send flight control configuration values
      Serial.print(0); // Windupguard
      comma();
      Serial.println(rx.speccyFactor);
      query = 'X';
      break;
    case 'L': // Send data filtering values
      Serial.print(gyro.factor);
      comma();
      Serial.print(accel.factor);
      comma();
      Serial.println(level.timeConstant);
      query = 'X';
      break;
    case 'N': // Send data filtering values
      for(int n=RX_ROLL;n<RX_AUX;n++) {
        Serial.print(rx.factor[n]);
        comma();
      }
      Serial.print(rx.factor[RX_AUX]);
      query = 'X';
      break;
    case 'Q': // Send sensor data
      Serial.print(gyro.value[SENSOR_ROLL]);
      comma();
      Serial.print(gyro.value[SENSOR_PITCH]);
      comma();
      Serial.print(gyro.value[SENSOR_YAW]);
      comma();
      Serial.print(accel.value[SENSOR_ROLL]);
      comma();
      Serial.print(accel.value[SENSOR_PITCH]);
      comma();
      Serial.print(accel.value[SENSOR_ZAXIS]);
      comma();
      Serial.print(level.adjustment[SENSOR_ROLL]);
      comma();
      Serial.print(level.adjustment[SENSOR_PITCH]);
      comma();
      Serial.print((int) (level.angle[SENSOR_ROLL]));
      comma();
      Serial.println((int) (level.angle[SENSOR_PITCH]));
      break;
    case 'R': // Send raw sensor data
      Serial.print(gyro.raw[SENSOR_ROLL]);
      comma();
      Serial.print(gyro.raw[SENSOR_PITCH]);
      comma();
      Serial.print(gyro.raw[SENSOR_YAW]);
      comma();
      Serial.print(accel.raw[SENSOR_ROLL]);
      comma();
      Serial.print(accel.raw[SENSOR_PITCH]);
      comma();
      Serial.println(accel.raw[SENSOR_ZAXIS]);
      break;
    case 'S': // Send all flight data
      Serial.print(int(time.deltaMicrosAvg/100.0)); // Deliver in deci of milliseconds
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(gyro.value[n]);
        comma();
      }
      Serial.print(rx.value[RX_THROTTLE]);
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(motor.axisCommand[n]);
        comma();
      }
      for(int n=0;n<4;n++) {
        Serial.print(motor.command[n]);
        comma();
      }
      Serial.print(motor.armed, BIN);
      comma();
      Serial.println(rx.value[RX_GEAR]);
      break;
    case 'T': // Send processed transmitter values
      Serial.print(rx.speccyFactor);
      comma();
      for(int n=0;n<3;n++) {
        Serial.print(rx.command[n]);
        comma();
      }
      Serial.print(level.adjustment[SENSOR_ROLL]);
      comma();
      Serial.print(level.adjustment[SENSOR_PITCH]);
      comma();
      Serial.print(motor.axisCommand[RX_ROLL]);
      comma();
      Serial.print(motor.axisCommand[RX_PITCH]);
      comma();
      Serial.println(motor.axisCommand[RX_YAW]);
      break;
    case 'U': // Send receiver values
      for (int n=0;n<5;n++) {
        Serial.print(rx.value[n]);
        comma();
      }
      Serial.println(rx.value[5]);
      break;
    case 'V': // Send receiver status
      for(int n=0;n<5;n++) {
        Serial.print(rx.command[n]);
        comma();
      }
      Serial.println(rx.command[5]);
      break;
    case 'P': // Send transmitter calibration data || NOT IN USE
      for(int n=0;n<5;n++) {
        Serial.print(1);
        comma();
        Serial.print(0);
        comma();
      }
      Serial.print(1);
      comma();
      Serial.println(0);
      query = 'X';
      break;
    case 'Z': // Send heading
      Serial.print(rx.command[RX_YAW]);
      comma();
      Serial.print(0);
      comma();
      Serial.print(0);
      comma();
      Serial.println(0);      
      break;
    case '6': // Report remote commands
      Serial.print(0);
      comma();
      Serial.print(0);
      comma();
      Serial.print(0);
      comma();
      Serial.println(0);
      break;
    case 'e':
      Serial.println(0);
      break;
    case '!': // Send flight software version
      Serial.println("1.4");
      query = 'X';
      break;
    default: // 'X'
      break;
  }
}

void repeat(int count, int value) {
  for(int n=0;n<count;n++) {
    Serial.print(value);
    comma();
  }
}

void comma() {
  Serial.print(',');
}

// Used to read integer values from the serial port
int readIntSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  
  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return (int) atof(data);
}
