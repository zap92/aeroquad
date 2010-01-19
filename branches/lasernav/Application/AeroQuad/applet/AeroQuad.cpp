 /*
  AeroQuad v1.5 - Novmeber 2009
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

/**************************************************************************** 
   Before flight, select the different user options for your AeroQuad below
   Also, consult the ReadMe.html file for additional details
   If you need additional assitance go to http://forum.AeroQuad.info
*****************************************************************************/

// Define Flight Configuration
#define plusConfig
//#define XConfig

// Calibration At Start Up
//#define CalibrationAtStartup
#define GyroCalibrationAtStartup

// Motor Control Method (ServoControl = higher ESC resolution, cooler motors, slight jitter in control, AnalogWrite = lower ESC resolution, warmer motors, no jitter)
// Before your first flight, to use the ServoControl method change the following line in Servo.h
// Which can be found in \arduino-0017\hardware\libraries\Servo\Servo.h
// For camera stabilization off, update line 54 with: #define REFRESH_INTERVAL 8000
// For camera stabilization on, update line 54 with: #define REFRESH_INTERVAL 12000
// For ServoControl method connect AUXPIN=3, MOTORPIN=8 for compatibility with PCINT
//#define ServoControl // This is only compatible with Arduino 0017 or greater
// For AnalogWrite method connect AUXPIN=8, MOTORPIN=3
#define AnalogWrite

// Camera Stabilization (experimental)
// Will move development to Arduino Mega (needs analogWrite support for additional pins)
//#define Camera

// Heading Hold (experimental)
// Currently uses yaw gyro which drifts over time, for Mega development will use magnetometer
//#define HeadingHold

// Auto Level (experimental)
//#define AutoLevel

// 5DOF IMU Version
//#define OriginalIMU // Use this if you have the 5DOF IMU which uses the IDG300

// Arduino Mega with AeroQuad Shield v1.x
// If you are using the Arduino Mega with an AeroQuad Shield v1.x, the receiver pins must be configured differently due to bug in Arduino core.
// Put a jumper wire between the Shield and Mega as indicated below
// For Roll (Aileron) Channel, place jumper between AQ Shield pin 2 and Mega AI13
// For Pitch (Elevator) Channel, place jumper between AQ Shield pin 5 and Mega AI11
// For Yaw (Rudder) Channel, place jumper between AQ Shield pin 6 and Mega AI10
// For Throttle Channel, place jumper between AQ Shield pin 4 and Mega AI12
// For Mode (Gear) Channel, place jumper between AQ Shield pin 7 and Mega AI9
// For Aux Channel, place jumper between AQ Shield 8 and Mega AI8
//#define Mega_AQ1x

// Yaw Gyro Type (experimental, used for investigation of lower cost gyros)
#define IDG // InvenSense
//#define LPY // STMicroelectronics

// Sensor Filter
// The Kalman Filter implementation is here for comparison against the Complementary Filter
// To adjust the KF parameters, look at initGyro1DKalman() found inside ConfigureFilter() in Filter.pde
//#define KalmanFilter

// *************************************************************

#include <stdlib.h>
#include <math.h>
#include <EEPROM.h>
#include <Servo.h>
#include "EEPROM_AQ.h"
#include "Filter.h"
#include "PID.h"
#include "Receiver.h"
#include "Sensors.h"
#include "Motors.h"
#include "AeroQuad.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
#include "WProgram.h"
void setup();
void loop ();
float readFloat(int address);
void writeFloat(float value, int address);
void readEEPROM();
void configureFilter(float timeConstant);
float filterData(float previousAngle, int gyroADC, float angle, float *filterTerm, float dt);
void initGyro1DKalman(struct Gyro1DKalman *filterdata, float Q_angle, float Q_gyro, float R_angle);
void predictKalman(struct Gyro1DKalman *filterdata, const float dotAngle, const float dt);
float updateKalman(struct Gyro1DKalman *filterdata, const float angle_m);
int smooth(int currentData, int previousData, float smoothFactor);
void configureMotors();
void commandMotors();
void commandAllMotors(int motorCommand);
void pulseMotors(byte quantity);
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters);
void zeroIntegralError();
void init_servodecode();
void configureReceiver();
unsigned int readReceiver(byte receiverPin);
int findMode(int *data, int arraySize);
void zeroGyros();
void autoZeroGyros();
void zeroAccelerometers();
int limitRange(int data, int minLimit, int maxLimit);
float arctan2(float y, float x);
void readSerialCommand();
float readFloatSerial();
void sendSerialTelemetry();
void comma();
void printInt(int data);
void setup() {
  Serial.begin(BAUD);
  analogReference(EXTERNAL); // Current external ref is connected to 3.3V
  pinMode (LEDPIN, OUTPUT);
  pinMode (AZPIN, OUTPUT);
  digitalWrite(AZPIN, LOW);
  delay(1);
  
  // Configure motors
  configureMotors();
  commandAllMotors(MINCOMMAND);

  // Read user values from EEPROM
  readEEPROM();
  
  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON)
     configureReceiver();
  
  //  Auto Zero Gyros
  autoZeroGyros();
  
  #ifdef CalibrationAtStartup
    // Calibrate sensors
    zeroGyros();
    zeroAccelerometers();
    zeroIntegralError();
  #endif
  #ifdef GyroCalibrationAtStartup
    zeroGyros();
  #endif
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Camera stabilization setup
  #ifdef Camera
    rollCamera.attach(ROLLCAMERAPIN);
    pitchCamera.attach(PITCHCAMERAPIN);
  #endif
  
  // Complementary filter setup
  configureFilter(timeConstant);
  
  previousTime = millis();
  digitalWrite(LEDPIN, HIGH);
  safetyCheck = 0;
}

// ************************************************************
// ******************** Main AeroQuad Loop ********************
// ************************************************************
void loop () {
  // Measure loop rate
  currentTime = millis();
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;
  #ifdef DEBUG
    if (testSignal == LOW) testSignal = HIGH;
    else testSignal = LOW;
    digitalWrite(LEDPIN, testSignal);
  #endif
  
// ************************************************************************
// ****************** Transmitter/Receiver Command Loop *******************
// ************************************************************************
  if ((currentTime > (receiverTime + RECEIVERLOOPTIME)) && (receiverLoop == ON)) { // 10Hz
    // Buffer receiver values read from pin change interrupt handler
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      receiverData[channel] = (mTransmitter[channel] * readReceiver(receiverPin[channel])) + bTransmitter[channel];
    // Smooth the flight control transmitter inputs (roll, pitch, yaw, throttle)
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      transmitterCommandSmooth[channel] = smooth(receiverData[channel], transmitterCommandSmooth[channel], smoothTransmitter[channel]);
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (channel = ROLL; channel < LASTAXIS; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
    // Read quad configuration commands from transmitter when throttle down
    if (receiverData[THROTTLE] < MINCHECK) {
      zeroIntegralError();
      // Disarm motors (left stick lower left corner)
      if (receiverData[YAW] < MINCHECK && armed == 1) {
        armed = 0;
        commandAllMotors(MINCOMMAND);
      }    
      // Zero sensors (left stick lower left, right stick lower right corner)
      if ((receiverData[YAW] < MINCHECK) && (receiverData[ROLL] > MAXCHECK) && (receiverData[PITCH] < MINCHECK)) {
        autoZeroGyros();
        zeroGyros();
        zeroAccelerometers();
        zeroIntegralError();
        pulseMotors(3);
      }   
      // Arm motors (left stick lower right corner)
      if (receiverData[YAW] > MAXCHECK && armed == 0 && safetyCheck == 1) {
        armed = 1;
        zeroIntegralError();
        minCommand = MINTHROTTLE;
        transmitterCenter[PITCH] = receiverData[PITCH];
        transmitterCenter[ROLL] = receiverData[ROLL];
      }
      // Prevents accidental arming of motor output if no transmitter command received
      if (receiverData[YAW] > MINCHECK) safetyCheck = 1; 
    }
    // Prevents too little power applied to motors during hard manuevers
    if (receiverData[THROTTLE] > (MIDCOMMAND - MINDELTA)) minCommand = receiverData[THROTTLE] - MINDELTA;
    if (receiverData[THROTTLE] < MINTHROTTLE) minCommand = MINTHROTTLE;
    // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
    //if ((receiverData[ROLL] < MINCHECK) || (receiverData[ROLL] > MAXCHECK) || (receiverData[PITCH] < MINCHECK) || (receiverData[PITCH] > MAXCHECK))
      //minCommand = MINTHROTTLE;
    receiverTime = currentTime;
  } 
/////////////////////////////
// End of transmitter loop //
/////////////////////////////
  
// ***********************************************************
// ********************* Analog Input Loop *******************
// ***********************************************************
  if ((currentTime > (analogInputTime + AILOOPTIME)) && (analogInputLoop == ON)) { // 500Hz
    // *********************** Read Sensors **********************
    // Apply low pass filter to sensor values and center around zero
    // Did not convert to engineering units, since will experiment to find P gain anyway
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
    }

    #ifndef OriginalIMU
      gyroADC[ROLL] = -gyroADC[ROLL];
      gyroADC[PITCH] = -gyroADC[PITCH];
    #endif
    #ifdef LPY
      gyroADC[YAW] = -gyroADC[YAW];
    #endif
  
    // Compiler seems to like calculating this in separate loop better
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor[GYRO]);
      accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor[ACCEL]);
    }

    // ****************** Calculate Absolute Angle *****************
    #ifndef KalmanFilter
      //filterData(previousAngle, gyroADC, angle, *filterTerm, dt)
      flightAngle[ROLL] = filterData(flightAngle[ROLL], gyroADC[ROLL], atan2(accelADC[ROLL], accelADC[ZAXIS]), filterTermRoll, AIdT);
      flightAngle[PITCH] = filterData(flightAngle[PITCH], gyroADC[PITCH], atan2(accelADC[PITCH], accelADC[ZAXIS]), filterTermPitch, AIdT);
      //flightAngle[ROLL] = filterData(flightAngle[ROLL], gyroData[ROLL], atan2(accelData[ROLL], accelData[ZAXIS]), filterTermRoll, AIdT);
      //flightAngle[PITCH] = filterData(flightAngle[PITCH], gyroData[PITCH], atan2(accelData[PITCH], accelData[ZAXIS]), filterTermPitch, AIdT);
    #endif
      
    #ifdef KalmanFilter
      predictKalman(&rollFilter, (gyroADC[ROLL]/1024) * aref * 8.72, AIdT);
      flightAngle[ROLL] = updateKalman(&rollFilter, atan2(accelADC[ROLL], accelADC[ZAXIS])) * 57.2957795;
      predictKalman(&pitchFilter, (gyroADC[PITCH]/1024) * aref * 8.72, AIdT);
      flightAngle[PITCH] = updateKalman(&pitchFilter, atan2(accelADC[PITCH], accelADC[ZAXIS])) * 57.2957795;
    #endif
    
    analogInputTime = currentTime;
  } 
//////////////////////////////
// End of analog input loop //
//////////////////////////////
  
// ********************************************************************
// *********************** Flight Control Loop ************************
// ********************************************************************
  if ((currentTime > controlLoopTime + CONTROLLOOPTIME) && (controlLoop == ON)) { // 500Hz

  // ********************* Check Flight Mode *********************
    #ifdef AutoLevel
      if (transmitterCommandSmooth[MODE] < 1500) {
        // Acrobatic Mode
        levelAdjust[ROLL] = 0;
        levelAdjust[PITCH] = 0;
      }
      else {
        // Stable Mode
        for (axis = ROLL; axis < YAW; axis++)
          levelAdjust[axis] = limitRange(updatePID(0, flightAngle[axis], &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
        // Turn off Stable Mode if transmitter stick applied
        if ((abs(receiverData[ROLL] - transmitterCenter[ROLL]) > levelOff)) {
          levelAdjust[ROLL] = 0;
          PID[axis].integratedError = 0;
        }
        if ((abs(receiverData[PITCH] - transmitterCenter[PITCH]) > levelOff)) {
          levelAdjust[PITCH] = 0;
          PID[PITCH].integratedError = 0;
        }
      }
    #endif
    
    // ************************** Update Roll/Pitch ***********************
    // updatedPID(target, measured, PIDsettings);
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
    motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL] + levelAdjust[ROLL], (gyroData[ROLL] * mMotorRate) + bMotorRate, &PID[ROLL]);
    motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH] - levelAdjust[PITCH], (gyroData[PITCH] * mMotorRate) + bMotorRate, &PID[PITCH]);

    // ***************************** Update Yaw ***************************
    // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
    // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
    #ifdef HeadingHold
      currentHeading += gyroData[YAW] * headingScaleFactor * controldT;
      if (transmitterCommand[THROTTLE] > MINCHECK ) { // apply heading hold only when throttle high enough to start flight
        if ((transmitterCommand[YAW] > (MIDCOMMAND + 25)) || (transmitterCommand[YAW] < (MIDCOMMAND - 25))) { // if commanding yaw, turn off heading hold
          headingHold = 0;
          heading = currentHeading;
        }
        else // no yaw input, calculate current heading vs. desired heading heading hold
          headingHold = updatePID(heading, currentHeading, &PID[HEADING]);
      }
      else {
        heading = 0;
        currentHeading = 0;
        headingHold = 0;
        PID[HEADING].integratedError = 0;
      }
      motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW] + headingHold, (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    #endif
    
    #ifndef HeadingHold
      motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW], (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    #endif
    
    // ****************** Calculate Motor Commands *****************
    if (armed && safetyCheck) {
      #ifdef plusConfig
        motorCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
        motorCommand[REAR] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
        motorCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
        motorCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
      #endif
      #ifdef XConfig
        // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
        motorCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] + motorAxisCommand[ROLL] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
        motorCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
        motorCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] + motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
        motorCommand[REAR] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[ROLL] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
      #endif
    }
  
    // If throttle in minimum position, don't apply yaw
    if (transmitterCommand[THROTTLE] < MINCHECK) {
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motorCommand[motor] = minCommand;
    }
    // If motor output disarmed, force motor output to minimum
    if (armed == 0) {
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
    
    // *********************** Command Motors **********************
    commandMotors();
    controlLoopTime = currentTime;
  } 
/////////////////////////
// End of control loop //
/////////////////////////
  
// *************************************************************
// **************** Command & Telemetry Functions **************
// *************************************************************
  if ((currentTime > telemetryTime + TELEMETRYLOOPTIME) && (telemetryLoop == ON)) { // 10Hz    
    readSerialCommand();
    sendSerialTelemetry();
    telemetryTime = currentTime;
  }
///////////////////////////
// End of telemetry loop //
///////////////////////////
  
// *************************************************************
// ******************* Camera Stailization *********************
// *************************************************************
#ifdef Camera // Development moved to Arduino Mega
  if ((currentTime > (cameraTime + CAMERALOOPTIME)) && (cameraLoop == ON)) { // 50Hz
    rollCamera.write((mCamera * flightAngle[ROLL]) + bCamera);
    pitchCamera.write(-(mCamera * flightAngle[PITCH]) + bCamera);
    cameraTime = currentTime;
  }
#endif
////////////////////////
// End of camera loop //
////////////////////////

// **************************************************************
// ***************** Fast Transfer Of Sensor Data ***************
// **************************************************************
  if ((currentTime > (fastTelemetryTime + FASTTELEMETRYTIME)) && (fastTransfer == ON)) { // 200Hz means up to 100Hz signal can be detected by FFT
    printInt(21845); // Start word of 0x5555
    for (axis = ROLL; axis < LASTAXIS; axis++) printInt(gyroADC[axis]);
    for (axis = ROLL; axis < LASTAXIS; axis++) printInt(accelADC[axis]);
    printInt(32767); // Stop word of 0x7FFF
    fastTelemetryTime = currentTime;
  }
////////////////////////////////
// End of fast telemetry loop //
////////////////////////////////

}
/*
  AeroQuad v1.5 - Novmeber 2009
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

#include "EEPROM_AQ.h"

// Utilities for writing and reading from the EEPROM
float readFloat(int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;
  
  for (int i = 0; i < 4; i++) 
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal;
}

void writeFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;
  
  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++) 
    EEPROM.write(address + i, floatIn.floatByte[i]);
}

void readEEPROM() {
  PID[ROLL].P = readFloat(PGAIN_ADR);
  PID[ROLL].I = readFloat(IGAIN_ADR);
  PID[ROLL].D = readFloat(DGAIN_ADR);
  PID[ROLL].lastPosition = 0;
  PID[ROLL].integratedError = 0;
  
  PID[PITCH].P = readFloat(PITCH_PGAIN_ADR);
  PID[PITCH].I = readFloat(PITCH_IGAIN_ADR);
  PID[PITCH].D = readFloat(PITCH_DGAIN_ADR);
  PID[PITCH].lastPosition = 0;
  PID[PITCH].integratedError = 0;
  
  PID[YAW].P = readFloat(YAW_PGAIN_ADR);
  PID[YAW].I = readFloat(YAW_IGAIN_ADR);
  PID[YAW].D = readFloat(YAW_DGAIN_ADR);
  PID[YAW].lastPosition = 0;
  PID[YAW].integratedError = 0;
  
  PID[LEVELROLL].P = readFloat(LEVEL_PGAIN_ADR);
  PID[LEVELROLL].I = readFloat(LEVEL_IGAIN_ADR);
  PID[LEVELROLL].D = readFloat(LEVEL_DGAIN_ADR);
  PID[LEVELROLL].lastPosition = 0;
  PID[LEVELROLL].integratedError = 0;  
  
  PID[LEVELPITCH].P = readFloat(LEVEL_PITCH_PGAIN_ADR);
  PID[LEVELPITCH].I = readFloat(LEVEL_PITCH_IGAIN_ADR);
  PID[LEVELPITCH].D = readFloat(LEVEL_PITCH_DGAIN_ADR);
  PID[LEVELPITCH].lastPosition = 0;
  PID[LEVELPITCH].integratedError = 0;
  
  PID[HEADING].P = readFloat(HEADING_PGAIN_ADR);
  PID[HEADING].I = readFloat(HEADING_IGAIN_ADR);
  PID[HEADING].D = readFloat(HEADING_DGAIN_ADR);
  PID[HEADING].lastPosition = 0;
  PID[HEADING].integratedError = 0;
  
  mTransmitter[THROTTLE] = readFloat(THROTTLESCALE_ADR);
  bTransmitter[THROTTLE] = readFloat(THROTTLEOFFSET_ADR);
  mTransmitter[ROLL] = readFloat(ROLLSCALE_ADR);
  bTransmitter[ROLL] = readFloat(ROLLOFFSET_ADR);
  mTransmitter[PITCH] = readFloat(PITCHSCALE_ADR);
  bTransmitter[PITCH] = readFloat(PITCHOFFSET_ADR);
  mTransmitter[YAW] = readFloat(YAWSCALE_ADR);
  bTransmitter[YAW] = readFloat(YAWOFFSET_ADR);
  mTransmitter[MODE] = readFloat(MODESCALE_ADR);
  bTransmitter[MODE] = readFloat(MODEOFFSET_ADR);
  mTransmitter[AUX] = readFloat(AUXSCALE_ADR);
  bTransmitter[AUX] = readFloat(AUXOFFSET_ADR);

  windupGuard = readFloat(WINDUPGUARD_ADR);
  levelLimit = readFloat(LEVELLIMIT_ADR);
  levelOff = readFloat(LEVELOFF_ADR);
  xmitFactor = readFloat(XMITFACTOR_ADR);
  smoothFactor[GYRO] = readFloat(GYROSMOOTH_ADR);
  smoothFactor[ACCEL] = readFloat(ACCSMOOTH_ADR);
  smoothTransmitter[THROTTLE] = readFloat(THROTTLESMOOTH_ADR);
  smoothTransmitter[ROLL] = readFloat(ROLLSMOOTH_ADR);
  smoothTransmitter[PITCH] = readFloat(PITCHSMOOTH_ADR);
  smoothTransmitter[YAW] = readFloat(YAWSMOOTH_ADR);
  smoothTransmitter[MODE] = readFloat(MODESMOOTH_ADR);
  smoothTransmitter[AUX] = readFloat(AUXSMOOTH_ADR);
  accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
  accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
  accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
  gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
  gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
  gyroZero[YAW] = readFloat(GYRO_YAW_ZERO_ADR);
  timeConstant = readFloat(FILTERTERM_ADR);
  smoothHeading = readFloat(HEADINGSMOOTH_ADR);
}
/*
  AeroQuad v1.5 - Novmeber 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
  
  Second order complementary filter written by LB Roy
  http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286
 
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

#include "Filter.h"

void configureFilter(float timeConstant) {
  #ifndef KalmanFilter
    flightAngle[ROLL] = atan2(analogRead(accelChannel[ROLL]) - accelZero[ROLL], analogRead(accelChannel[ZAXIS]) - accelZero[ZAXIS]) * 57.2957795;
    flightAngle[PITCH] = atan2(analogRead(accelChannel[PITCH]) - accelZero[PITCH], analogRead(accelChannel[ZAXIS]) - accelZero[ZAXIS]) * 57.2957795;
    filterTermRoll[2] = -(analogRead(gyroChannel[ROLL]) - gyroZero[ROLL]) / 29473.792 * 57.2957795;
    filterTermPitch[2] = -(analogRead(gyroChannel[PITCH]) - gyroZero[PITCH]) / 29473.792 * 57.2957795;
  #endif
  #ifdef KalmanFilter
    // These parameters need to be further optimized
    initGyro1DKalman(&rollFilter, 0.001, 0.003, 0.03);
    initGyro1DKalman(&pitchFilter, 0.001, 0.003, 0.03);
  #endif
}


#ifndef KalmanFilter
float filterData(float previousAngle, int gyroADC, float angle, float *filterTerm, float dt) {
  // Written by RoyLB at:
  // http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286
  static float filter;
  float accel, gyro;
  
  // For Sparkfun 5DOF IMU
  // accelerometerOutput = (N-512)/1024*(double)10.78; (rad)
  // gyroOutput = (N-512)/1024*(double)28.783; (rad/sec)
  accel = angle * 57.2957795;
  //gyro = (N-512)/1024 * (double) 28.783;
  gyro = (gyroADC / 1024) * aref / 0.002;
  
  ///////////////////////
  // constants or parameters:
  // timeConstant - bandwidth of filter (1/sec). Need to tune this to match sensor performance.
  // T - iteration rate of the filter (sec)
  ///////////////////////
  // variables:
  // int_x1 (filterTerm[0]) - input to the first integrator (deg/sec/sec)
  // int_x2 (filterTerm[1]) - input to the second integrator (deg/sec)
  // int_y1 (filterTerm[2]) - output of the first integrator (deg/sec). This needs to be saved each iteration
  //////////////////////
  // inputs:
  // gyro - gyro output (deg/sec)
  // accel - accelerometer input to filter (deg)
  // x_accel - accelerometer output in x-axis (g)
  // z_accel - accelerometer output in z-axis (g)
  // accel_ang - derived angle based on arctan(x_accel,z_accel), (deg)
  //////////////////////
  // outputs:
  // filter - complementary filter output (and output of second integrator), (deg)
  //            This also needs to be saved each iteration.
  ///////////////////////

  filterTerm[0] = (accel - previousAngle) * timeConstant * timeConstant;
  filterTerm[2] = (dt * filterTerm[0]) + filterTerm[2];
  filterTerm[1] = filterTerm[2] + (accel - previousAngle) * 2 * timeConstant + gyro;
  return (dt * filterTerm[1]) + previousAngle;
}
#endif

#ifdef KalmanFilter
// The Kalman filter implementation is directly taken from the work
// of Tom Pycke at: http://tom.pycke.be/mav/90/sparkfuns-5dof
void initGyro1DKalman(struct Gyro1DKalman *filterdata, float Q_angle, float Q_gyro, float R_angle) {
	filterdata->Q_angle = Q_angle;
	filterdata->Q_gyro  = Q_gyro;
	filterdata->R_angle = R_angle;
}

void predictKalman(struct Gyro1DKalman *filterdata, const float dotAngle, const float dt) {
	filterdata->x_angle += dt * (dotAngle - filterdata->x_bias);
	filterdata->P_00 +=  - dt * (filterdata->P_10 + filterdata->P_01) + filterdata->Q_angle * dt;
	filterdata->P_01 +=  - dt * filterdata->P_11;
	filterdata->P_10 +=  - dt * filterdata->P_11;
	filterdata->P_11 +=  + filterdata->Q_gyro * dt;
}

float updateKalman(struct Gyro1DKalman *filterdata, const float angle_m) {
	const float y = angle_m - filterdata->x_angle;
	const float S = filterdata->P_00 + filterdata->R_angle;
	const float K_0 = filterdata->P_00 / S;
	const float K_1 = filterdata->P_10 / S;
	
	filterdata->x_angle +=  K_0 * y;
	filterdata->x_bias  +=  K_1 * y;
	filterdata->P_00 -= K_0 * filterdata->P_00;
	filterdata->P_01 -= K_0 * filterdata->P_01;
	filterdata->P_10 -= K_1 * filterdata->P_00;
	filterdata->P_11 -= K_1 * filterdata->P_01;

	return filterdata->x_angle;
}
#endif

int smooth(int currentData, int previousData, float smoothFactor) {
  return (previousData * (1 - smoothFactor) + (currentData * smoothFactor));
}

// fast way to approximate atan2
/*uns8 Arctan(uns8 niprop) {
	if( niprop >= 16 )
		return((uns8)45);
	skip(niprop);     /* 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15 */
/*	#pragma return[] =   0,  2,  6,  9, 13, 16, 19, 23, 26, 28, 31, 34, 36, 38, 41, 43
}*/
/*
  AeroQuad v1.5 - Novmeber 2009
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

#include "Motors.h"

void configureMotors() {
  #ifdef ServoControl
    frontMotor.attach(FRONTMOTORPIN);
    rearMotor.attach(REARMOTORPIN);
    rightMotor.attach(RIGHTMOTORPIN);
    leftMotor.attach(LEFTMOTORPIN);
  #endif
  #ifdef AnalogWrite
    analogWrite(FRONTMOTORPIN, 124);		
    analogWrite(REARMOTORPIN, 124);		
    analogWrite(RIGHTMOTORPIN, 124);		
    analogWrite(LEFTMOTORPIN, 124);		
  #endif
}

void commandMotors() {
  #ifdef ServoControl
    frontMotor.write(motorCommand[FRONT]);
    rearMotor.write(motorCommand[REAR]);
    rightMotor.write(motorCommand[RIGHT]);
    leftMotor.write(motorCommand[LEFT]);
  #endif
  #ifdef AnalogWrite		
    analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * mMotorCommand) + bMotorCommand);		
    analogWrite(REARMOTORPIN, (motorCommand[REAR] * mMotorCommand) + bMotorCommand);		
    analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);		
    analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * mMotorCommand) + bMotorCommand);		
  #endif  
}

// Sends commands to all motors
void commandAllMotors(int motorCommand) {
  #ifdef ServoControl
    frontMotor.write(motorCommand);
    rearMotor.write(motorCommand);
    rightMotor.write(motorCommand);
    leftMotor.write(motorCommand);
  #endif
  #ifdef AnalogWrite		
    analogWrite(FRONTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    analogWrite(REARMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    analogWrite(RIGHTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    analogWrite(LEFTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
  #endif
}

void pulseMotors(byte quantity) {
  for (byte i = 0; i < quantity; i++) {      
    commandAllMotors(MINCOMMAND + 100);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}
/*
  AeroQuad v1.5 - Novmeber 2009
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

#include "PID.h"

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters)
{
  float error;
  float dTerm;

  error = targetPosition - currentPosition;
  
  PIDparameters->integratedError += error;
  if (PIDparameters->integratedError < -windupGuard) PIDparameters->integratedError = -windupGuard;
  else if (PIDparameters->integratedError > windupGuard) PIDparameters->integratedError = windupGuard;
  
  dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition);
  PIDparameters->lastPosition = currentPosition;
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
}

void zeroIntegralError() {
  for (axis = ROLL; axis < LASTLEVELAXIS; axis++)
    PID[axis].integratedError = 0;
}
/*
  AeroQuad v1.5 - Novmeber 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
  
  Interrupt based method inspired by Dror Caspi
  http://www.rcgroups.com/forums/showpost.php?p=12356667&postcount=1639
  
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

#include "Receiver.h"
#include <ServoDecode.h>
char * stateStrings[] = {
  "NOT_SYNCHED", "ACQUIRING", "READY", "in Failsafe"};

#ifndef Mega_AQ1x
void init_servodecode()
{
  //Serial.begin(38400);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  ServoDecode.begin();
  ServoDecode.setFailsafe(3,1234); // set channel 3 failsafe pulse  width
}
// INIT Arduino Standard
#endif

#ifdef Mega_AQ1x
// INIT MEGA_AQ1x
#endif

// Configure each receiver pin for PCINT
void configureReceiver() {
  #ifndef Mega_AQ1x
  // pinMode(THROTTLEPIN, INPUT);
  // pinMode(ROLLPIN, INPUT);
  // pinMode(PITCHPIN, INPUT);
  // pinMode(YAWPIN, INPUT);
  // pinMode(MODEPIN, INPUT);
  // pinMode(AUXPIN, INPUT);
  init_servodecode();
  for (channel = ROLL; channel < LASTCHANNEL; channel++)
      pinData[receiverChannel[channel]].edge == FALLING_EDGE;

  // for (channel = ROLL; channel < LASTCHANNEL; channel++) {
  //   attachPinChangeInterrupt(receiverChannel[channel]);
  //   pinData[receiverChannel[channel]].edge == FALLING_EDGE;
  //}
  #endif
  
  #ifdef Mega_AQ1x
  initializeMegaPcInt2();
  for (channel = ROLL; channel < LASTCHANNEL; channel++)
    pinData[receiverChannel[channel]].edge == FALLING_EDGE;
  #endif
}

// Calculate PWM pulse width of receiver data
// If invalid PWM measured, use last known good time
unsigned int readReceiver(byte receiverPin) {
  
int pulsewidth;
uint16_t data;

  // print the decoder state
  if( ServoDecode.getState()!= READY_state) {
    //		Serial.print("The decoder is ");
    //		Serial.println(stateStrings[ServoDecode.getState()]);
    //for ( int i =0; i <=LASTCHANNEL; i++ )
    //		{ // print the status of the first four channels
    //	Serial.print("Cx"); // if you see this, the decoder does not have a valid signal
    //	Serial.print(i);
    //	Serial.print("= ");
    //	pulsewidth = ServoDecode.GetChannelPulseWidth(i);
    //	Serial.print(pulsewidth);
    //	Serial.print("  ");
    //	}
    //Serial.println("");
  }
  else {
    // decoder is ready, print the channel pulse widths
    // for ( int i =1; i <=LASTCHANNEL; i++ ){ // print the status of the first four channels
    //	Serial.print("Ch");
    //	Serial.print(i);
    //	Serial.print("= ");
    //	pulsewidth = ServoDecode.GetChannelPulseWidth(i);
    //	Serial.print(pulsewidth);
    //	Serial.print("  ");
    //}
    // Serial.println("");
   
    data=ServoDecode.GetChannelPulseWidth((int)receiverPin);
    pinData[receiverPin].lastGoodWidth=data; 
    pinData[receiverPin].edge=RISING_EDGE;
  
    //Serial.print("Ch");
    //Serial.print((int)receiverPin);
    //Serial.print("=");
    //Serial.print(data);
    //Serial.println("");
  //uint8_t oldSREG;
    
  //oldSREG = SREG;
  //cli();
  //data = pinData[receiverPin].lastGoodWidth;
  //SREG = oldSREG;  
   
  digitalWrite(12,LOW);
  digitalWrite(13,LOW);
  }

     return data;
}
/*
  AeroQuad v1.5 - Novmeber 2009
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

#include "Sensors.h"

int findMode(int *data, int arraySize) {
  // The mode of a set of numbers is the value that occurs most frequently
  boolean done = 0;
  byte i;
  int temp, maxData, frequency, maxFrequency;
  
  // Sorts numbers from lowest to highest
  while (done != 1) {        
    done = 1;
    for (i=0; i<(arraySize-1); i++) {
      if (data[i] > data[i+1]) {     // numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        done = 0;
      }
    }
  }
  
  temp = 0;
  frequency = 0;
  maxFrequency = 0;
  
  // Count number of times a value occurs in sorted array
  for (i=0; i<arraySize; i++) {
    if (data[i] > temp) {
      frequency = 0;
      temp = data[i];
      frequency++;
    } else if (data[i] == temp) frequency++;
    if (frequency > maxFrequency) {
      maxFrequency = frequency;
      maxData = data[i];
    }
  }
  return maxData;
}

// Allows user to zero gyros on command
void zeroGyros() {
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    for (int i=0; i<FINDZERO; i++) findZero[i] = analogRead(gyroChannel[axis]);
    gyroZero[axis] = findMode(findZero, FINDZERO);
  }
  writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
  writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
  writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
}

void autoZeroGyros() {
  digitalWrite(AZPIN, HIGH);
  delayMicroseconds(750);
  digitalWrite(AZPIN, LOW);
  delay(8);
}

// Allows user to zero accelerometers on command
void zeroAccelerometers() {
  for (axis = ROLL; axis < YAW; axis++) {
    for (int i=0; i<FINDZERO; i++) findZero[i] = analogRead(accelChannel[axis]);
    accelZero[axis] = findMode(findZero, FINDZERO);
  }
  accelZero[ZAXIS] = ZMAX - ((ZMAX - ZMIN)/2);
  writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
  writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
  writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
}

// Works faster and is smaller than the constrain() function
int limitRange(int data, int minLimit, int maxLimit) {
  if (data < minLimit) return minLimit;
  else if (data > maxLimit) return maxLimit;
  else return data;
}

float arctan2(float y, float x) {
  // Taken from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
   float coeff_1 = PI/4;
   float coeff_2 = 3*coeff_1;
   float abs_y = abs(y)+1e-10;      // kludge to prevent 0/0 condition
   float r, angle;
   
   if (x >= 0) {
     r = (x - abs_y) / (x + abs_y);
     angle = coeff_1 - coeff_1 * r;
   }
   else {
     r = (x + abs_y) / (abs_y - x);
     angle = coeff_2 - coeff_1 * r;
   }
   if (y < 0)
     return(-angle);     // negate if in quad III or IV
   else
     return(angle);
}

/*
  AeroQuad v1.5 - Novmeber 2009
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

void readSerialCommand() {
  // Check for serial message
  if (Serial.available()) {
    digitalWrite(LEDPIN, LOW);
    queryType = Serial.read();
    switch (queryType) {
    case 'A': // Receive roll and pitch gyro PID
      PID[ROLL].P = readFloatSerial();
      PID[ROLL].I = readFloatSerial();
      PID[ROLL].D = readFloatSerial();
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
      PID[PITCH].P = readFloatSerial();
      PID[PITCH].I = readFloatSerial();
      PID[PITCH].D = readFloatSerial();
      PID[PITCH].lastPosition = 0;
      PID[PITCH].integratedError = 0;
      break;
    case 'C': // Receive yaw PID
      PID[YAW].P = readFloatSerial();
      PID[YAW].I = readFloatSerial();
      PID[YAW].D = readFloatSerial();
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
      PID[HEADING].P = readFloatSerial();
      PID[HEADING].I = readFloatSerial();
      PID[HEADING].D = readFloatSerial();
      PID[HEADING].lastPosition = 0;
      PID[HEADING].integratedError = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      PID[LEVELROLL].P = readFloatSerial();
      PID[LEVELROLL].I = readFloatSerial() / 100;
      PID[LEVELROLL].D = readFloatSerial();
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;
      PID[LEVELPITCH].P = readFloatSerial();
      PID[LEVELPITCH].I = readFloatSerial() / 100;
      PID[LEVELPITCH].D = readFloatSerial();
      PID[LEVELPITCH].lastPosition = 0;
      PID[LEVELPITCH].integratedError = 0;
      break;
    case 'G': // Receive auto level configuration
      levelLimit = readFloatSerial();
      levelOff = readFloatSerial();
      break;
    case 'I': // Receive flight control configuration
      windupGuard = readFloatSerial();
      xmitFactor = readFloatSerial();
      break;
    case 'K': // Receive data filtering values
      smoothFactor[GYRO] = readFloatSerial();
      smoothFactor[ACCEL] = readFloatSerial();
      timeConstant = readFloatSerial();
      break;
    case 'M': // Receive transmitter smoothing values
      smoothTransmitter[ROLL] = readFloatSerial();
      smoothTransmitter[PITCH] = readFloatSerial();
      smoothTransmitter[YAW] = readFloatSerial();
      smoothTransmitter[THROTTLE] = readFloatSerial();
      smoothTransmitter[MODE] = readFloatSerial();
      smoothTransmitter[AUX] = readFloatSerial();
      break;
    case 'O': // Receive transmitter calibration values
      mTransmitter[ROLL] = readFloatSerial();
      bTransmitter[ROLL] = readFloatSerial();
      mTransmitter[PITCH] = readFloatSerial();
      bTransmitter[PITCH] = readFloatSerial();
      mTransmitter[YAW] = readFloatSerial();
      bTransmitter[YAW] = readFloatSerial();
      mTransmitter[THROTTLE] = readFloatSerial();
      bTransmitter[THROTTLE] = readFloatSerial();
      mTransmitter[MODE] = readFloatSerial();
      bTransmitter[MODE] = readFloatSerial();
      mTransmitter[AUX] = readFloatSerial();
      bTransmitter[AUX] = readFloatSerial();
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
      testCommand = readFloatSerial();
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
        remoteCommand[motor] = readFloatSerial();
      break;
    case 'a': // Enable/disable fast data transfer of sensor data
      queryType = 'X'; // Stop any other telemetry streaming
      if (readFloatSerial() == 1)
        fastTransfer = ON;
      else
        fastTransfer = OFF;
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

// Used to read floating point values from the serial port
float readFloatSerial() {
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
  }  while ((data[limitRange(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}
/*
  AeroQuad v1.5 - Novmeber 2009
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

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case 'B': // Send roll and pitch gyro PID values
    Serial.print(PID[ROLL].P);
    comma();
    Serial.print(PID[ROLL].I);
    comma();
    Serial.print(PID[ROLL].D);
    comma();
    Serial.print(PID[PITCH].P);
    comma();
    Serial.print(PID[PITCH].I);
    comma();
    Serial.println(PID[PITCH].D);
    queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    Serial.print(PID[YAW].P);
    comma();
    Serial.print(PID[YAW].I);
    comma();
    Serial.print(PID[YAW].D);
    comma();
    Serial.print(PID[HEADING].P);
    comma();
    Serial.print(PID[HEADING].I);
    comma();
    Serial.println(PID[HEADING].D);
    queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    Serial.print(PID[LEVELROLL].P);
    comma();
    Serial.print(PID[LEVELROLL].I * 100);
    comma();
    Serial.print(PID[LEVELROLL].D);
    comma();
    Serial.print(PID[LEVELPITCH].P);
    comma();
    Serial.print(PID[LEVELPITCH].I * 100);
    comma();
    Serial.println(PID[LEVELPITCH].D);
    queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    Serial.print(levelLimit);
    comma();
    Serial.println(levelOff);
    queryType = 'X';
    break;
  case 'J': // Send flight control configuration values
    Serial.print(windupGuard);
    comma();
    Serial.println(xmitFactor);
    queryType = 'X';
    break;
  case 'L': // Send data filtering values
    Serial.print(smoothFactor[GYRO]);
    comma();
    Serial.print(smoothFactor[ACCEL]);
    comma();
    Serial.println(timeConstant);
    queryType = 'X';
    break;
  case 'N': // Send motor smoothing values
    for (axis = ROLL; axis < AUX; axis++) {
      Serial.print(smoothTransmitter[axis]);
      comma();
    }
    Serial.println(smoothTransmitter[AUX]);
    queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (axis = ROLL; axis < AUX; axis++) {
      Serial.print(mTransmitter[axis]);
      comma();
      Serial.print(bTransmitter[axis]);
      comma();
    }
    Serial.print(mTransmitter[AUX]);
    comma();
    Serial.println(bTransmitter[AUX]);
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyroADC[axis]);
      comma();
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(accelADC[axis]);
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      comma();
    }
    Serial.print(flightAngle[ROLL]);
    comma();
    Serial.print(flightAngle[PITCH]);
    Serial.println();
    break;
  case 'R': // Send raw sensor data
    Serial.print(analogRead(ROLLRATEPIN));
    comma();
    Serial.print(analogRead(PITCHRATEPIN));
    comma();
    Serial.print(analogRead(YAWRATEPIN));
    comma();
    Serial.print(analogRead(ROLLACCELPIN));
    comma();
    Serial.print(analogRead(PITCHACCELPIN));
    comma();
    Serial.println(analogRead(ZACCELPIN));
    break;
  case 'S': // Send all flight data
    Serial.print(deltaTime);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyroData[axis]);
      comma();
    }
    Serial.print(transmitterCommand[THROTTLE]);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(motorAxisCommand[axis]);
      comma();
    }
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      Serial.print(motorCommand[motor]);
      comma();
    }
     Serial.print(armed, BIN);
    comma();
    #ifdef AutoLevel
      Serial.println(transmitterCommandSmooth[MODE]);
    #endif
    #ifndef AutoLevel
      Serial.println(1000);
    #endif
    break;
   case 'T': // Send processed transmitter values
    Serial.print(xmitFactor);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(transmitterCommand[axis]);
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      comma();
    }
    Serial.print(motorAxisCommand[ROLL]);
    comma();
    Serial.print(motorAxisCommand[PITCH]);
    comma();
    Serial.println(motorAxisCommand[YAW]);
    break;
  case 'U': // Send smoothed receiver values
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(transmitterCommandSmooth[channel]);
      comma();
    }
    Serial.println(transmitterCommandSmooth[AUX]);
    break;
  case 'V': // Send receiver status
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(receiverData[channel]);
      comma();
    }
    Serial.println(receiverData[AUX]);
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    Serial.print(transmitterCommand[YAW]);
    comma();
    Serial.print(headingHold);
    comma();
    Serial.print(heading);
    comma();
    Serial.println(currentHeading);
    break;
  case '6': // Report remote commands
    for (motor = FRONT; motor < LEFT; motor++) {
      Serial.print(remoteCommand[motor]);
      comma();
    }
    Serial.println(remoteCommand[LEFT]);
    break;
  case '!': // Send flight software version
    Serial.println("1.5");
    queryType = 'X';
    break;
  }
}

void comma() {
  Serial.print(',');
}

void printInt(int data) {
  byte msb, lsb;
  
  msb = data >> 8;
  lsb = data & 0xff;
  
  Serial.print(msb, BYTE);
  Serial.print(lsb, BYTE);
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

