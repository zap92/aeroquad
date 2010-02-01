 /*
  AeroQuad v2.0 - January 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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
//#define plusConfig
#define XConfig

// Calibration At Start Up
//#define CalibrationAtStartup
#define GyroCalibrationAtStartup

// Camera Stabilization (experimental)
// Will move development to Arduino Mega (needs analogWrite support for additional pins)
//#define Camera

// Heading Hold (experimental)
// Currently uses yaw gyro which drifts over time, for Mega development will use magnetometer
//#define HeadingHold

// Auto Level (experimental)
//#define AutoLevel

// Sensor Filter
// The Kalman Filter implementation is here for comparison against the Complementary Filter
// To adjust the KF parameters, look at initGyro1DKalman() found inside ConfigureFilter() in Filter.pde
//#define KalmanFilter

// *************************************************************

#include <stdlib.h>
#include <math.h>
#include "AeroQuad.h"
#include "Filter.h"
#include "PID.h"
#include "Motors.h"

#include "Eeprom.h"
Eeprom eeprom;

#include "Sensors.h"
Sensors sensors;

#include "Receiver.h"
Receiver receiver;

#include "Motors.h"
Motors motors;

#include "SerialComs.h"
SerialComs serialcoms;

#include "GPS.h"
GPS gps;

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  Serial.begin(BAUD);
  analogReference(EXTERNAL); // Current external ref is connected to 3.3V
  pinMode(LEDPIN, OUTPUT);
  pinMode(11, INPUT);
  analogRead(11);
  
  // Read user values from EEPROM
  eeprom.initialize();

  // Setup and calibrate sensors
  sensors.initialize(2, 0);
  sensors.zeroGyros();
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Setup receiver pins for pin change interrupts
  receiver.intialize();

  // Configure motors
  motors.initialize();

  // Compass setup
  if (compassLoop == ON)
    configureCompass();
  
 
  // Camera stabilization setup
  #ifdef Camera
    rollCamera.attach(ROLLCAMERAPIN);
    pitchCamera.attach(PITCHCAMERAPIN);
  #endif
  
  // Complementary filter setup
  configureFilter(timeConstant);
  
  serialcoms.assignSerialPort(&Serial);
  serialcoms.assignSerialPort(&Serial1);
  serialcoms.initialize(100, 50);
  
  gps.assignSerialPort(&Serial2);
  gps.initialize(100, 75);
  
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
  
  sensors.process();  // Measure sensor output
  receiver.process(); // Read R/C receiver and execute pilot commands
  flightcontrol.process();
  motors.process()
  serialcoms.process(currentTime); // Process serial command and telemetry
  gps.process(currentTime); // Read GPS
  
  
// *************************************************************
// ******************* Camera Stailization *********************
// *************************************************************
#ifdef Camera // Development moved to Arduino Mega

  if ((currentTime > (cameraTime + CAMERALOOPTIME)) && (cameraLoop == ON)) { // 50Hz
    //rollCamera.write((mCamera * flightAngle[ROLL]) + bCamera);
    //pitchCamera.write((mCamera * flightAngle[PITCH]) + bCamera);
    rollCamera.write((int)flightAngle[ROLL]+90);
    pitchCamera.write((int)flightAngle[PITCH]+90);
    cameraTime = currentTime;
  }
  SoftwareServo::refresh();
  /*if ((currentTime > (rollCameraTime + rollCameraLoop)) && (cameraLoop == ON)) { // 50Hz
    Serial.print(rollState, DEC); Serial.print(" - "); Serial.print(currentTime); Serial.print(" - ");
    Serial.println(rollCameraLoop);
    if (rollState == HIGH) {
      rollCameraLoop = 20000;
      digitalWrite(ROLLCAMERAPIN, LOW);
      rollState = LOW;
    }
    else { // rollState = LOW
      rollCameraLoop = (mCamera * flightAngle[ROLL]) + bCamera;
      digitalWrite(ROLLCAMERAPIN, HIGH);
      rollState = HIGH;
    }
    rollCameraTime = currentTime;
  }*/  
  /*if ((currentTime > (pitchCameraTime + pitchCameraLoop)) && (cameraLoop == ON)) { // 50Hz
    if (pitchState == HIGH) {
      pitchCameraLoop = 20000;
      digitalWrite(PITCHCAMERAPIN, LOW);
      pitchState = LOW;
    }
    else { // rollState = LOW
      pitchCameraLoop = (mCamera * flightAngle[PITCH]) + bCamera;
      digitalWrite(PITCHCAMERAPIN, HIGH);
      pitchState = HIGH;
    }
    pitchCameraTime = currentTime;
  }*/  
#endif
////////////////////////
// End of camera loop //
////////////////////////
}
