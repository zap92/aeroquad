 /*
  AeroQuad v1.6 - March 2010
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
//#define XConfig

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

#include "AeroQuad.h"

#include "Eeprom.h"
Eeprom eeprom;

#include "Sensors.h"
Sensors sensors;

#include "Filter.h"
Filter filter;

#include "Attitude.h"
Attitude_CompFilter attitude;
//Attitude_KalmanFilter attitude;

#include "Receiver.h"
Receiver_Duemilanove receiver;
//Receiver_Mega receiver;

#include "Motors.h"
Motors_PWM motors;
//Motors_I2C motors;

#include "FlightControl.h"
FlightControl_PID flightcontrol;

#include "SerialComs.h"
SerialComs serialcoms;

#include "Blinkie.h"
Blinkie blinkie;

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  // Read user values from EEPROM
  eeprom.initialize();

  // Setup and calibrate sensors reading every 2ms (500Hz)
  sensors.initialize(2, 0);
  attitude.intialize(2,0);
  flightcontrol.initialize(2,0);
  
  // Configure motors and command motors every 2ms (500Hz)
  motors.initialize(2,1);

  // Setup receiver pins for pin change interrupts and read every 100ms (10Hz) starting
  receiver.intialize(100,25);

  // Complementary filter setup
  filter.initialize(eeprom.readFilterSetting());
  
  // Configure the serial port and read commands/telemetry every 100ms (10Hz) starting
  serialcoms.assignSerialPort(&Serial);
  serialcoms.initialize(100, 50);
  
  // Start blinking the LED every 1000ms (1Hz)
  blinkie.initialize(1000,0);
}

// ************************************************************
// ******************** Main AeroQuad Loop ********************
// ************************************************************
void loop () {
  currentTime = millis();
  
  // Measure sensor output at 500Hz rate
  sensors.process(currentTime);
  
  // Calculate flight angle to AeroQuad at 500Hz rate
  attitude.process(currentTime);
  
  // Process sensor data and generate motor commands at 500Hz rate
  flightcontrol.process(currentTime);
  
  // Read R/C receiver and execute pilot commands at 100Hz rate
  receiver.process(currentTime); 
  
  // Command motors at 500Hz rate
  motors.process(currentTime);
  
  // Process serial command and telemetry at 10Hz rate
  serialcoms.process(currentTime); 
}
