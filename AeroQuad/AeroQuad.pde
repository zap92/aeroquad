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


#include <EEPROM.h>
#include <stdlib.h>
#include <math.h>
#include "WProgram.h"

#define uint8_t byte
#define byte uint8_t
#define LEDPIN 13
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define LASTCHANNEL 6
#define LEVELROLL 3
#define LEVELPITCH 4
#define LASTLEVELAXIS 5
#define HEADING 5
#define ZAXIS 2
#define LASTAXIS 3
#define ON 1
#define OFF 0

///// Includes for classes to use /////
#include "Eeprom.h"
Eeprom eeprom;

#include "Sensors.h"
Sensors sensors;

#include "FlightCommand.h"
FlightCommand_Duemilanove flightCommand;
//Receiver_Mega receiver;
SIGNAL(PCINT0_vect) {flightCommand.measurePulseWidthISR(0);}
SIGNAL(PCINT1_vect) {flightCommand.measurePulseWidthISR(1);}
SIGNAL(PCINT2_vect) {flightCommand.measurePulseWidthISR(2);}

#include "Motors.h"
Motors_PWM motors;
//Motors_I2C motors;

#include "FlightControl.h"
FlightControl flightControl;

#include "SerialComs.h"
SerialComs serialcoms;

#include "Blinkie.h"
Blinkie blinkie;
////////////////////////////////////////



static unsigned long currentTime = 0;

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  sensors.initialize(2, 0);// Setup and calibrate sensors, read every 2ms (500Hz)
  sensors.setGyroInvert(ROLL, ON);
  sensors.setGyroInvert(PITCH, ON);
  sensors.setGyroInvert(YAW, ON);
  sensors.setAccelInvert(ROLL, OFF);
  sensors.setAccelInvert(PITCH, OFF);
  sensors.setAccelInvert(YAW, OFF);
  
  flightControl.initialize(2,0); // Calculate motor commands every 2ms (500Hz) after reading sensors
  flightControl.setAutoLevel(OFF);
  flightControl.setHeadingHold(OFF);
  
  flightCommand.initialize(100,25); // Setup receiver pins for pin change interrupts and read every 100ms (10Hz) starting

  motors.initialize(2,1); // Configure motors and command motors every 2ms (500Hz), but offset by 1 ms

  serialcoms.assignSerialPort(&Serial); // Configure the serial port 1
  serialcoms.initialize(100, 50); // Read commands/telemetry every 100ms (10Hz) starting
  
  // Use this class to learn how SubSystem works
  blinkie.initialize(1000,0); // Start blinking the LED every 1000ms (1Hz)
}

// ************************************************************
// ******************** Main AeroQuad Loop ********************
// ************************************************************
void loop () {
  currentTime = millis();
  
  // Read R/C receiver and execute pilot commands at 100Hz rate
  flightCommand.process(currentTime); 
  
  // Measure sensor output at 500Hz rate
  sensors.process(currentTime);
  
  // Process sensor data and generate motor commands at 500Hz rate
  flightControl.process(currentTime);
  
  // Command motors at 500Hz rate
  motors.process(currentTime);
  
  // Process serial command and telemetry at 10Hz rate
  serialcoms.process(currentTime); 
}
