#include <WProgram.h>
#include <Wire.h>

#define VERSION "0.1"

#include <APM_RC.h>

#include "LanguageExtensions.h"
#include "HardwareComponent.h"

#include "LED.h"
LED led;

#include "SerialComs.h"
SerialComs serialcoms;

#include "GPS.h"
GPS gps;

#include "IMU.h"
IMU imu;

#include "Sensors.h"
Sensors sensors;

/*
#include "Receiver.h"
Receiver receiver;

#include "Navigation.h"
Navigation navigation;
*/

#include "FlightControl.h"
FlightControl flightcontrol;

#import "Motors.h"
Motors motors;

/*#import "Camera.h"
Camera camera;*/

void setup()
{	
	{
		//Assign the 1st and 3rd serial ports for serial telemetry
		serialcoms.assignSerialPort(&Serial);				//Use for any debugging output
		serialcoms.assignSerialPort(&Serial3);				//Telemetry output to Xbee
		
		//Run serial telemetry at 10hz offset 50ms (100ms cycle time)
		serialcoms.initialize(100, 50);
	}
	
	{
		//Assign the 2nd serial port for the GPS
		gps.assignSerialPort(&Serial1);
		//gps.setHardwareType(GPS::HardwareTypeNMEA);
		//gps.setHardwareType(GPS::HardwareTypeUBOX);
		gps.setHardwareType(GPS::HardwareTypeMTK);
		
		//Run the GPS at 2hz offset 25ms (500ms cycle time)
		gps.initialize(500, 25);
	}

	{
		imu.setHardwareType(IMU::ArduPilotOilPan);
		imu.setSuplimentalYawSource(IMU::HMC5843);
		//imu.setFilterType(IMU::SimpleAccellOnly);			//Works fine.  In all its noisy glory
		//imu.setFilterType(IMU::Complimentry);				//Works fine, but not a very good filter
		//imu.setFilterType(IMU::DCM);						//Implimentation doesn't quite work yet.  Returns 0's all the time
		//imu.setFilterType(IMU::Quaternion);				//Implimentation not yet complete.  Not working
		imu.setFilterType(IMU::Kalman);						//Works fine.
		
		//Run the IMU at 50hz (20ms cycle time)
		imu.initialize(20,0);
		imu.calibrateZero();
	}
	
	{
		
		sensors.setPressorSensorType(Sensors::PressureSensorBMP085);
		sensors.setHeightSensorType(Sensors::MaxBotixMaxSonar);
		//sensors.setPowerSensorType(Sensors::PowerSensorAnalogIn);
		
		//Run the sensors at 50hz (20ms cycle time)
		sensors.initialize(20,0);
	}
	
	{
		//Run the receiver at 50hz
		//receiver.initialize(20,0);
		//receiver.setHardwareType(Receiver::HardwareTypeI2C);
		//receiver.setHardwareType(Receiver::HardwareTypeFake);
		//receiver.disable();
	}
	
	{
		//flightcontrol.enableAutoLevel();
		//flightcontrol.enableHeadingHold();
		
		//Run flight control at 50hz (20ms cycle time)
		flightcontrol.initialize(20,0);
	}
	
	{		
		//Run the navigation at 25hz (40ms cycle time)
		//navigation.initialize(40, 0);	
	}
	
	{
		motors.setOrientationType(Motors::QuadPlusMotor);			//Quad Motors in a + configuration
		//motors.setOrientationType(Motors::QuadXMotor);				//Quad Motors in a - configuration
		motors.setHardwareType(Motors::HardwareTypeDebugSerial);
		motors.setHardwareType(Motors::HardwareTypeAPM);
		
		//Run the motors at 50hz
		motors.initialize(20,0);
	}
	
	{
		//Run the camera at 10hz offset 50ms
		//camera.initialize(100, 50);
	}
	
	{
		led.setPatternType(LED::PatternSweep);
		
		//Run the LEDs at 10hz offset 75ms (100ms cycle time)
		led.initialize(100, 75);		
	}
	
	//Setup Done.
	serialcoms.print("!"VERSION);
	serialcoms.println();
}


static unsigned long currentTime = 0;
static unsigned long previousTime = currentTime;
static unsigned int deltaTime = 0;
void loop()
{
	currentTime = millis();
	deltaTime = currentTime - previousTime;
	previousTime - currentTime;
	
	//process all the "inputs" to the system
	{
		gps.process(currentTime);
		//receiver.process(currentTime);
		sensors.process(currentTime);
		imu.process(currentTime);
	}
	
	//process flight control
	{
		flightcontrol.process(currentTime);
		//navigation.process(currentTime);
	}
	
	//process all the "outputs" from the system
	{
		motors.process(currentTime);
		//camera.process(currentTime);
	}
	
	//process accessory sub systems
	led.process(currentTime);
	serialcoms.process(currentTime);
}