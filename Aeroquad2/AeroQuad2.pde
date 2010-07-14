#include <WProgram.h>
#include <Wire.h>

#define VERSION "0.1"

#include "LanguageExtensions.h"
#include "HardwareComponent.h"

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

#include "FlightControl.h"
FlightControl flightcontrol;

#import "Motors.h"
Motors motors;

#import "Camera.h"
Camera camera;*/

#include "LED.h"
LED led;

void setup()
{	
	{
		//Assign the 1st and 3rd serial ports for serial telemetry
		serialcoms.assignSerialPort(&Serial, true);		//Use for any debugging output
		serialcoms.assignSerialPort(&Serial3);			//Telemetry output to Xbee
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
		//imu.setFilterType(IMU::SimpleAccellOnly);		//Works fine.  In all its noisy glory
		//imu.setFilterType(IMU::Complimentry);			//Works fine, but not a very good filter
		//imu.setFilterType(IMU::DCM);					//Implimentation doesn't quite work yet.  Returns 0's all the time
		//imu.setFilterType(IMU::Quaternion);			//Implimentation not yet complete.  Not working
		imu.setFilterType(IMU::Kalman);					//Works fine.
		//Run the IMU at 50hz (10ms cycle time)
		imu.initialize(20,0);
		imu.calibrateZero();
	}
	
	{
		//Run the sensors at 50hz offset 25ms
		sensors.setPressorSensorType(Sensors::PressureSensorBMP085);
		//sensors.setHeightSensorType(Sensors::HeightSensorAnalogIn);
		//sensors.setPowerSensorType(Sensors::PowerSensorAnalogIn);
		sensors.initialize(20,0);
	}
	
	/*//Run the receiver at 50hz
	receiver.initialize(20,0);
	receiver.setHardwareType(Receiver::HardwareTypeI2C);
	//receiver.setHardwareType(Receiver::HardwareTypeFake);
	receiver.disable();
	
	//Run flight control at 50hz
	flightcontrol.initialize(20,0);
	flightcontrol.enableAutoLevel();
	flightcontrol.enableHeadingHold();
	flightcontrol.disable();

	//Run the camera at 10hz offset 50ms
	camera.initialize(100, 50);
	camera.disable();
	
	//Run the motors at 50hz
	motors.initialize(20,0);
	motors.setHardwareType(Motors::HardwareTypeOnboard);
	motors.setOrientationType(MotorOrientationType::QuadPlusMotor);
	motors.disable();
	*/
	
	//Run the LEDs at 5hz offset 75ms
	led.initialize(200, 75);
	led.setPatternType(LED::PatternSweep);
	
	//Setup Done.
	serialcoms.debugPrintln("!"VERSION);
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
	gps.process(currentTime);
	//receiver.process(currentTime);
	//sensors.process(currentTime);
	imu.process(currentTime);
	
	//process flight control
	//flightcontrol.process(currentTime);
	
	//process all the "outputs" from the system
	//motors.process(currentTime);
	//camera.process(currentTime);
	
	//process accessory sub systems
	led.process(currentTime);
	serialcoms.process(currentTime);
}