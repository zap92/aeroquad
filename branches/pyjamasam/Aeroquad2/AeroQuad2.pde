#include <WProgram.h>
#include <Wire.h>

#define VERSION "0.1"

#include <APM_RC.h>
#include <APM_ADC.h>

#include "LanguageExtensions.h"
#include "HardwareComponent.h"
#include "PID.h"

#include "SerialComs.h"

#include "LED.h"
LED led;

#include "Settings.h"
Settings settings;

#include "Receiver.h"
Receiver receiver;

#include "GPS.h"
GPS gps;

#include "Sensors.h"
Sensors sensors;

#include "IMU.h"
IMU imu;


/*#include "Navigation.h"
Navigation navigation;
*/

#include "FlightControl.h"
FlightControl flightcontrol;

#import "Motors.h"
Motors motors;

#import "Camera.h"
Camera camera;

void setup()
{	
	APM_ADC.Init();
	APM_RC.Init();
	
	{
		//Assign the 1st and 3rd serial ports for serial telemetry
		serialcoms.assignSerialPort(&Serial, true);				//Use USB serial for any debugging output
		serialcoms.assignSerialPort(&Serial3);					//Telemetry output to Xbee
		
		//Run serial telemetry at 10hz offset 50ms (100ms cycle time)
		serialcoms.initialize(100, 50);
	}
	
	{
		//Prepare the settings system for use
		settings.initialize();
	}
	
	{		
		receiver.setHardwareType(Receiver::HardwareTypeAPM);
		
		//Run the receiver at 50hz (20ms cycle time)
		receiver.initialize(20,0);
		receiver.calibrate();
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
		//flightcontrol.setFlightControlType(FlightControl::FlightControlTypeAcrobatic);
		flightcontrol.setFlightControlType(FlightControl::FlightControlTypeStable);
		//flightcontrol.setFlightControlType(FlightControl::FlightControlTypeAutonomous);
		
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
		//motors.setHardwareType(Motors::HardwareTypeDebugSerial);
		//motors.setHardwareType(Motors::HardwareTypeAPM);
		
		//Run the motors at 50hz
		motors.initialize(20,0);
	}
	
	{
		//Run the camera at 50hz offset 50ms (cycleTime)
		camera.initialize(20, 50);
	}
	
	{		
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
		receiver.process(currentTime);
		gps.process(currentTime);
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
		//motors.process(currentTime);
		camera.process(currentTime);
	}
	
	//process accessory sub systems
	led.process(currentTime);
	serialcoms.process(currentTime);
}