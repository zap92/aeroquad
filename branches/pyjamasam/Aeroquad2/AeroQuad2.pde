#include <WProgram.h>

#define PROGRAMNAME "AQ+"
#define VERSION "0.1"

#include <APM_RC.h>
#include <APM_ADC.h>
#include <Wire.h>

#include "SerialComs.h"
#include "LED.h"
#include "Settings.h"

#include "Receiver.h"
#include "GPS.h"
#include "Sensors.h"

#include "IMU.h"

#include "FlightControl.h"

#import "Camera.h"
#import "Motors.h"

/**

#include "FlightControl.h"
FlightControl flightcontrol;

*/

const ArduinoShellCallback::callbackReturn _performanceStatistics(ArduinoShell &shell, const int argc, const char* argv[]);

void setup()
{	
	APM_ADC.Init();
	APM_RC.Init();
	
	{
		Serial.begin(115200);
		
		serialcoms.setProgramName(PROGRAMNAME);
		serialcoms.setProgramVersion(VERSION);
		
		//Run serial telemetry at 40hz offset 50ms (25ms cycle time)
		serialcoms.initialize(25, 50);
	}
	
	{
		//Prepare the settings system for use
		settings.initialize();
	}
	
	{		
		receiver.setHardwareType(Receiver::HardwareTypeAPM);
		
		//Run the receiver at 50hz (20ms cycle time)
		receiver.initialize(20,0);		
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
		sensors.setPressorSensorType(Sensors::PressureSensorBMP085);
		sensors.setHeightSensorType(Sensors::MaxbotixSonar);
		//sensors.setPowerSensorType(Sensors::PowerSensorAnalogIn);
		
		//Run the sensors at 50hz (20ms cycle time)
		sensors.initialize(20,0);
	}
	
	{
		imu.setHardwareType(IMU::ArduPilotOilPan);
		imu.setSuplimentalYawSource(IMU::HMC5843);
		//imu.setFilterType(IMU::SimpleAccellOnly);			//Works fine.  In all its noisy glory
		//imu.setFilterType(IMU::Complimentry);				//Works fine, but not a very good filter
		//imu.setFilterType(IMU::DCM);								//Implimentation doesn't quite work yet.  Returns 0's all the time
		//imu.setFilterType(IMU::Quaternion);						//Implimentation not yet complete.  Not working
		imu.setFilterType(IMU::Kalman);								//Works fine.
		
		//Run the IMU at 100hz (10ms cycle time)
		imu.initialize(10,0);
		//imu.calibrateZero();
	}
	
	
	{		
		flightcontrol.setFlightControlType(FlightControl::FlightControlTypeAcrobatic);
		//flightcontrol.setFlightControlType(FlightControl::FlightControlTypeStable);
		//flightcontrol.setFlightControlType(FlightControl::FlightControlTypeAutonomous);
		
		//Run flight control at 100hz (10ms cycle time)
		flightcontrol.initialize(10,0);
	}
	
	/*{		
		//Run the navigation at 25hz (40ms cycle time)
		//navigation.initialize(40, 0);	
	}*/
	
	{
		//motors.setHardwareType(Motors::HardwareTypeDebugSerial);
		motors.setHardwareType(Motors::HardwareTypeAPM);		
		//motors.setOrientationType(Motors::QuadPlusMotor);					//Quad Motors in a + configuration
		motors.setOrientationType(Motors::QuadXMotor);							//Quad Motors in a x configuration
				
		//Run the motors at 100hz (10ms cycle time)
		motors.initialize(10,0);
	}
	
	{
		//Run the camera at 50hz offset 50ms (cycleTime)
		camera.initialize(20, 50);
	}
	
	{		
		//Run the LEDs at 10hz offset 75ms (100ms cycle time)
		led.initialize(100, 75);		
	}
	
	serialcoms.shell()->registerKeyword("performanceStatistics", "performanceStatistics", _performanceStatistics , true);
}


static unsigned long currentTime = 0;

static unsigned int totalRunTimeSampleCount = 0;
static float totalRunTimeTotal = 0;

void loop()
{
	currentTime = millis();
		
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
		motors.process(currentTime);
		camera.process(currentTime);
	}
	
	//process accessory sub systems
	led.process(currentTime);
	serialcoms.process(currentTime);
			
	totalRunTimeTotal += (millis() - currentTime);
	totalRunTimeSampleCount++;
		
	if (totalRunTimeSampleCount >= 1000)
	{		
		totalRunTimeTotal = (float) totalRunTimeTotal / (float)totalRunTimeSampleCount;
		totalRunTimeSampleCount = 1;
	}	
}

const ArduinoShellCallback::callbackReturn _performanceStatistics(ArduinoShell &shell, const int argc, const char* argv[])
{
	shell << (float)totalRunTimeTotal / (float)totalRunTimeSampleCount<< ","
			<< receiver.averageProcessingTime() << ","
			<< gps.averageProcessingTime() << ","
			<< sensors.averageProcessingTime() << ","
			<< imu.averageProcessingTime() << ","
			<< flightcontrol.averageProcessingTime() << ","
			//<< navigation.averageProcessingTime() << ","
			<< motors.averageProcessingTime() << ","
			<< camera.averageProcessingTime() << ","
			<< led.averageProcessingTime() << endl;
			
	return ArduinoShellCallback::Success;			
}

