#include <WProgram.h>
#include <Wire.h>

#include "LanguageExtensions.h"
#include "HardwareComponent.h"

#include "SerialComs.h"
SerialComs serialcoms;

#include "GPS.h"
GPS gps;

#include "IMU.h"
IMU imu;

#include "Receiver.h"
Receiver receiver;

#include "Sensors.h"
Sensors sensors;

#include "FlightControl.h"
FlightControl flightcontrol;

#import "Motors.h"
Motors motors;

#import "Camera.h"
Camera camera;

#include "LED.h"
LED led;

void setup()
{
	analogReference(EXTERNAL);
	HardwareComponent::setReferenceVoltage(2760.0);
	
	//Initialize I2C and SPI
	Wire.begin();
	//TODO: SPI
	
	
	//Assign the 1st and 3rd serial ports for serial telemetry
	serialcoms.assignSerialPort(&Serial, true);		//Use for any debugging output
	serialcoms.assignSerialPort(&Serial2);
	
	//Assign the 2nd serial port for the GPS
	gps.assignSerialPort(&Serial1);
	
	//Run serial telemetry at 10hz offset 50ms
	serialcoms.initialize(100, 50);

	//Run the GPS at 10hz offset 25ms
	gps.initialize(100, 25);
	
	//Run the IMU at 200hz
	imu.initialize(5,0);
	imu.setHardwareType(IMU::HardwareTypeRazor);
	imu.setFilterType(IMU::FilterTypeSimplifiedKalman);
	
	//Run the sensors at 50hz offset 25ms
	sensors.initialize(20,25);
	
	//Run the receiver at 50hz
	receiver.initialize(20,0);
	receiver.setHardwareType(Receiver::HardwareTypeI2C);
	//receiver.setHardwareType(Receiver::HardwareTypeFake);
	
	//Run flight control at 200hz
	flightcontrol.initialize(5,0);
	
	//Run the camera at 10hz offset 50ms
	camera.initialize(100, 50);
	
	//Run the motors at 200hz
	motors.initialize(5,0);
	motors.setHardwareType(Motors::HardwareTypeOnboard);
	motors.setOrientationType(MotorHardware::OrientationTypePlus4Motor);
	
	//Run the LEDs at 5hz offset 75ms
	led.initialize(200, 75);
	led.setPatternType(LED::LEDPatternInsideOut);
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
	imu.process(currentTime);
	gps.process(currentTime);
	receiver.process(currentTime);
	sensors.process(currentTime);
	
	//process flight control
	flightcontrol.process(currentTime);
	
	//process all the "outputs" from the system
	motors.process(currentTime);
	camera.process(currentTime);
	
	//process accessory sub systems
	led.process(currentTime);
	serialcoms.process(currentTime);
}