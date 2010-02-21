#include <WProgram.h>
#include <Wire.h>

#include "LanguageExtensions.h"
#include "HardwareComponent.h"

#include "SerialComs.h"
SerialComs serialcoms;

#include "Receiver.h"
Receiver receiver;

#include "GPS.h"
GPS gps;

#include "IMU.h"
IMU imu;

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
	SPIMaster::getInstance()->begin();
	
	//Assign the 1st and 3rd serial ports for serial telemetry
	serialcoms.assignSerialPort(&Serial, true);		//Use for any debugging output
	serialcoms.assignSerialPort(&Serial2);
	
	//Assign the 2nd serial port for the GPS
	gps.assignSerialPort(&Serial1);
	
	//Run serial telemetry at 10hz offset 50ms
	serialcoms.initialize(100, 50);

	//Run the GPS at 5hz offset 25ms
	gps.initialize(200, 25);
	//gps.disable();
	
	//Run the IMU at 50hz
	imu.initialize(20,0);
	imu.setHardwareType(IMU::HardwareTypeRazor);
	imu.setFilterType(IMU::FilterTypeSimplifiedKalman);
	imu.calibrateZero();
	imu.disable();
	
	//Run the sensors at 5hz offset 25ms
	sensors.initialize(200,25);
	sensors.setPressorSensorType(Sensors::PressureSensorSPI);
	sensors.setHeightSensorType(Sensors::HeightSensorAnalogIn);
	//sensors.disable();
	
	//Run the receiver at 50hz
	receiver.initialize(20,0);
	receiver.setHardwareType(Receiver::HardwareTypeI2C);
	//receiver.setHardwareType(Receiver::HardwareTypeFake);
	//receiver.disable();
	
	//Run flight control at 50hz
	flightcontrol.initialize(20,0);
	flightcontrol.enableAutoLevel();
	flightcontrol.enableHeadingHold();
	//flightcontrol.disable();
	
	//Run the camera at 10hz offset 50ms
	camera.initialize(100, 50);
	//camera.disable();
	
	//Run the motors at 50hz
	motors.initialize(20,0);
	motors.setHardwareType(Motors::HardwareTypeOnboard);
	motors.setOrientationType(MotorOrientationType::QuadPlusMotor);
	//motors.disable();
	
	//Run the LEDs at 5hz offset 75ms
	led.initialize(200, 75);
	led.setPatternType(LED::LEDPatternInsideOut);
	//led.disable();
	
	pinMode(22,OUTPUT);
}


static unsigned long currentTime = 0;
static unsigned long previousTime = currentTime;
static unsigned int deltaTime = 0;
void loop()
{
	currentTime = millis();
	deltaTime = currentTime - previousTime;
	previousTime - currentTime;
	
	//for the purposes of timing the main loop we toggle a digital pin high and then low at the end
	digitalWrite(22, HIGH);
	
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
	
	digitalWrite(22, LOW);
}