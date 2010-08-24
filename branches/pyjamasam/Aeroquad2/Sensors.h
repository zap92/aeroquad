#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "SubSystem.h"
#include "HardwareComponent.h"

#include "SerialComs.h"

class PowerSensor : public AnalogInHardwareComponent
{
	public:
		PowerSensor();
		
		virtual void initialize();				
		virtual void process(const unsigned long currentTime);
};

class PressureSensor : public HardwareComponent
{
	protected:
		long _currentPressure;				
		long _currentTemperature;
		
	public:
		PressureSensor();

		virtual void initialize();		
		virtual void process(const unsigned long currentTime);
		
		const long temperature();
		
		const long pressure();		
		const float altitude();
};

class BMP085PressureSensor : public PressureSensor
{
	private:
		byte _samplingMode;
		int _state;
		
		long _rawTemperature;
		long _rawPressure;				
		
		//Calibration data that will be read from the sensor as per the datasheet		
		int _calibrationAC1;
		int _calibrationAC2;
		int _calibrationAC3;
		unsigned int _calibrationAC4;
    	unsigned int _calibrationAC5;
    	unsigned int _calibrationAC6;  
		int _calibrationB1;
		int _calibrationB2;
		int _calibrationMB;
		int _calibrationMC;
		int _calibrationMD;
		
		void _initiateRawTemperatureReading();		
		const long _readRawTempature();		
		
		void _initiateRawPressureReading();		
		const int32_t _readRawPressure();		
		
		void _processReadings();
    		
	public:		
		BMP085PressureSensor();
		
		virtual void initialize();		
		virtual void process(unsigned long currentTime);
			
};

class HeightSensor : public AnalogInHardwareComponent
{
	public:
		HeightSensor();

		virtual void initialize();		
		virtual void process(const unsigned long currentTime);
		
		const float height();
};

class MaxbotixSonarHeightSensor : public HeightSensor
{
	public:		
		MaxbotixSonarHeightSensor();
		
		virtual void initialize();		
		virtual const int readRawValue(const unsigned int channel);
};

class Sensors : public SubSystem
{
	private:
		PressureSensor *_pressureSensor;
		PowerSensor *_powerSensor;
		HeightSensor *_heightSensor;

		
	public:
		typedef enum { PressureSensorNone = 0, PressureSensorBMP085} PressureSensorType;
		typedef enum { HeightSensorNone = 0, MaxbotixSonar } HeightSensorType;
		typedef enum { PowerSensorNone = 0 } PowerSensorType;
		
		Sensors();
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0);		
		virtual void process(const unsigned long currentTime);
		
		void setPressorSensorType(const PressureSensorType hardwareType);		
		void setPowerSensorType(const PowerSensorType hardwareType);		
		void setHeightSensorType(const HeightSensorType hardwareType);
				
		//accessors for sensor values
		const long temperature();		
		const long pressure();		
		const float altitudeUsingPressure();		
		const float altitudeUsingHeightSensor();		
};

extern Sensors sensors;

const ArduinoShellCallback::callbackReturn _monitorSensors(ArduinoShell &shell, const int argc, const char* argv[]);

#endif //#ifndef __SENSORS_H__