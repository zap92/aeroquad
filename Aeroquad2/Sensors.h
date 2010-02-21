#include "SubSystem.h"
#include "HardwareComponent.h"

class PressureSensor : public HardwareComponent
{
	protected:
		float _currentHeight;
		
	public:
		PressureSensor() : HardwareComponent()
		{
			_currentHeight = 0.0;
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			HardwareComponent::process(currentTime);	
		}
		
		const float getHeight()
		{
			return _currentHeight;
		}
};

#define BARO_CSB 47
#define PRESSURE_MSB_REGISTER 0x1F   //Pressure 3 MSB
#define PRESSURE_LSB_REGISTER 0x20   //Pressure 16 LSB

class SPIPressureSensor : public PressureSensor, SPIDevice
{
	private:
		unsigned long _referencePressure;
		
	public:		
		void initialize()
		{
			PressureSensor::initialize();
			SPIDevice::initialize(BARO_CSB);
			
			_referencePressure = 101330;
						
			//Switch the operation mode to continious high speed for the sensor
			this->sendByte(0x03, 0x09);
		}
		
		void process(const unsigned long currentTime)
		{	
			unsigned long pressureMsb = this->readByte(PRESSURE_MSB_REGISTER);
		  	pressureMsb &= B00000111;
		
		  	unsigned long pressureLsb = this->readInt(PRESSURE_LSB_REGISTER);
		  
			unsigned long pressure = (((pressureMsb) << 16) | (pressureLsb));
		  	pressure /= 4;
		
			//float altitude = (((pow(pressure/_referencePressure,0.19026)) - 1) * 288.15) / 0.00198122;
			//TODO convert the pressure reading to an altitude reading
		
			//byte revId = this->readByte(0x00); //Snag the revid
			//DEBUGSERIALPRINT(altitude);
			//DEBUGSERIALPRINTLN("");
			
			PressureSensor::process(currentTime);
		}
};


class HeightSensor : public HardwareComponent
{
	protected:
		float _currentHeight;
		
	public:
		HeightSensor() : HardwareComponent()
		{
			_currentHeight = 0.0;
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			HardwareComponent::process(currentTime);	
		}
		
		const float getHeight()
		{
			return _currentHeight;
		}
};

#define HEIGHTSENSOR_INPUTPIN 10
class AnalogInHeightSensor : public HeightSensor
{
	public:		
		void initialize()
		{
			HeightSensor::initialize();
			
			pinMode(HEIGHTSENSOR_INPUTPIN, INPUT);
		}
		
		void process(const unsigned long currentTime)
		{	
			static float tmpf;	        //temporary variable
			
			int rawAnalogValue = analogRead(HEIGHTSENSOR_INPUTPIN);
			tmpf = rawAnalogValue * HardwareComponent::getReferenceVoltage() / 1024.0f;  //voltage (mV)
		  	tmpf /= 3.2;    		//input sensitivity in mV/Unit
			
			//convert from CM to Meters
			tmpf /= 100;
			_currentHeight = tmpf;
					
			HeightSensor::process(currentTime);
		}
};





class Sensors : public SubSystem
{
	private:
		float _headingFromCompass;
		
		float _voltage;
		float _current;
		
		PressureSensor *_pressureSensor;
		HeightSensor *_heightSensor;
		
	public:
		
		typedef enum { PressureSensorNone = 0, PressureSensorSPI = 1} PressureSensorType;
		typedef enum { HeightSensorNone = 0, HeightSensorAnalogIn = 1} HeightSensorType;
			
		Sensors() : SubSystem()
		{	
			_headingFromCompass = 0;
			
			_voltage = 0;
			_current = 0;
			
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				//TODO
				//Read compass and process compass reading
				/*Wire.beginTransmission(0x21);
				Wire.send('A');
				Wire.endTransmission();
				Wire.requestFrom(0x21, 2); 
				int heading = Wire.receive() << 8;
				heading += Wire.receive();
				
				_headingFromCompass = heading / 10.0f;*/
				
				//Read and process pressure reading
				if (_pressureSensor)
				{
					_pressureSensor->process(currentTime);
				}
				
				//Read and process height reading
				if (_heightSensor)
				{
					_heightSensor->process(currentTime);
				}
				
				//Read and process current and voltage
			}
		}
		
		void setPressorSensorType(const PressureSensorType hardwareType)
		{
			switch (hardwareType)
			{
				case PressureSensorSPI:
				{
					_pressureSensor = new SPIPressureSensor();
					break;
				}
			}
			
			if (_pressureSensor)
			{
				_pressureSensor->initialize();
			}
		}
		
		void setHeightSensorType(const HeightSensorType hardwareType)
		{
			switch (hardwareType)
			{
				case HeightSensorAnalogIn:
				{
					_heightSensor = new AnalogInHeightSensor();
					break;
				}
			}
			
			if (_heightSensor)
			{
				_heightSensor->initialize();
			}
		}
		
		//accessors for sensor values
		
		//Altitude
		const float heightFromPressure()
		{
			if (_pressureSensor)
			{
				return _pressureSensor->getHeight();
			}
		}
		
		const float heightFromUltrasonic()
		{
			if (_heightSensor)
			{
				return _heightSensor->getHeight();
			}
		}
		
		//Heading
		const float headingFromCompass()
		{
			return _headingFromCompass;
		}
		
		//Power sensors
		const float currentVoltage()
		{
			return _voltage;
		}
		
		const float currentCurrentConsumptionRate()
		{
			return _current;
		}
};