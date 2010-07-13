#include "SubSystem.h"
#include "HardwareComponent.h"




#define PRESSURE_MINIMUM 94190.0
#define PRESSURE_MAXIMUM 103100.0
#define PRESSURE_HEIGHT_MAXIMUM 610.0
#define PRESSURE_HEIGHT_MINIMUM -153.0
class PressureSensor : public HardwareComponent
{
	protected:
		float _currentHeight;
		float _currentPressure;
		
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
			_currentHeight = fconstrain(fmap(_currentPressure, PRESSURE_MINIMUM, PRESSURE_MAXIMUM, PRESSURE_HEIGHT_MAXIMUM, PRESSURE_HEIGHT_MINIMUM), PRESSURE_HEIGHT_MINIMUM, PRESSURE_HEIGHT_MAXIMUM);
			
			HardwareComponent::process(currentTime);	
		}
		
		const float getHeight()
		{
			return _currentHeight;
		}
};

/*#define BARO_CSB 47
#define BARO_DRDY 46
#define PRESSURE_MSB_REGISTER 0x1F   //Pressure 3 MSB
#define PRESSURE_LSB_REGISTER 0x20   //Pressure 16 LSB

class SPIPressureSensor : public PressureSensor, SPIDevice
{		
	private:
	
		const byte readByteFromRegister(const byte registerAddress)
		{
		    byte adjustedRegisterAddress = registerAddress << 2;
		    adjustedRegisterAddress &= B11111100; //Read command

			this->activate();			
		    this->sendByte(adjustedRegisterAddress);
			byte returnByte = this->readByte();	
			this->deactivate();
			
			return returnByte;
		}
		
		const int readIntFromRegister(const byte registerAddress)
		{
		    byte adjustedRegisterAddress = registerAddress << 2;
		    adjustedRegisterAddress &= B11111100; //Read command

			this->activate();	
		    this->sendByte(adjustedRegisterAddress);
		    byte in_byte1 = this->readByte();	  
		    byte in_byte2 = this->readByte();
			this->deactivate();		
		
			return (((in_byte1) << 8) | (in_byte2));
		}
		
		const byte sendByteToRegister(const byte registerAddress, const byte registerValue)
		{
		    byte adjustedRegisterAddress = registerAddress << 2;
		    adjustedRegisterAddress |= B00000010; //Write command

			this->activate();			
		    this->sendByte(adjustedRegisterAddress);
			byte returnByte = this->sendByte(registerValue);	
			this->deactivate();
			
			return returnByte;
		}
	
	public:		
		void initialize()
		{
			PressureSensor::initialize();
			SPIDevice::initialize(BARO_CSB);
			
			pinMode(BARO_DRDY, INPUT);
			
			this->sendByteToRegister(0x03, 0x0A);
		}
		
		void process(const unsigned long currentTime)
		{	
			unsigned long pressureMsb = this->readByteFromRegister(PRESSURE_MSB_REGISTER);
		  	pressureMsb &= B00000111;
	
		  	unsigned long pressureLsb = this->readIntFromRegister(PRESSURE_LSB_REGISTER);
				  
			_currentPressure = (((pressureMsb) << 16) | (pressureLsb));
		  	_currentPressure /= 4;
		
			PressureSensor::process(currentTime);
		}
};*/


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
			
			//convert from cm to meters
			_currentHeight = tmpf / 100;
					
			HeightSensor::process(currentTime);
		}
};

class Sensors : public SubSystem
{
	private:
		PressureSensor *_pressureSensor;
		HeightSensor *_heightSensor;
		
	public:
		
		typedef enum { PressureSensorNone = 0, PressureSensorSPI} PressureSensorType;
		typedef enum { HeightSensorNone = 0, HeightSensorAnalogIn} HeightSensorType;
		typedef enum { CompassNone = 0, CompassHMC5843} CompassType;
			
		Sensors() : SubSystem()
		{				
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
						
			//Read and process pressure reading
			if (_pressureSensor)
			{
				_pressureSensor->initialize();
			}
			
			//Read and process height reading
			if (_heightSensor)
			{
				_heightSensor->initialize();
			}
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
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
				
				//TODO
				//Read and process current and voltage
			}
		}
		
		void setPressorSensorType(const PressureSensorType hardwareType)
		{
			switch (hardwareType)
			{
				/*case PressureSensorSPI:
				{
					//_pressureSensor = new SPIPressureSensor();
					break;
				}*/
				
				default:
				{
					serialcoms.debugPrintln("ERROR: Unknown Pressure Sensor type selected.");
					break;
				}
			}
		}
		
		void setHeightSensorType(const HeightSensorType hardwareType)
		{
			switch (hardwareType)
			{
				/*case HeightSensorAnalogIn:
				{
					_heightSensor = new AnalogInHeightSensor();
					break;
				}*/
				
				default:
				{
					serialcoms.debugPrintln("ERROR: Unknown Height Sensor type selected.");
					break;
				}
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
			
			return 0.0;
		}
		
		const float heightFromUltrasonic()
		{
			if (_heightSensor)
			{
				return _heightSensor->getHeight();
			}
			
			return 0.0;
		}
		
		
		
		//Power sensors
		const float currentVoltage()
		{
			return 0.0f;
		}
		
		const float currentCurrentConsumptionRate()
		{
			return 0.0f;
		}
};