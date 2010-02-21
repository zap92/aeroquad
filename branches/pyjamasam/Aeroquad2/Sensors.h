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
				  
			unsigned long pressure = (((pressureMsb) << 16) | (pressureLsb));
		  	pressure /= 4;
		
			_currentHeight = fmap(pressure, 103100, 94190, -153, 610);  //Map the pressures between -153m and 610m
			DEBUGSERIALPRINT(_currentHeight);
			DEBUGSERIALPRINTLN("");
		
			
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
			
			//convert from cm to meters
			_currentHeight = tmpf / 100;
					
			HeightSensor::process(currentTime);
		}
};







class Compass : public HardwareComponent
{
	protected:
		typedef enum { AxisX = 0, AxisY = 1, AxisZ = 2} CompassAxis;
		float _currentHeading;
		
		float _process3dReading(const int xReading, const int yReading, const int zReading)
		{
			float currentRollAngle = imu.currentRollAngle();
			float currentPitchAngle = imu.currentPitchAngle();
			
			float CMx = (xReading * cos(currentPitchAngle)) + (yReading *sin(currentRollAngle) * sin(currentPitchAngle)) - (zReading * cos(currentRollAngle) * sin(currentPitchAngle));
			float CMy = (yReading * cos(currentRollAngle)) + (zReading * sin(currentRollAngle));

			float heading = abs(degrees(atan(CMy/CMx)));
			if (CMx >= 0 && CMy >= 0) {heading = 180 - heading;}
		    if (CMx >= 0 && CMy < 0) {heading = heading + 180;}
		    if (CMx < 0 && CMy < 0) {heading = 360 - heading;}
			
			return heading;
		}
	
	public:
		Compass() : HardwareComponent()
		{
			_currentHeading = 0.0;
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
	
		virtual void process(const unsigned long currentTime)
		{
			HardwareComponent::process(currentTime);	
		}
	
		const float getHeading()
		{
			return _currentHeading;
		}	
};

#define COMPASS_CSB 53
#define COMPASS_DRDY 48
#define COMPASS_RESET 49
#define RESET_WAIT_TIME 3
class MicroMag3Compass : public Compass, SPIDevice
{
	private:
		const int _readAxis(CompassAxis axis)
		{
			digitalWrite(COMPASS_RESET, HIGH);
			delayMicroseconds(RESET_WAIT_TIME);
			digitalWrite(COMPASS_RESET, LOW);

			// Send command byte
		  	// Description found on page 9 of MicroMag3 data sheet
		  	// First nibble defines speed/accuracy of measurement
		  	// Use 0x70 for the slowest/best accuracy, 0x10 for fastest/least accuracy
		  	// Last nibble defines axis (X = 0x01, Y = 0x02, Z - 0x03)
			switch (axis) 
			{
				case AxisX:
					this->sendByte(0x61);
					break;
				case AxisY:
					this->sendByte(0x62);
					break;
				case AxisZ:
					this->sendByte(0x63);
					break;
			}

			 // Wait for data to be ready, then read two bytes
			 while(digitalRead(COMPASS_DRDY) == LOW);  
			
			 byte in_byte1 = this->readByte();	  
			 byte in_byte2 = this->readByte();
			
			return (((in_byte1) << 8) | (in_byte2));
		}
		
	public:
		void initialize()
		{
			Compass::initialize();
			SPIDevice::initialize(COMPASS_CSB);
		
			pinMode(COMPASS_DRDY, INPUT);
			pinMode(COMPASS_RESET, OUTPUT);
		
		}
		
		void process(const unsigned long currentTime)
		{
			if (digitalRead(COMPASS_DRDY) == HIGH)
			{
				this->activate();
				int xAxisReading = this->_readAxis(AxisX);
				int yAxisReading = this->_readAxis(AxisY);
				int zAxisReading = this->_readAxis(AxisZ);
				this->deactivate();
				
				_currentHeading = this->_process3dReading(xAxisReading, yAxisReading, zAxisReading);
			}
			
			Compass::process(currentTime);
		}
		
};

#define COMPAS_I2C_ADDRESS 0x1E
class HMC5843Compass : public Compass
{
	public:
		void initialize()
		{
			Compass::initialize();
			
			//Set the compass to continous read mode
			Wire.beginTransmission(COMPAS_I2C_ADDRESS);
			Wire.send(0x02);
			Wire.send(0x00);
			Wire.endTransmission();
		}
		
		void process(const unsigned long currentTime)
		{
			Wire.requestFrom(COMPAS_I2C_ADDRESS, 6);
			 
			byte xAxisReadingHigh = Wire.receive();
			byte xAxisReadingLow = Wire.receive();
			int xAxisReading = (((xAxisReadingHigh) << 8) | (xAxisReadingLow));
			
			byte yAxisReadingHigh = Wire.receive();
			byte yAxisReadingLow = Wire.receive();
			int yAxisReading = (((xAxisReadingHigh) << 8) | (xAxisReadingLow));
		
			byte zAxisReadingHigh = Wire.receive();
			byte zAxisReadingLow = Wire.receive();
			int zAxisReading = (((xAxisReadingHigh) << 8) | (xAxisReadingLow));
			
			_currentHeading = this->_process3dReading(xAxisReading, yAxisReading, zAxisReading);
			
			//DEBUGSERIALPRINT(_currentHeading);
			//DEBUGSERIALPRINT(":");
			//DEBUGSERIALPRINT(_currentHeight);
			//DEBUGSERIALPRINTLN("");
			
			Compass::process(currentTime);
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
		Compass *_compass;
		
	public:
		
		typedef enum { PressureSensorNone = 0, PressureSensorSPI = 1} PressureSensorType;
		typedef enum { HeightSensorNone = 0, HeightSensorAnalogIn = 1} HeightSensorType;
		typedef enum { CompassNone = 0, CompassMicroMag3 = 1, CompassHMC5843 = 2 } CompassType;
			
		Sensors() : SubSystem()
		{				
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
				//Read compass and process compass reading
				if (_compass)
				{
					_compass->process(currentTime);
				}
				
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
		
		void setCompassType(const CompassType hardwareType)
		{
			switch (hardwareType)
			{
				case CompassMicroMag3:
				{
					_compass = new MicroMag3Compass();
					break;
				}
				case CompassHMC5843:
				{
					_compass = new HMC5843Compass();
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
		
		//Heading
		const float headingFromCompass()
		{
			if (_compass)
			{
				return _compass->getHeading();
			}
			return 0.0;
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