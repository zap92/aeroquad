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

class Compass : public HardwareComponent
{
	protected:
		float _currentHeadingInRadians;
		float _currentHeadingInDegrees;
		
		float _xMagnetMax, _xMagnetMin;
		float _yMagnetMax, _yMagnetMin;
		float _zMagnetMax, _zMagnetMin;
		
		void _process3dReading(const int xReading, const int yReading, const int zReading)
		{
			//Double check the compass limits.  This ensures we are scaling correctly
			/*if (xReading > _xMagnetMax) 
			{
				_xMagnetMax = xReading;
			}
			if (yReading > _yMagnetMax) 
			{
				_yMagnetMax = yReading;
			}
			if (zReading > _zMagnetMax) 
			{
				_zMagnetMax = zReading;
			}
			
			if (xReading < _xMagnetMin) 
			{
				_xMagnetMin = xReading;
			}
			if (yReading < _yMagnetMin) 
			{
				_yMagnetMin = yReading;
			}
			if (zReading < _zMagnetMin) 
			{
				_zMagnetMin = zReading;
			}

			//Map the incoming Data from -1 to 1
			float xMagnetMaped = fmap(xReading, _xMagnetMin, _xMagnetMax, -300000, 300000)/300000.0;
			float yMagnetMaped = fmap(yReading, _yMagnetMin, _yMagnetMax, -300000, 300000)/300000.0;
			float zMagnetMaped = fmap(zReading, _zMagnetMin, _zMagnetMax, -300000, 300000)/300000.0;


			//normalize the magnetic vector
			float norm = sqrt( sq(xMagnetMaped) + sq(yMagnetMaped) + sq(zMagnetMaped));
			xMagnetMaped /=norm;
			yMagnetMaped /=norm;
			zMagnetMaped /=norm;*/
			
			float flatHeadingInRadians = atan2(yReading, xReading);
			
			/*if (flatHeadingInRadians > 0)
			{
				flatHeadingInRadians = fabs(flatHeadingInRadians);
			}
			else
			{
				flatHeadingInRadians = (2*M_PI) - flatHeadingInRadians;
			}*/



			//http://www.ssec.honeywell.com/magnetic/datasheets/lowcost.pdf
			float currentRollAngle = -imu.currentRollAngleInRadians();
			float currentPitchAngle = -imu.currentPitchAngleInRadians();

			float cos_roll = cos(currentRollAngle);
			float sin_roll = sin(currentRollAngle);
			float cos_pitch = cos(currentPitchAngle);
			float sin_pitch = sin(currentPitchAngle);
			
			// Tilt compensated Magnetic field X
			float compensatedX = (xReading*cos_pitch) + (yReading*sin_roll*sin_pitch) + (zReading*cos_roll*sin_pitch);
			// Tilt compensated Magnetic field Y
			float compensatedY = (yReading*cos_roll) - (zReading*sin_roll);

			// Compensated Magnetic Heading
			_currentHeadingInRadians = atan2(compensatedY,compensatedX);			
			
			
			/*if (_currentHeadingInRadians < 0)
			{
				_currentHeadingInRadians = fabs(_currentHeadingInRadians);
			}
			else
			{
				_currentHeadingInRadians = (2*M_PI) - _currentHeadingInRadians;
			}*/
			
			_currentHeadingInDegrees = ToDeg(flatHeadingInRadians);
			
			
			
						
			serialcoms.debugPrint("!ANG:");
			serialcoms.debugPrint(imu.currentRollAngleInRadians());
			serialcoms.debugPrint(",");			
			serialcoms.debugPrint(imu.currentPitchAngleInRadians());
			serialcoms.debugPrint(",");
			serialcoms.debugPrint(flatHeadingInRadians);
			serialcoms.debugPrintln("");
		}
	
	public:
		Compass() : HardwareComponent()
		{
			_currentHeadingInDegrees = _currentHeadingInRadians = 0.0;
		}
		
		void setLimits(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
		{
			_xMagnetMax = xMax;
			_xMagnetMin = xMin;
			
			_yMagnetMax = yMax;
			_yMagnetMin = yMin;
			
			_zMagnetMax = zMax;
			_zMagnetMin = zMin;
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
	
		virtual void process(const unsigned long currentTime)
		{
			HardwareComponent::process(currentTime);	
		}
	
		const float getHeadingInRadians()
		{
			return _currentHeadingInRadians;
		}
			
		const float getHeadingInDegrees()
		{
			return _currentHeadingInDegrees;
		}	
};

#define HMC5843COMPAS_I2C_ADDRESS 0x1E
class HMC5843Compass : public Compass
{
	public:
		void initialize()
		{
			Compass::initialize();
			
			//Set the compass to continous read mode
			Wire.begin();
			Wire.beginTransmission(HMC5843COMPAS_I2C_ADDRESS);
			Wire.send(0x02);
			Wire.send(0x00);         //Set to continous streaming mode
			Wire.endTransmission();  

			delay(5);                //HMC5843 needs some ms before communication
		}
		
		void process(const unsigned long currentTime)
		{	
			Compass::process(currentTime);
			
			Wire.beginTransmission(HMC5843COMPAS_I2C_ADDRESS);
			Wire.send(0x03);        //sends address to read from
			Wire.endTransmission(); //end transmission
			 
			Wire.requestFrom(HMC5843COMPAS_I2C_ADDRESS, 6);    // request 6 bytes from device
			byte xAxisReadingHigh = Wire.receive();
			byte xAxisReadingLow = Wire.receive();
			int xAxisReading = (((xAxisReadingHigh) << 8) | (xAxisReadingLow));
						
			byte yAxisReadingHigh = Wire.receive();
			byte yAxisReadingLow = Wire.receive();
			int yAxisReading = (((yAxisReadingHigh) << 8) | (yAxisReadingLow));
			
			byte zAxisReadingHigh = Wire.receive();
			byte zAxisReadingLow = Wire.receive();
			int zAxisReading = (((zAxisReadingHigh) << 8) | (zAxisReadingLow));
			
			Wire.endTransmission(); //end transmission
			
			//The APM and DIYDrones HMC5843 board combination need things swapped around to because of mounting differences
			this->_process3dReading(xAxisReading, yAxisReading, -zAxisReading);
		}
};




class Sensors : public SubSystem
{
	private:
		PressureSensor *_pressureSensor;
		HeightSensor *_heightSensor;
		Compass *_compass;
		
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
			
			if (_compass)
			{
				_compass->initialize();
			}
			
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
		
		void setCompassType(const CompassType hardwareType)
		{
			switch (hardwareType)
			{
				case CompassHMC5843:
				{
					_compass = new HMC5843Compass();
					break;
				}
				
				default:
				{
					serialcoms.debugPrintln("ERROR: Unknown Compass type selected.");
					break;
				}
			}
		}
		
		void setCompassLimits(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
		{
			if (_compass)
			{
				_compass->setLimits(xMin, xMax, yMin, yMax, zMin, zMax);
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
		const float headingFromCompassInRadians()
		{
			if (_compass)
			{
				return _compass->getHeadingInRadians();
			}
			return 0.0;
		}
		const float headingFromCompassInDegrees()
		{
			if (_compass)
			{
				return _compass->getHeadingInDegrees();
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