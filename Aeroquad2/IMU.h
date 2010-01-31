#include "SubSystem.h"
#include "HardwareComponent.h"

class IMUHardware : public HardwareComponent
{
	protected:
		enum { IMUPin4xGyroX = 7, IMUPin4xGyroY = 6, IMUPin4xGyroZ = 9, IMUPin1xGyroX = 5, IMUPin1xGyroY = 4, IMUPin1xGyroZ = 8, IMUPinAcclX = 0, IMUPinAcclY = 1, IMUPinAcclZ = 2};
		
		float _lastReading[9];
		
		typedef struct {					
			byte inpInvert;    			//invert input
		  	int zeroLevel;     			// zero level (mV) @ 0
		  	float inpSens;   			// input sensitivity (mv/unit) 
		} inputConfiguration;
		
		inputConfiguration _inputConfigurations[9];
		
		int _inputLayout[9];
		
	public:
		IMUHardware() : HardwareComponent()
		{

		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
			
			for (int i = 0; i < 9; i++)
			{
				_inputLayout[i] = -1;
			}
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			static float tmpf;	        //temporary variable
			
			for (int i = 0; i < 9; i++)
			{
				if (_inputLayout[i] != -1)
				{
					int rawAnalogValue = analogRead(_inputLayout[i]);
					tmpf = rawAnalogValue * HardwareComponent::getReferenceVoltage() / 1024.0f;  //voltage (mV)
					tmpf -= _inputConfigurations[i].zeroLevel; 		 	//voltage relative to zero level (mV)
				  	tmpf /= _inputConfigurations[i].inpSens;    		//input sensitivity in mV/Unit
					if (_inputConfigurations[i].inpInvert)
					{
				  		tmpf *= -1;  		//invert axis value according to configuration 
					}
					_lastReading[i] = tmpf;
				}
			}
		
			HardwareComponent::process(currentTime);	
		}
		
		const float *getCurrentReadings()
		{
			return _lastReading;
		}
};

class RazorIMU : public IMUHardware
{
	public:
		RazorIMU() : IMUHardware()
		{
			
		}
		
		void initialize()
		{
			IMUHardware::initialize();
			
			//Setup sensor input layout
			_inputLayout[0] = IMUPinAcclX;
			_inputLayout[1] = IMUPinAcclY;
			_inputLayout[2] = IMUPinAcclZ;
			
			_inputLayout[3] = IMUPin4xGyroX;
			_inputLayout[4] = IMUPin4xGyroY;
			_inputLayout[5] = IMUPin4xGyroZ;
			
			_inputLayout[6] = IMUPin1xGyroX;
			_inputLayout[7] = IMUPin1xGyroY;
			_inputLayout[8] = IMUPin1xGyroZ;
			
			//Setup sensor details
			//These numbers are based on the spec sheets for the components on the razor
			//http://www.sparkfun.com/datasheets/Sensors/IMU/lpr530al.pdf
			//http://www.sparkfun.com/datasheets/Sensors/IMU/LY530ALH.pdf
			//http://www.sparkfun.com/datasheets/Components/SMD/adxl335.pdf
			for(int i=0; i<=2; i++)
		 	{                  // X,Y axis accels
				_inputConfigurations[i].zeroLevel = 1600;
				_inputConfigurations[i].inpSens = 300;			//mV/g
				_inputConfigurations[i].inpInvert = 0; 
			}
			
			for(int i=3; i<=5; i++)
			{					//X,Y,Z 4x gyro
				_inputConfigurations[i].zeroLevel = 1200;
			 	_inputConfigurations[i].inpSens = 3300;			//mV/deg/ms
				_inputConfigurations[i].inpInvert = 1; 
			}
			
			for(int i=6; i<=8; i++)
			{					//X,Y,Z 1x gyro
				_inputConfigurations[i].zeroLevel = 1230;
			 	_inputConfigurations[i].inpSens = 830;			//mV/deg/ms
				_inputConfigurations[i].inpInvert = 1;
			} 			
		} 
		
		void process(const unsigned long currentTime)
		{
			//Any processing that the Razor IMU needs
			
			//Call the generic IMU Hardware process method
			IMUHardware::process(currentTime);	
		}
};

class IMUFilter
{
	protected:
		unsigned long lastProcessTime;
		
		float _accelRaw[3];
		float _gyro4Raw[3];
		float _gyro1Raw[3];

		float _processedRate[3];
		float _processedAngle[3];
		
	public:
		IMUFilter()
		{
			lastProcessTime = 0;
		}
		
		virtual void initialize()
		{
			
		}
		
		virtual void filter(const unsigned long currentTime)
		{
			lastProcessTime = currentTime;
		}
		
		void setCurrentReadings(const float *currentReadings)
		{
			/*Readings are expected in the following order (9 floats)
			xAccel, yAccel, zAccel, 4.5XGyro, 4.5YGyro, 4.5ZGyro, 1XGyro, 1YGyro, 1ZGyro*/
				
			_accelRaw[0] = currentReadings[0];
			_accelRaw[1] = currentReadings[1];
			_accelRaw[2] = currentReadings[2];
            
			_gyro4Raw[0] = currentReadings[3];
			_gyro4Raw[1] = currentReadings[4];
			_gyro4Raw[2] = currentReadings[5];
			
			_gyro1Raw[0] = currentReadings[6];
			_gyro1Raw[1] = currentReadings[7];
			_gyro1Raw[2] = currentReadings[8];
		}
		
		//Accessors for the processed values
		const float currentRollRate()
		{
			return _processedRate[1];
		}
		
		const float currentPitchRate()
		{
			return _processedRate[0];
		}
		
		const float currentYawRate()
		{
			return _processedRate[2];
		}
		
		const float currentRollAngle()
		{
			return _processedAngle[1];
		}
		
		const float currentPitchAngle()
		{
			return _processedAngle[0];
		}
};

class SimplifiedKalmanIMUFilter : public IMUFilter
{
	private:
		char _firstSample;	  //marks first sample
		unsigned long _lastMicros;	
		unsigned long _interval; //interval since previous analog samples
		
		
		//Notation "w" stands for one of the axes, so for example RwAcc[0],RwAcc[1],RwAcc[2] means RxAcc,RyAcc,RzAcc
		float _rwEst[3];    	 //Rw estimated from combining RwAcc and RwGyro
		float _rwGyro[3];        //Rw obtained from last estimated value and gyro movement
		
		float _gyroWeight;		// gyro weight/smooting factor
		
		void _normalize3DVector(float* vector)
		{
			static float R;  
			R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
			vector[0] /= R;
			vector[1] /= R;  
			vector[2] /= R;  
		}

		const float _square(const float x)
		{
			return x*x;
		}
	
	public:
		SimplifiedKalmanIMUFilter() : IMUFilter()
		{
			
		}
		
		void initialize()
		{
			IMUFilter::initialize();
			
			_firstSample = 1;
			
			_gyroWeight = 10;
		}
		
		void filter(const unsigned long currentTime)
		{	
			static int i,w;
			static float tmpf,tmpf2;
			static unsigned long newMicros;			
			static char signRzGyro;  

			//compute interval since last filter time
			newMicros = micros();			 
			_interval = newMicros - _lastMicros;    //please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2 
			_lastMicros = newMicros;               //save for next loop, please note interval will be invalid in first sample but we don't use it
			
			//normalize vector (convert to a vector with same direction and with length 1)
			_normalize3DVector(_accelRaw);
			
			if (_firstSample)
			{
			    for(w=0;w<=2;w++) 
				{
					_rwEst[w] = _accelRaw[w];    //initialize with accelerometer readings
				}
			
				_firstSample = 0;
			}
			else
			{
			    //evaluate RwGyro vector
			    if(abs(_rwEst[2]) < 0.1)
				{
			     	//Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
					//in this case skip the gyro data and just use previous estimate
					for(w=0;w<=2;w++) 
					{
						_rwGyro[w] = _rwEst[w];
					}
			    }
				else
				{
			    	//get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
			      	for(w=0;w<=1;w++)
					{
			        	tmpf = _gyro4Raw[w];                   //get current gyro rate in deg/ms
			        	tmpf *= _interval / 1000.0f;                     //get angle change in deg
			        	_processedAngle[w] = atan2(_rwEst[w],_rwEst[2]) * 180 / PI;   //get angle and convert to degrees        
			        	_processedAngle[w] += tmpf;                                 //get updated angle according to gyro movement
			      	}

			      //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
			      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
			      signRzGyro = ( cos(_processedAngle[0] * PI / 180) >=0 ) ? 1 : -1;

			      //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
			      for(w=0;w<=1;w++)
				  {
			        _rwGyro[0] = sin(_processedAngle[0] * PI / 180);
			        _rwGyro[0] /= sqrt( 1 + _square(cos(_processedAngle[0] * PI / 180)) * _square(tan(_processedAngle[1] * PI / 180)) );
			        _rwGyro[1] = sin(_processedAngle[1] * PI / 180);
			        _rwGyro[1] /= sqrt( 1 + _square(cos(_processedAngle[1] * PI / 180)) * _square(tan(_processedAngle[0] * PI / 180)) );        
			      }
			      _rwGyro[2] = signRzGyro * sqrt(1 - _square(_rwGyro[0]) - _square(_rwGyro[1]));
			    }

			    //combine Accelerometer and gyro readings
			    for(w=0;w<=2;w++) 
				{
					_rwEst[w] = (_accelRaw[w] + _gyroWeight * _rwGyro[w]) / (1 + _gyroWeight);
				}

			    _normalize3DVector(_rwEst);  
			}
			
			//Processed gyro rate is just the rate read from the sensor
			for(w=0;w<=2;w++) 
			{
				_processedRate[w] = _gyro4Raw[w];
			}
			
			IMUFilter::filter(currentTime);
		}
};


class IMU : public SubSystem
{
	private:
		IMUHardware *_imuHardware;
		IMUFilter *_imuFilter;
		
	public:
		typedef enum { HardwareTypeGeneric = 0, HardwareTypeRazor = 1} HardwareType;
		typedef enum { FilterTypeComplimentry = 0, FilterTypeSimplifiedKalman = 1} FilterType;
			
		IMU() : SubSystem()
		{
			_imuHardware = NULL;
			_imuFilter = NULL;
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
		}
		
		void setHardwareType(const HardwareType hardwareType)
		{
			switch (hardwareType)
			{
				case HardwareTypeRazor:
				{
					_imuHardware = new RazorIMU();
					break;
				}
			}
			
			if (_imuHardware)
			{
				_imuHardware->initialize();
			}
		}
		
		void setFilterType(const FilterType filterType)
		{
			switch (filterType)
			{
				case FilterTypeSimplifiedKalman:
				{
					_imuFilter = new SimplifiedKalmanIMUFilter();
					break;
				}
			}
			
			if (_imuFilter)
			{
				_imuFilter->initialize();
			}
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				if (_imuHardware)
				{
					_imuHardware->process(currentTime);

					_imuFilter->setCurrentReadings(_imuHardware->getCurrentReadings());

					_imuFilter->filter(currentTime);
					
					/*DEBUGSERIALPRINT(this->currentRollRate());
					DEBUGSERIALPRINT(",");
					DEBUGSERIALPRINT(this->currentPitchRate());
					DEBUGSERIALPRINT(",");
					DEBUGSERIALPRINT(this->currentYawRate());
					DEBUGSERIALPRINTLN("");*/
					
					/*DEBUGSERIALPRINT(this->currentRollAngle());
					DEBUGSERIALPRINT(",");
					DEBUGSERIALPRINT(this->currentPitchAngle());
					DEBUGSERIALPRINTLN("");*/
				}
			}
		}
		
		//Accessors for the processed values
		const float currentRollRate()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentRollRate();
			}
			
			return 0;
		}
		
		const float currentPitchRate()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentPitchRate();
			}
			
			return 0;	
		}
		
		const float currentYawRate()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentYawRate();
			}
			
			return 0;
		}
		
		const float currentRollAngle()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentRollAngle();
			}
			
			return 0;
		}
		
		const float currentPitchAngle()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentPitchAngle();
			}
			
			return 0;
		}
};