#include "SubSystem.h"
#include "HardwareComponent.h"

class IMUHardware : public HardwareComponent
{
	protected:
		enum {IMUGyroX = 0, IMUGyroY, IMUGyroZ, IMUAcclX, IMUAcclY, IMUAcclZ};
		
		float _lastReading[6];
		int _rawReadings[6];
		
		typedef struct {					
			bool inputInUse;
			bool invert;    				//invert input
		  	int zeroLevel;     				// zero level (mV) @ 0
		  	float sensitivity;   			// input sensitivity (mv/unit) 
			int hardwarePin;
		} inputConfiguration;
		
		inputConfiguration _inputConfigurations[6];

	public:
		IMUHardware() : HardwareComponent()
		{
			for (int i = IMUGyroX; i <= IMUAcclZ; i++)
			{
				_inputConfigurations[i].inputInUse = false;
				_inputConfigurations[i].invert = false;
			}
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
		
		virtual const int readRawValue(const unsigned int axis)
		{
			return analogRead(axis);
		}
		
		virtual void process(const unsigned long currentTime)
		{
			this->scaleRawReadingsToEngineeringValues();		
			HardwareComponent::process(currentTime);	
		}
		
		void scaleRawReadingsToEngineeringValues()
		{
			static float tmpf;	        //temporary variable
			
			float dacMvValue = this->getReferenceVoltage() / (pow(2,this->getDacPrecision()) - 1);
			
			for (int i = IMUGyroX; i <= IMUAcclZ; i++)
			{
				if (_inputConfigurations[i].inputInUse)
				{
					int rawAnalogValue = _rawReadings[i];
					tmpf = (float)rawAnalogValue * dacMvValue;
					//DEBUGSERIALPRINT(tmpf);
					//DEBUGSERIALPRINT(":");
					tmpf -= _inputConfigurations[i].zeroLevel; 		 		//voltage relative to zero level (mV)
					//DEBUGSERIALPRINT(tmpf);
					//DEBUGSERIALPRINT(":");	
				  	tmpf /= _inputConfigurations[i].sensitivity;    		//input sensitivity in mV/Unit
					//DEBUGSERIALPRINT(tmpf);
					//DEBUGSERIALPRINT(":");
					if (_inputConfigurations[i].invert)
					{						
				  		tmpf *= -1;  		//invert axis value according to configuration 
					}
					_lastReading[i] = tmpf;
					
					//serialcoms.debugPrint(_lastReading[i]);
					//serialcoms.debugPrint(",");
				}
			}
			
			//serialcoms.debugPrintln("");
		}
		
#define ZEROLEVELSAMPLECOUNT 50
		virtual void calibrateZero()
		{
			int zeroLevelSamples[ZEROLEVELSAMPLECOUNT];
			float dacMvValue = this->getReferenceVoltage() / (pow(2,this->getDacPrecision()) - 1);

			//Calibrate each axis of gyro (Because they can drift over time)
			for (int i = IMUGyroX; i <= IMUGyroZ; i++)
			{
				if (_inputConfigurations[i].inputInUse)
				{
					//Sample the analog reading a bunch of times
					for (int j = 0; j < ZEROLEVELSAMPLECOUNT; j++) 
					{
						zeroLevelSamples[j] = this->readRawValue(_inputConfigurations[i].hardwarePin);
						delay(10);	//Esure that some more readings can be taken.						
					}
					
					//find the mode of the sampled data
					int sampleMode = findMode(zeroLevelSamples, ZEROLEVELSAMPLECOUNT);
					
					//convert the mode value to mV and assign it to the related _inputConfigurations structre for this input
					_inputConfigurations[i].zeroLevel = (float)sampleMode * dacMvValue;
					/*DEBUGSERIALPRINT(i);
					DEBUGSERIALPRINT(":");
					DEBUGSERIALPRINT(_inputConfigurations[i].zeroLevel);
					DEBUGSERIALPRINT("\t");*/
				}
			}
						
			/*DEBUGSERIALPRINTLN;
			delay(4000);*/
		}
		
		const float *getCurrentReadings()
		{
			return _lastReading;
		}
};

class OilpanIMU : public IMUHardware
{
	private:
		static const int _AdcChipSelect = 33;
		
		//Holder variables for the ADC readings
		unsigned char _adc_cmd[9];
		volatile long _adc_value[8];
		volatile unsigned char _adc_counter[8];
		
		//This IMU has a interrupt so we need a singleton pattern to get back to it when the interrupt fires
		static OilpanIMU *_staticInstance;
		
		
		unsigned char _ADC_SPI_transfer(unsigned char data)
		{
		  /* Wait for empty transmit buffer */
		  while ( !( UCSR2A & (1<<UDRE2)) );
		  /* Put data into buffer, sends the data */
		  UDR2 = data;
		  /* Wait for data to be received */
		  while ( !(UCSR2A & (1<<RXC2)) );
		  /* Get and return received data from buffer */
		  return UDR2;
		}
		
		
	public:
		static OilpanIMU* getInstance()
		{
			return _staticInstance;
		}
		
		OilpanIMU() : IMUHardware()
		{
			_staticInstance = this;
			
			//SPI Commands needed to read the DAC
			_adc_cmd[0] = 0x87;
			_adc_cmd[1] = 0xC7;
			_adc_cmd[2] = 0x97;
			_adc_cmd[3] = 0xD7;
			_adc_cmd[4] = 0xA7;
			_adc_cmd[5] = 0xE7;
			_adc_cmd[6] = 0xB7;
			_adc_cmd[7] = 0xF7;
			_adc_cmd[8] = 0x00;
			
			
			memset((void*)_adc_value, 0, 8);
			memset((void*)_adc_counter,0,8);
			
			this->setReferenceVoltage(3300);
			this->setDacPrecision(12);
		}
		
		void initialize()
		{
			IMUHardware::initialize();
			
			//Setup sensor details
			//X,Y,Z Gyros
			
			//With the settings below gyros give us deg/ms
			_inputConfigurations[IMUGyroX].zeroLevel = 1370;			//in mv
			_inputConfigurations[IMUGyroX].sensitivity = 2.2;			//in mV/deg/ms
			_inputConfigurations[IMUGyroX].inputInUse = true; 
			_inputConfigurations[IMUGyroX].hardwarePin = 2;
			_inputConfigurations[IMUGyroX].invert = true;
						
			_inputConfigurations[IMUGyroY].zeroLevel = 1370;			//in mv
			_inputConfigurations[IMUGyroY].sensitivity = 2.2;			//in mV/deg/ms
			_inputConfigurations[IMUGyroY].inputInUse = true; 
			_inputConfigurations[IMUGyroY].hardwarePin = 1;
			
			_inputConfigurations[IMUGyroZ].zeroLevel = 1370;			//in mv
			_inputConfigurations[IMUGyroZ].sensitivity = 2.2;			//in mV/deg/ms
			_inputConfigurations[IMUGyroZ].inputInUse = true; 
			_inputConfigurations[IMUGyroZ].hardwarePin = 0;
			
			
			//X,Y,Z Accels
			//With the settings below accels give us G
			_inputConfigurations[IMUAcclX].zeroLevel = 1650;			//in mv
		 	_inputConfigurations[IMUAcclX].sensitivity = 310;			//in mv/G
			_inputConfigurations[IMUAcclX].inputInUse = true; 
			_inputConfigurations[IMUAcclX].hardwarePin = 4;
			_inputConfigurations[IMUAcclX].invert = true;

			_inputConfigurations[IMUAcclY].zeroLevel = 1650;			//in mv
		 	_inputConfigurations[IMUAcclY].sensitivity = 310;			//in mv/G
			_inputConfigurations[IMUAcclY].inputInUse = true; 
			_inputConfigurations[IMUAcclY].hardwarePin = 5;
			_inputConfigurations[IMUAcclY].invert = true;

			_inputConfigurations[IMUAcclZ].zeroLevel = 1650;			//in mv
		 	_inputConfigurations[IMUAcclZ].sensitivity = 310;			//in mv/G
			_inputConfigurations[IMUAcclZ].inputInUse = true; 
			_inputConfigurations[IMUAcclZ].hardwarePin = 6;
			

 	
			//Since the Oilpan uses a dedicated DAC lets setup what we need to get it running.
			//This all comes from the APM_ADC code provided by DIYDrones
			{
				// Disable device (Chip select is active low)
				pinMode(_AdcChipSelect,OUTPUT);
				digitalWrite(_AdcChipSelect,HIGH); 

				// Setup Serial Port2 in SPI mode
				UBRR2 = 0;   
				DDRH |= (1<<PH2);  // SPI clock XCK2 (PH2) as output. This enable SPI Master mode
				// Set MSPI mode of operation and SPI data mode 0.
				UCSR2C = (1<<UMSEL21)|(1<<UMSEL20); //|(0<<UCPHA2)|(0<<UCPOL2);
				// Enable receiver and transmitter.
				UCSR2B = (1<<RXEN2)|(1<<TXEN2);
				// Set Baud rate
				UBRR2 = 2; // SPI clock running at 2.6MHz


				// Enable Timer2 Overflow interrupt to capture ADC data
				TIMSK2 = 0;  // Disable interrupts 
				TCCR2A = 0;  // normal counting mode 
				TCCR2B = _BV(CS21)|_BV(CS22);     // Set prescaler of 256
				TCNT2 = 0;
				TIFR2 = _BV(TOV2);  // clear pending interrupts; 
				TIMSK2 =  _BV(TOIE2) ; // enable the overflow interrupt
			}
		} 		
		
	 	const int readRawValue(const unsigned int axis)
		{
			//All we need to do here is just grab the value from the variable that holds it
			int result;

			cli();  // We stop interrupts to read the variables
			if (_adc_counter[axis]>0)
			{
				result = _adc_value[axis]/_adc_counter[axis];
			}
			else
			{
				result = 0;
			}
			_adc_value[axis] = 0;    // Initialize for next reading
			_adc_counter[axis] = 0;
			sei();						//turn interrupts back on
			
			return(result);
		}
		
		
		void process(const unsigned long currentTime)
		{
			for (int i = IMUGyroX; i <= IMUAcclZ; i++)
			{
				if (_inputConfigurations[i].inputInUse)
				{
					_rawReadings[i] = this->readRawValue(_inputConfigurations[i].hardwarePin);
					//DEBUGSERIALPRINT(_rawReadings[i]);
					//DEBUGSERIALPRINT(",");
				}
			}
			
			//DEBUGSERIALPRINTLN;
			
			//Call the generic IMU Hardware process method
			IMUHardware::process(currentTime);	
		}
		
		//This is called via an Interrupt to pull the data from the ADC.
		//We are running this at 400hz via a timer so it can oversample the data
		void fetchReadings()
		{
			uint8_t ch;
			unsigned int adc_tmp;

			//bit_set(PORTC,0); // To test performance with a scope
			Bit_Clear(PORTC,4);             // Enable Chip Select (PIN PC4)
			_ADC_SPI_transfer(_adc_cmd[0]);       // Command to read the first channel

			for (ch=0;ch<8;ch++)
			{
				adc_tmp = _ADC_SPI_transfer(0)<<8;    // Read first byte
				adc_tmp |= _ADC_SPI_transfer(_adc_cmd[ch+1]);  // Read second byte and send next command
				_adc_value[ch] += adc_tmp>>3;     // Shift to 12 bits
				_adc_counter[ch]++;               // Number of samples
			}

			Bit_Set(PORTC,4);                // Disable Chip Select (PIN PH6)
			//bit_clear(PORTC,0); // To test performance
		}
};

OilpanIMU *OilpanIMU::_staticInstance = NULL;

//Timer used to trigger the frequency that we read the ADC
ISR (TIMER2_OVF_vect)
{
	OilpanIMU::getInstance()->fetchReadings();
	
	//reset the timer
	TCNT2 = 104;        // 400 Hz
}

class IMUFilter
{
	protected:
		typedef enum { IMUFilterAxisX = 0, IMUFilterAxisY, IMUFilterAxisZ } IMUFilterAxis;
		
		unsigned long _lastProcessTime;
		unsigned int  _deltaTime;
		
		bool _firstPass;
		
		float _accelRaw[3];
		float _gyroRaw[3];

		float _processedRate[3];
		float _processedAngleInDegrees[2];
		float _processedAngleInRadians[2];
		
	public:
		IMUFilter()
		{
			_lastProcessTime = 0;
			_deltaTime = 0;
			
			_firstPass = true;
			
			_processedRate[IMUFilterAxisX] = _processedRate[IMUFilterAxisY] = _processedRate[IMUFilterAxisZ] = 0;
			_processedAngleInDegrees[IMUFilterAxisX] = _processedAngleInDegrees[IMUFilterAxisY] = 0;
			_processedAngleInRadians[IMUFilterAxisX] = _processedAngleInRadians[IMUFilterAxisY] = 0;
		}
		
		virtual void initialize()
		{
			
		}
		
		virtual void filter(const unsigned long currentTime)
		{
			_deltaTime = currentTime - _lastProcessTime;
			_lastProcessTime = currentTime;
			
			//Copy over the gyro rates.  If the subclassed filters overwrite then so be it, but this is the default
			_processedRate[IMUFilterAxisX] = _gyroRaw[IMUFilterAxisX];
			_processedRate[IMUFilterAxisY] = _gyroRaw[IMUFilterAxisY];
			_processedRate[IMUFilterAxisZ] = _gyroRaw[IMUFilterAxisZ];			
		}
		
		void setCurrentReadings(const float *currentReadings)
		{
			//Readings are expected in the following order (6 floats)
			//XGyro, YGyro, ZGyro, xAccel, yAccel, zAccel
				
			_gyroRaw[IMUFilterAxisX] = currentReadings[0];	//X
			_gyroRaw[IMUFilterAxisY] = currentReadings[1];	//Y
			_gyroRaw[IMUFilterAxisZ] = currentReadings[2];	//Z
				
			_accelRaw[IMUFilterAxisX] = currentReadings[3];	//X
			_accelRaw[IMUFilterAxisY] = currentReadings[4];	//Y
			_accelRaw[IMUFilterAxisZ] = currentReadings[5];	//Z
		}
		
		//Accessors for the processed values
		const float currentRollRate()
		{
			return _processedRate[IMUFilterAxisX];
		}
		
		const float currentPitchRate()
		{
			return _processedRate[IMUFilterAxisY];
		}
		
		const float currentYawRate()
		{
			return _processedRate[IMUFilterAxisZ];
		}
		
		const float currentRollAngleInDegrees()
		{
			//The roll angle rotates "around" the Y axis
			return _processedAngleInDegrees[IMUFilterAxisY];
		}
		
		const float currentPitchAngleInDegrees()
		{
			//The roll angle rotates "around" the X axis
			return _processedAngleInDegrees[IMUFilterAxisX];
		}
		
		const float currentRollAngleInRadians()
		{
			//The roll angle rotates "around" the Y axis
			return _processedAngleInRadians[IMUFilterAxisY];
		}
		
		const float currentPitchAngleInRadians()
		{
			//The roll angle rotates "around" the X axis
			return _processedAngleInRadians[IMUFilterAxisX];
		}
};

//Working
//Just a simple Accel only filter.
class SimpleAccellOnlyIMUFilter : public IMUFilter
{
	public:
		SimpleAccellOnlyIMUFilter() : IMUFilter()
		{
			
		}
		
		void initialize()
		{
			IMUFilter::initialize();
		}
		
		void filter(const unsigned long currentTime)
		{
			IMUFilter::filter(currentTime);
			
			//Very simplistic angle determination.
			//Doesn't use the gyros at all.
			float R = sqrt(sq(_accelRaw[IMUFilterAxisX]) + sq(_accelRaw[IMUFilterAxisY]) + sq(_accelRaw[IMUFilterAxisZ]));
			
			_processedAngleInRadians[IMUFilterAxisX] = fmap(acos(_accelRaw[IMUFilterAxisX]/R), 0, M_PI, -(M_PI/2), (M_PI/2));
			_processedAngleInRadians[IMUFilterAxisY] = fmap(acos(_accelRaw[IMUFilterAxisY]/R), 0, M_PI, -(M_PI/2), (M_PI/2));
			
			_processedAngleInDegrees[IMUFilterAxisX] = ToDeg(_processedAngleInRadians[IMUFilterAxisX]);
			_processedAngleInDegrees[IMUFilterAxisY] = ToDeg(_processedAngleInRadians[IMUFilterAxisY]);
			
			if (_firstPass) {_firstPass = false;}
		}
};

//Working
//Based on the code by RoyB at: http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286    
class ComplimentryMUFilter : public IMUFilter
{
	private:
		float timeConstantCF;
		
		float previousAngle_roll;
		float newAngle_roll;
		float newRate_roll;
		float filterTerm0_roll;
		float filterTerm1_roll;
		float filterTerm2_roll;
		
		float previousAngle_pitch;
		float newAngle_pitch;
		float newRate_pitch;
		float filterTerm0_pitch;
		float filterTerm1_pitch;
		float filterTerm2_pitch;

	public:
		ComplimentryMUFilter() : IMUFilter()
		{
		    filterTerm0_roll = filterTerm0_pitch = 0;
			filterTerm1_roll = filterTerm1_pitch = 0;
			filterTerm2_roll = filterTerm2_pitch = 0;
		}
		
		void initialize()
		{
			previousAngle_roll = 0;
			previousAngle_pitch = 0;

		    timeConstantCF = 4.0;
		
			IMUFilter::initialize();
		}
		
		void filter(const unsigned long currentTime)
		{			
			IMUFilter::filter(currentTime);
			
			float dt = _deltaTime / 1000.0;
			
			float newRollAngle = atan2(_accelRaw[IMUFilterAxisX],_accelRaw[IMUFilterAxisZ]);
			float newPitchAngle = atan2(_accelRaw[IMUFilterAxisY],_accelRaw[IMUFilterAxisZ]);
			
			float newRollRate = ToRad(_gyroRaw[IMUFilterAxisX]);
			float newPitchRate = ToRad(_gyroRaw[IMUFilterAxisY]);
									
			if (_firstPass)
			{
				previousAngle_roll = newRollAngle;
				filterTerm2_roll = newRollRate;
				
				previousAngle_pitch = newPitchAngle;
				filterTerm2_pitch = newPitchRate;
				
				_firstPass = false;
			}
			else
			{		
				filterTerm0_roll = (newRollAngle - previousAngle_roll) * timeConstantCF *  timeConstantCF;
				filterTerm2_roll += (filterTerm0_roll * dt);
				filterTerm1_roll = filterTerm2_roll + (newRollAngle - previousAngle_roll) * 2 *  timeConstantCF + newRollRate;
				previousAngle_roll = (filterTerm1_roll * dt) + previousAngle_roll;
				
				filterTerm0_pitch = (newPitchAngle - previousAngle_pitch) * timeConstantCF *  timeConstantCF;
				filterTerm2_pitch += filterTerm0_pitch * dt;
				filterTerm1_pitch = filterTerm2_pitch + (newPitchAngle - previousAngle_pitch) * 2 *  timeConstantCF + newPitchRate;
				previousAngle_pitch = (filterTerm1_pitch * dt) + previousAngle_pitch;
			}
			
			//This filter calculates things backwards so we need to invert the results
			_processedAngleInRadians[IMUFilterAxisX] = -previousAngle_roll;
			_processedAngleInRadians[IMUFilterAxisY] = -previousAngle_pitch;
			
			_processedAngleInDegrees[IMUFilterAxisX] = ToDeg(_processedAngleInRadians[IMUFilterAxisX]);
			_processedAngleInDegrees[IMUFilterAxisY] = ToDeg(_processedAngleInRadians[IMUFilterAxisY]);
		}
};

//Not Working.
//Seams to "Lock" at certian angles
//Based on the code at: http://www.starlino.com/imu_guide.html
class SimplifiedKalmanIMUFilter : public IMUFilter
{
	private:
		//Notation "w" stands for one of the axes, so for example RwAcc[0],RwAcc[1],RwAcc[2] means RxAcc,RyAcc,RzAcc
		float _rwEst[3];    	 //Rw estimated from combining RwAcc and RwGyro
		float _rwGyro[3];        //Rw obtained from last estimated value and gyro movement
		float _rwAcc[3];        //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
		float _awz[2];
		
		float _gyroWeight;		// gyro weight/smooting factor	
	public:
		SimplifiedKalmanIMUFilter() : IMUFilter()
		{
			
		}
		
		void initialize()
		{
			IMUFilter::initialize();
			
			_gyroWeight = 10;
		}
		
		void filter(const unsigned long currentTime)
		{	
			IMUFilter::filter(currentTime);  
			
			static float tmpf;  			//temp variable
			static int w;

			//get accelerometer readings in g, gives us RwAcc vector
			for(w=0;w<=2;w++)
			{
				_rwAcc[w] = _accelRaw[w];
			}

			//normalize vector (convert to a vector with same direction and with length 1)
			normalize3DVector(_rwAcc);

			if (_firstPass)
			{
			    for(w=0;w<=2;w++)
				{
					_rwEst[w] = _rwAcc[w];    //initialize with accelerometer readings
				}
				_firstPass = false;
			}
			else
			{
			    //evaluate RwGyro vector
			    if(fabs(_rwEst[2]) < 0.1)
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
						tmpf = (_gyroRaw[w] / 1000.0f) / 1000.0f;                   //get current gyro rate in deg/ns
						tmpf *= (_deltaTime / 1000.0f);                     		//get angle change in deg
			        	_awz[w] = (atan2(_rwEst[w],_rwEst[2]));   		//get angle and convert to degrees        
			        	_awz[w] += ToRad(tmpf);                         //get updated angle according to gyro movement
			      	}

			      	//estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
			      	//RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
			      	int signRzGyro = ( cos(_awz[0]) >=0 ) ? 1 : -1;

			      	//reverse calculation of RwGyro from Awz angles, for formula deductions see  http://starlino.com/imu_guide.html
			      	for(w=0;w<=1;w++)
					{
			        	_rwGyro[w] = sin(_awz[w]);
			        	_rwGyro[w] /= sqrt( 1 + sq(cos(_awz[w])) * sq(tan(_awz[1-w])) );
			   		}
			    
					_rwGyro[2] = signRzGyro * sqrt(1 - sq(_rwGyro[0]) - sq(_rwGyro[1]));
			    
			    	//combine Accelerometer and gyro readings
			    	for(w=0;w<=2;w++)
			 		{
						_rwEst[w] = (_rwAcc[w] + _gyroWeight * _rwGyro[w]) / (1 + _gyroWeight);
					}
				
			    	normalize3DVector(_rwEst);
				}
			}
			
			//This algo returns us data with the signs backwards to what the rest of the system expects.  Invert
			_processedAngleInRadians[IMUFilterAxisX] = -_rwEst[0];
			_processedAngleInRadians[IMUFilterAxisY] = -_rwEst[1];
			
			_processedAngleInDegrees[IMUFilterAxisX] = ToDeg(_processedAngleInRadians[IMUFilterAxisX]);
			_processedAngleInDegrees[IMUFilterAxisY] = ToDeg(_processedAngleInRadians[IMUFilterAxisX]);
		}
};

//Not working yet.
//Just returns 0's
class DCMIMUFilter : public IMUFilter
{
	private:
		float _dt;
		
		float _DCMMatrix[3][3];		
		float _updateMatrix[3][3];
		float _temporaryMatrix[3][3];
		
		float _errorRollPitch[3];
		float _errorYaw[3];
		
		float _omegaVector[3]; //Corrected Gyro_Vector data
		float _omegaP[3];//Omega Proportional correction
		float _omegaI[3];//Omega Integrator
		float _omega[3];
		
		float _kpRollPitch;
		float _kiRollPitch;
		float _kpYaw;
		float _kiYaw;
		
		float _accelVector[3];
		float _gyroVector[3];
		
		float _accelWeight;
		
		void _matrixUpdate()
		{
			
			_gyroVector[0] = ToRad(_gyroRaw[IMUFilterAxisX]);
			_gyroVector[1] = ToRad(_gyroRaw[IMUFilterAxisY]);
			_gyroVector[2] = ToRad(_gyroRaw[IMUFilterAxisZ]);

			_accelVector[0] = _accelRaw[IMUFilterAxisX];
			_accelVector[1] = _accelRaw[IMUFilterAxisY];
			_accelVector[2] = _accelRaw[IMUFilterAxisZ];

			// Low pass filter on accelerometer data (to filter vibrations)
			//Accel_Vector[0]=Accel_Vector[0]*0.5 + (float)read_adc(3)*0.5; // acc x
			//Accel_Vector[1]=Accel_Vector[1]*0.5 + (float)read_adc(4)*0.5; // acc y
			//Accel_Vector[2]=Accel_Vector[2]*0.5 + (float)read_adc(5)*0.5; // acc z

			Vector_Add(&_omega[0], &_gyroVector[0], &_omegaI[0]);//adding integrator
			Vector_Add(&_omegaVector[0], &_omega[0], &_omegaP[0]);//adding proportional

			//Accel_adjust();//adjusting centrifugal acceleration. // Not used for quadcopter

			_updateMatrix[0][0] = 0;
			_updateMatrix[0][1] = -_dt * _omegaVector[2];//-z
			_updateMatrix[0][2] = _dt * _omegaVector[1];//y
			_updateMatrix[1][0] = _dt * _omegaVector[2];//z
			_updateMatrix[1][1] = 0;
			_updateMatrix[1][2] = -_dt * _omegaVector[0];//-x
			_updateMatrix[2][0] = -_dt * _omegaVector[1];//-y
			_updateMatrix[2][1] = _dt * _omegaVector[0];//x
			_updateMatrix[2][2] = 0;

			Matrix_Multiply(_DCMMatrix,_updateMatrix,_temporaryMatrix); //a*b=c

			for(int x=0; x<3; x++)  //Matrix Addition (update)
			{
				for(int y=0; y<3; y++)
				{
					_DCMMatrix[x][y] += _temporaryMatrix[x][y];
				} 
			}
		}
		
		void _normalize()
		{
			float error=0;
			float temporary[3][3];
			float renorm=0;

			error= -Vector_Dot_Product(&_DCMMatrix[0][0],&_DCMMatrix[1][0])*.5; //eq.19

			Vector_Scale(&temporary[0][0], &_DCMMatrix[1][0], error); //eq.19
			Vector_Scale(&temporary[1][0], &_DCMMatrix[0][0], error); //eq.19

			Vector_Add(&temporary[0][0], &temporary[0][0], &_DCMMatrix[0][0]);//eq.19
			Vector_Add(&temporary[1][0], &temporary[1][0], &_DCMMatrix[1][0]);//eq.19

			Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

			renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
			Vector_Scale(&_DCMMatrix[0][0], &temporary[0][0], renorm);

			renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
			Vector_Scale(&_DCMMatrix[1][0], &temporary[1][0], renorm);

			renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
			Vector_Scale(&_DCMMatrix[2][0], &temporary[2][0], renorm);	
		}
		
		void _driftCorrection()
		{
			//Compensation the Roll, Pitch and Yaw drift. 
			float errorCourse;
			static float Scaled_Omega_P[3];
			static float Scaled_Omega_I[3];
			float Accel_magnitude;
			float Accel_weight;

			//*****Roll and Pitch***************
			Vector_Cross_Product(&_errorRollPitch[0],&_accelVector[0],&_DCMMatrix[2][0]); //adjust the ground of reference
			Vector_Scale(&_omegaP[0],&_errorRollPitch[0],_kpRollPitch*Accel_weight);

			Vector_Scale(&Scaled_Omega_I[0],&_errorRollPitch[0],_kpRollPitch*Accel_weight);
			Vector_Add(_omegaI,_omegaI,Scaled_Omega_I);

			//*****YAW***************
			// We make the gyro YAW drift correction based on compass magnetic heading 
			if (0) 
			{
				/*errorCourse = (DCM_Matrix[0][0]*APM_Compass.Heading_Y) - (DCM_Matrix[1][0]*APM_Compass.Heading_X);  //Calculating YAW error
				Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

				Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
				Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

				Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
				Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I*/
			}
		}
		
		void _eulerAngles()
		{
			// Euler angles from DCM matrix
			_processedAngleInDegrees[IMUFilterAxisX] = ToDeg(atan2(_DCMMatrix[2][1],_DCMMatrix[2][2]));
			_processedAngleInDegrees[IMUFilterAxisY] = ToDeg(asin(-_DCMMatrix[2][0]));
			//yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
		}
		
	public:
		DCMIMUFilter() : IMUFilter()
		{
			
		}
		
		void initialize()
		{
			IMUFilter::initialize();
			
			_kpRollPitch = 0.002;
			_kiRollPitch = 0.0000005;
			_kpYaw = 1.5;
			_kiYaw = 0.00005;
			
			_accelWeight = 1.0;
		
			//Initialize all the matrix
			{
				_DCMMatrix[0][0] = 1;
				_DCMMatrix[0][1] = 0;
				_DCMMatrix[0][2] = 0;
			
				_DCMMatrix[1][0] = 0;
				_DCMMatrix[1][1] = 1;
				_DCMMatrix[1][2] = 0;
			
				_DCMMatrix[2][0] = 0;
				_DCMMatrix[2][1] = 0;
				_DCMMatrix[2][2] = 1;


				_updateMatrix[0][0] = 0;
				_updateMatrix[0][1] = 1;
				_updateMatrix[0][2] = 2;
			
				_updateMatrix[1][0] = 3;
				_updateMatrix[1][1] = 4;
				_updateMatrix[1][2] = 5;
			
				_updateMatrix[2][0] = 6;
				_updateMatrix[2][1] = 7;
				_updateMatrix[2][2] = 8;
			

				_temporaryMatrix[0][0] = 0;
				_temporaryMatrix[0][1] = 0;
				_temporaryMatrix[0][2] = 0;
			
				_temporaryMatrix[1][0] = 0;
				_temporaryMatrix[1][1] = 0;
				_temporaryMatrix[1][2] = 0;
			
				_temporaryMatrix[2][0] = 0;
				_temporaryMatrix[2][1] = 0;
				_temporaryMatrix[2][2] = 0;

		
				_omegaVector[0] = 0;
				_omegaVector[1] = 0;
				_omegaVector[2] = 0;
			

				_omegaP[0] = 0;
				_omegaP[1] = 0;
				_omegaP[2] = 0;
			

				_omegaI[0] = 0;
				_omegaI[1] = 0;
				_omegaI[2] = 0;
			

				_omega[0] = 0;
				_omega[1] = 0;
				_omega[2] = 0;
			

				_errorRollPitch[0] = 0;
				_errorRollPitch[1] = 0;
				_errorRollPitch[2] = 0;
			

				_errorYaw[0] = 0;
				_errorYaw[1] = 0;
				_errorYaw[2] = 0;


				_accelVector[0] = 0;
				_accelVector[1] = 0;
				_accelVector[2] = 0;


				_gyroVector[0] = 0;
				_gyroVector[1] = 0;
				_gyroVector[2] = 0;
			}
		}
		
		void filter(const unsigned long currentTime)
		{
			IMUFilter::filter(currentTime);
			
			//get the delta time in seconds
			_dt = _deltaTime / 1000.0f;
			
			_matrixUpdate(); 
			_normalize();
			_driftCorrection();
			_eulerAngles();
			
			if (_firstPass) {_firstPass = false;}
		}
};

//Working 
//Based on code at: http://gluonpilot.com
class KalmanIMUFilter : public IMUFilter
{
private:
	struct Gyro1DKalman
	{
		/* These variables represent our state matrix x */
		float x_angle,
		      x_bias;

		/* Our error covariance matrix */
		float P_00,
		      P_01,
		      P_10,
		      P_11;	

		/* 
		 * Q is a 2x2 matrix of the covariance. Because we
		 * assuma the gyro and accelero noise to be independend
		 * of eachother, the covariances on the / diagonal are 0.
		 *
		 * Covariance Q, the process noise, from the assumption
		 *    x = F x + B u + w
		 * with w having a normal distribution with covariance Q.
		 * (covariance = E[ (X - E[X])*(X - E[X])' ]
		 * We assume is linair with dt
		 */
		float Q_angle, Q_gyro;
		/*
		 * Covariance R, our observation noise (from the accelerometer)
		 * Also assumed to be linair with dt
		 */
		float R_angle;
	};
	
	struct Gyro1DKalman _rollFilterData;
	struct Gyro1DKalman _pitchFilterData;
	
	// Initializing the struct
	void _init(struct Gyro1DKalman *filterdata, float Q_angle, float Q_gyro, float R_angle)
	{
		memset (filterdata, 0, sizeof (struct Gyro1DKalman));
		filterdata->Q_angle = Q_angle;
		filterdata->Q_gyro  = Q_gyro;
		filterdata->R_angle = R_angle;
	}
	
	// Kalman predict
	/*
	 * The predict function. Updates 2 variables:
	 * our model-state x and the 2x2 matrix P
	 *     
	 * x = [ angle, bias ]' 
	 * 
	 *   = F x + B u
	 *
	 *   = [ 1 -dt, 0 1 ] [ angle, bias ] + [ dt, 0 ] [ dotAngle 0 ]
	 *
	 *   => angle = angle + dt (dotAngle - bias)
	 *      bias  = bias
	 *
	 *
	 * P = F P transpose(F) + Q
	 *
	 *   = [ 1 -dt, 0 1 ] * P * [ 1 0, -dt 1 ] + Q
	 *
	 *  P(0,0) = P(0,0) - dt * ( P(1,0) + P(0,1) ) + dt² * P(1,1) + Q(0,0)
	 *  P(0,1) = P(0,1) - dt * P(1,1) + Q(0,1)
	 *  P(1,0) = P(1,0) - dt * P(1,1) + Q(1,0)
	 *  P(1,1) = P(1,1) + Q(1,1)
	 *
	 *
	 */
	void _predict(struct Gyro1DKalman *filterdata, const float dotAngle, const float dt)
	{
		filterdata->x_angle += dt * (dotAngle - filterdata->x_bias);

		filterdata->P_00 +=  - dt * (filterdata->P_10 + filterdata->P_01) + filterdata->Q_angle * dt;
		filterdata->P_01 +=  - dt * filterdata->P_11;
		filterdata->P_10 +=  - dt * filterdata->P_11;
		filterdata->P_11 +=  + filterdata->Q_gyro * dt;
	}
	
	// Kalman update
	/*
	 *  The update function updates our model using 
	 *  the information from a 2nd measurement.
	 *  Input angle_m is the angle measured by the accelerometer.
	 *
	 *  y = z - H x
	 *
	 *  S = H P transpose(H) + R
	 *    = [ 1 0 ] P [ 1, 0 ] + R
	 *    = P(0,0) + R
	 * 
	 *  K = P transpose(H) S^-1
	 *    = [ P(0,0), P(1,0) ] / S
	 *
	 *  x = x + K y
	 *
	 *  P = (I - K H) P
	 *
	 *    = ( [ 1 0,    [ K(0),
	 *          0 1 ] -   K(1) ] * [ 1 0 ] ) P
	 *
	 *    = [ P(0,0)-P(0,0)*K(0)  P(0,1)-P(0,1)*K(0),
	 *        P(1,0)-P(0,0)*K(1)  P(1,1)-P(0,1)*K(1) ]
	 */
	float _update(struct Gyro1DKalman *filterdata, const float angle_m)
	{
		const float y = angle_m - filterdata->x_angle;

		const float S = filterdata->P_00 + filterdata->R_angle;
		const float K_0 = filterdata->P_00 / S;
		const float K_1 = filterdata->P_10 / S;

		filterdata->x_angle +=  K_0 * y;
		filterdata->x_bias  +=  K_1 * y;

		filterdata->P_00 -= K_0 * filterdata->P_00;
		filterdata->P_01 -= K_0 * filterdata->P_01;
		filterdata->P_10 -= K_1 * filterdata->P_00;
		filterdata->P_11 -= K_1 * filterdata->P_01;

		return filterdata->x_angle;
	}
	
	
public:
	KalmanIMUFilter() : IMUFilter()
	{
		
	}
	
	void initialize()
	{
		IMUFilter::initialize();
		
		_init(&_rollFilterData, 0.001, 0.003, 0.03);
		_init(&_pitchFilterData, 0.001, 0.003, 0.03);
	}
	
	void filter(const unsigned long currentTime)
	{
		IMUFilter::filter(currentTime);

		float dt = _deltaTime / 1000.0f;  //delta time in seconds
		float R = sqrt(sq(_accelRaw[IMUFilterAxisX]) + sq(_accelRaw[IMUFilterAxisY]) + sq(_accelRaw[IMUFilterAxisZ]));
		
		float rawRollAngle = acos(_accelRaw[IMUFilterAxisX]/R);
		float rawPitchAngle = acos(_accelRaw[IMUFilterAxisY]/R);
		
		if (_firstPass)
		{
			//make sure to initialize things the first time through.
			_rollFilterData.x_angle = rawRollAngle;
			_pitchFilterData.x_angle = rawPitchAngle;
			
			_firstPass = false;
		}
		else
		{
			//Roll
			_predict(&_rollFilterData, ToRad(_gyroRaw[IMUFilterAxisX]), dt);
		 	_update(&_rollFilterData, rawRollAngle);
		
			//Pitch
			_predict(&_pitchFilterData, ToRad(_gyroRaw[IMUFilterAxisY]), dt);
			_update(&_pitchFilterData, rawPitchAngle);
		}
		
		_processedAngleInRadians[IMUFilterAxisX] = fmap(_rollFilterData.x_angle, 0, M_PI, -(M_PI/2), (M_PI/2));
		_processedAngleInRadians[IMUFilterAxisY] = fmap(_pitchFilterData.x_angle, 0, M_PI, -(M_PI/2), (M_PI/2));
		
		_processedAngleInDegrees[IMUFilterAxisX] = ToDeg(_processedAngleInRadians[IMUFilterAxisX]);
		_processedAngleInDegrees[IMUFilterAxisY] = ToDeg(_processedAngleInRadians[IMUFilterAxisY]);
	}	
};



class IMU : public SubSystem
{
	private:
		IMUHardware *_imuHardware;
		IMUFilter *_imuFilter;
		
	public:
		typedef enum { HardwareTypeOilPan } HardwareType;
		typedef enum { FilterTypeSimpleAccellOnly = 0, FilterTypeComplimentry, FilterTypeSimplifiedKalman, FilterTypeDCM, FilterTypeKalman } FilterType;
			
		IMU() : SubSystem()
		{
			_imuHardware = NULL;
			_imuFilter = NULL;
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 	
			SubSystem::initialize(frequency, offset);

			if (_imuHardware)
			{
				_imuHardware->initialize();
			}
			
			if (_imuFilter)
			{
				_imuFilter->initialize();
			}
		}
		
		void setHardwareType(const HardwareType hardwareType)
		{
			switch (hardwareType)
			{	
				case HardwareTypeOilPan:
				{
					_imuHardware = new OilpanIMU();
					break;
				}
				default:
				{
					serialcoms.debugPrintln("ERROR: Unknown IMU Hardware type selected.");
					break;
				}
			}
		}
		
		void setFilterType(const FilterType filterType)
		{
			switch (filterType)
			{
				case FilterTypeSimpleAccellOnly:
				{
					_imuFilter = new SimpleAccellOnlyIMUFilter();
					break;
				}
				
				case FilterTypeComplimentry:
				{
					_imuFilter = new ComplimentryMUFilter();
					break;
				}
				
				case FilterTypeSimplifiedKalman:
				{
					_imuFilter = new SimplifiedKalmanIMUFilter();
					break;
				}
				
				case FilterTypeDCM:
				{
					_imuFilter = new DCMIMUFilter();
					break;
				}
				
				case FilterTypeKalman:
				{
					_imuFilter = new KalmanIMUFilter();
					break;
				}
								
				default:
				{
					serialcoms.debugPrintln("ERROR: Unknown IMU Filter type selected.");
					break;
				}
			}
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				if (_imuHardware)
				{
					_imuHardware->process(currentTime);

					if (_imuFilter)
					{
						_imuFilter->setCurrentReadings(_imuHardware->getCurrentReadings());
						_imuFilter->filter(currentTime);
						
						//Pitch is defined as the angle between the aircraft's longitudinal axis and the local horizontal plane (positive for nose up). 
						//Roll is defined as the angle about the longitudinal axis between the local horizontal plane and the actual flight orientation (positive for right wing down).
					}
					
					
					/*int rollTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel1);
					int pitchTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel2);
					int throttleTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel3);
					int yawTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel4);
					
					//DEBUGSERIALPRINT(throttleTransmitterCommand);
					//DEBUGSERIALPRINT(",");
					//DEBUGSERIALPRINT(yawTransmitterCommand);
					//DEBUGSERIALPRINTLN("");
					
					//Check for some basic "Command" stick positions
					if ((throttleTransmitterCommand < 1100 && yawTransmitterCommand < 1100) && (rollTransmitterCommand > 1900 && pitchTransmitterCommand < 1100))
					{
						this->calibrateZero();
					}*/
				}
			}
		}
		
		void calibrateZero()
		{
			if (_imuHardware)
			{
				_imuHardware->calibrateZero();
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
		
		const float currentRollAngleInDegrees()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentRollAngleInDegrees();
			}
			
			return 0;
		}
		
		const float currentPitchAngleInDegrees()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentPitchAngleInDegrees();
			}
			
			return 0;
		}
		
		const float currentRollAngleInRadians()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentRollAngleInRadians();
			}
			
			return 0;
		}
		
		const float currentPitchAngleInRadians()
		{
			if (_imuFilter)
			{
				return _imuFilter->currentPitchAngleInRadians();
			}
			
			return 0;
		}
};