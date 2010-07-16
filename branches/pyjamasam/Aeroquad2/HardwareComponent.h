#ifndef __HARDWARECOMPONENT_H__
#define __HARDWARECOMPONENT_H__

class HardwareComponent
{
	private:
		unsigned long _lastProcessTime;
		
	public:
		HardwareComponent() 
		{
		}

		void _initialize()
		{
		}

		virtual void initialize() 
		{ 
			this->_initialize(); 
		}
		
		virtual void process(const unsigned long currentTime)
		{
			//this method should be called after the subclass is done its processing
			_lastProcessTime = currentTime;
		}
};

#define MAXINPUTCOUNT 20
class AnalogInHardwareComponent : public HardwareComponent
{
	private:
		float _referenceVoltage;
		int _adcPrecision;
		
	protected:
		float _lastReadings[MAXINPUTCOUNT];
		int _rawReadings[MAXINPUTCOUNT];
		
		int _inputCount;

		typedef struct {					
			bool inputInUse;
			bool invert;    				//invert input
		  	int zeroLevel;     				// zero level (mV) @ 0
		  	float sensitivity;   			// input sensitivity (mv/unit) 
			int hardwarePin;
		} inputConfiguration;

		inputConfiguration _inputConfigurations[MAXINPUTCOUNT];
		
	private:
		void _scaleRawReadingsToEngineeringValues()
		{
			static float tmpf;	        //temporary variable
			
			float dacMvValue = this->referenceVoltage() / (pow(2,this->adcPrecision()) - 1);
			
			for (int i = 0; i <= _inputCount; i++)
			{
				if (_inputConfigurations[i].inputInUse)
				{
					int rawAnalogValue = _rawReadings[i];
					tmpf = (float)rawAnalogValue * dacMvValue;
					tmpf -= _inputConfigurations[i].zeroLevel; 		 		//voltage relative to zero level (mV)
				  	tmpf /= _inputConfigurations[i].sensitivity;    		//input sensitivity in mV/Unit
					if (_inputConfigurations[i].invert)
					{						
				  		tmpf *= -1;  		//invert axis value according to configuration 
					}
					_lastReadings[i] = tmpf;
				}
			}
		}

	public:
		AnalogInHardwareComponent(const unsigned int inputCount) : HardwareComponent()
		{
			this->_referenceVoltage = 5.0;
			this->_adcPrecision = 10;
			
			_inputCount = inputCount;
			
			//Initalize ALL the inputs, even if we aren't going to use them
			for (int i = 0; i < MAXINPUTCOUNT; i++)
			{
				_inputConfigurations[i].inputInUse = false;
				_inputConfigurations[i].invert = false;
				
				_lastReadings[i] = 0.0f;
				_rawReadings[i] = 0;
			}
		}
		
		void setReferenceVoltage(const float referenceVoltage)
		{
			_referenceVoltage = referenceVoltage;
		}
		const float referenceVoltage()
		{
			return _referenceVoltage;
		}

		void setAdcPrecision(const int adcPrecision)
		{
			_adcPrecision = adcPrecision;
		}

		const int adcPrecision()
		{
			return _adcPrecision;
		}
		
		virtual const int readRawValue(const unsigned int axis)
		{
			return analogRead(axis);
		}
		
		virtual void process(const unsigned long currentTime)
		{
			for (int i = 0; i < _inputCount; i++)
			{
				if (_inputConfigurations[i].inputInUse)
				{
					_rawReadings[i] = this->readRawValue(_inputConfigurations[i].hardwarePin);
				}
			}
			
			this->_scaleRawReadingsToEngineeringValues();		
			HardwareComponent::process(currentTime);
		}
};

#endif



