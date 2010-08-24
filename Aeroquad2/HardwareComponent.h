#ifndef __HARDWARECOMPONENT_H__
#define __HARDWARECOMPONENT_H__

class HardwareComponent
{
	private:
		unsigned long _lastProcessTime;
		
	public:
		HardwareComponent();

		void _initialize();

		virtual void initialize();		
		virtual void process(const unsigned long currentTime);
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
		void _scaleRawReadingsToEngineeringValues();

	public:
		AnalogInHardwareComponent(const unsigned int inputCount);
		
		void setReferenceVoltage(const float referenceVoltage);
		const float referenceVoltage();

		void setAdcPrecision(const int adcPrecision);

		const int adcPrecision();
		
		virtual const int readRawValue(const unsigned int axis);		
		virtual void process(const unsigned long currentTime);
};

#endif




