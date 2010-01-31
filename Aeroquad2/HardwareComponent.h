#ifndef __HARDWARECOMPONENT_H__
#define __HARDWARECOMPONENT_H__

class HardwareComponent
{
	private:
		unsigned long _lastProcessTime;
		
		static float _referenceVoltage;

	public:
		static void setReferenceVoltage(const float referenceVoltage)
		{
			_referenceVoltage = referenceVoltage;
		}
		static const float getReferenceVoltage()
		{
			return _referenceVoltage;
		}
		
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

float HardwareComponent::_referenceVoltage = 0;

#endif



