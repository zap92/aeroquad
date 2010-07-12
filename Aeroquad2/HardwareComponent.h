#ifndef __HARDWARECOMPONENT_H__
#define __HARDWARECOMPONENT_H__

class HardwareComponent
{
	private:
		unsigned long _lastProcessTime;
		
		float _referenceVoltage;
		int _dacPrecision;

	public:
		void setReferenceVoltage(const float referenceVoltage)
		{
			_referenceVoltage = referenceVoltage;
		}
		const float getReferenceVoltage()
		{
			return _referenceVoltage;
		}
		
		void setDacPrecision(const int dacPrecision)
		{
			_dacPrecision = dacPrecision;
		}
		
		const int getDacPrecision()
		{
			return _dacPrecision;
		}
		
		HardwareComponent() 
		{
			this->_referenceVoltage = 5.0;
			this->_dacPrecision = 10;
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

#endif



