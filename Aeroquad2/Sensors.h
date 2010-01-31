#include "SubSystem.h"

class Sensors : public SubSystem
{
	private:
	
		unsigned int _altitudeFromPressure;
		unsigned int _altitudeFromUltrasonic;
		
		int _headingFromCompass;
		
		float _voltage;
		float _current;
	
	public:
		Sensors() : SubSystem()
		{
			_altitudeFromPressure = 0;
			_altitudeFromUltrasonic = 0;
			
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
				
				//Read and process pressure reading
				
				//Read and process ultrasonic reading
				
				//Read and process current and voltage
			}
		}
		
		
		//accessors for sensor values
		
		//Altitude
		const unsigned int altitudeFromPressure()
		{
			return _altitudeFromPressure;
		}
		
		const unsigned int altitudeFromUltrasonic()
		{
			return _altitudeFromUltrasonic;
		}
		
		//Heading
		const int headingFromCompass()
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