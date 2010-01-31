#include "SubSystem.h"

class LED : public SubSystem
{
	private: 
		bool _readyLEDState;
		
	public:
		LED() : SubSystem()
		{
			_readyLEDState = false;
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
			
			pinMode(31, OUTPUT);
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				if (_readyLEDState)
				{
					digitalWrite(31, HIGH);
				}
				else
				{
					digitalWrite(31, LOW);
				}
				
				_readyLEDState = !_readyLEDState;
			}
		}
};