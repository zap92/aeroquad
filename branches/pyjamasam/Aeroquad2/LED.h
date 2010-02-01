#include "SubSystem.h"

class LED : public SubSystem
{
	public:
		typedef enum { LEDPatternOff = 0, LEDPatternInsideOut = 1, LEDPatternOutsideIn = 2} LEDPattern;
		
	private: 
		bool _readyLEDState;
		
		int _patternState;
		
		LEDPattern _activePattern;
		
	public:
		LED() : SubSystem()
		{
			_readyLEDState = false;
			_activePattern = LEDPatternOff;
			
			_patternState = 0;
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);

			for(int i = 24; i < 32; i++)
			{
				pinMode(i, OUTPUT);
				digitalWrite(i, LOW);				
			}
		}
		
		void setPatternType(const LEDPattern patternType)
		{
			_activePattern = patternType;
			_patternState = 0;
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
				
				//Process any light patterns that are needed
				switch (_activePattern)
				{
					case LEDPatternOff:
					{
						//No pattern active.
						break;
					}
					
					case LEDPatternInsideOut:
					{
						//sweep from the center of the quad to the edge of the arms.
						if (_patternState == 0)
						{
							digitalWrite(24, HIGH);
							digitalWrite(26, HIGH);
							digitalWrite(28, HIGH);
							
							digitalWrite(25, LOW);
							digitalWrite(27, LOW);
							digitalWrite(29, LOW);
						}
						else if (_patternState == 1)
						{
							digitalWrite(24, HIGH);
							digitalWrite(26, HIGH);
							digitalWrite(28, HIGH);
							
							digitalWrite(25, HIGH);
							digitalWrite(27, HIGH);
							digitalWrite(29, HIGH);
						}
						else if (_patternState == 2)
						{
							digitalWrite(24, LOW);
							digitalWrite(26, LOW);
							digitalWrite(28, LOW);
							
							digitalWrite(25, HIGH);
							digitalWrite(27, HIGH);
							digitalWrite(29, HIGH);
						}
						else if (_patternState == 3)
						{
							digitalWrite(24, LOW);
							digitalWrite(26, LOW);
							digitalWrite(28, LOW);
							
							digitalWrite(25, LOW);
							digitalWrite(27, LOW);
							digitalWrite(29, LOW);
						}
						else if (_patternState == 4)
						{
							_patternState = -1;
						}
						
						_patternState++;
						
						break;
					}
					
					case LEDPatternOutsideIn:
					{
						//Sweep from the outside of the quad towards the center
						if (_patternState == 0)
						{
							digitalWrite(24, LOW);
							digitalWrite(26, LOW);
							digitalWrite(28, LOW);
							
							digitalWrite(25, HIGH);
							digitalWrite(27, HIGH);
							digitalWrite(29, HIGH);
						}
						else if (_patternState == 1)
						{
							digitalWrite(24, HIGH);
							digitalWrite(26, HIGH);
							digitalWrite(28, HIGH);
							
							digitalWrite(25, HIGH);
							digitalWrite(27, HIGH);
							digitalWrite(29, HIGH);
						}
						else if (_patternState == 2)
						{
							digitalWrite(24, HIGH);
							digitalWrite(26, HIGH);
							digitalWrite(28, HIGH);
							
							digitalWrite(25, LOW);
							digitalWrite(27, LOW);
							digitalWrite(29, LOW);
						}
						else if (_patternState == 3)
						{
							digitalWrite(24, LOW);
							digitalWrite(26, LOW);
							digitalWrite(28, LOW);
							
							digitalWrite(25, LOW);
							digitalWrite(27, LOW);
							digitalWrite(29, LOW);
						}
						else if (_patternState == 4)
						{
							_patternState = -1;
						}
						
						_patternState++;
						
						break;
					}
				}
			}
		}
};