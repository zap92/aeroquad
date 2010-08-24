#include <WProgram.h>

#include "LED.h"

#define GREENLED 37
#define YELLOWLED 36
#define REDLED 35

LED::LED() : SubSystem()
{
	_readyLEDState = false;
	_activePattern = PatternOff;
	
	_patternState = 0;
}

void LED::initialize(const unsigned int frequency, const unsigned int offset) 
{ 
	SubSystem::initialize(frequency, offset);

	pinMode(REDLED, OUTPUT);
	digitalWrite(REDLED, LOW);				

	pinMode(YELLOWLED, OUTPUT);
	digitalWrite(YELLOWLED, LOW);				

	pinMode(GREENLED, OUTPUT);
	digitalWrite(GREENLED, LOW);				
}

void LED::setPatternType(const LEDPattern patternType)
{
	_activePattern = patternType;
	_patternState = 0;
}

void LED::process(const unsigned long currentTime)
{
	if (this->_canProcess(currentTime))
	{
		SubSystem::recordProcessingStartTime();
		
		if (_readyLEDState)
		{
			//digitalWrite(31, HIGH);
		}
		else
		{
			//digitalWrite(31, LOW);
		}
		
		_readyLEDState = !_readyLEDState;
		
		//Process any light patterns that are needed
		switch (_activePattern)
		{
			case PatternOff:
			{
				//No pattern active.
				break;
			}
			
			case PatternChase:
			{
				//leds will chase each other...  2 on 1 off..
				if (_patternState == 0)
				{
					digitalWrite(REDLED, HIGH);				
					digitalWrite(YELLOWLED, LOW);				
					digitalWrite(GREENLED, LOW);
				}
				else if (_patternState == 1)
				{
					digitalWrite(REDLED, LOW);				
					digitalWrite(YELLOWLED, HIGH);				
					digitalWrite(GREENLED, LOW);
				}
				else if (_patternState == 2)
				{
					digitalWrite(REDLED, LOW);				
					digitalWrite(YELLOWLED, LOW);				
					digitalWrite(GREENLED, HIGH);

					_patternState = -1;
				}
				
				_patternState++;
				
				break;
			}
			
			case PatternSweep:
			{
				//Sweep from the outside of the quad towards the center
				if (_patternState == 0)
				{
					digitalWrite(REDLED, HIGH);				
					digitalWrite(YELLOWLED, LOW);				
					digitalWrite(GREENLED, LOW);
				}
				else if (_patternState == 1)
				{
					digitalWrite(REDLED, LOW);				
					digitalWrite(YELLOWLED, HIGH);				
					digitalWrite(GREENLED, LOW);
				}
				else if (_patternState == 2)
				{
					digitalWrite(REDLED, LOW);				
					digitalWrite(YELLOWLED, LOW);				
					digitalWrite(GREENLED, HIGH);
				}
				else if (_patternState == 3)
				{
					//Nothing in this state.  Keep it like it was.
				}
				else if (_patternState == 4)
				{
					digitalWrite(REDLED, LOW);				
					digitalWrite(YELLOWLED, HIGH);				
					digitalWrite(GREENLED, LOW);
				}
				else if (_patternState == 5)
				{
					digitalWrite(REDLED, HIGH);				
					digitalWrite(YELLOWLED, LOW);				
					digitalWrite(GREENLED, LOW);
					
					_patternState = -1;
				}
				
				_patternState++;
				
				break;
			}
			
			case PatternAlternate:
			{
				if (_patternState == 0)
				{
					digitalWrite(REDLED, HIGH);				
					digitalWrite(YELLOWLED, LOW);				
					digitalWrite(GREENLED, HIGH);
				}
				else if (_patternState == 1)
				{
					digitalWrite(REDLED, LOW);				
					digitalWrite(YELLOWLED, HIGH);				
					digitalWrite(GREENLED, LOW);
					
					_patternState = -1;
				}
				
				_patternState++;
				break;
			}
		}
	
		SubSystem::recordProcessingEndTime();
		
	}
}

LED led;