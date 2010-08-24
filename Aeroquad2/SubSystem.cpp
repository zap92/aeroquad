#include <WProgram.h>

#include "SubSystem.h"

SubSystem* SubSystem::getInstance()
{
	return _staticInstance;
}

SubSystem::SubSystem() 
{
	_enabled = 0;
	_lastRunTime = 0;
	_frequency = 0;
	
	_previousRunTimeSampleCount = 0;
	_previousRunTimeTotal = 0;
	
	_staticInstance = this;
}

void SubSystem::initialize(const unsigned int frequency, const unsigned int offset) 
{ 
	_frequency = frequency;
	_lastRunTime = offset;
	_enabled = 1;
}

void SubSystem::enable()
{
	_enabled = 1;
}

void SubSystem::disable()
{
	_enabled = 0;
}

unsigned int SubSystem::enabled()
{
	return _enabled;
}

unsigned int SubSystem::_canProcess(const unsigned long currentTime)
{
	if (_enabled == 1)
	{
		if (currentTime > (_lastRunTime + _frequency))
		{
			 _lastRunTime = currentTime;
			 return 1;
		}
	}   
	return 0;
}

void SubSystem::process(const unsigned long currentTime) 
{

}

void SubSystem::recordProcessingStartTime()
{
	_processingStartTime = millis();
}

void SubSystem::recordProcessingEndTime()
{
	_previousRunTimeTotal += (millis() - _processingStartTime);
	_previousRunTimeSampleCount++;
		
	if (_previousRunTimeSampleCount >= 1000)
	{		
		_previousRunTimeTotal = (float) _previousRunTimeTotal / (float)_previousRunTimeSampleCount;
		_previousRunTimeSampleCount = 1;
	}			
}

const float SubSystem::averageProcessingTime()
{
	if (_previousRunTimeSampleCount > 0)
	{
		return (float) _previousRunTimeTotal / (float) _previousRunTimeSampleCount;
	}
	
	return 0;
}

SubSystem* SubSystem::_staticInstance = NULL;




