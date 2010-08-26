#include <WProgram.h>


#include "PID.h"

PID::PID()
{
	_firstPass = true;
	
	this->reset();			
}

void PID::setParameters(float *p, float *i, float *d, float *windupGuard)
{
	_P = p;
	_I = i;
	_D = d;
	_windupGuard = windupGuard;
}

void PID::reset()
{
	_integral = 0;	
}

const float PID::update(const float targetValue, const float currentValue, unsigned long currentTime)
{	
	unsigned long deltaTime = currentTime - _previousTime;
	_previousTime = currentTime;

	float error = targetValue - currentValue;
	
	if (_firstPass)
	{
		_firstPass = false;
		return constrain(error, -*_windupGuard, *_windupGuard);
	}
	else
	{
		// Calculate integral and limit integrated_error to +/-windup_guard  
		_integral  += (error * deltaTime);
		_integral = constrain(_integral, -*_windupGuard, *_windupGuard);

		// Calculate derivative   
		float derivative   = (error - _lastError) / deltaTime;		
		
		_lastError = error;

		return (*_P * error) + (*_I * _integral) + (*_D * derivative);
	}
}


