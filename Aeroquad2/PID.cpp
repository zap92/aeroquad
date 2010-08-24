#include <WProgram.h>


#include "PID.h"

PID::PID()
{
	_firstPass = true;
	
	this->reset();			
}

void PID::setParameters(const float p, const float i, const float d, const float windupGuard)
{
	_P = p;
	_I = i;
	_D = d;
	_windupGuard = windupGuard;
}

void PID::setP(const float p)
{
	_P = p;
}	
const float PID::getP()
{
	return _P;
}

void PID::setI(const float i)
{
	_I = i;
}
const float PID::getI()
{
	return _I;
}		

void PID::setD(const float d)
{
	_D = d;
}
const float PID::getD()
{
	return _D;
}			

void PID::setWindupGuard(float windupGuard)
{
	_windupGuard = windupGuard;
}
const float PID::getWindupGuard()
{
	return _windupGuard;
}

void PID::reset()
{
	_integeratedError = 0;	
}

const float PID::update(const float targetValue, const float currentValue)
{
	float error = targetValue - currentValue;
	
	if (_firstPass)
	{
		_firstPass = false;
		return constrain(error, -_windupGuard, _windupGuard);
	}
	else
	{
			// Calculate error integral and limit integrated_error to +/-windup_guard  
		_integeratedError += error;
		_integeratedError = constrain(_integeratedError, -_windupGuard, _windupGuard);

			// Calculate error derivative   
			float dTerm = _D * (error - _lastError);
			_lastError = error;

			return (_P * error) + (_I * _integeratedError) + dTerm;
		}
}


