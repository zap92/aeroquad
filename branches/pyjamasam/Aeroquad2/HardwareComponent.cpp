#include <WProgram.h>

#include "HardwareComponent.h"

HardwareComponent::HardwareComponent() 
{
}

void HardwareComponent::_initialize()
{
}

void HardwareComponent::initialize() 
{ 
	this->_initialize(); 
}

void HardwareComponent::process(const unsigned long currentTime)
{
	//this method should be called after the subclass is done its processing
	_lastProcessTime = currentTime;
}

void AnalogInHardwareComponent::_scaleRawReadingsToEngineeringValues()
{
	static float tmpf;	        //temporary variable
	
	float dacMvValue = this->referenceVoltage() / (pow(2,this->adcPrecision()) - 1);
	
	for (int i = 0; i <= _inputCount; i++)
	{
		if (_inputConfigurations[i].inputInUse)
		{
			int rawAnalogValue = _rawReadings[i];
			tmpf = (float)rawAnalogValue * dacMvValue;
			tmpf -= _inputConfigurations[i].zeroLevel; 		 		//voltage relative to zero level (mV)
		  	tmpf /= _inputConfigurations[i].sensitivity;    		//input sensitivity in mV/Unit
			if (_inputConfigurations[i].invert)
			{						
		  		tmpf *= -1;  		//invert axis value according to configuration 
			}
			_lastReadings[i] = tmpf;
		}
	}
}


AnalogInHardwareComponent::AnalogInHardwareComponent(const unsigned int inputCount) : HardwareComponent()
{
	this->_referenceVoltage = 5.0;
	this->_adcPrecision = 10;
	
	_inputCount = inputCount;
	
	//Initalize ALL the inputs, even if we aren't going to use them
	for (int i = 0; i < MAXINPUTCOUNT; i++)
	{
		_inputConfigurations[i].inputInUse = false;
		_inputConfigurations[i].invert = false;
		
		_lastReadings[i] = 0.0f;
		_rawReadings[i] = 0;
	}
}

void AnalogInHardwareComponent::setReferenceVoltage(const float referenceVoltage)
{
	_referenceVoltage = referenceVoltage;
}
const float AnalogInHardwareComponent::referenceVoltage()
{
	return _referenceVoltage;
}

void AnalogInHardwareComponent::setAdcPrecision(const int adcPrecision)
{
	_adcPrecision = adcPrecision;
}

const int AnalogInHardwareComponent::adcPrecision()
{
	return _adcPrecision;
}

const int AnalogInHardwareComponent::readRawValue(const unsigned int axis)
{
	return analogRead(axis);
}

void AnalogInHardwareComponent::process(const unsigned long currentTime)
{
	for (int i = 0; i < _inputCount; i++)
	{
		if (_inputConfigurations[i].inputInUse)
		{
			_rawReadings[i] = this->readRawValue(_inputConfigurations[i].hardwarePin);
		}
	}
	
	this->_scaleRawReadingsToEngineeringValues();		
	HardwareComponent::process(currentTime);
}




