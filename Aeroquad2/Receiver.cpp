#include <WProgram.h>

#include <APM_RC.h>

#include "LanguageExtensions.h"

#include "Receiver.h"

#include "Settings.h"
#include "SerialComs.h"
#include "Motors.h"

ReceiverHardware::ReceiverHardware() : HardwareComponent()
{
	//get the receiver calibration data from the settings system
	const int *settingsReceiverCalibrationMax = settings.getReceiverCalibrationMax();
	const int *settingsReceiverCalibrationMin = settings.getReceiverCalibrationMin();
		
	for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
	{
		_channelMax[i] = settingsReceiverCalibrationMax[i];
		_channelMin[i] = settingsReceiverCalibrationMin[i];
	}
}

void ReceiverHardware::initialize()
{
	HardwareComponent::initialize();
}	

void ReceiverHardware::process(const unsigned long currentTime)
{
	//Scale the raw values to our sane -500 <-> 500 values;
	for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
	{
		_currentReceiverValues[i] = map(_currentRawReceiverValues[i], _channelMin[i], _channelMax[i], -500, 500);
	}
				
	HardwareComponent::process(currentTime);
}
		
const int ReceiverHardware::channelValue(ChannelIndex channelIndex)
{
	return _currentReceiverValues[channelIndex];
}


void ReceiverHardware::startCalibration()
{
	//Setup some dummy values so we can actually calibrate correctly.
	for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
	{
		_channelMax[i] = 0;
		_channelMin[i] = 3000;
	}		
}

void ReceiverHardware::performCalibrationCycle()
{	
	//let the hardware run so we get a new reading
	this->process(millis());
	
	for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
	{
		_channelMax[i] = fmax(_channelMax[i], _currentRawReceiverValues[i]);
		_channelMin[i] = fmin(_channelMin[i], _currentRawReceiverValues[i]);		
	}	
}

const int* ReceiverHardware::channelMaxValues()
{
	return _channelMax;
}

const int* ReceiverHardware::channelMinValues()
{
	return _channelMin;
}

void ReceiverHardware::calibrationComplete()
{
	//Store out our channel min and max values so we can load them from EEPROM later
	settings.setReceiverCalibrationMax(_channelMax);
	settings.setReceiverCalibrationMin(_channelMin);
	
	settings.save();
}


void ReceiverHardwareAPM::initialize()
{
	ReceiverHardware::initialize();
}	

void ReceiverHardwareAPM::process(const unsigned long currentTime)
{
	if (APM_RC.GetState())
	{
		//Only process new readings if we actually have new readings.
		for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
		{
			_currentRawReceiverValues[i] = APM_RC.InputCh(i);
		}
	}
	
	ReceiverHardware::process(currentTime);	
}


//Generic method to scale a normal stick value to a rate or angle.
const float Receiver::_scaleStickValue(const int stickValue, const float scaleValue)
{
	//We have a -500 to 500 range with the normal channel readout.
	return fmap(stickValue, -500, 500, -scaleValue, scaleValue);
}		


Receiver::Receiver() : SubSystem()
{
	
}

void Receiver::initialize(const unsigned int frequency, const unsigned int offset) 
{ 
	SubSystem::initialize(frequency, offset);
	
	if (_receiverHardware)
	{
		_receiverHardware->initialize();
	}
	
	serialcoms.shell()->registerKeyword("monitorReceiver", "monitorReceiver", _monitorReceiver , true);
	serialcoms.shell()->registerKeyword("calibrateReceiver", "calibrateReceiver", _calibrateReceiver);
}

void Receiver::setHardwareType(const HardwareType hardwareType)
{
	switch (hardwareType)
	{
		case HardwareTypeAPM:
		{
			_receiverHardware = new ReceiverHardwareAPM();
			break;
		}
		
		default:
		{
			serialcoms.println("ERROR: Unknown Receiver Hardware type selected.");					
			break;
		}
	}
}

void Receiver::process(const unsigned long currentTime)
{
	if (this->_canProcess(currentTime))
	{
		SubSystem::recordProcessingStartTime();
		if (_receiverHardware)
		{
			_receiverHardware->process(currentTime);
		}				
		SubSystem::recordProcessingEndTime();
	}
}

void Receiver::startCalibration()
{
	if (_receiverHardware)
	{
		_receiverHardware->startCalibration();
	}
}	

void Receiver::performCalibrationCycle()
{	
	if (_receiverHardware)
	{
		_receiverHardware->performCalibrationCycle();
	}
}

void Receiver::calibrationComplete()
{
	if (_receiverHardware)
	{
		_receiverHardware->calibrationComplete();
	}		
}

const int* Receiver::channelMaxValues()
{
	if (_receiverHardware)
	{
		return _receiverHardware->channelMaxValues();
	}
	return NULL;	
}

const int* Receiver::channelMinValues()
{
	if (_receiverHardware)
	{
		return _receiverHardware->channelMinValues();
	}
	return NULL;
}

//Accessor for current channel value
const int Receiver::channelValue(ReceiverHardware::ChannelIndex channelIndex)
{
	if (_receiverHardware)
	{
		return _receiverHardware->channelValue(channelIndex);
	}
	else
	{
		return 0;
	}
}

const float Receiver::channelValueAsAngleInRadians(ReceiverHardware::ChannelIndex channelIndex)
{
	//Convert the stick position into an angle (in this case radians)
	if (_receiverHardware)
	{
		return this->_scaleStickValue(_receiverHardware->channelValue(channelIndex), MAXIMUM_STICK_ANGLE_IN_RADIANS);
	}
	else
	{
		return 0;
	}
}
const float Receiver::channelValueAsAngleInDegrees(ReceiverHardware::ChannelIndex channelIndex)
{
	return ToDeg(this->channelValueAsAngleInRadians(channelIndex));
}


const float Receiver::channelValueAsRateInRadians(ReceiverHardware::ChannelIndex channelIndex)
{
	//Convert the stick position into an angle (in this case radians/s)
	if (_receiverHardware)
	{	
		return this->_scaleStickValue(_receiverHardware->channelValue(channelIndex), MAXIMUM_STICK_RATE_IN_RADIANS_PER_SECOND);
	}
	else 
	{
		return 0;
	}
}
const float Receiver::channelValueAsRateInDegrees(ReceiverHardware::ChannelIndex channelIndex)
{
	return ToDeg(this->channelValueAsRateInRadians(channelIndex));
}


const float Receiver::channelValueAsRateInMeters(ReceiverHardware::ChannelIndex channelIndex)
{
	//Convert the stick position into an rate (in this case m/s)
	if (_receiverHardware)
	{
		return this->_scaleStickValue(_receiverHardware->channelValue(channelIndex), MAXIMUM_STICK_RATE_IN_METERS_PER_SECOND);
	}
	else 
	{
		return 0;
	}
}

Receiver receiver;

const ArduinoShellCallback::callbackReturn _calibrateReceiver(ArduinoShell &shell, const int argc, const char* argv[])
{	
	//Check to ensure that we don't have armed motors before we do this (as its a disruptive action and takes cpu cycles from the main loop) and disarmed motors generally means not flying)
	if (motors.armed())
	{
	}
	else
	{		
		shell << "Starting Receiver calibration in 1 second..." << endl;
		delay(1000);	
		
		const int* channelMaxValues = receiver.channelMaxValues();
		const int* channelMinValues = receiver.channelMinValues();
		
		receiver.startCalibration();	
		
		while (1)
		{			
			unsigned int bytesAvailable = shell.serialPort()->available();
		
			if (bytesAvailable >= 1)
			{
				if (shell.serialPort()->read() == 'x')
				{
					//All done calibrating.  Time to bail out
					break;
				}
			}
			
			receiver.performCalibrationCycle();				
			
			for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
			{
				shell << _DEC(i + 1) << ":" << channelMaxValues[i] << "," << channelMinValues[i] << " ";
			}	
			
			shell << endl;
			delay(30);
		}
			
		shell << "Finishing Receiver calibration..." << endl;
		receiver.calibrationComplete();
		shell << "Receiver calibration completed" << endl;
	}
		
	return ArduinoShellCallback::Success;	
}

const ArduinoShellCallback::callbackReturn _monitorReceiver(ArduinoShell &shell, const int argc, const char* argv[])
{
	shell << receiver.channelValue(ReceiverHardware::Channel1) << ","
			<< receiver.channelValue(ReceiverHardware::Channel2) << ","
			<< receiver.channelValue(ReceiverHardware::Channel3) << ","
			<< receiver.channelValue(ReceiverHardware::Channel4) << ","
			<< receiver.channelValue(ReceiverHardware::Channel5) << ","
			<< receiver.channelValue(ReceiverHardware::Channel6) << ","
			<< receiver.channelValue(ReceiverHardware::Channel7) << ","
			<< receiver.channelValue(ReceiverHardware::Channel8) << endl;
		
	return ArduinoShellCallback::Success;
}

