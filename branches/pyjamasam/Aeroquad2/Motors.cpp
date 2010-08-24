#include <WProgram.h>

#include "Motors.h"

#include <APM_RC.h>
#include "Receiver.h"
#include "FlightControl.h"

void MotorHardware::zeroOutputs() 
{
	//Push every output down below where anything should happen
	for (int i = 0; i < _motorCount; i++)
	{
		_motorOutput[i] = _disarmedMotorCommand;
	}
}

void MotorHardware::writeOutputs()
{
	//default implimentation does nothing
}


MotorHardware::MotorHardware(unsigned int motorCount) : HardwareComponent()
{
	_motorCount = (int)fmin(motorCount, 8);
	_armed = false;
}

void MotorHardware::initialize()
{
	this->zeroOutputs();
	HardwareComponent::initialize();
}	

void MotorHardware::process(const unsigned long currentTime)
{
	if (_armed)
	{
		//Push the outputs out
		this->writeOutputs();
	}
	else
	{
		//Force the outputs to zero to ensure they are off
		this->zeroOutputs();
		this->writeOutputs();
	}
	HardwareComponent::process(currentTime);	
}

void MotorHardware::processForESCCalibration()
{
	this->writeOutputs();	
}

void MotorHardware::arm()
{
	if (!_armed)
	{	
		this->zeroOutputs();
		_armed = true;
	}
}

void MotorHardware::disarm()
{
	if (_armed)
	{	
		this->zeroOutputs();
		_armed = false;
	}
}

const bool MotorHardware::armed()
{
	return _armed;		
}

void MotorHardware::setMotorOutput(const unsigned int motorIndex, const int motorCommand)
{
	if (motorIndex < _motorCount)
	{				
		//Ensure that we don't have a value past the max a motor can actually support
		_motorOutput[motorIndex] = constrain(_minimumMotorCommand + constrain(motorCommand, 0, 1000), _minimumMotorCommand, _maximumMotorCommand);
	}
}

void MotorHardware::setAllMotorOutputToMax()
{
	for (unsigned int i = 0; i < _motorCount; i++)
	{
		_motorOutput[i] = _maximumMotorCommand;
	}	
}

void MotorHardware::setAllMotorOutputToDisarmed()
{
	for (unsigned int i = 0; i < _motorCount; i++)
	{
		_motorOutput[i] = _disarmedMotorCommand;
	}	
}


//Uses the APM RC output library to drive the outputs
void MotorHardwareAPM::writeOutputs()
{			
	APM_RC.OutputCh(0, _motorOutput[0]);
	APM_RC.OutputCh(1, _motorOutput[1]);
	APM_RC.OutputCh(2, _motorOutput[2]);
	APM_RC.OutputCh(3, _motorOutput[3]);
	  
	// InstantPWM
	APM_RC.Force_Out0_Out1();
	APM_RC.Force_Out2_Out3();
}
		
MotorHardwareAPM::MotorHardwareAPM() : MotorHardware(4)
{
}

//For just debuggging the output that should go to the motors
void MotorhardwareDebugSerial::writeOutputs()
{			
	Serial.print(_motorOutput[0]);
	Serial.print(",");
	Serial.print(_motorOutput[1]);
	Serial.print(",");
	Serial.print(_motorOutput[2]);
	Serial.print(",");
	Serial.print(_motorOutput[3]);
	Serial.println();
}
		
MotorhardwareDebugSerial::MotorhardwareDebugSerial() : MotorHardware(4)
{	
}


Motors::Motors() : SubSystem()
{
	
}

void Motors::initialize(const unsigned int frequency, const unsigned int offset) 
{ 
	SubSystem::initialize(frequency, offset);
	
	if (_motorHardware)
	{
		_motorHardware->initialize();
	}
	
	serialcoms.shell()->registerKeyword("calibrateESCs", "calibrateESCs", _calibrateESCs);
}

void Motors::setHardwareType(const HardwareType hardwareType)
{
	switch (hardwareType)
	{
		case HardwareTypeAPM:
		{
			_motorHardware = new MotorHardwareAPM();
			break;
		}
		
		case HardwareTypeDebugSerial:
		{
			_motorHardware = new MotorhardwareDebugSerial();
			break;
		}
		default:
		{
			serialcoms.println("ERROR: Unknown Motor hardware type selected.");	
			break;
		}
	}
}

void Motors::setOrientationType(const OrientationType orientationType)
{
	_orientationType = orientationType;
}
		
void Motors::process(const unsigned long currentTime)
{
	if (this->_canProcess(currentTime))
	{
		if (_motorHardware)
		{
			int throttleTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel3);
			int yawTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel4);										
								
			//Check for some basic "Command" stick positions
			if (throttleTransmitterCommand < -480)
			{
				if (yawTransmitterCommand < -480)
				{
					//Throttle/Yaw Stick is in the bottom left position.  Disarm the motors
					_motorHardware->disarm();
				}
				else if (yawTransmitterCommand > 480)
				{
					//Throttle/Yaw stick is int he bottom right position.  Arm the motors
					_motorHardware->arm();
				}
			}
			
			//Main control system.
			float flightControlRollCommand = flightcontrol.rollCommand();
			float flightControlPitchCommand = flightcontrol.pitchCommand();
			float flightControlYawCommand = flightcontrol.yawCommand();
			float flightControlThrottleCommand = flightcontrol.throttleCommand();										
			
			int mixedMotorCommand0 = 0;
			int mixedMotorCommand1 = 0;
			int mixedMotorCommand2 = 0;
			int mixedMotorCommand3 = 0;
			
			//Calculate the mix for each of the motors.
			switch (_orientationType)
			{
				case QuadPlusMotor:
				{							
					// Right motor		(CCW)
					mixedMotorCommand0 = flightControlThrottleCommand - flightControlRollCommand - flightControlYawCommand;
					
					// Left motor		(CCW)
					mixedMotorCommand1 = flightControlThrottleCommand + flightControlRollCommand - flightControlYawCommand;
					
					// Front motor		(CW)
					mixedMotorCommand2 = flightControlThrottleCommand + flightControlPitchCommand + flightControlYawCommand;
					
					// Back motor		(CW)
					mixedMotorCommand3 = flightControlThrottleCommand - flightControlPitchCommand + flightControlYawCommand;
					break;
				}
				
				case QuadXMotor:
				{							
					// Right-Front motor		(CCW)				
					mixedMotorCommand0 = flightControlThrottleCommand - flightControlRollCommand  + flightControlPitchCommand - flightControlYawCommand;
					
					// Right-Back motor		(CW)
					mixedMotorCommand1 = flightControlThrottleCommand - flightControlRollCommand - flightControlPitchCommand + flightControlYawCommand;
					
					//Left-Front motor		(CCW)
					mixedMotorCommand2 = flightControlThrottleCommand + flightControlRollCommand + flightControlPitchCommand + flightControlYawCommand;
					
					// Left-Back motor		(CW)
					mixedMotorCommand3 = 	flightControlThrottleCommand + flightControlRollCommand - flightControlPitchCommand - flightControlYawCommand;				
					break;
				}
			}
			
			_motorHardware->setMotorOutput(0,mixedMotorCommand0);
			_motorHardware->setMotorOutput(1,mixedMotorCommand1);
			_motorHardware->setMotorOutput(2,mixedMotorCommand2);
			_motorHardware->setMotorOutput(3,mixedMotorCommand3);
																		
			_motorHardware->process(currentTime);
		}
	}
}

const bool Motors::armed()
{
	if (_motorHardware)
	{
		return _motorHardware->armed();
	}	
	else
	{
		return false;
	}	
}

MotorHardware* Motors::motorHardware()
{
	return _motorHardware;
}
		
		
	

const ArduinoShellCallback::callbackReturn _calibrateESCs(ArduinoShell &shell, const int argc, const char* argv[])
{
	MotorHardware *motorHardware = motors.motorHardware();
	
	if (motorHardware)
	{		
		//ensure that the hardware is disarmed as a percaution
		motorHardware->disarm();
			
		shell << "Starting ESC calibration" << endl;
		shell << "Please make sure there is no power applied to them. (Hit any key to continue)..." << endl;			
		while (shell.serialPort()->available() == 0) {}
		shell.serialPort()->read();
		
		motorHardware->setAllMotorOutputToMax();
		motorHardware->processForESCCalibration();
		
		shell << "Attach power to the ESCs now. (Hit any key to continue)..." << endl;		
		while (shell.serialPort()->available() == 0) {}
		shell.serialPort()->read();
		
		shell << " - Calibrating maximum value";
		for (byte i = 0; i < 4; i++)
		{
			delay(1000);
			shell << ".";
		}
		shell << " done" << endl;
		
		motorHardware->setAllMotorOutputToDisarmed();
		motorHardware->processForESCCalibration();
		shell << " - Calibrating minimum value";	
		for (byte i = 0; i < 4; i++)
		{
			delay(1000);
			shell << ".";
		}

		shell << " done" << endl;
		
		shell << "ESC calibration completed" << endl;
	}
	else
	{
		shell << "Error: No motor hardware to calibrate" << endl;
	}

	return ArduinoShellCallback::Success;
}

Motors motors;
