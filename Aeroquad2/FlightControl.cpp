#include <WProgram.h>

#include "FlightControl.h"

#include "Receiver.h"
#include "IMU.h"
#include "LED.h"
#include "Settings.h"

		
void FlightControl::_processAcrobaticFlight(const unsigned long currentTime)
{			
	//All controls are rate based.  The simplest form of control.
	_throttleCommand = receiver.channelValue(ReceiverHardware::Channel3);
	
	//use some pids to get the command we need to apply to a channel.
	_rollCommand = _rollRatePID.update(receiver.channelValueAsRateInRadians(ReceiverHardware::Channel1), imu.rollRateInRadians());
	_pitchCommand = _pitchRatePID.update(receiver.channelValueAsRateInRadians(ReceiverHardware::Channel2), imu.pitchRateInRadians());
	_yawCommand = _yawRatePID.update(receiver.channelValueAsRateInRadians(ReceiverHardware::Channel4), imu.yawRateInRadians());
	
	//normalize the resaultant channel commands to somthing thats common (for the motor controls (0-1000))	
	_throttleCommand += 500.0;
	
	//Scale the outputs from the PID loops up a bit.  The rest of the tuning will be performed via the PID settings.
	_rollCommand *= 100.0;
	_pitchCommand *= 100.0;
	_yawCommand *= 100.0;						
}


void FlightControl::_processStableRollAndPitch(const float expectedRollAngle, const float expectedPitchAngle)
{
	_rollCommand = _rollAnglePID.update(receiver.channelValueAsAngleInRadians(ReceiverHardware::Channel1), expectedRollAngle);
	_pitchCommand = _pitchAnglePID.update(receiver.channelValueAsAngleInRadians(ReceiverHardware::Channel2), expectedPitchAngle);
	
	//Normalize the channel commands to somthing thats common (for the motor controls (0-1000))
	//TODO: 
}

void FlightControl::_processStableFlight(const unsigned long currentTime)
{
	//Roll and pitch are angle based, yaw stays rate based (cause we can't actualy do angle based for yaw) but includes heading hold now
	//and throttle becomes altitude based with hold
	//Software is assisting flight now
	this->_processStableRollAndPitch(imu.rollAngleInRadians(), imu.pitchAngleInRadians());
	
	
	//Process Yaw and deal with heading hold if needed
	const float transmitterYawRateInRadians = receiver.channelValueAsRateInRadians(ReceiverHardware::Channel4);
	
	//Yaw is always a rate, case we can't always "center" based on the stick like we can with roll and pitch (would ne north all the time)
	if (fabs(transmitterYawRateInRadians) > 0.122173048) // 7 deg/s  (this is the dead zone when moving the stick)
	{
		//Pilot is requesting Yaw command (disabling heading hole and letting the quad yaw to the pilots desired heading)
		_yawCommand = _yawRatePID.update(transmitterYawRateInRadians, imu.yawRateInRadians());	
		
		//Capture when the last time a yaw command was used.
		//This is used to trigger the heading hold after a short time delay (to allow a final bit of "coast" before it locks on)
		_lastYawCommandTime = currentTime;							
		_headingHoldSetPoint = 0;
		_headingHoldActive = false;
	}
	else
	{	
		//Heading hold enabled and no yaw command from the pilot.
		if (!_headingHoldActive)
		{
			//heading hold is not yet activated yet.  
			//Check to see if we have waited long enough to capture the current heading to lock onto 
			if ((currentTime - _lastYawCommandTime) > 500)
			{
				//We have waited long enough.  So lets capture the heading and lock onto it.
				_headingHoldSetPoint = imu.yawAngleInRadians();						
				_headingHoldActive = true;
			}

			//We are still rate based.  Untill the next processing cycle after we lock on to a heading
			_yawCommand = _yawRatePID.update(transmitterYawRateInRadians, imu.yawRateInRadians());
		}
		else
		{
			//We have a heading to hold so we switch to an angle based PID to now try and keep that heading.
			_yawCommand = _headingPID.update(_headingHoldSetPoint, imu.yawAngleInRadians());	
		}
	}
	
	
	//Process throttle and deal with altitude hold if needed
	float transmitterThrottleRateInMeters = receiver.channelValueAsRateInMeters(ReceiverHardware::Channel3);
	
	//Throttle is a rate, case we can't always "center" based on the stick like we can with roll and pitch (would ne north all the time)
	if (fabs(transmitterThrottleRateInMeters) > 0.01) 
	{
		//Pilot is requesting Yaw command (disabling heading hole and letting the quad yaw to the pilots desired heading)
		_yawCommand = _throttleRatePID.update(transmitterThrottleRateInMeters, imu.altitudeRateInMeters());
		
		//Capture when the last time a yaw command was used.
		//This is used to trigger the heading hold after a short time delay (to allow a final bit of "coast" before it locks on)
		_lastThrottleCommandTime = currentTime;							
		_altitudeHoldActive = false;
	}
	else
	{	
		//Altitude hold enabled and no throttle command from the pilot.
		if (!_altitudeHoldActive)
		{
			//altitude hold is not yet activated yet.  
			//Check to see if we have waited long enough to capture the current altitude to lock onto 
			if ((currentTime - _lastThrottleCommandTime) > 1500)
			{
				//We have waited long enough.  So lets capture the heading and lock onto it.
				_altitudeHoldSetPoint = imu.altitudeInMeters();					
				_altitudeHoldActive = true;
			}
		
			//We are still rate based.  Untill the next processing cycle after we lock on to a altitude
			_throttleCommand = _throttleRatePID.update(transmitterThrottleRateInMeters, imu.altitudeRateInMeters());
		}
		else
		{
			//We have an altitude to hold so we switch to an altitude based PID to now try and keep that altitude.
			_throttleCommand = _altitudePID.update(_altitudeHoldSetPoint, imu.altitudeInMeters());	
		}
	}
}

void FlightControl::_processAutonomousFlight(const unsigned long currentTime)
{
	//Navigation subsystem does most of the work.  We just aim to keep the quad stable
	//So infact its farily simple for the purposes of flight control.
	this->_processStableRollAndPitch(imu.rollAngleInRadians(), imu.pitchAngleInRadians());
}
		
FlightControl::FlightControl() : SubSystem()
{
	_headingHoldActive = false;
	_lastYawCommandTime = 0;
	_headingHoldSetPoint = 0;
	
	_altitudeHoldActive = false;
	_lastThrottleCommandTime = 0;
	_altitudeHoldSetPoint = 0;
				
	_flightControlType = FlightControlTypeAcrobatic;		//Good old standby.  Rate control mode.
	led.setPatternType(LED::PatternAlternate);
}

void FlightControl::initialize(const unsigned int frequency, const unsigned int offset) 
{ 
	SubSystem::initialize(frequency, offset);
	
	//Pull in all the pid settings.
	const Settings::PIDParameters *pidSettings = settings.getRollRatePIDParameters();
	_rollRatePID.setParameters(pidSettings->P,pidSettings->I,pidSettings->D,pidSettings->windupGuard);
	
	pidSettings = settings.getRollAnglePIDParameters();
	_rollAnglePID.setParameters(pidSettings->P,pidSettings->I,pidSettings->D,pidSettings->windupGuard);
	
	
	pidSettings = settings.getPitchRatePIDParameters();
	_pitchRatePID.setParameters(pidSettings->P,pidSettings->I,pidSettings->D,pidSettings->windupGuard);
	
	pidSettings = settings.getPitchAnglePIDParameters();
	_pitchAnglePID.setParameters(pidSettings->P,pidSettings->I,pidSettings->D,pidSettings->windupGuard);
	
	
	pidSettings = settings.getYawRatePIDParameters();
	_yawRatePID.setParameters(pidSettings->P,pidSettings->I,pidSettings->D,pidSettings->windupGuard);
	
	pidSettings = settings.getHeadingPIDParameters();
	_headingPID.setParameters(pidSettings->P,pidSettings->I,pidSettings->D,pidSettings->windupGuard);
	
	
	pidSettings = settings.getThrottleRatePIDParameters();
	_throttleRatePID.setParameters(pidSettings->P,pidSettings->I,pidSettings->D,pidSettings->windupGuard);
	
	pidSettings = settings.getAltitudePIDParameters();
	_altitudePID.setParameters(pidSettings->P,pidSettings->I,pidSettings->D,pidSettings->windupGuard);
}
		
void FlightControl::process(const unsigned long currentTime)
{
	if (this->_canProcess(currentTime))
	{
		SubSystem::recordProcessingStartTime();
		
		switch (_flightControlType)
		{
			case FlightControlTypeAcrobatic:
			{
				this->_processAcrobaticFlight(currentTime);																								
				break;
			}
			
			case FlightControlTypeStable:
			{
				this->_processStableFlight(currentTime);
				break;
			}
			
			case FlightControlTypeAutonomous:
			{
				this->_processAutonomousFlight(currentTime);
				break;
			}
		}	
		
		SubSystem::recordProcessingEndTime();								
	}
	
	SubSystem::process(currentTime);
}

void FlightControl::setFlightControlType(FlightControl::FlightControlType flightControlType)
{
	_flightControlType = flightControlType;
	
	switch (_flightControlType)
	{
		case FlightControlTypeAcrobatic:
		{
			//Essentualy free flight mode.  With just the minimal flight assistance to keep the quad in the air	
												
			led.setPatternType(LED::PatternAlternate);
			break;
		}
		
		case FlightControlTypeStable:
		{
			//Full assistance enabled.
							
			led.setPatternType(LED::PatternChase);
			break;
		}
		
		case FlightControlTypeAutonomous:
		{
			//Complete autonomous control.  Full assistance enabled
			//Some other restrictions kick in though.  TX commands aren't actually used at all.
			//Navigation subsystem does all the work to move the quad around.
			
			led.setPatternType(LED::PatternSweep);
			break;
		}
	}
}

const FlightControl::FlightControlType FlightControl::flightControlType()
{
	return _flightControlType;
}


//Accessors for the flight control command data
const float FlightControl::pitchCommand()
{
	return _pitchCommand;
}

const float FlightControl::rollCommand()
{
	return _rollCommand;
}

const float FlightControl::yawCommand()
{
	return _yawCommand;
}

const float FlightControl::throttleCommand()
{
	return _throttleCommand;
}

FlightControl flightcontrol;