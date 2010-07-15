#include "SubSystem.h"

class FlightControl : public SubSystem
{
	public:
		typedef enum { FlightControlTypeRateControl = 0, FlightControlTypeAtitudeControl, FlightControlTypeAutonomous } FlightControlType; 
	private:
		unsigned int _pitchCommand;
		unsigned int _rollCommand;
		unsigned int _yawCommand;
		unsigned int _throttleCommand;
		
		bool _autoLevelEnabled;
		bool _headingHoldEnabled;
		
		FlightControlType _flightControlType;
		
		int _tickDirection;
				
	public:
		FlightControl() : SubSystem()
		{
			_autoLevelEnabled = false;
			_headingHoldEnabled = false;
			
			_flightControlType = FlightControlTypeRateControl;		//Good old standby.  Rate control mode.
			
			//Just for testing to ensure the commands make it all the way through to the motors
			_tickDirection = 10;
			_rollCommand = _yawCommand = _pitchCommand = _throttleCommand = 100;
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
		}
				
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				//Testing code.  Checking to see if commands make it all the way to the motors
				_throttleCommand += _tickDirection;
				_rollCommand = _yawCommand = _pitchCommand = _throttleCommand;
				
				if (_rollCommand > 1000)
				{
					_tickDirection *= -1;
				}
				else if (_rollCommand < 100)
				{
					_tickDirection *= -1;
				}
				
				/*int rollTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel1);
				int pitchTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel2);
				int yawTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel4);
				
				float currentPitchAngle = imu.currentPitchAngle();
				float currentRollAngle = imu.currentRollAngle();
				
				float currentPitchRate = imu.currentPitchRate();
				float currentRollRate = imu.currentRollRate();
				float currentYawRate = imu.currentYawRate();
				
				int rollAdjust = 0;
				int pitchAdjust = 0;
				int yawAdjust = 0;
				
				if (_autoLevel)
				{
					//Process adjustments needed to autolevel the quad
					//We always target a 0 angle.  So let the PID sort out how to get us there
					
					// Check to see if the roll stick is anywhere but a deadzone around "0"
					if (rollTransmitterCommand > (1500 - _autoLevelTriggerLimit) && rollTransmitterCommand < (1500 + _autoLevelTriggerLimit))
					{
						//Roll stick is in the dead zone.  Apply auto level for the roll axis
						rollAdjust = _updatePID(0, currentRollAngle, &_PIDs[FlightControl::PIDTypeRollAngle]);
					}
					else
					{
						//Roll transmitter stick is outside the dead zone.  Don't bother with auto level for this axis.  And zero out the pid
						_PIDs[FlightControl::PIDTypeRollAngle].integratedError = 0;
					}
					
					// Check to see if the pitch stick is anywhere but a deadzone around "0"
					if (pitchTransmitterCommand > (1500 - _autoLevelTriggerLimit) && pitchTransmitterCommand < (1500 + _autoLevelTriggerLimit))
					{
						//pitch stick is in the dead zone.  Apply auto level for the pitch axis
						pitchAdjust = _updatePID(0, currentPitchAngle, &_PIDs[FlightControl::PIDTypePitchAngle]);
					}
					else
					{
						//Pitch transmitter stick is outside the dead zone.  Don't bother with auto level for this axis.  And zero out the pid
						_PIDs[FlightControl::PIDTypePitchAngle].integratedError = 0;
					}
				}
				
				if (_headingHold)
				{
					//Process adjustments needed to keep the quad pointed in the right direction
					//TODO
				}
				
				//Sort out the current scaled gyro rates
				float currentScaledRollRate = (currentRollRate * 465) + 1500;
				float currentScaledPitchRate = (currentPitchRate * 465) + 1500;
				float currentScaledYawRate = (currentYawRate * 465) + 1500;
				
				_rollCommand = _updatePID(rollTransmitterCommand + rollAdjust, currentScaledRollRate, &_PIDs[FlightControl::PIDTypeRoll]);
				_pitchCommand = _updatePID(pitchTransmitterCommand + pitchAdjust, currentScaledPitchRate, &_PIDs[FlightControl::PIDTypePitch]);
				_yawCommand = _updatePID(yawTransmitterCommand + yawAdjust, currentScaledYawRate, &_PIDs[FlightControl::PIDTypeYaw]);				*/
			}
		}
		
		//Accessors for the flight control options
		void enableAutoLevel()
		{
			_autoLevelEnabled = true;
		}
		
		void disableAutoLevel()
		{
			_autoLevelEnabled = false;
		}
		
		const bool autoLevel()
		{
			return _autoLevelEnabled;
		}
		
		void enableHeadingHold()
		{
			_headingHoldEnabled = true;
		}
		
		void disableHeadingHold()
		{
			_headingHoldEnabled = false;
		}
		
		const bool headingHold()
		{
			return _headingHoldEnabled;
		}
		
		void setFlightControlType(FlightControlType flightControlType)
		{
			_flightControlType = flightControlType;
		}
		
		const FlightControlType getFlightControlType()
		{
			return _flightControlType;
		}

		
		//Accessors for the flight control data
		const unsigned int getPitchCommand()
		{
			return _pitchCommand;
		}
		
		const unsigned int getRollCommand()
		{
			return _rollCommand;
		}
		
		const unsigned int getYawCommand()
		{
			return _yawCommand;
		}
		
		const unsigned int getThrottleCommand()
		{
			return _throttleCommand;
		}
};