#include "SubSystem.h"

class FlightControl : public SubSystem
{
	public:
		typedef enum { PIDTypeRoll = 0, PIDTypePitch = 1, PIDTypeYaw = 2, PIDTypeRollAngle = 3, PIDTypePitchAngle = 4, PIDTypeHeading = 5 } PIDType;
	
	private:
		int _pitchCommand;
		int _rollCommand;
		int _yawCommand;
		
		int _windupGuard;
		int _autoLevelTriggerLimit;
		
		struct PIDdata {
		  float P, I, D;
		  float lastPosition;
		  float integratedError;
		} _PIDs[6];
		
		bool _autoLevel;
		bool _headingHold;
		
		const int _updatePID(const float targetPosition, const float currentPosition, struct PIDdata *PIDparameters)
		{
		  	float error;
			float dTerm;

		  	error = targetPosition - currentPosition;

			PIDparameters->integratedError += error;
			PIDparameters->integratedError = constrain(PIDparameters->integratedError, -_windupGuard, _windupGuard);
			
			dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition);
			PIDparameters->lastPosition = currentPosition;
			return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
		}
		
	public:
		FlightControl() : SubSystem()
		{
			_autoLevel = false;
			_headingHold = false;
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
			
			_PIDs[FlightControl::PIDTypeRoll].P = 3.15;
			_PIDs[FlightControl::PIDTypeRoll].I = 0;
			_PIDs[FlightControl::PIDTypeRoll].D = 0;
			
			_PIDs[FlightControl::PIDTypePitch].P = 3.15;
			_PIDs[FlightControl::PIDTypePitch].I = 0;
			_PIDs[FlightControl::PIDTypePitch].D = 0;
			
			_PIDs[FlightControl::PIDTypeYaw].P = 5;
			_PIDs[FlightControl::PIDTypeYaw].I = 0;
			_PIDs[FlightControl::PIDTypeYaw].D = 0;
			
			_PIDs[FlightControl::PIDTypeRollAngle].P = 3.15;
			_PIDs[FlightControl::PIDTypeRollAngle].I = 0;
			_PIDs[FlightControl::PIDTypeRollAngle].D = 0;
			
			_PIDs[FlightControl::PIDTypePitchAngle].P = 3.15;
			_PIDs[FlightControl::PIDTypePitchAngle].I = 0;
			_PIDs[FlightControl::PIDTypePitchAngle].D = 0;
			
			_PIDs[FlightControl::PIDTypeHeading].P = 3.15;
			_PIDs[FlightControl::PIDTypeHeading].I = 0;
			_PIDs[FlightControl::PIDTypeHeading].D = 0;
			
			_windupGuard = 1000;
			_autoLevelTriggerLimit = 100;
		}
		
		//PID accessors
		void setPidParameters(const float P, const float I, const float D, PIDType pidType)
		{
			_PIDs[pidType].P = P;
			_PIDs[pidType].I = I;
			_PIDs[pidType].D = D;
		}
		
		void getPidParameters(float *P, float *I, float *D, PIDType pidType)
		{
			if (P) *P = _PIDs[pidType].P;
			if (I) *I = _PIDs[pidType].I;
			if (D) *D = _PIDs[pidType].D;
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				int rollTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel1);
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
				_yawCommand = _updatePID(yawTransmitterCommand + yawAdjust, currentScaledYawRate, &_PIDs[FlightControl::PIDTypeYaw]);				
				
				/*DEBUGSERIALPRINT(_rollCommand);
				DEBUGSERIALPRINT(",");
				DEBUGSERIALPRINT(_yawCommand);
				DEBUGSERIALPRINT(",");
				DEBUGSERIALPRINT(_yawCommand);
				DEBUGSERIALPRINTLN("");*/
			}
		}
		
		//Setters for the flight control options
		void enableAutoLevel()
		{
			_autoLevel = true;
		}
		
		void disableAutoLevel()
		{
			_autoLevel = false;
		}
		
		const bool autoLevel()
		{
			return _autoLevel;
		}
		
		void enableHeadingHold()
		{
			_headingHold = true;
		}
		
		void disableHeadingHold()
		{
			_headingHold = false;
		}
		
		const bool headingHold()
		{
			return _headingHold;
		}
		
		//Accessors for the flight control data
		const int pitchCommand()
		{
			return _pitchCommand;
		}
		
		const int rollCommand()
		{
			return _rollCommand;
		}
		
		const int yawCommand()
		{
			return _yawCommand;
		}
};