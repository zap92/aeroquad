#include "SubSystem.h"

class FlightControl : public SubSystem
{
	public:
		typedef enum { PIDTypeRoll = 0, PIDTypePitch = 1, PIDTypeYaw = 2 } PIDType;
	
	private:
		int _pitchCommand;
		int _rollCommand;
		int _yawCommand;
		
		int _windupGuard;
		
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
			if (PIDparameters->integratedError < -_windupGuard) PIDparameters->integratedError = -_windupGuard;
			else if (PIDparameters->integratedError > _windupGuard) PIDparameters->integratedError = _windupGuard;

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
			
			_PIDs[FlightControl::PIDTypeRoll].P = 5;
			_PIDs[FlightControl::PIDTypeRoll].I = 0;
			_PIDs[FlightControl::PIDTypeRoll].D = 0;
			
			_PIDs[FlightControl::PIDTypePitch].P = 5;
			_PIDs[FlightControl::PIDTypePitch].I = 0;
			_PIDs[FlightControl::PIDTypePitch].D = 0;
			
			_PIDs[FlightControl::PIDTypeYaw].P = 5;
			_PIDs[FlightControl::PIDTypeYaw].I = 0;
			_PIDs[FlightControl::PIDTypeYaw].D = 0;
			
			_windupGuard = 1000;
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
					//TODO
				}
				
				if (_headingHold)
				{
					//Process adjustments needed to keep the quad pointed in the right direction
					//TODO
				}
				
				float currentRollPosition = currentRollRate * 465;
				float currentPitchPosition = currentPitchRate * 465;
				float currentYawPosition = currentYawRate * 465;
				
				_rollCommand = _updatePID(rollTransmitterCommand, (currentRollPosition + 1500), &_PIDs[FlightControl::PIDTypeRoll]);
				_pitchCommand = _updatePID(pitchTransmitterCommand, (currentPitchPosition + 1500), &_PIDs[FlightControl::PIDTypePitch]);
				_yawCommand = _updatePID(yawTransmitterCommand, (currentYawPosition + 1500), &_PIDs[FlightControl::PIDTypeYaw]);				
				
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