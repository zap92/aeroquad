#include "SubSystem.h"

class FlightControl : public SubSystem
{
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
		
		const int _updatePID(const float targetPosition, const float currentPosition, struct PIDdata *PIDparameters)
		{
		  	float error;
		  	float dTerm;
		
			error = targetPosition - currentPosition;

			PIDparameters->integratedError = constrain(PIDparameters->integratedError + error, -_windupGuard, _windupGuard);

			dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition);
			PIDparameters->lastPosition = currentPosition;
			
			return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
		}
		
	public:
		FlightControl() : SubSystem()
		{
			
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
			
			_PIDs[0].P = 3;
			_PIDs[0].I = 0.0;
			_PIDs[0].D = -10.0;
			
			_PIDs[1].P = 3;
			_PIDs[1].I = 0.0;
			_PIDs[1].D = -10.0;
			
			_PIDs[2].P = 12;
			_PIDs[2].I = 0.0;
			_PIDs[2].D = 0.0;
			
			_windupGuard = 1000;
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				int pitchTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel2);
				int rollTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel1);
				int yawTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel4);
				int throttleTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel3);
				
				float currentPitchAngle = imu.currentPitchAngle();
				float currentRollAngle = imu.currentRollAngle();
				
				float currentPitchRate = imu.currentPitchRate();
				float currentRollRate = imu.currentRollRate();
				float currentYawRate = imu.currentYawRate();
				
				int rollAdjust = 0;
				int pitchAdjust = 0;
				int yawAdjust = 0;
				
				int currentRollPosition = fconstrain(fmap(currentRollRate, -1,1, 1000,2000), 1000, 2000);
				int currentPitchPosition = fconstrain(fmap(currentPitchRate, -1,1, 1000,2000), 1000,2000);
				int currentYawPosition = fconstrain(fmap(currentYawRate, -1,1, 1000,2000), 1000,2000);
				
				_rollCommand = _updatePID(rollTransmitterCommand + rollAdjust, currentRollPosition, &_PIDs[0]);
				_pitchCommand = _updatePID(pitchTransmitterCommand + pitchAdjust, currentPitchPosition, &_PIDs[1]);
				_yawCommand = _updatePID(yawTransmitterCommand + yawAdjust, currentYawPosition, &_PIDs[2]);
				
				/*DEBUGSERIALPRINT(rollTransmitterCommand);
				DEBUGSERIALPRINT(",");
				DEBUGSERIALPRINT(pitchTransmitterCommand);
				DEBUGSERIALPRINT(",");
				DEBUGSERIALPRINT(yawTransmitterCommand);
				DEBUGSERIALPRINTLN("");*/
			}
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