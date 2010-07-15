#include "SubSystem.h"

class FlightControl : public SubSystem
{
	public:
		typedef enum { FlightControlTypeAcrobatic = 0, FlightControlTypeStable, FlightControlTypeAutonomous } FlightControlType; 
	private:
		unsigned int _pitchCommand;
		unsigned int _rollCommand;
		unsigned int _yawCommand;
		unsigned int _throttleCommand;
		
		bool _autoLevelEnabled;
		
		
		bool _headingHoldEnabled;
		bool _headingHoldActive;									//A flag to indicate if we are actually trying to hold our heading
		unsigned long _lastYawCommandTime;				//The last time there was a yaw command made by the pilot
		float	_headingHoldYaw;										//The heading we need to try and keep when heading hold is enabled
		
		
		bool _altitudeHoldEnabled;
		
		FlightControlType _flightControlType;
		
		int _tickDirection;
		
		//Used for rate control when enabled
		PID	_rollRatePID;
		PID	_pitchRatePID;
		PID	_yawRatePID;
		
		//Used for angle control when enabled
		PID	_rollAnglePID;
		PID	_pitchAnglePID;		
		PID	_yawAnglePID;
		
		//Used all the time
		PID	_throttlePID;
				
	public:
		FlightControl() : SubSystem()
		{
			_autoLevelEnabled = false;
			
			_headingHoldEnabled = false;
			_headingHoldYaw = 0;
			
			_altitudeHoldEnabled = false;			
			
			_flightControlType = FlightControlTypeAcrobatic;		//Good old standby.  Rate control mode.
			led.setPatternType(LED::PatternAlternate);
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
						
			_rollRatePID.setParameters(5,0,-10,2000);
			_rollAnglePID.setParameters(5,0,-1,2000);
			
			_pitchRatePID.setParameters(5,0,-10,2000);			
			_pitchAnglePID.setParameters(5,0,-1,2000);						
			
			_yawRatePID.setParameters(12,0,0,2000);
			_yawAnglePID.setParameters(12,0,0,2000);
		}
				
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				int throttleTransmitterCommand = 0; 	//receiver.channelValue(ReceiverHardware::Channel3);
				int rollTransmetterCommand = 0; 			//receiver.channelValue(ReceiverHardware::Channel4);
				int pitchTransmetterCommand = 0;		//receiver.channelValue(ReceiverHardware::Channel1);
				int yawTransmetterCommand = 0;		//receiver.channelValue(ReceiverHardware::Channel2);
				
				float outputRollCommand = 0;
				float outputPitchCommand = 0;
				float outputYawCommand = 0;
				float outputThrottleCommand = 0;
				
				//Autonomous flight control handles Yaw in the navigation system.  So we only do the yaw calculation for Acro and Stable modes
				if (_flightControlType != FlightControlTypeAutonomous)
				{
					//Yaw is always a rate, case we can't always "center" based on the stick like we can with roll and pitch (would ne north all the time)
					float transmetterYawRateInRadians = 0;
					float currentYawRateInRadians = imu.currentYawRateInRadians();
					
					if (fabs(transmetterYawRateInRadians) > 0.0872664626) /* 5 deg/s */
					{
						//Pilot is requesting Yaw command
						outputYawCommand = _yawRatePID.update(transmetterYawRateInRadians, currentYawRateInRadians);	
						
						//Capture when the last time a yaw command was used.
						//This is used to trigger the heading hold after a short time delay (to allow a final bit of "coast" when yawing before it locks on)
						_lastYawCommandTime = currentTime;							
						_headingHoldActive = false;
					}
					else
					{
						float currentYawAngleInRadians = imu.currentYawAngleInRadians();
						
						//No yaw command from the pilot.
						if (!_headingHoldActive && (currentTime - _lastYawCommandTime) > 1500)
						{
							//We aren't activly trying to hold out heading and its been 1.5 seconds since the last Yaw command.  
							//Record the current heading as the one to try and hold and enable heading hold
							_headingHoldYaw = currentYawAngleInRadians;						
							_headingHoldActive = true;
						}
						
						outputYawCommand = _yawAnglePID.update(_headingHoldYaw, currentYawAngleInRadians);	
					}
				}
				else
				{
					outputYawCommand = 0;		//Yaw is controlled by the navigation subsystem and will "point" the quad where it needs to go.
				}
				
				
				//Process roll and pitch as the flight control type requires				
				switch (_flightControlType)
				{
					case FlightControlTypeAcrobatic:
					{			
						//This mode is fully controlled by the pilot.  We only use the gyro rates and the transmitter sticks become rate based.						
						float transmetterRollRateInRadians = 0;				//TODO: convert from TX commands to radians/s
						float transmetterPitchRateInRadians = 0;		
						
						float currentRollRateInRadians = imu.currentRollRateInRadians();
						float currentPitchRateInRadians = imu.currentPitchRateInRadians();												
				
						outputRollCommand = _rollRatePID.update(transmetterRollRateInRadians, currentRollRateInRadians);
						outputPitchCommand = _pitchRatePID.update(transmetterPitchRateInRadians, currentPitchRateInRadians);						
									
						break;
					}
					case FlightControlTypeStable:
					{
						//This mode has the software helping out and keeping things relativly stable.  Transmitter sticks become angle based.
						float transmetterRollAngleInRadians = 0;				//TODO: convert from TX commands to radians
						float transmetterPitchAngleInRadians = 0;						
						
						float currentRollAngleInRadians = imu.currentRollAngleInRadians();
						float currentPitchAngleInRadians = imu.currentPitchAngleInRadians();												
				
						outputRollCommand = _rollAnglePID.update(transmetterRollAngleInRadians, currentRollAngleInRadians);
						outputPitchCommand = _pitchAnglePID.update(transmetterPitchAngleInRadians, currentPitchAngleInRadians);
																			
						break;
					}
					case FlightControlTypeAutonomous:
					{
						//Transmitter sticks are ignored and software controls everything.  Flight control basically just keeps the craft stable.  Navigation takes over movement.						
						float currentRollAngleInRadians = imu.currentRollAngleInRadians();
						float currentPitchAngleInRadians = imu.currentPitchAngleInRadians();												
				
						//Always try and keep the quad level.  The Navigation system will attempt to direct the quad where it needs to go
						outputRollCommand = _rollAnglePID.update(0, currentRollAngleInRadians);
						outputPitchCommand = _pitchAnglePID.update(0, currentPitchAngleInRadians);						
						break;
					}
				}		
				
				//TODO: Altitude Hold
				
				//TODO: Scale the outputs to the 1000-2000 numbers that motor control requires.
				serialcoms.print(outputRollCommand);
				serialcoms.print(",");
				serialcoms.print(outputPitchCommand);
				serialcoms.print(",");
				serialcoms.print(outputYawCommand);
				serialcoms.print(",");
				serialcoms.print(outputThrottleCommand);
				serialcoms.println();																
			}
		}
		
		//Accessors for the flight control options				
		void enableHeadingHold()
		{
			//heading hold can only be enabled in stable and autonomous modes
			if (_flightControlType != FlightControlTypeAcrobatic)
			{
				_headingHoldEnabled = true;
			}
		}
		
		void disableHeadingHold()
		{
			//ensure that heading hold can't be turned off in autonomous mode
			if (_flightControlType != FlightControlTypeAutonomous)
			{
				_headingHoldEnabled = false;
			}
		}
		
		const bool headingHold()
		{
			return _headingHoldEnabled;
		}
		
		void enableAltitudeHold()
		{
			//heading hold can only be enabled in stable and autonomous modes
			if (_flightControlType != FlightControlTypeAcrobatic)
			{
				_altitudeHoldEnabled = true;
			}
		}
		
		void disableAltitudeHold()
		{
			//ensure that altitude hold can't be turned off in autonomous mode
			if (_flightControlType != FlightControlTypeAutonomous)
			{
				_altitudeHoldEnabled = false;
			}
		}
		
		const bool altitudeHold()
		{
			return _altitudeHoldEnabled;
		}
		
		
		
		void setFlightControlType(FlightControlType flightControlType)
		{
			_flightControlType = flightControlType;
			
			switch (_flightControlType)
			{
				case FlightControlTypeAcrobatic:
				{
					//Essentualy free flight mode.  With just the minimal flight assistance to keep the quad in the air					
					_autoLevelEnabled = false;
					_headingHoldEnabled = false;
					_altitudeHoldEnabled = false;
					
					led.setPatternType(LED::PatternAlternate);
					break;
				}
				case FlightControlTypeStable:
				{
					//Slightly more flight assistance.  Autoleveling enabled.
					_autoLevelEnabled = true;
					
					//Heading hold can be enabled if so desired.
									
					led.setPatternType(LED::PatternChase);
					break;
				}
				case FlightControlTypeAutonomous:
				{
					//Complete autonomous control.  Full assistance enabled
					_autoLevelEnabled = true;
					_headingHoldEnabled = true;		
					_altitudeHoldEnabled = true;								
					
					led.setPatternType(LED::PatternSweep);
					break;
				}
			}
		}
		
		const FlightControlType getFlightControlType()
		{
			return _flightControlType;
		}

		
		//Accessors for the flight control command data
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