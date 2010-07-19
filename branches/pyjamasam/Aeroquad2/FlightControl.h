#include "SubSystem.h"

class FlightControl : public SubSystem
{
	public:
		typedef enum { FlightControlTypeAcrobatic = 0, FlightControlTypeStable, FlightControlTypeAutonomous } FlightControlType; 
	private:
		float _pitchCommand;
		float _rollCommand;
		float _yawCommand;
		float _throttleCommand;
				
		bool _headingHoldActive;									//A flag to indicate if we are actually trying to hold our heading
		unsigned long _lastYawCommandTime;							//The last time there was a yaw command made by the pilot
		float	_headingHoldSetPoint;								//The heading we need to try and keep when heading hold is enabled
		
		bool _altitudeHoldActive;									//A flag to indicate if we are actually trying to hold our altitude
		unsigned long _lastThrottleCommandTime;						//The last time there was a throttle command made by the pilot
		float	_altitudeHoldSetPoint;								//The altitude we need to try and keep when altitude hold is enabled
		
		FlightControlType _flightControlType;
		
		int _tickDirection;
		
		//Used for acrobatic control when enabled
		PID	_rollRatePID;
		PID	_pitchRatePID;
		PID	_yawRatePID;
		PID _throttleRatePID;
		
		//Used for stable control when enabled
		PID	_rollAnglePID;
		PID	_pitchAnglePID;		
		PID	_headingPID;
		PID	_altitudePID;
		
		void _processAcrobaticFlight(const unsigned long currentTime)
		{
			//All controls are rate based.  The simplest form of control.  No software assistance.
			_throttleCommand = receiver.channelValue(ReceiverHardware::Channel3);
			
			_rollCommand = _rollRatePID.update(receiver.channelValueAsRateInRadians(ReceiverHardware::Channel1), imu.rollRateInRadians());
			_pitchCommand = _pitchRatePID.update(receiver.channelValueAsRateInRadians(ReceiverHardware::Channel2), imu.pitchRateInRadians());
			_yawCommand = _yawRatePID.update(receiver.channelValueAsRateInRadians(ReceiverHardware::Channel4), imu.yawRateInRadians());
		}
		
		
		void _processStableRollAndPitch(const float expectedRollAngle, const float expectedPitchAngle)
		{
			_rollCommand = _rollAnglePID.update(receiver.channelValueAsAngleInRadians(ReceiverHardware::Channel1), expectedRollAngle);
			_pitchCommand = _pitchAnglePID.update(receiver.channelValueAsAngleInRadians(ReceiverHardware::Channel2), expectedPitchAngle);
		}
		
		void _processStableFlight(const unsigned long currentTime)
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
			
			/*Serial.print(transmitterYawRateInRadians);
			Serial.print(",");
			Serial.print(_headingHoldActive);
			Serial.print(",");
			Serial.print(_headingHoldSetPoint);
			Serial.print(",");			
			Serial.print(_yawCommand);
			Serial.println();*/
			
			
			
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
		
		void _processAutonomousFlight(const unsigned long currentTime)
		{
			//Navigation subsystem does most of the work.  We just aim to keep the quad stable
			//So infact its farily simple for the purposes of flight control.
			this->_processStableRollAndPitch(imu.rollAngleInRadians(), imu.pitchAngleInRadians());
		}
				
	public:
		FlightControl() : SubSystem()
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
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0) 
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
				
		virtual void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				
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
				
				/*serialcoms.print(_throttleCommand);
				serialcoms.print(",");
				serialcoms.print(_rollCommand);
				serialcoms.print(",");
				serialcoms.print(_pitchCommand);
				serialcoms.print(",");
				serialcoms.print(_yawCommand);
				serialcoms.println("");*/
			}
			
			SubSystem::process(currentTime);
		}
		
		void setFlightControlType(FlightControlType flightControlType)
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
		
		const FlightControlType flightControlType()
		{
			return _flightControlType;
		}

		
		//Accessors for the flight control command data
		const float pitchCommand()
		{
			return _pitchCommand;
		}
		
		const float rollCommand()
		{
			return _rollCommand;
		}
		
		const float yawCommand()
		{
			return _yawCommand;
		}
		
		const float throttleCommand()
		{
			return _throttleCommand;
		}
};