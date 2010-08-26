#ifndef __FLIGHTCONTROL_H__
#define __FLIGHTCONTROL_H__

#include "SubSystem.h"

#include "PID.h"

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
		PID   _throttleRatePID;
		
		//Used for stable control when enabled
		PID	_rollAnglePID;
		PID	_pitchAnglePID;		
		PID	_headingPID;
		PID	_altitudePID;
		
		void _processAcrobaticFlight(const unsigned long currentTime);				
		void _processStableRollAndPitch(const float expectedRollAngle, const float expectedPitchAngle, const unsigned long currentTime);		
		void _processStableFlight(const unsigned long currentTime);
		
		void _processAutonomousFlight(const unsigned long currentTime);
				
	public:
		FlightControl();
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0);				
		virtual void process(const unsigned long currentTime);
		
		void setFlightControlType(FlightControlType flightControlType);
		
		const FlightControlType flightControlType();
		
		//Accessors for the flight control command data
		const float pitchCommand();	
		const float rollCommand();		
		const float yawCommand();		
		const float throttleCommand();
};

extern FlightControl flightcontrol;

#endif //#ifndef __FLIGHTCONTROL_H__