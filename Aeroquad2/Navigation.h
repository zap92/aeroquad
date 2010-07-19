#include "SubSystem.h"

class Waypoint
{
	public:
		static const float IgnoreHeading;
	private:
		bool _isHome;

		float _latitude;
		float _longitude;

		float _altitude;
		float _heading;

		int _loiterTime;

	public:
		Waypoint()
		{
			_isHome = false;
	
			_latitude = _longitude = 0;
			
			_altitude = 0;
			_heading = 0;
			
			_loiterTime = 0;
		}
		
		Waypoint (const bool isHome, const float latitude, const float longitude, const float altitude, const float heading = IgnoreHeading, const int loiterTime = 0) : Waypoint()
		{
			_isHome = isHome;
			
			_latitude = latitude;
			_longitude = longitude;
			
			_altitude = altitude;
			_heading = heading;
			
			_loiterTime = loiterTime;
		}
		
		const bool isHome()
		{
			return _isHome;
		}
		void setIsHome(const bool isHome)
		{
			_isHome = isHome;
		}
		
		const float latitude()
		{
			return _latitude;
		}
		void setLatitude(const float latitude)
		{
			_latitude = latitude;
		}
		
		const float longitude()
		{
			return _longitude;
		}
		void setLongitude(const float longitude)
		{
			_longitude = longitude;
		}
		
		const float altitude()
		{
			return _altitude;
		}
		void setAltitude(const float altitude)
		{
			_altitude = altitude;
		}
		
		const float heading()
		{
			return _heading;
		}
		void setHeading(const float heading)
		{
			_heading = heading;
		}
		
		const float loiterTime()
		{
			return _loiterTime;
		}
		void setLoiterTime(const float loiterTime)
		{
			_loiterTime = loiterTime;
		}		
};
static const int Waypoint::IgnoreHeading = -99;

//presently we support a maximum of 200 waypoints
class Navigation : public Subsystem
{
	private:
		Waypoint *_homeWaypoint;
		
		Waypoint *_activeWaypoint;
		
		unsigned int _waypointCount;
		unsigned int _activeWaypointIndex;
		
	public:
		Navigation : Subsystem()
		{
			_waypointCount = 0;
		}
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset) 
		{ 
			Subsystem::initialize(frequency, offset);
		}
		
		virtual void process(const unsigned long currentTime) 
		{
			Subsystem::process(currentTime);
		}
		
		//Waypoint Accessors
		const int waypointCount()
		{
			return _waypointCount;
		}
};