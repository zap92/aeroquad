#include "SubSystem.h"

class Camera : public SubSystem
{
	public:
		Camera() : SubSystem()
		{
			
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				float currentPitchAngle = imu.currentPitchAngle();
				float currentRollAngle = imu.currentRollAngle();
				
				//Invert the angle and drive the servos as needed.
				//TODO
			}
		}
};