#include "SubSystem.h"

class Camera : public SubSystem
{
	public:
		Camera() : SubSystem()
		{
			
		}
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
		}
		
		virtual void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				float currentPitchAngle = -imu.currentPitchAngle();
				float currentRollAngle = -imu.currentRollAngle();
				
				//TODO
				//Tell the servos where to go now that we know what angle we need to set them to.
			}
		}
};