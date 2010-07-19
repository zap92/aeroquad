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
				float currentPitchAngle = -imu.pitchAngleInRadians();
				float currentRollAngle = -imu.rollAngleInRadians();
								
				//We can't do more then 45 degrees in either way.
				currentPitchAngle = constrain(currentPitchAngle, -0.785398163, 0.785398163);
				currentRollAngle = constrain(currentRollAngle, -0.785398163, 0.785398163);

				int pitchServoCommand = fmap(currentPitchAngle, -0.785398163, 0.785398163, 1000, 2000);
				int rollServoCommand = fmap(currentRollAngle, -0.785398163, 0.785398163, 1000, 2000);				
				
				//Tell the servos where to go now that we know what angle we need to set them to.
				APM_RC.OutputCh(4, pitchServoCommand);
				APM_RC.OutputCh(5, rollServoCommand);
				
			}
		}
};