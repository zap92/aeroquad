#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "SubSystem.h"

class Camera : public SubSystem
{
	public:
		Camera();
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0);		
		virtual void process(const unsigned long currentTime);		
};

extern Camera camera;

#endif //#ifndef __CAMERA_H__