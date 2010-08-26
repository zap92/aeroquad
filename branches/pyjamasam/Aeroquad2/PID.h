#ifndef __PID_H__
#define __PID_H__

//PID Loop based on: http://caspiquad.googlecode.com/svn/tags/flying_well_2010_0206/CaspiQuad/PID.cpp
class PID
{
	private: 
		float *_P;
		float *_I;
		float *_D;
		
		float *_windupGuard;
		
		float _lastError;
		float _integral;	
		
		bool _firstPass;
		
		unsigned long _previousTime;
		
	public:
		PID();
		
		void setParameters(float *p, float *i, float *d, float *windupGuard);				
		
		void reset();
		
		const float update(const float targetValue, const float currentValue, unsigned long currentTime);
};

#endif //#ifndef __PID_H__