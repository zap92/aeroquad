#ifndef __PID_H__
#define __PID_H__

//PID Loop based on: http://caspiquad.googlecode.com/svn/tags/flying_well_2010_0206/CaspiQuad/PID.cpp
class PID
{
	private: 
		float _P;
		float _I;
		float _D;
		
		float _windupGuard;
		
		float _lastError;
		float _integeratedError;	
		
		bool _firstPass;
		
	public:
		PID();
		
		void setParameters(const float p, const float i, const float d, const float windupGuard);
		
		void setP(const float p);
		const float getP();
		
		void setI(const float i);
		const float getI();
		
		void setD(const float d);
		const float getD();
		
		void setWindupGuard(float windupGuard);
		const float getWindupGuard();
		
		void reset();
		
		const float update(const float targetValue, const float currentValue);
};

#endif //#ifndef __PID_H__