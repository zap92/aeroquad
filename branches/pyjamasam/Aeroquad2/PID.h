
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
		PID()
		{
			_firstPass = true;
			
			this->reset();			
		}
		
		void setParameters(const float p, const float i, const float d, const float windupGuard)
		{
			_P = p;
			_I = i;
			_D = d;
			_windupGuard = windupGuard;
		}
		
		void setP(const float p)
		{
			_P = p;
		}	
		const float getP()
		{
			return _P;
		}
		
		void setI(const float i)
		{
			_I = i;
		}
		const float getI()
		{
			return _I;
		}		
		
		void setD(const float d)
		{
			_D = d;
		}
		const float getD()
		{
			return _D;
		}			
		
		void setWindupGuard(float windupGuard)
		{
			_windupGuard = windupGuard;
		}
		const float getWindupGuard()
		{
			return _windupGuard;
		}
		
		void reset()
		{
			_integeratedError = 0;	
		}
		
		const float update(const float targetValue, const float currentValue)
		{
			float error = targetValue - currentValue;
			
			if (_firstPass)
			{
				_firstPass = false;
				return constrain(error, -_windupGuard, _windupGuard);
			}
			else
			{
	  			// Calculate error integral and limit integrated_error to +/-windup_guard  
				_integeratedError += error;
				_integeratedError = constrain(_integeratedError, -_windupGuard, _windupGuard);
	
	  			// Calculate error derivative   
	  			float dTerm = _D * (error - _lastError);
	  			_lastError = error;
	  
	  			return (_P * error) + (_I * _integeratedError) + dTerm;
  			}
		}
};