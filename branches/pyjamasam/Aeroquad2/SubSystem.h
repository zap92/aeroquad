#ifndef __SUBSYSTEM_H__
#define __SUBSYSTEM_H__

class SubSystem
{
	
private:
	static SubSystem* _staticInstance;
	
	protected:
		unsigned int _enabled;
		unsigned long _lastRunTime;

		unsigned int _frequency;

	public:
		static SubSystem* getInstance()
		{
			return _staticInstance;
		}
		
		SubSystem() 
		{
			_enabled = 0;
			_lastRunTime = 0;
			_frequency = 0;
			
			_staticInstance = this;
		}

		virtual void initialize(const unsigned int frequency, const unsigned int offset) 
		{ 
			_frequency = frequency;
			_lastRunTime = offset;
			_enabled = 1;
		}

		void enable()
		{
			_enabled = 1;
		}

		void disable()
		{
			_enabled = 0;
		}

		unsigned int enabled()
		{
			return _enabled;
		}

		unsigned int _canProcess(const unsigned long currentTime)
		{
			if (_enabled == 1)
			{
				if (currentTime > (_lastRunTime + _frequency))
				{
					 _lastRunTime = currentTime;
					 return 1;
				}
			}   
			return 0;
		}

		virtual void process(const unsigned long currentTime) 
		{
		
		}
};

SubSystem* SubSystem::_staticInstance = NULL;

#endif



