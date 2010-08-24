#ifndef __SUBSYSTEM_H__
#define __SUBSYSTEM_H__

//#define NUMBER_OF_STATISTICS_SAMPLES 50
class SubSystem
{
	
private:
	static SubSystem* _staticInstance;
	
	protected:
		unsigned int _enabled;
		unsigned long _lastRunTime;
		
		unsigned long _processingStartTime;
				
		unsigned int _previousRunTimeSampleCount;
		float _previousRunTimeTotal;
		
		unsigned int _frequency;				

	public:
		static SubSystem* getInstance();
		
		SubSystem();

		virtual void initialize(const unsigned int frequency, const unsigned int offset);
		virtual void process(const unsigned long currentTime);
		
		void recordProcessingStartTime();
		void recordProcessingEndTime();
		
		const float averageProcessingTime();
		
		void enable();
		void disable();
		
		unsigned int enabled();

		unsigned int _canProcess(const unsigned long currentTime);
};

#endif




