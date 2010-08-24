#ifndef __LED_H__
#define __LED_H__

#include "SubSystem.h"

class LED : public SubSystem
{
	public:
		typedef enum { PatternOff = 0, PatternChase, PatternSweep, PatternAlternate } LEDPattern;
		
	private: 
		bool _readyLEDState;
		
		int _patternState;
		
		LEDPattern _activePattern;
		
	public:
		LED();
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0);
		virtual void process(const unsigned long currentTime);
		
		void setPatternType(const LEDPattern patternType);
};

extern LED led;

#endif //#ifndef __LED_H__