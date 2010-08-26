#ifndef __SERIALCOMS_H__
#define __SERIALCOMS_H__

#include "SubSystem.h"

#include <stdio.h>

#define BAUD 115200
#define PROMPT				">> "

#define MAX_COMMAND_LINE_LENGTH 256
#define MAX_SUPPORTED_KEYWORDS 20
#define MAX_SUPPORTED_ARGUMENTS	 20

class ArduinoShellCallback
{
	public:
		typedef enum { Success = 0, Failure, InvalidParameterCount } callbackReturn;
};

class ArduinoShell;
typedef const ArduinoShellCallback::callbackReturn (*ArduinoShellKeywordCallback)(ArduinoShell&, const int argc, const char* argv[]); 

class ArduinoShell
{
	private:
		const char* _programName;
		const char* _programVersion;
		
		typedef struct keywordStruct 
		{			
			const char *keyword;
			const char *helpText;
			ArduinoShellKeywordCallback callback;
			bool streamingOutput;
		} keywordStruct;
		
		keywordStruct _supportedKeywords[MAX_SUPPORTED_KEYWORDS];
		unsigned int _supportedKeywordCount;
		
		HardwareSerial *_serialPort;
		
		char _currentCommandBuffer[MAX_COMMAND_LINE_LENGTH];
		
		ArduinoShellKeywordCallback _activeStreamingCallback;
		bool _streamingActivated;
		
		void _printPrompt();
		void _printVersion();
		void _printMemoryInfo();
		
		void _outputHelp();
		
		void _processCommandLine();
		
		void _runCallback(ArduinoShellKeywordCallback callback, const int argc, const char* argv[]);
		
	public:		
		ArduinoShell(HardwareSerial* serialPort);
		
		void setProgramName(const char *programName);
  		void setProgramVersion(const char* programVersion);
		
		void begin(const unsigned long baudRate);	
		
		HardwareSerial *serialPort();		
		
		void registerKeyword(const char* keyword, const char* helpText, ArduinoShellKeywordCallback callback, bool streamingOutput = false);				
		void process();
		
		const float getArgumentAsFloat(const char * arg);
};

extern ArduinoShell Serial3Shell;

template<class T> 
inline ArduinoShell &operator <<(ArduinoShell &obj, T arg) 
{ 
	obj.serialPort()->print(arg); 
	return obj; 	
}


// Specialization for class _BASED
// Thanks to Arduino forum user Ben Combee who suggested this 
// clever technique to allow for expressions like
//   Serial << _HEX(a);

struct _BASED 
{ 
  long val; 
  int base;
  _BASED(long v, int b): val(v), base(b) {}
};

#define _HEX(a)     _BASED(a, HEX)
#define _DEC(a)     _BASED(a, DEC)
#define _OCT(a)     _BASED(a, OCT)
#define _BIN(a)     _BASED(a, BIN)
#define _BYTE(a)    _BASED(a, BYTE)

inline ArduinoShell &operator <<(ArduinoShell &obj, const _BASED &arg)
{ 
	obj.serialPort()->print(arg.val, arg.base); 
	return obj; 
	
} 

#if ARDUINO >= 18
// Specialization for class _FLOAT
// Thanks to Michael Margolis for suggesting a way
// to accommodate Arduino 0018's floating point precision
// feature like this:
//   Serial << _FLOAT(gps_latitude, 6); // 6 digits of precision

struct _FLOAT
{
  float val;
  int digits;
  _FLOAT(double v, int d): val(v), digits(d){}
};

inline ArduinoShell &operator <<(ArduinoShell &obj, const _FLOAT &arg)
{ 
	obj.serialPort()->print(arg.val, arg.digits); 
	return obj; 	
}
#endif

// Specialization for enum _EndLineCode
// Thanks to Arduino forum user Paul V. who suggested this
// clever technique to allow for expressions like
//   Serial << "Hello!" << endl;
enum _EndLineCode { endl };
inline ArduinoShell &operator <<(ArduinoShell &obj, _EndLineCode arg) 
{ 
	obj.serialPort()->println(); 
	return obj; 	
}









class SerialComs : public SubSystem, public Print
{
private:
		virtual void write(uint8_t v);
				
	public:		
  		SerialComs();
  		
  		void setProgramName(const char *programName);
  		void setProgramVersion(const char* programVersion);
		
		ArduinoShell *shell();
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0);
		void process(const unsigned long currentTime);
};

extern SerialComs serialcoms;

#endif // #ifndef __SERIALCOMS_H__