#ifndef __MOTORS_H__
#define __MOTORS_H__

#include "SubSystem.h"
#include "HardwareComponent.h"

#include "SerialComs.h"

class MotorHardware : public HardwareComponent
{
	private:
		unsigned int _motorCount;
		
		static const int _disarmedMotorCommand = 1000;
		static const int _minimumMotorCommand = _disarmedMotorCommand + 100;
		static const int _maximumMotorCommand = 2000;
		
	protected:
		unsigned int _motorOutput[8];
		bool _armed;
		
		void zeroOutputs();
		
		virtual void writeOutputs();
		
	public:
		MotorHardware(unsigned int motorCount);
		
		const unsigned int getMotorCount();

		virtual void initialize();		
		virtual void process(const unsigned long currentTime);
		void processForESCCalibration();
		
		void arm();
		void disarm();
		
		const bool armed();
		
		void setMotorOutput(const unsigned int motorIndex, const int motorCommand);
		const unsigned int* getMotorOutput();
		
		
		void setAllMotorOutputToMax();
		void setAllMotorOutputToDisarmed();
};

//Uses the APM RC output library to drive the outputs
class MotorHardwareAPM : public MotorHardware
{
	protected:
		virtual void writeOutputs();
		
	public:
		MotorHardwareAPM();
};

//For just debuggging the output that should go to the motors
class MotorhardwareDebugSerial : public MotorHardware
{
	protected:
		virtual void writeOutputs();
		
	public:
		MotorhardwareDebugSerial();
};

class Motors : public SubSystem
{
	public:
		typedef enum { HardwareTypeAPM = 0, HardwareTypeDebugSerial } HardwareType;
		typedef enum { QuadPlusMotor = 0, QuadXMotor } OrientationType;
	
	private:
		MotorHardware *_motorHardware;
		OrientationType _orientationType;
		
	public:		
		Motors();
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0);
		virtual void process(const unsigned long currentTime);	
		
		const bool armed();	
		
		void setHardwareType(const HardwareType hardwareType);		
		void setOrientationType(const OrientationType orientationType);
		
		MotorHardware *motorHardware();
};

extern Motors motors;

const ArduinoShellCallback::callbackReturn _calibrateESCs(ArduinoShell &shell, const int argc, const char* argv[]);
const ArduinoShellCallback::callbackReturn _monitorMotors(ArduinoShell &shell, const int argc, const char* argv[]);

#endif //#ifndef __MOTORS_H__