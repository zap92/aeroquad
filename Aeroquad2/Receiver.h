#ifndef __RECEIVER_H__
#define __RECEIVER_H__

#include "SubSystem.h"
#include "HardwareComponent.h"

#include "SerialComs.h"

#define NUMBER_OF_CHANNELS 8

class ReceiverHardware : public HardwareComponent
{	
	protected:
		unsigned int _currentRawReceiverValues[NUMBER_OF_CHANNELS];
		int _channelMax[NUMBER_OF_CHANNELS];
		int _channelMin[NUMBER_OF_CHANNELS];
		
		int _currentReceiverValues[NUMBER_OF_CHANNELS];
		
		
	public:
		typedef enum {Channel1 = 0, Channel2, Channel3, Channel4, Channel5, Channel6, Channel7, Channel8 } ChannelIndex;
		ReceiverHardware();

		virtual void initialize();		
		virtual void process(const unsigned long currentTime);
				
		const int channelValue(ChannelIndex channelIndex);
				
		void startCalibration();		
		void performCalibrationCycle();		
		void calibrationComplete();
		
		const int* channelMaxValues();
		const int* channelMinValues();
};

class ReceiverHardwareAPM : public ReceiverHardware
{
	public:
		virtual void initialize();		
		virtual void process(const unsigned long currentTime);
};



//We supoort a maximum of 1.04719755 radians/s (60 degrees/s) using the sticks.
#define MAXIMUM_STICK_RATE_IN_RADIANS_PER_SECOND 1.04719755

//We support up to a 45 degree angle using the sticks.
#define MAXIMUM_STICK_ANGLE_IN_RADIANS 0.785398163

//We support a maximum of 2 meters/second using the sticks.
#define MAXIMUM_STICK_RATE_IN_METERS_PER_SECOND 2.0f


class Receiver : public SubSystem
{
	private:
		ReceiverHardware *_receiverHardware;
		
		//Generic method to scale a normal stick value to a rate or angle.
		const float _scaleStickValue(const int stickValue, const float scaleValue);
		
	public:
		typedef enum { HardwareTypeAPM = 0 } HardwareType;
		
		Receiver();
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0);
		
		void setHardwareType(const HardwareType hardwareType);
		
		virtual void process(const unsigned long currentTime);
		
		void startCalibration();		
		void performCalibrationCycle();		
		void calibrationComplete();
		
		const int* channelMaxValues();
		const int* channelMinValues();
		
		//Accessor for current channel value
		const int channelValue(ReceiverHardware::ChannelIndex channelIndex);
		
		const float channelValueAsAngleInRadians(ReceiverHardware::ChannelIndex channelIndex);
		const float channelValueAsAngleInDegrees(ReceiverHardware::ChannelIndex channelIndex);
				
		const float channelValueAsRateInRadians(ReceiverHardware::ChannelIndex channelIndex);
		const float channelValueAsRateInDegrees(ReceiverHardware::ChannelIndex channelIndex);
				
		const float channelValueAsRateInMeters(ReceiverHardware::ChannelIndex channelIndex);
};

extern Receiver receiver;

const ArduinoShellCallback::callbackReturn _calibrateReceiver(ArduinoShell &shell, const int argc, const char* argv[]);
const ArduinoShellCallback::callbackReturn _monitorReceiver(ArduinoShell &shell, const int argc, const char* argv[]);

#endif //#ifndef __RECEIVER_H__