#include "SubSystem.h"
#include "HardwareComponent.h"

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
		ReceiverHardware() : HardwareComponent()
		{
			//get the receiver calibration data from the settings system
			const int *settingsReceiverCalibrationMax = settings.getReceiverCalibrationMax();
			const int *settingsReceiverCalibrationMin = settings.getReceiverCalibrationMin();
				
			for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
			{
				_channelMax[i] = settingsReceiverCalibrationMax[i];
				_channelMin[i] = settingsReceiverCalibrationMin[i];
			}
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			//Scale the raw values to our sane -500 <-> 500 values;
			for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
			{
				_currentReceiverValues[i] = map(_currentRawReceiverValues[i], _channelMin[i], _channelMax[i], -500, 500);
			}
						
			HardwareComponent::process(currentTime);
		}
				
		const int channelValue(ChannelIndex channelIndex)
		{
			return _currentReceiverValues[channelIndex];
		}
		
		
		void startForCalibration()
		{
			//Setup some dummy values so we can actually calibrate correctly.
			for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
			{
				_channelMax[i] = 0;
				_channelMin[i] = 3000;
			}	
			
			serialcoms.debugPrint("Starting Receiver calibration in 1 second...");
			serialcoms.debugPrintln();
			delay(1000);
		}
		
		void performCalibrationCycle(const unsigned long currentTime)
		{
			this->process(currentTime);
			for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
			{
				_channelMax[i] = fmax(_channelMax[i], _currentRawReceiverValues[i]);
				_channelMin[i] = fmin(_channelMin[i], _currentRawReceiverValues[i]);
				serialcoms.debugPrint(_channelMax[i]);
				serialcoms.debugPrint(":");
				serialcoms.debugPrint(_channelMin[i]);
				serialcoms.debugPrint(",");
			}
			serialcoms.debugPrintln();
		}
		
		void calibrationComplete()
		{
			//Store out our channel min and max values so we can load them from EEPROM later
			settings.setReceiverCalibrationMax(_channelMax);
			settings.setReceiverCalibrationMin(_channelMin);
		}
};

class ReceiverHardwareAPM : public ReceiverHardware
{
	public:
		virtual void initialize()
		{
			ReceiverHardware::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			if (APM_RC.GetState())
			{
				//Only process new readings if we actually have new readings.
				for (int i = 0; i < 8; i++)
				{
					_currentRawReceiverValues[i] = APM_RC.InputCh(i);
				}
			}
			
			ReceiverHardware::process(currentTime);	
		}
};



//We supoort a maximum of 0.392699082 radians/s (22.5 degrees/s) using the sticks.
#define MAXIMUM_STICK_RATE_IN_RADIANS_PER_SECOND 0.392699082

//We support up to a 45 degree angle using the sticks.
#define MAXIMUM_STICK_ANGLE_IN_RADIANS 0.785398163

//We support a maximum of 2 meters/second using the sticks.
#define MAXIMUM_STICK_RATE_IN_METERS_PER_SECOND 2.0f


class Receiver : public SubSystem
{
	private:
		ReceiverHardware *_receiverHardware;
		
		//Generic method to scale a normal stick value to a rate or angle.
		const float _scaleStickValue(const int stickValue, const float scaleValue)
		{
			//We have a -500 to 500 range with the normal channel readout.
			return fmap(stickValue, -500, 500, -scaleValue, scaleValue);
		}		
		
	public:
		typedef enum { HardwareTypeAPM = 0 } HardwareType;
		
		Receiver() : SubSystem()
		{
			
		}
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
			
			if (_receiverHardware)
			{
				_receiverHardware->initialize();
			}
		}
		
		void setHardwareType(const HardwareType hardwareType)
		{
			switch (hardwareType)
			{
				case HardwareTypeAPM:
				{
					_receiverHardware = new ReceiverHardwareAPM();
					break;
				}
				
				default:
				{
					serialcoms.debugPrint("ERROR: Unknown Receiver Hardware type selected.");
					serialcoms.debugPrintln();
					break;
				}
			}
		}
		
		virtual void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				if (_receiverHardware)
				{
					_receiverHardware->process(currentTime);
				}
			}
		}
		
		void calibrate()
		{
			if (_receiverHardware)
			{
				if (digitalRead(40))
				{
					//Prepare the receiverhardware for calibration
					_receiverHardware->startForCalibration();
				
					unsigned long currentTime = millis();
				
					while (digitalRead(40))
					{
						currentTime = millis();
						if (this->_canProcess(currentTime))
						{
							_receiverHardware->performCalibrationCycle(currentTime);
						}
					}
				
					_receiverHardware->calibrationComplete();
				}
			}
			else
			{
				serialcoms.debugPrint("ERROR: No Receiver Hardware, unable to calibrate.");
				serialcoms.debugPrintln();
			}
		}
		
		//Accessor for current channel value
		const int channelValue(ReceiverHardware::ChannelIndex channelIndex)
		{
			if (_receiverHardware)
			{
				_receiverHardware->channelValue(channelIndex);
			}
			else
			{
				return 0;
			}
		}
		
		const float channelValueAsAngleInRadians(ReceiverHardware::ChannelIndex channelIndex)
		{
			//Convert the stick position into an angle (in this case radians)
			if (_receiverHardware)
			{
				return this->_scaleStickValue(_receiverHardware->channelValue(channelIndex), MAXIMUM_STICK_ANGLE_IN_RADIANS);
			}
			else
			{
				return 0;
			}
		}
		const float channelValueAsAngleInDegrees(ReceiverHardware::ChannelIndex channelIndex)
		{
			return ToDeg(this->channelValueAsAngleInRadians(channelIndex));
		}
		
		
		const float channelValueAsRateInRadians(ReceiverHardware::ChannelIndex channelIndex)
		{
			//Convert the stick position into an angle (in this case radians/s)
			if (_receiverHardware)
			{	
				return this->_scaleStickValue(_receiverHardware->channelValue(channelIndex), MAXIMUM_STICK_RATE_IN_RADIANS_PER_SECOND);
			}
			else 
			{
				return 0;
			}
		}
		const float channelValueAsRateInDegrees(ReceiverHardware::ChannelIndex channelIndex)
		{
			return ToDeg(this->channelValueAsRateInRadians(channelIndex));
		}
		
		
		const float channelValueAsRateInMeters(ReceiverHardware::ChannelIndex channelIndex)
		{
			//Convert the stick position into an rate (in this case m/s)
			if (_receiverHardware)
			{
				return this->_scaleStickValue(_receiverHardware->channelValue(channelIndex), MAXIMUM_STICK_RATE_IN_METERS_PER_SECOND);
			}
			else 
			{
				return 0;
			}
		}
};