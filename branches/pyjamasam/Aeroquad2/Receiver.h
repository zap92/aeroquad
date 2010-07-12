#include "SubSystem.h"
#include "HardwareComponent.h"

class ReceiverHardware : public HardwareComponent
{
	protected:
		uint16_t _currentReceiverValues[8];
		
	public:
		typedef enum {Channel1 = 0, Channel2, Channel3, Channel4, Channel5, Channel6, Channel7, Channel8 } ChannelIndex;
		ReceiverHardware() : HardwareComponent()
		{
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			HardwareComponent::process(currentTime);	
		}
		
		const int rawChannelValue(ChannelIndex channelIndex)
		{
			return _currentReceiverValues[channelIndex];
		}
};

class Receiver : public SubSystem
{
	private:
		ReceiverHardware *_receiverHardware;
		
	public:
		typedef enum { HardwareTypeOnboard = 0 } HardwareType;
		
		Receiver() : SubSystem()
		{
			
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
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
				
			}
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				if (_receiverHardware)
				{
					_receiverHardware->process(currentTime);
				}
			}
		}
		
		//Accessor for current channel value
		//This will return a -500 <-> 500 range.  It converts the 1000-2000 range from the RX hardware to a sanner value
		const unsigned int normalizedChannelValue(ReceiverHardware::ChannelIndex channelIndex)
		{
			if (_receiverHardware)
			{
				unsigned int rawReceiverValue = _receiverHardware->channelValue(channelIndex);
				return map(rawReceiverValue, 1000, 2000, -500, 500);
			}
			else
			{
				return 0;
			}
		}
};