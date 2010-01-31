#include "SubSystem.h"
#include "HardwareComponent.h"

class ReceiverHardware : public HardwareComponent
{
	protected:
		uint16_t _currentReceiverValues[6];
		
		float _transmitterFactor;
		
	public:
		typedef enum {Channel1 = 0, Channel2 = 1, Channel3 = 2, Channel4 = 3, Channel5 = 4, Channel6 = 5 } ChannelIndex;
		ReceiverHardware() : HardwareComponent()
		{
			_transmitterFactor = 0.5;
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			HardwareComponent::process(currentTime);	
		}
		
		const int channelValue(ChannelIndex channelIndex)
		{
			return _currentReceiverValues[channelIndex];
		}
};

class I2CReceiver : public ReceiverHardware
{
	private:
		unsigned int _I2CAddress;
		
	public:
		I2CReceiver() : ReceiverHardware()
		{
			_I2CAddress = 0x63;
		}
		
		virtual void initialize()
		{
			ReceiverHardware::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			static byte bytesRead[12];
			
			Wire.requestFrom(_I2CAddress, 12);
			if (Wire.available() == 12)
			{
				for (int i = 0; i < 12 ; i++)
				{
					byte byteRead = Wire.receive();
					bytesRead[i] = byteRead;
				}

				memcpy(_currentReceiverValues, bytesRead, 12);
			}
			
			/*for (int i = 0; i < 6; i++)
			{
				if (i > 0) DEBUGSERIALPRINT(",");
				DEBUGSERIALPRINT((int)_currentReceiverValues[i]);
			}
			DEBUGSERIALPRINTLN("");*/
			
			ReceiverHardware::process(currentTime);	
		}
};

class FakeTestReceiver : public ReceiverHardware
{
	public:
		FakeTestReceiver() : ReceiverHardware()
		{
		}
		
		virtual void initialize()
		{
			ReceiverHardware::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			for (int i = 0; i < 6 ; i++)
			{
				_currentReceiverValues[i] = 1500;
			}

			ReceiverHardware::process(currentTime);	
		}
};

class Receiver : public SubSystem
{
	private:
		ReceiverHardware *_receiverHardware;
		
	public:
		typedef enum { HardwareTypeOnboard = 0, HardwareTypeI2C = 1, HardwareTypeFake = 99} HardwareType;
		
		Receiver() : SubSystem()
		{
			
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
		}
		
		void setHardwareType(const HardwareType hardwareType)
		{
			switch (hardwareType)
			{
				case HardwareTypeI2C:
				{
					_receiverHardware = new I2CReceiver();
					break;
				}
				
				case HardwareTypeFake:
				{
					_receiverHardware = new FakeTestReceiver();
				}
			}
			
			if (_receiverHardware)
			{
				_receiverHardware->initialize();
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
		const unsigned int channelValue(ReceiverHardware::ChannelIndex channelIndex)
		{
			if (_receiverHardware)
			{
				return _receiverHardware->channelValue(channelIndex);
			}
			else
			{
				return 0;
			}
		}
};