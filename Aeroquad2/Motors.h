#include "SubSystem.h"
#include "HardwareComponent.h"

class MotorHardware : public HardwareComponent
{
	public:
		typedef enum { OrientationTypePlus4Motor = 0, OrientationTypeX4Motor = 1, OrientationType8Motor} OrientationType;
		static const int MinimumMotorCommand = 1100;
		static const int MaximumMotorCommand = 2000;
	
	private:
		OrientationType _orientationType;
	
	protected:
		int _motorOutput[8];
		bool _armed;
		
		void zeroOutputs() 
		{
			for (int i = 0; i < 8; i++)
			{
				_motorOutput[i] = 10;
			}
		}
		
		virtual void writeOutputs()
		{
			
		}
		
	public:
		MotorHardware() : HardwareComponent()
		{
			_armed = false;
		}

		virtual void initialize()
		{
			HardwareComponent::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			HardwareComponent::process(currentTime);	
		}
		
		void arm()
		{
			if (!_armed)
			{
				DEBUGSERIALPRINTLN("Arming");	
				this->zeroOutputs();
				_armed = true;
			}
		}
		
		void disarm()
		{
			if (_armed)
			{
				DEBUGSERIALPRINTLN("Disarming");	
				this->zeroOutputs();
				_armed = false;
			}
		}
		
		void setOrientationType(const MotorHardware::OrientationType orientationType)
		{
			_orientationType = orientationType;
		}
		
		const MotorHardware::OrientationType orientationType()
		{
			return _orientationType;
		}
};

class MotorControlOnboard : public MotorHardware
{
	private:
		unsigned int _motorPins[8];
		
	protected:
		void writeOutputs()
		{
			for (int i = 0; i < 8;i ++)
			{
				int rawOutputValue = constrain(_motorOutput[i], MinimumMotorCommand, MaximumMotorCommand);
				int analogOutputValue = map(rawOutputValue, 1000, 2000, 0, 255);
								
				if (i >= 4)
				{
					if (i > 4) DEBUGSERIALPRINT(",");
					DEBUGSERIALPRINT(analogOutputValue);
				}
				
				analogWrite(_motorPins[i], analogOutputValue);
			}	
			DEBUGSERIALPRINTLN("");
		}
		
	public:
		MotorControlOnboard() : MotorHardware()
		{
			//Pins used by the shield for motor output
			_motorPins[0] = 2;
			_motorPins[1] = 3;
			_motorPins[2] = 4;
			_motorPins[3] = 5;
			_motorPins[4] = 6;
			_motorPins[5] = 7;
			_motorPins[6] = 8;
			_motorPins[7] = 9;
		}
		
		void initialize()
		{
			MotorHardware::initialize();
			
			this->zeroOutputs();
			this->writeOutputs();
		}
		
		void process(const unsigned long currentTime)
		{
			unsigned int throttleCommand = receiver.channelValue(ReceiverHardware::Channel3);
			
			int flightControlPitchCommand = flightcontrol.pitchCommand();
			int flightControlRollCommand = flightcontrol.rollCommand();
			int flightControlYawCommand = flightcontrol.yawCommand();
			
			this->zeroOutputs();
			
			if (_armed)
			{
				switch (this->orientationType())
				{
					case OrientationTypePlus4Motor:
					{
						//Front
						_motorOutput[4] = (throttleCommand - flightControlPitchCommand + flightControlYawCommand);
					
						//Back
						_motorOutput[5] = (throttleCommand + flightControlPitchCommand + flightControlYawCommand);
					
						//Left
						_motorOutput[6] = (throttleCommand + flightControlRollCommand - flightControlYawCommand);
					
						//Right
						_motorOutput[7] = (throttleCommand - flightControlRollCommand - flightControlYawCommand);
						break;
					}
				
					case OrientationTypeX4Motor:
					{
						_motorOutput[4] = constrain(throttleCommand - flightControlPitchCommand + flightControlRollCommand - flightControlYawCommand, MinimumMotorCommand, MaximumMotorCommand);
						_motorOutput[5] = constrain(throttleCommand - flightControlPitchCommand - flightControlRollCommand + flightControlYawCommand, MinimumMotorCommand, MaximumMotorCommand);
						_motorOutput[6] = constrain(throttleCommand + flightControlPitchCommand + flightControlRollCommand + flightControlYawCommand, MinimumMotorCommand, MaximumMotorCommand);
						_motorOutput[7] = constrain(throttleCommand + flightControlPitchCommand - flightControlRollCommand - flightControlYawCommand, MinimumMotorCommand, MaximumMotorCommand);
						break;
					}
				}
			}
			
			this->writeOutputs();
			
			MotorHardware::process(currentTime);
		}
};

class Motors : public SubSystem
{
	private:
		MotorHardware *_motorHardware;
	
	public:
		typedef enum { HardwareTypeOnboard = 0, HardwareTypeI2C = 1} HardwareType;
		Motors() : SubSystem()
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
				case HardwareTypeOnboard:
				{
					_motorHardware = new MotorControlOnboard();
					break;
				}
			}
			
			if (_motorHardware)
			{
				_motorHardware->initialize();
			}
		}
		
		void setOrientationType(const MotorHardware::OrientationType orientationType)
		{
			if (_motorHardware)
			{
				_motorHardware->setOrientationType(orientationType);
			}
		}
		
		const MotorHardware::OrientationType orientationType()
		{
			if (_motorHardware)
			{
				return _motorHardware->orientationType();
			}
			else
			{
				return MotorHardware::OrientationTypePlus4Motor;
			}
		}
		
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				if (_motorHardware)
				{
					int rollTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel1);
					int pitchTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel2);
					int throttleTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel3);
					int yawTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel4);
					
					//DEBUGSERIALPRINT(throttleTransmitterCommand);
					//DEBUGSERIALPRINT(",");
					//DEBUGSERIALPRINT(yawTransmitterCommand);
					//DEBUGSERIALPRINTLN("");
					
					//Check for some basic "Command" stick positions
					if (throttleTransmitterCommand < 1100)
					{
						if (yawTransmitterCommand < 1100)
						{
							//Throttle/Yaw Stick is in the bottom left position.  Disarm the motors
							_motorHardware->disarm();
						}
						else if (yawTransmitterCommand > 1900)
						{
							//Throttle/Yaw stick is int he bottom right position.  Arm the motors
							_motorHardware->arm();
						}
					}
					
					_motorHardware->process(currentTime);
				}
			}
		}
};