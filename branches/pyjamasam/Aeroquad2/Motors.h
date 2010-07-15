#include "SubSystem.h"
#include "HardwareComponent.h"

class MotorHardware : public HardwareComponent
{
	private:
		unsigned int _motorCount;
	public:
		static const int MinimumMotorCommand = 1100;
		static const int MaximumMotorCommand = 2000;
		
	protected:
		unsigned int _motorOutput[8];
		bool _armed;
		
		void zeroOutputs() 
		{
			//Push every output down below where anything should happen
			for (int i = 0; i < _motorCount; i++)
			{
				_motorOutput[i] = 800;
			}
		}
		
		virtual void writeOutputs()
		{
			//default implimentation does nothing
		}
		
	public:
		MotorHardware(unsigned int motorCount) : HardwareComponent()
		{
			_motorCount = (int)fmin(motorCount, 8);
			_armed = true;
		}

		virtual void initialize()
		{
			this->zeroOutputs();
			HardwareComponent::initialize();
		}	
		
		virtual void process(const unsigned long currentTime)
		{
			if (_armed)
			{
				//Push the outputs out
				this->writeOutputs();
			}
			else
			{
				//Force the outputs to zero to ensure they are off
				this->zeroOutputs();
				this->writeOutputs();
			}
			HardwareComponent::process(currentTime);	
		}
		
		void arm()
		{
			if (!_armed)
			{	
				this->zeroOutputs();
				_armed = true;
			}
		}
		
		void disarm()
		{
			if (_armed)
			{	
				this->zeroOutputs();
				_armed = false;
			}
		}
		
		void setMotorOutput(const unsigned int motorIndex, const unsigned int motorCommand)
		{
			if (motorIndex < _motorCount)
			{
				_motorOutput[motorIndex] = motorCommand;
			}
		}
};

//Uses the APM RC output library to drive the outputs
class MotorHardwareAPM : public MotorHardware
{
	protected:
		void writeOutputs()
		{
			APM_RC.OutputCh(0, _motorOutput[0]);    // Right motor (+) / Right-Back motor (X)
			APM_RC.OutputCh(1, _motorOutput[1]);    // Left motor (+) / Left-Front motor (X)
			APM_RC.OutputCh(2, _motorOutput[2]);   // Front motor (+) / Right-Front motor (X)
			APM_RC.OutputCh(3, _motorOutput[3]);   // Back motor (+) / Left-Back motor (X) 
			  
			// InstantPWM
			APM_RC.Force_Out0_Out1();
			APM_RC.Force_Out2_Out3();
		}
		
	public:
		MotorHardwareAPM() : MotorHardware(4)
		{
			
		}
};

//For just debuggging the output that should go to the motors
class MotorhardwareDebugSerial : public MotorHardware
{
	protected:
		void writeOutputs()
		{
			serialcoms.print(_motorOutput[0]);
			serialcoms.print(",");
			serialcoms.print(_motorOutput[1]);
			serialcoms.print(",");
			serialcoms.print(_motorOutput[2]);
			serialcoms.print(",");
			serialcoms.print(_motorOutput[3]);
			
			serialcoms.println();
		}
		
	public:
		MotorhardwareDebugSerial() : MotorHardware(4)
		{
			
		}
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
		
		Motors() : SubSystem()
		{
			
		}
		
		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{ 
			SubSystem::initialize(frequency, offset);
			
			if (_motorHardware)
			{
				_motorHardware->initialize();
			}
		}
		
		void setHardwareType(const HardwareType hardwareType)
		{
			switch (hardwareType)
			{
				case HardwareTypeAPM:
				{
					_motorHardware = new MotorHardwareAPM();
					break;
				}
				
				case HardwareTypeDebugSerial:
				{
					_motorHardware = new MotorhardwareDebugSerial();
					break;
				}
			}
		}
		
		void setOrientationType(const OrientationType orientationType)
		{
			_orientationType = orientationType;
		}
				
		void process(const unsigned long currentTime)
		{
			if (this->_canProcess(currentTime))
			{
				if (_motorHardware)
				{
					/*int rollTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel1);
					int pitchTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel2);
					int throttleTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel3);
					int yawTransmitterCommand = receiver.channelValue(ReceiverHardware::Channel4);*/
										
					//Check for some basic "Command" stick positions
					/*if (throttleTransmitterCommand < 1100)
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
					}*/
					
					//details given to us by each of the main subsystems (navigation is enabled, and flightcontrol)
					int navigationRollCommand = 0;
					int navigationPitchCommand = 0;
					int navigationYawCommand = 0;
					int navigationThrottleCommand = 0;
					
					//if (flightcontrol.navigationIsEnabled)
					//{
					//	navigationRollCommand = navigation.getRollCommand();
					//	navigationPitchCommand = navigation.getPitchCommand();
					//	navigationYawCommand = navigation.getYawCommand();
					//	navigationThrottleCommand = navigation.getThrottleCommand();
					//}
					
					//Main control system.
					int flightControlRollCommand = flightcontrol.getRollCommand();
					int flightControlPitchCommand = flightcontrol.getPitchCommand();
					int flightControlYawCommand = flightcontrol.getYawCommand();
					int flightControlThrottleCommand = flightcontrol.getThrottleCommand();
					
					//Calculate the mix for each of the motors.
					switch (_orientationType)
					{
						case QuadPlusMotor:
						{
							break;
						}
						
						case QuadXMotor:
						{
							break;
						}
					}
					
					_motorHardware->setMotorOutput(0,flightControlRollCommand);
					_motorHardware->setMotorOutput(1,flightControlPitchCommand);
					_motorHardware->setMotorOutput(2,flightControlYawCommand);
					_motorHardware->setMotorOutput(3,flightControlThrottleCommand);
																				
					
					_motorHardware->process(currentTime);
				}
			}
		}
};