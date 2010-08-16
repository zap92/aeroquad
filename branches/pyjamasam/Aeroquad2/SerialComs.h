#include "SubSystem.h"

#include <stdio.h>

#define BAUD 115200
#define MAXASSIGNEDSERIALPORTS 2

class FastSerialPort : public HardwareSerial
{
	
};

class SerialComs : public SubSystem
{
	private:
		struct serialPortInformation
		{
			FastSerialPort *serialPort;
			
			bool debugPort;
			
			unsigned long lastRunTime;
			
			unsigned int normalFrequency;
			unsigned int activeFrequency;
		};
		
  		unsigned int _serialPortCount;
		serialPortInformation	_assignedSerialPorts[MAXASSIGNEDSERIALPORTS];

	public:
		
  		SerialComs() : SubSystem()
		{
			_serialPortCount = 0;
		}
		
		void assignSerialPort(HardwareSerial *serialPort, const bool debugPort = false)
		{
			if (_serialPortCount < MAXASSIGNEDSERIALPORTS)
			{
				_assignedSerialPorts[_serialPortCount].serialPort = serialPort;
				_assignedSerialPorts[_serialPortCount].debugPort = debugPort;
				
				_serialPortCount++;
			}
		}

		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{
			if (_serialPortCount > 0)
			{
				for(int i = 0; i < _serialPortCount; i++)
				{
					_assignedSerialPorts[i].lastRunTime = offset;
				
					_assignedSerialPorts[i].normalFrequency = frequency;
					_assignedSerialPorts[i].activeFrequency = frequency;    
				
					_assignedSerialPorts[i].serialPort->begin(BAUD);
				}
				
				this->enable();
			}
			else
			{
				//No serial ports.  Disable this subsystem
				this->disable();
			}
		}

		void process(const unsigned long currentTime)
		{
			//since we are doing customized timing we need to make sure we are enabled
			if (this->enabled())
			{
				for (unsigned int i = 0; i < _serialPortCount; i++)
				{
					if (currentTime > (_assignedSerialPorts[i].lastRunTime + _assignedSerialPorts[i].activeFrequency))
					{	
						//Check for any incomming commands
						
						//Send any outgoing telementry data

						_assignedSerialPorts[i].lastRunTime = currentTime;
					}
				}
			}
  		}
};

SerialComs serialcoms;
























