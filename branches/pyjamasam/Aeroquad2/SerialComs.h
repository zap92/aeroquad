#include "SubSystem.h"

#define BAUD 115200
#define MAXASSIGNEDSERIALPORTS 4

#define USESERIALDEBUG

#ifdef USESERIALDEBUG
#define DEBUGSERIALPRINT(value) {serialcoms.debugPrint(value);}
#define DEBUGSERIALPRINTLN(value) {serialcoms.debugPrintln(value);}
#else
#define DEBUGSERIALPRINT(value)
#define DEBUGSERIALPRINTLN(value)
#endif

class SerialComs : public SubSystem
{
	private:
  		unsigned int _serialPortCount;
		HardwareSerial *_serialPorts[MAXASSIGNEDSERIALPORTS];
		bool _serialPortDebugFlag[MAXASSIGNEDSERIALPORTS];

		unsigned char _lastTelemetryType[MAXASSIGNEDSERIALPORTS];
		unsigned char _fastTransferOn[MAXASSIGNEDSERIALPORTS];

		unsigned long _lastRunTime[MAXASSIGNEDSERIALPORTS];
		unsigned int _normalFrequency[MAXASSIGNEDSERIALPORTS];
		unsigned int _activeFrequency[MAXASSIGNEDSERIALPORTS];

		float _readFloatString(HardwareSerial *serialPort) 
		{
			unsigned char index = 0;
			unsigned char timeout = 0;
			char data[128] = "";

			do 
			{
				if (serialPort->available() == 0) 
				{
					delay(10);
					timeout++;
				}
				else 
				{
					data[index] = serialPort->read();
					timeout = 0;
					index++;
				}
			}  
			while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
			
			return atof(data);
		}

  		void _outputComma(HardwareSerial *serialPort) 
		{
			serialPort->print(',');
		}
  
  		void _printBinaryInteger(int data, HardwareSerial *serialPort) 
		{
			serialPort->print(data >> 8, BYTE); //MSB
			serialPort->print(data & 0xff, BYTE);  //LSB
		}


	public:
		
  		SerialComs() : SubSystem()
		{
			_serialPortCount = 0;
		}
		
		void assignSerialPort(HardwareSerial *serialPort, bool isDebug = false)
		{
			if (_serialPortCount <= MAXASSIGNEDSERIALPORTS)
			{
				_serialPorts[_serialPortCount] = serialPort;
				_serialPortDebugFlag[_serialPortCount] = isDebug;
				
				_serialPortCount++;
			}
		}

		void initialize(const unsigned int frequency, const unsigned int offset = 0) 
		{
			for(int i = 0; i< MAXASSIGNEDSERIALPORTS; i++)
			{
				_lastTelemetryType[i] = 'X';
				_fastTransferOn[i] = 0;
				_lastRunTime[i] = offset;
				_normalFrequency[i] = frequency;
				_activeFrequency[i] = frequency;    
			}

			if (_serialPortCount > 0)
			{
				for (unsigned int i = 0; i < _serialPortCount; i++)
				{
					_serialPorts[i]->begin(BAUD);
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
				//Check to see if any of the serial ports are running in fast transfer mode
				for (unsigned int i = 0; i < _serialPortCount; i++)
				{
					if (currentTime > (_lastRunTime[i] + _activeFrequency[i]))
					{	
						//Check for any incomming command
						/*_readCommand(i);

						//This serial port has crossed the time threshold and has somthing to do
						if (_fastTransferOn[i])
						{
							//Fast binary telemetry mode
							_sendBinaryTelemetry(i);
						}
						else
						{
							//Normal ascii telemetry mode
							_sendAsciiTelemetry(i);
						}*/

						_lastRunTime[i] = currentTime;
					}
				}
			}
  		}

		void debugPrint(const char* value)
		{
			for (unsigned int i = 0; i < _serialPortCount; i++)
			{
				if (_serialPortDebugFlag[i])
				{
					_serialPorts[i]->print(value);
				}
			}
		}
		
		void debugPrint(const float value)
		{
			for (unsigned int i = 0; i < _serialPortCount; i++)
			{
				if (_serialPortDebugFlag[i])
				{
					_serialPorts[i]->print(value);
				}
			}
		}
		
		void debugPrint(const int value)
		{
			for (unsigned int i = 0; i < _serialPortCount; i++)
			{
				if (_serialPortDebugFlag[i])
				{
					_serialPorts[i]->print(value);
				}
			}
		}
		
		void debugPrintln(const char* value)
		{
			for (unsigned int i = 0; i < _serialPortCount; i++)
			{
				if (_serialPortDebugFlag[i])
				{
					_serialPorts[i]->println(value);
				}
			}
		}
};

























