#include <WProgram.h>

#include "SerialComs.h"

#include "LanguageExtensions.h"






/*
#define TX_BUFFER_SIZE 80

class FastSerialPort : public Print
{
	private: 
		HardwareSerial *_serialPort;
		unsigned int _serialPortNumber;
		
		unsigned int _tx_buffer[TX_BUFFER_SIZE];
		volatile int _tx_buffer_head;
		volatile int _tx_buffer_tail;
		
		static FastSerialPort *_fastSerialPortList[4];
				
	public:
		
		static FastSerialPort* getFastSerialPort(const unsigned int portNumber)
		{
			return _fastSerialPortList[portNumber];		
		}
		
		FastSerialPort(const unsigned int portNumber)
		{
			_tx_buffer_head = 0;
			_tx_buffer_tail = 0;
			
			switch (portNumber)
			{
				case 0:
				{
					_serialPort = &Serial;
					break;
				}						
				case 3:
				{
					_serialPort = &Serial3;
					break;
				}
			}
			
			_fastSerialPortList[portNumber] = this;
		}
		
		
		//Methods to handle the transmit ring buffer
		const bool hasData()
		{
			return _tx_buffer_tail == _tx_buffer_head;
		}
		
		const unsigned int getNextByteToSend()
		{
				unsigned int data = _tx_buffer[_tx_buffer_tail];
				_tx_buffer_tail = (_tx_buffer_tail + 1) % TX_BUFFER_SIZE;   
				
				return data;
		}
		
		
		//Methods to make this look like a HardwareSerial object
		void begin(const unsigned long baudRate)
		{
			if (_serialPort)
			{
				_serialPort->begin(baudRate);
			} 	
		}
		
		void end()
		{
			if (_serialPort)
			{
				_serialPort->end();
			}		
		}
		
		const unsigned int available()
		{
			if (_serialPort)
			{
				return _serialPort->available();
			}		
			
			return 0;			
		}
		
		const int read()
		{
			if (_serialPort)
			{
				return _serialPort->read();
			}		
		}
		
		void flush()
		{
			if (_serialPort)
			{
				_serialPort->flush();
			}	
		}
    
		void write(uint8_t byteToSend)
		{
  			bool enableInterrupt = false;
  			
  			if (!this->hasData())
  			{
  				//There wasn't any data in the buffer, so the interrupt wasn't enabled.  We need to start it up.
	  			enableInterrupt = true;
  			}
  			
  			unsigned int newHead = (_tx_buffer_head + 1) % TX_BUFFER_SIZE;  // Move to next position in the ring buffer
			
  			if (newHead == _tx_buffer_tail)
	  		{
	  			//Error: Buffer overflow
	  			return;		
	  		}
	  		
  			_tx_buffer[_tx_buffer_head] = byteToSend;  			// Add data to the tx buffer
    		_tx_buffer_head = newHead;              					// Update head pointer

			if (enableInterrupt)
			{
				switch (_serialPortNumber)
				{
					case 0:
					{
						UCSR0B |= _BV(UDRIE0);   // Enable Serial TX interrupt
						break;
					}		
					case 3:
					{
						UCSR3B |= _BV(UDRIE3);   // Enable Serial TX interrupt
						break;	
					}
				}
			}    
	}
};

// Serial0 interrupt
ISR(SIG_USART0_DATA)
{
	FastSerialPort* fsp = FastSerialPort::getFastSerialPort(0);
	
	if (fsp)
	{
		if (fsp->hasData())
		{
			//No data to send.  						
			UCSR0B &= ~(_BV(UDRIE0));    // Disable interrupt
		}		
		else
		{
			//Data to send.  
			UDR0 = fsp->getNextByteToSend();		
		}
	}
}

// Serial3 interrupt
ISR(SIG_USART3_DATA)
{
	FastSerialPort* fsp = FastSerialPort::getFastSerialPort(3);
	
	if (fsp)
	{
		if (fsp->hasData())
		{
			//No data to send.  						
			UCSR3B &= ~(_BV(UDRIE3));    // Disable interrupt
		}		
		else
		{
			//Data to send.  
			UDR3 = fsp->getNextByteToSend();		
		}
	}
}

FastSerialPort *FastSerialPort::_fastSerialPortList[4];

FastSerialPort fastSerial0(0);
FastSerialPort fastSerial3(3);
*/




void ArduinoShell::_printPrompt()
{
		_serialPort->print(PROMPT);
}	

void ArduinoShell::_printVersion()
{
	_serialPort->print(_programName);
	_serialPort->print("sh v");
	_serialPort->println(_programVersion);		
}

void ArduinoShell::_printMemoryInfo()
{	
	_serialPort->print(freeMemory());		
	_serialPort->println();		
}

void ArduinoShell::_outputHelp()
{			
	_serialPort->println("Available commands");
	_serialPort->println("------------------");
	
	_serialPort->println(" - ?/help");
	_serialPort->println(" - v/version");
	_serialPort->println(" - m/meminfo");
	
	for (int j = 0; j < _supportedKeywordCount; j++)
	{
		_serialPort->print(" - ");
		if (_supportedKeywords[j].helpText)
		{
			_serialPort->println(_supportedKeywords[j].helpText);
		}
		else
		{
			_serialPort->println(_supportedKeywords[j].keyword);
		}
		delay(5);
	}						
}		

void ArduinoShell::_runCallback(ArduinoShellKeywordCallback callback, const int argc, const char* argv[])
{
	//Time to call the callback with our new parameters
	ArduinoShellCallback::callbackReturn returnValue = callback(*this, argc, argv);
	if (returnValue != ArduinoShellCallback::Success)
	{
		switch (returnValue)
		{
			case ArduinoShellCallback::InvalidParameterCount:
			{
				_serialPort->println("Error: Invalid parameter count");
				break;
			}
			
			default:
			{
				_serialPort->print("Error: Unknown error (");
				_serialPort->print(returnValue, DEC);
				_serialPort->println(")");
				break;
			}
		}		
	}		
}

void ArduinoShell::_processCommandLine()
{
	int i = 0;
	
	_streamingActivated = false;
	
	// skip leading whitespace
	for(i = 0; _currentCommandBuffer[i] && _currentCommandBuffer[i] == ' '; i++);
	
	//Find the first word (its the keyword)
	char * keyword = _currentCommandBuffer + i;
	
	i = 0;						
	while(keyword[i] != 0 && keyword[i] != ' ')
	{
		// skip ahead to the next word
		i++;
	}
	
	//delimit the keyword
	keyword[i] = 0;			
	char* argumentListStart = keyword + i + 1;
							
	bool foundKeyword = false;			
	//Try and find the keyword is our supported list
	if (strcasecmp(keyword, "help") == 0 || strcasecmp(keyword, "?") == 0 )
	{
		//Special keyword thats internal		
		foundKeyword = true;
		
		//We ignore the rest of the arguments for this keyword.				
		this->_outputHelp();
	}
	else if (strcasecmp(keyword, "version") == 0 || strcasecmp(keyword, "v") == 0 )
	{
		//Special keyword thats internal		
		foundKeyword = true;
		
		//We ignore the rest of the arguments for this keyword.				
		this->_printVersion();
	}
	else if (strcasecmp(keyword, "meminfo") == 0 || strcasecmp(keyword, "m") == 0 )
	{
		//Special keyword thats internal		
		foundKeyword = true;
		
		//We ignore the rest of the arguments for this keyword.				
		this->_printMemoryInfo();
	}
	else
	{								
		for (int j = 0; j < _supportedKeywordCount; j++)
		{
			if (strcasecmp(keyword, _supportedKeywords[j].keyword) == 0)
			{				
				//Found the keyword.  Lets finish the argument processing and call the callback.
				foundKeyword = true;
				
				unsigned int argc = 0;
				const char* argv[MAX_SUPPORTED_ARGUMENTS];	
				
				if (_supportedKeywords[j].streamingOutput)
				{
					//This is a streaming callback.  Ignore the arguments (we don't support streaming with arguments at this point)
					_activeStreamingCallback = _supportedKeywords[j].callback;
					_streamingActivated = true;
				}
				else
				{
					char *foundToken = strtok(argumentListStart, ",");
					while (foundToken)
					{
						//trim the space from the front and back of the token
						for(i = 0; foundToken && foundToken[0] == ' '; foundToken++);
						
						i = 0;
						while(foundToken[i] != 0 && foundToken[i] != ' ')
						{
							// skip ahead to the next word
							i++;
						}
						
						foundToken[i] = 0;
						
						argv[argc++] = foundToken;
						
						foundToken = strtok(NULL, ",");
					}
				}
				
				this->_runCallback(_supportedKeywords[j].callback, argc, argv);				
					
				break;		
			}
		}		
	}
	
	if (!foundKeyword)
	{
		//Unable to find this keyword.  Output and error.		
		_serialPort->println("Error: Unknown command");
	}
	
	if (!_streamingActivated)
	{
		this->_printPrompt();		
	}	
}
	
ArduinoShell::ArduinoShell(HardwareSerial* serialPort)
{
	_serialPort = serialPort;	
	_supportedKeywordCount = 0;		
	
	_activeStreamingCallback = NULL;		
	_streamingActivated = false;			
}	

void ArduinoShell::setProgramName(const char *programName)
{
	_programName = programName;
}

void ArduinoShell::setProgramVersion(const char* programVersion)
{
	_programVersion = programVersion;
}

HardwareSerial *ArduinoShell::serialPort()
{
	return _serialPort;		
}

void ArduinoShell::registerKeyword(const char* keyword, const char* helpText, ArduinoShellKeywordCallback callback, bool streamingOutput)
{
	if (_supportedKeywordCount < MAX_SUPPORTED_KEYWORDS)
	{
		_supportedKeywords[_supportedKeywordCount].keyword = keyword;
		_supportedKeywords[_supportedKeywordCount].helpText = helpText;
		_supportedKeywords[_supportedKeywordCount].callback = callback;
		_supportedKeywords[_supportedKeywordCount].streamingOutput = streamingOutput;
		
		_supportedKeywordCount++;
	}
	else
	{
		_serialPort->println("Error: Maximum supported keywords reached!");
	}				
}

void ArduinoShell::begin(const unsigned long baudRate)
{			
	_serialPort->begin(baudRate);
				
	_serialPort->println();				
	
	this->_printVersion();			
	
	this->_printPrompt();	
}

void ArduinoShell::process()
{
	//ensure we don't read to much and lock up the system in a single pass	
	unsigned int bytesAvailable = constrain(_serialPort->available(), 0, 50);
	
	if (_activeStreamingCallback)
	{
		//When we have an active streaming callback all we do is check to see if there is a byte available.
		if (bytesAvailable >= 1)
		{
			if (_serialPort->read() == 'x')
			{
				//Turn off the streaming feature and switch back to the shell
				_activeStreamingCallback = NULL;
				
				//Back to normal.  Output the prompt
				this->_printPrompt();
			}		
		}
		else
		{
			unsigned int argc = 0;
			const char* argv[1];
			
			this->_runCallback(_activeStreamingCallback, argc, argv);		
		}
	}
	else
	{	
		while(bytesAvailable--)
		{
			char c = _serialPort->read();
			
			if (c == '\r' || c == '\n')
			{
				//End of line.  Process the line for a command
				_serialPort->println();
				if (_currentCommandBuffer[0])
				{
					this->_processCommandLine();
				}
				else
				{
					//No command line.  Just output the prompt again
					this->_printPrompt();		
				}
				
				_currentCommandBuffer[0] = 0;														
			}
			else
			{
				unsigned int currentCommandBufferLength = strlen(_currentCommandBuffer);	
				
				if((c == 0x08 || c == 0x7f) && currentCommandBufferLength != 0)
				{
					//Backspace
					_currentCommandBuffer[currentCommandBufferLength-1] = 0;
					_serialPort->print(0x08, BYTE);
					_serialPort->print(0x20, BYTE);
					_serialPort->print(0x08, BYTE);
				}										
				else if(currentCommandBufferLength < MAX_COMMAND_LINE_LENGTH - 1 && c >= 0x20 && c < 0x7f)
				{
					_currentCommandBuffer[currentCommandBufferLength] = c;
					_currentCommandBuffer[currentCommandBufferLength+1] = 0;
					_serialPort->print(c);
				}					
			}
		}
	}				
}		

const float ArduinoShell::getArgumentAsFloat(const char * arg)
{		
	return atof(arg);	
}	

ArduinoShell Serial3Shell(&Serial3);





void SerialComs::write(uint8_t v)
{
	Serial.write(v);	
}

SerialComs::SerialComs() : SubSystem()
{
				
}

void SerialComs::setProgramName(const char *programName)
{
	Serial3Shell.setProgramName(programName);	
}

void SerialComs::setProgramVersion(const char* programVersion)
{
	Serial3Shell.setProgramVersion(programVersion);		
}

ArduinoShell *SerialComs::shell()
{
	return &Serial3Shell;
}

void SerialComs::initialize(const unsigned int frequency, const unsigned int offset) 
{			
	SubSystem::initialize(frequency, offset);
	
	//Serial port 0 is used for debug
	Serial.begin(BAUD);
	
	//Serial port 3 is used for the shell
	Serial3Shell.begin(BAUD);				
	
	this->enable();
}

void SerialComs::process(const unsigned long currentTime)
{
	if (this->_canProcess(currentTime))
	{								
		Serial3Shell.process();
	}			
}  	

SerialComs serialcoms;

























