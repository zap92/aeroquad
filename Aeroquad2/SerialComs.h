#include "SubSystem.h"

#include <stdio.h>

#define BAUD 115200
#define MAXASSIGNEDSERIALPORTS 2

class SerialComs : public SubSystem
{
	private:
		struct serialPortInformation
		{
			HardwareSerial *serialPort;
			
			bool debugPort;
			
			unsigned long lastRunTime;
			
			unsigned int normalFrequency;
			unsigned int activeFrequency;
		};
		
  		unsigned int _serialPortCount;
		serialPortInformation	_assignedSerialPorts[MAXASSIGNEDSERIALPORTS];
		
		void printNumber(unsigned long n, uint8_t base)
		{
		 	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
			unsigned long i = 0;

			if (n == 0) 
			{
				print('0');
				return;
			} 

			while (n > 0) 
			{
				buf[i++] = n % base;
				n /= base;
			}

			for (; i > 0; i--)
			{
				print((char) (buf[i - 1] < 10 ? '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));
			}
		}
	
	
		void debugPrintNumber(unsigned long n, uint8_t base)
		{
		 	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
			unsigned long i = 0;

			if (n == 0) 
			{
				debugPrint('0');
				return;
			} 

			while (n > 0) 
			{
				buf[i++] = n % base;
				n /= base;
			}

			for (; i > 0; i--)
			{
				debugPrint((char) (buf[i - 1] < 10 ? '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));
			}
		}

		void printFloat(double number, uint8_t digits) 
		{ 
			// Handle negative numbers
		  	if (number < 0.0)
		  	{
		     	print('-');
		     	number = -number;
		  	}

		  	// Round correctly so that print(1.999, 2) prints as "2.00"
		  	double rounding = 0.5;
		  	for (uint8_t i=0; i<digits; ++i)
		    	rounding /= 10.0;

		  	number += rounding;

		  	// Extract the integer part of the number and print it
			unsigned long int_part = (unsigned long)number;
			double remainder = number - (double)int_part;
			print(int_part);

		  	// Print the decimal point, but only if there are digits beyond
		  	if (digits > 0)
		    	print("."); 

		  	// Extract digits from the remainder one at a time
		  	while (digits-- > 0)
		  	{
		    	remainder *= 10.0;
		    	int toPrint = int(remainder);
		    	print(toPrint);
		    	remainder -= toPrint; 
		  	} 
		}
		
		void debugPrintFloat(double number, uint8_t digits) 
		{ 
			// Handle negative numbers
		  	if (number < 0.0)
		  	{
		     	debugPrint('-');
		     	number = -number;
		  	}

		  	// Round correctly so that print(1.999, 2) prints as "2.00"
		  	double rounding = 0.5;
		  	for (uint8_t i=0; i<digits; ++i)
		    	rounding /= 10.0;

		  	number += rounding;

		  	// Extract the integer part of the number and print it
			unsigned long int_part = (unsigned long)number;
			double remainder = number - (double)int_part;
			debugPrint(int_part);

		  	// Print the decimal point, but only if there are digits beyond
		  	if (digits > 0)
		    	debugPrint("."); 

		  	// Extract digits from the remainder one at a time
		  	while (digits-- > 0)
		  	{
		    	remainder *= 10.0;
		    	int toPrint = int(remainder);
		    	debugPrint(toPrint);
		    	remainder -= toPrint; 
		  	} 
		}

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

		//Overloaded from Print
		void write(uint8_t c)
		{
			for (unsigned int i = 0; i < _serialPortCount; i++)
			{
				_assignedSerialPorts[i].serialPort->write(c);
			}
		}
	    void write(const char *str)
		{
	 		while (*str)
				write(*str++);
		}
	    void write(const uint8_t *buffer, size_t size)
		{
			while (size--)
		    	write(*buffer++);
		}

		void debugWrite(uint8_t c)
		{
			for (unsigned int i = 0; i < _serialPortCount; i++)
			{
				if (_assignedSerialPorts[i].debugPort)
				{
					_assignedSerialPorts[i].serialPort->write(c);
				}
			}
		}
	    void debugWrite(const char *str)
		{
	 		while (*str)
				debugWrite(*str++);
		}
	    void debugWrite(const uint8_t *buffer, size_t size)
		{
			while (size--)
		    	debugWrite(*buffer++);
		}
		
		void print(const char str[])
		{
			write(str);
		}
	    void print(char c, int base = BYTE)
		{
			print((long) c, base);
		}
	    void print(unsigned char b , int base = BYTE)
		{
			print((unsigned long) b, base);
		}
	    void print(int n, int base = DEC)
		{
			print((long) n, base);	
		}
	    void print(unsigned int n, int base = DEC)
		{
			print((unsigned long) n, base);	
		}
	    void print(long n, int base = DEC)
		{
			if (base == 0) 
			{
		    	write(n);
		  	}
		 	else if (base == 10) 
			{
		    	if (n < 0) 
				{
		    		print('-');
		      		n = -n;
		    	}
		    	printNumber(n, 10);
		  	} 
			else 
			{
		    	printNumber(n, base);
		  	}
		}
	    void print(unsigned long n, int base = DEC)
		{
			if (base == 0) 
			{
				write(n);
			}
		  	else 
			{
				printNumber(n, base);
			}
		}
	    void print(double n, int digits = 2)
		{
			printFloat(n, digits);
		}
		void println(void)
		{
			print('\r');
			print('\n');  
		}
		
		void debugPrint(const char str[])
		{
			debugWrite(str);
		}
	    void debugPrint(char c, int base = BYTE)
		{
			debugPrint((long) c, base);
		}
	    void debugPrint(unsigned char b , int base = BYTE)
		{
			debugPrint((unsigned long) b, base);
		}
	    void debugPrint(int n, int base = DEC)
		{
			debugPrint((long) n, base);	
		}
	    void debugPrint(unsigned int n, int base = DEC)
		{
			debugPrint((unsigned long) n, base);	
		}
	    void debugPrint(long n, int base = DEC)
		{
			if (base == 0) 
			{
		    	debugWrite(n);
		  	}
		 	else if (base == 10) 
			{
		    	if (n < 0) 
				{
		    		print('-');
		      		n = -n;
		    	}
		    	debugPrintNumber(n, 10);
		  	} 
			else 
			{
		    	debugPrintNumber(n, base);
		  	}
		}
	    void debugPrint(unsigned long n, int base = DEC)
		{
			if (base == 0) 
			{
				debugWrite(n);
			}
		  	else 
			{
				debugPrintNumber(n, base);
			}
		}
	    void debugPrint(double n, int digits = 2)
		{
			debugPrintFloat(n, digits);
		}	
		void debugPrintln(void)
		{
			debugPrint('\r');
			debugPrint('\n');  
		}


};

SerialComs serialcoms;
























