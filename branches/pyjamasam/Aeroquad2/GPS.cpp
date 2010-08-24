#include <WProgram.h>

#include "GPS.h"

#include "SerialComs.h"
	

GPSHardware::GPSHardware() : HardwareComponent()
{
	_lastFix = 0;
}

void GPSHardware::initialize(HardwareSerial *serialPort)
{
	HardwareComponent::initialize();
}	

void GPSHardware::process(const unsigned long currentTime, const byte dataByte)
{
	HardwareComponent::process(currentTime);	
}

unsigned int GPSHardware::baudRate()
{
	return 9600;
}

const GPSHardware::GPSFixType GPSHardware::fixType()
{
	return _fixState;
}

const unsigned long GPSHardware::lastFix()
{
	return _lastFix;
}

const long GPSHardware::longitude()
{
	return _lng;
}

const long GPSHardware::latitude()
{
	return _lat;
}

const int GPSHardware::altitude()
{
	return _alt;	
}

const int GPSHardware::groundSpeed()
{
	return _groundSpeed;
}

const int GPSHardware::groundCourse()
{
	return _groundCourse;
}


void MTKGPSHardware::_resetStateMachine()
{
	_calculatedChecksumA = _calculatedChecksumB = 0;

	_step = 0;
	_class = 0;
	_id = 0;

	_payloadChecksumA = _payloadChecksumB = 0;

 	_payloadLengthHi = 0;
	_payloadLengthLo = 0;
	_payloadCounter = 0;
}

void MTKGPSHardware::_updateRunningCheckup(const byte dataByte)
{
	_calculatedChecksumA += dataByte;
	_calculatedChecksumB += _calculatedChecksumA; 
}

long MTKGPSHardware::_join4Bytes(unsigned char buffer[])
{
  	union long_union {
		int32_t dword;
		uint8_t  byte[4];
	} longUnion;

	longUnion.byte[3] = *buffer;
	longUnion.byte[2] = *(buffer+1);
	longUnion.byte[1] = *(buffer+2);
	longUnion.byte[0] = *(buffer+3);
	
	return(longUnion.dword);
}

void MTKGPSHardware::_parsePacket(void)
{
  	int j;
	//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other MTK classes.. 
	//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
  	if (_class == 0x01) 
  	{
    	switch(_id)	//Checking the MTK ID
    	{
    		case 0x05: //ID Custom
			{
     			j=0;
				_lng = _join4Bytes(&_buffer[j]); // lon*1000000
				j+=4;
				_lat = _join4Bytes(&_buffer[j]); // lat*1000000
				j+=4;
				_alt = _join4Bytes(&_buffer[j]);
				j+=4;
				_groundSpeed = _join4Bytes(&_buffer[j]);
				j+=4;
				_groundCourse = _join4Bytes(&_buffer[j]);
				j+=4;
				_numberOfSatellites = _buffer[j];
				j++;
				_fixState = (GPSHardware::GPSFixType)_buffer[j];
				j++;
				_time = _join4Bytes(&_buffer[j]);
				
				_newData = 1;
      			break;
			}
      	}
    }   
}


unsigned int MTKGPSHardware::baudRate()
{
	return 38400;
}

void MTKGPSHardware::initialize(HardwareSerial *serialPort)
{
	GPSHardware::initialize(serialPort);
	
	//Ensure that the GPS is running in 4hz mode
	//TODO: This doesn't seam to work yet
	serialPort->println("$PMTK220,250*2F");
	
	this->_resetStateMachine();
}

void MTKGPSHardware::process(const unsigned long currentTime, const byte dataByte)
{
	GPSHardware::process(currentTime, dataByte);	
	
	 switch(_step)     //Normally we start from zero. This is a state machine
     {
      	case 0: 
		{
        	if (dataByte == 0xB5)  // MTK sync char 1
			{
          		_step++;   //OH first data packet is correct, so jump to the next step
			}
        	break;
			}

      	case 1:  
		{
        	if (dataByte == 0x62)  // MTK sync char 2
			{
          		_step++;   //ooh! The second data packet is correct, jump to the step 2
			}
        	else 
          	{
				_step = 0;   //Nop, is not correct so restart to step zero and try again.     
			}
        	break;
		}
		
      	case 2:
		{
        	_class = dataByte;
        	_updateRunningCheckup(_class);
        	_step++;
        	break;
		}
		
      	case 3:
		{
        	_id = dataByte;
        	_step = 4;
	        _payloadLengthHi = 26;
			_payloadLengthLo = 0;
			_payloadCounter = 0;

	        _updateRunningCheckup(_id);
        	break;
		}
		
      	case 4:
		{
			if (_payloadCounter < _payloadLengthHi)  // We stay in this state until we reach the payload_length
        	{
          		_buffer[_payloadCounter] = dataByte;
          		_updateRunningCheckup(dataByte);
          		_payloadCounter++;

          		if (_payloadCounter == _payloadLengthHi)
            	{
					_step++;
				}
       		}
        	break;
		}
		
      	case 5:
		{
        	_payloadChecksumA = dataByte;   // First checksum byte
        	_step++;
        	break;
		}
		
      	case 6:
		{
			_payloadChecksumB = dataByte;	// Second checksum byte
		
	  		// We end the GPS read...
        	if((_calculatedChecksumA == _payloadChecksumA) && (_calculatedChecksumB == _payloadChecksumB))   // Verify the received checksum with the generated checksum.. 
			{
	  			this->_parsePacket();
	
				_lastFix = millis();			//record the last time we got a fix.  We can use this to check for a drop out
			}
        	else
		  	{
		  		//bad checksum
				//Ignore this packet
		  	}
		
			//reset things to get ready for the next packet
			this->_resetStateMachine();
        	break;
	  	}
	}
}


GPS::GPS() : SubSystem()
{
}

void GPS::assignSerialPort(HardwareSerial *serialPort)
{
	_serialPort = serialPort;
}

void GPS::setHardwareType(const HardwareType hardwareType)
{
	switch (hardwareType)
	{
		case HardwareTypeMTK:
		{
			_gpsHardware = new MTKGPSHardware();
			break;
		}
		
		default:
		{
			serialcoms.println("ERROR: Unknown GPS Hardware type selected.");					
			break;
		}
	}
}

void GPS::initialize(const unsigned int frequency, const unsigned int offset) 
{ 
	SubSystem::initialize(frequency, offset); 

	if (_serialPort && _gpsHardware)
	{
		_serialPort->begin(_gpsHardware->baudRate());
		_gpsHardware->initialize(_serialPort);
	}
	else
	{
		this->disable();
	}
	
	serialcoms.shell()->registerKeyword("monitorGPS", "monitorGPS", _monitorGPS , true);
}

	void GPS::process(const unsigned long currentTime)
{
	if (this->_canProcess(currentTime))
	{
		SubSystem::recordProcessingStartTime();
		
		unsigned int numberBytesAvailable = _serialPort->available();
		if (numberBytesAvailable > 0)
		{			
			for (int i=0; i < numberBytesAvailable; i++)
		    {
				_gpsHardware->process(currentTime, _serialPort->read());
			}
		}	
		
		SubSystem::recordProcessingEndTime();	
	}
}

//Public accessors for GPS data
const GPSHardware::GPSFixType GPS::fixType()
{
	if (_gpsHardware)
	{
		if (millis() - _gpsHardware->lastFix() <= 2000)
			{	
			return _gpsHardware->fixType();
		}
		else
		{
			//haven't gotten a fix in more then 2 seconds.  This isn't good.
			return GPSHardware::FixNone;
		}
	}
	else
	{
		//No GPS hardware available.
		return GPSHardware::FixNone;
	}
}

const unsigned long GPS::fixAge()
{
	if (_gpsHardware)
	{
		return millis() - _gpsHardware->lastFix();
	}
	else
	{
		return 0;
	}
}

const long GPS::longitude()
{
	if (_gpsHardware)
	{
		return _gpsHardware->longitude();
	}
	else
	{
		return 0;
	}
}

const long GPS::latitude()
{
	if (_gpsHardware)
	{
		return _gpsHardware->latitude();
	}
	else
	{
		return 0;
	}
}

const int GPS::altitude()
{
	if (_gpsHardware)
	{
		return _gpsHardware->altitude();
	}
	else
	{
		return 0;
	}	
}

const int GPS::groundSpeed()
{
	if (_gpsHardware)
	{
		return _gpsHardware->groundSpeed();
	}
	else
	{
		return 0;
	}	
}

const int GPS::groundCourse()
{
	if (_gpsHardware)
	{
		return _gpsHardware->groundCourse();
	}
	else
	{
		return 0;
	}	
}

GPS gps;

const ArduinoShellCallback::callbackReturn _monitorGPS(ArduinoShell &shell, const int argc, const char* argv[])
{
	shell << gps.fixType() << ","
			<< gps.fixAge() << ","
			<< gps.latitude() << ","
			<< gps.longitude() << ","
			<< gps.altitude() << ","
			<< gps.groundSpeed() << ","
			<< gps.groundCourse() << ","
			<< endl;

	return ArduinoShellCallback::Success;	
}
