#ifndef __GPS_H__
#define __GPS_H__

#include "SubSystem.h"
#include "HardwareComponent.h"

#include "SerialComs.h"

class GPSHardware : public HardwareComponent
{
	public:
		typedef enum { FixNone = 0, Fix2D, Fix3D } GPSFixType;
	
	
	protected:
		unsigned long _lastFix;
		
	 	long _lng;
		long _lat;
		
		int _alt;
		
		int _groundSpeed;
		int _groundCourse;
		
		unsigned int _numberOfSatellites;
		GPSFixType _fixState;
		
		unsigned long _time;
		
		bool _newData;
		
	public:
		GPSHardware();

		virtual void initialize(HardwareSerial *serialPort);		
		virtual void process(const unsigned long currentTime, const byte dataByte);
		
		virtual unsigned int baudRate();
		
		virtual const GPSFixType fixType();		
		const unsigned long lastFix();		
		const long longitude();		
		const long latitude();		
		const int altitude();		
		const int groundSpeed();		
		const int groundCourse();
		
};

class MTKGPSHardware : public GPSHardware
{
	private:
		uint8_t _calculatedChecksumA;     // Running checksum of the packet
		uint8_t _calculatedChecksumB;
		
		uint8_t _step;
		uint8_t _class;
		uint8_t _id;

		uint8_t _payloadChecksumA;		//packet checksum received over the wire
		uint8_t _payloadChecksumB;
		
		uint8_t _payloadLengthHi;
		uint8_t _payloadLengthLo;
		uint8_t _payloadCounter;
		
		uint8_t _buffer[60];
		
		void _resetStateMachine();		
		void _updateRunningCheckup(const byte dataByte);		
		long _join4Bytes(unsigned char buffer[]);		
		void _parsePacket(void);
	
	public:
		virtual unsigned int baudRate();		
		virtual void initialize(HardwareSerial *serialPort);		
		virtual void process(const unsigned long currentTime, const byte dataByte);
};

class GPS : public SubSystem
{
	private:
		HardwareSerial *_serialPort;
		GPSHardware *_gpsHardware;

	public:
		typedef enum { HardwareTypeNMEA = 0, HardwareTypeUBOX, HardwareTypeMTK } HardwareType;
		
		GPS();

		void assignSerialPort(HardwareSerial *serialPort);		
		void setHardwareType(const HardwareType hardwareType);
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0) ;
  		virtual void process(const unsigned long currentTime);
		
		//Public accessors for GPS data
	    const GPSHardware::GPSFixType fixType();		
		const unsigned long fixAge();		
		const long longitude();		
		const long latitude();		
		const int altitude();		
		const int groundSpeed();		
		const int groundCourse();
};

extern GPS gps;

const ArduinoShellCallback::callbackReturn _monitorGPS(ArduinoShell &shell, const int argc, const char* argv[]);

#endif //#ifndef __GPS_H__