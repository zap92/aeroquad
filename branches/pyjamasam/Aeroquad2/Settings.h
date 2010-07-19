#include <avr/eeprom.h>

//Some fancy macros to make it easier to read and write settings
//And it saves some typing when defining new settings.
#define SETTING_TYPE_FLOAT_DEFINATION(name, size) float _##name

#define SETTING_TYPE_FLOAT_DEFAULT(name, defaultData) {\
		_settingsDataStore._##name = defaultData;\
	}\
}

#define SETTING_TYPE_FLOAT_IMPLIMENTATION(name) void set##name(const float newData)\
{ \
	_settingsDataStore._##name = newData;\
	_reCalculateCheckSum();\
	_save();\
}\
const float get##name()\
{\
	return _settingsDataStore._##name;\
}




#define SETTING_TYPE_PID_DEFINATION(name) PIDParameters _##name

#define SETTING_TYPE_PID_DEFAULT(name, defaultData) {\
		_settingsDataStore._##name.P = defaultData.P;\
		_settingsDataStore._##name.I = defaultData.I;\
		_settingsDataStore._##name.D = defaultData.D;\
		_settingsDataStore._##name.windupGuard = defaultData.windupGuard;\
}

#define SETTING_TYPE_PID_IMPLIMENTATION(name) void set##name(const PIDParameters *newData)\
{ \
	_settingsDataStore._##name.P = newData->P;\
	_settingsDataStore._##name.I = newData->I;\
	_settingsDataStore._##name.D = newData->D;\
	_settingsDataStore._##name.windupGuard = newData->windupGuard;\
	\
	_reCalculateCheckSum();\
	_save();\
}\
const PIDParameters *get##name()\
{\
	return &_settingsDataStore._##name;\
}




#define SETTING_TYPE_INT_ARRAY_DEFINATION(name, size) int _##name[size]

#define SETTING_TYPE_INT_ARRAY_DEFAULT(name, defaultData) {\
	unsigned int arraySize = sizeof(_settingsDataStore._##name) / sizeof(int);\
	for (int i = 0; i < arraySize; i++) \
	{\
		_settingsDataStore._##name[i] = defaultData[i];\
	}\
}

#define SETTING_TYPE_INT_ARRAY_IMPLIMENTATION(name) void set##name(const int *newData)\
{ \
   	unsigned int arraySize = sizeof(_settingsDataStore._##name) / sizeof(int);\
	for (int i = 0; i < arraySize; i++) \
	{\
		_settingsDataStore._##name[i] = newData[i];\
	}\
	_reCalculateCheckSum();\
	_save();\
}\
const int *get##name()\
{\
	return _settingsDataStore._##name;\
}





#define HOUSEKEEPINGSIZE 3
#define SETTINGS_LOCATION 0x00

#define SETTINGS_VERSION 1

class Settings
{
	public:
		struct PIDParameters
		{
			float P;
			float I;
			float D;
			
			float windupGuard;
		};
	
	
	private:
		bool _online;
      
  		struct _settingsDataStoreStructure
		{
			//Housekeeping data
			byte version;
			unsigned int checksum;

			//Settings data - Add new settings below
			SETTING_TYPE_INT_ARRAY_DEFINATION(ReceiverCalibrationMax, 8);
			SETTING_TYPE_INT_ARRAY_DEFINATION(ReceiverCalibrationMin, 8);
			
			SETTING_TYPE_PID_DEFINATION(RollRatePIDParameters);
			SETTING_TYPE_PID_DEFINATION(PitchRatePIDParameters);
			SETTING_TYPE_PID_DEFINATION(YawRatePIDParameters);
			SETTING_TYPE_PID_DEFINATION(ThrottleRatePIDParameters);
			
			SETTING_TYPE_PID_DEFINATION(RollAnglePIDParameters);
			SETTING_TYPE_PID_DEFINATION(PitchAnglePIDParameters);
			SETTING_TYPE_PID_DEFINATION(HeadingPIDParameters);
			SETTING_TYPE_PID_DEFINATION(AltitudePIDParameters);
		};

  		_settingsDataStoreStructure _settingsDataStore;

		unsigned int _KIRSP_cal_CRC16(unsigned char *CRC16_Data_Array, unsigned int CRC16_Data_Array_Len)
		{
			static unsigned char KIRSP_CRC16_Hi_Byte  = 0xFF;  // Do not modify
			static unsigned char KIRSP_CRC16_Low_Byte = 0xFF;  // Do not modify

			unsigned int x = 0xFFFF;
			unsigned int y;
			int i;

			x ^= *CRC16_Data_Array;
			for (i = 0; i < 8; ++i)
			{
				if (x & 1)
				{
					x = (x >> 1) ^ 0xA001; // <----The key
				}
				else
				{
					x = (x >> 1);
				}
			}

			KIRSP_CRC16_Hi_Byte = highByte(x);
			KIRSP_CRC16_Low_Byte = lowByte(x);
			y = x;  

			CRC16_Data_Array_Len--;
			*CRC16_Data_Array++;  
			while (CRC16_Data_Array_Len--)
			{
				y ^= *CRC16_Data_Array;
				for (i = 0; i < 8; ++i)
				{
					if (y & 1)
					{
						y = (y >> 1) ^ 0xA001; // <---The Key
					}
					else
					{
						y = (y >> 1);
					}
				}
				KIRSP_CRC16_Hi_Byte = highByte(y);
				KIRSP_CRC16_Low_Byte = lowByte(y);  
				*CRC16_Data_Array++;
			}  
			
			KIRSP_CRC16_Hi_Byte = lowByte(y);   // write to global variable
			KIRSP_CRC16_Low_Byte = highByte(y); // write to global variable
			
			return y;
		} // end of KIRSP_cal_CRC16 function

		void _reCalculateCheckSum()
		{
			//Don't include the version, length, or CRC in the CRC calc
			unsigned int newChecksum = _KIRSP_cal_CRC16((unsigned char*)&_settingsDataStore + HOUSEKEEPINGSIZE, sizeof(_settingsDataStoreStructure) - HOUSEKEEPINGSIZE);

			_settingsDataStore.checksum = newChecksum;
		}
  
		void _save()
		{
			eeprom_write_block((unsigned char*)&_settingsDataStore, (void*)SETTINGS_LOCATION, sizeof(_settingsDataStoreStructure));
		}

	public:	
  		Settings(unsigned int spaceAllowedToUse = 1024)
		{
			_settingsDataStore.version = SETTINGS_VERSION;

			_settingsDataStore.checksum = 0;

			if (sizeof(_settingsDataStoreStructure) > spaceAllowedToUse)
			{
				//Settings take up more room then we are allowed to use.  Throw an error and just give up.
				//Flag the settings as offline.  This disables all writes.
				_online = false;
			}
			else
			{
				_online = true;
			}
		}

		void initialize()
		{
			if (_online)
			{
				_settingsDataStoreStructure settingsInEEPROM;
				eeprom_read_block(&settingsInEEPROM, (const void*)SETTINGS_LOCATION, sizeof(_settingsDataStoreStructure));

				if (settingsInEEPROM.version != _settingsDataStore.version)
				{
					//Version mismatch.  We need to re-init
					serialcoms.debugPrint("Settings version mismatch: ");
					serialcoms.debugPrint(_settingsDataStore.version);
					serialcoms.debugPrint(" != ");
					serialcoms.debugPrint(settingsInEEPROM.version);
					serialcoms.debugPrintln();

					setupDefaults();
				}
				else
				{
					unsigned int checksumFromLoadedData = _KIRSP_cal_CRC16((unsigned char*)&settingsInEEPROM + HOUSEKEEPINGSIZE, sizeof(_settingsDataStoreStructure) - HOUSEKEEPINGSIZE);

					if (checksumFromLoadedData == settingsInEEPROM.checksum)
					{
						//Checksums match.  Lets finish loading the data
						//We just copy the data into the settings structure for use later
						memcpy((unsigned char*)&_settingsDataStore, (unsigned char*)&settingsInEEPROM, sizeof(_settingsDataStoreStructure) );
					}
					else
					{
						//checksum mismatch.  We need to re-init
						serialcoms.debugPrint("Settings Checksum mismatch: ");
						serialcoms.debugPrint(settingsInEEPROM.checksum);
						serialcoms.debugPrint(" != ");
						serialcoms.debugPrint(checksumFromLoadedData);
						serialcoms.debugPrintln();

						setupDefaults();
					}
				}
			}
		}

		void setupDefaults()
		{
			//Add settings defaults below
			int maxDefaults[] = {2000,2000,2000,2000,2000,2000,2000,2000};
			SETTING_TYPE_INT_ARRAY_DEFAULT(ReceiverCalibrationMax, maxDefaults);
			
			int minDefaults[] = {1000,1000,1000,1000,1000,1000,1000,1000};
			SETTING_TYPE_INT_ARRAY_DEFAULT(ReceiverCalibrationMin, minDefaults);
			
			PIDParameters params = {1.0,0.0,1.0,2000};
			SETTING_TYPE_PID_DEFAULT(RollRatePIDParameters, params);
			SETTING_TYPE_PID_DEFAULT(PitchRatePIDParameters, params);
			SETTING_TYPE_PID_DEFAULT(YawRatePIDParameters, params);
			SETTING_TYPE_PID_DEFAULT(ThrottleRatePIDParameters, params);
			
			SETTING_TYPE_PID_DEFAULT(RollAnglePIDParameters, params);
			SETTING_TYPE_PID_DEFAULT(PitchAnglePIDParameters, params);
			SETTING_TYPE_PID_DEFAULT(HeadingPIDParameters, params);
			SETTING_TYPE_PID_DEFAULT(AltitudePIDParameters, params);
			
			_reCalculateCheckSum();
			_save();
		}

		SETTING_TYPE_INT_ARRAY_IMPLIMENTATION(ReceiverCalibrationMax);
		SETTING_TYPE_INT_ARRAY_IMPLIMENTATION(ReceiverCalibrationMin);
		
		SETTING_TYPE_PID_IMPLIMENTATION(RollRatePIDParameters);
		SETTING_TYPE_PID_IMPLIMENTATION(PitchRatePIDParameters);
		SETTING_TYPE_PID_IMPLIMENTATION(YawRatePIDParameters);
		SETTING_TYPE_PID_IMPLIMENTATION(ThrottleRatePIDParameters);
		
		SETTING_TYPE_PID_IMPLIMENTATION(RollAnglePIDParameters);
		SETTING_TYPE_PID_IMPLIMENTATION(PitchAnglePIDParameters);
		SETTING_TYPE_PID_IMPLIMENTATION(HeadingPIDParameters);
		SETTING_TYPE_PID_IMPLIMENTATION(AltitudePIDParameters);
};