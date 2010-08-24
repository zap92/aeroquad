#include <WProgram.h>

#include "LanguageExtensions.h"
#include "Settings.h"

#include "SerialComs.h"

unsigned int Settings::_KIRSP_cal_CRC16(unsigned char *CRC16_Data_Array, unsigned int CRC16_Data_Array_Len)
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

void Settings::_reCalculateCheckSum()
{
	//Don't include the version, length, or CRC in the CRC calc
	unsigned int newChecksum = _KIRSP_cal_CRC16((unsigned char*)&_settingsDataStore + HOUSEKEEPINGSIZE, sizeof(_settingsDataStoreStructure) - HOUSEKEEPINGSIZE);

	_settingsDataStore.checksum = newChecksum;
}

void Settings::_save()
{
	eeprom_write_block((unsigned char*)&_settingsDataStore, (void*)SETTINGS_LOCATION, sizeof(_settingsDataStoreStructure));
}


	Settings::Settings(unsigned int spaceAllowedToUse)
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

void Settings::initialize()
{
	if (_online)
	{
		_settingsDataStoreStructure settingsInEEPROM;
		eeprom_read_block(&settingsInEEPROM, (const void*)SETTINGS_LOCATION, sizeof(_settingsDataStoreStructure));

		if (settingsInEEPROM.version != _settingsDataStore.version)
		{
			//Version mismatch.  We need to re-init
			serialcoms.print("Settings version mismatch: ");
			serialcoms.print(_settingsDataStore.version);
			serialcoms.print(" != ");
			serialcoms.print(settingsInEEPROM.version);
			serialcoms.println();

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
				serialcoms.print("Settings Checksum mismatch: ");
				serialcoms.print(settingsInEEPROM.checksum);
				serialcoms.print(" != ");
				serialcoms.print(checksumFromLoadedData);
				serialcoms.println();

				setupDefaults();
			}
		}
	}
}

void Settings::setupDefaults()
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


Settings settings;

