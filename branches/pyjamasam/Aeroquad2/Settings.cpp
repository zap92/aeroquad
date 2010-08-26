#include <WProgram.h>

#include "LanguageExtensions.h"
#include "Settings.h"

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

void Settings::_recalculateChecksum()
{
	//Don't include the version, length, or CRC in the CRC calc
	unsigned int newChecksum = _KIRSP_cal_CRC16((unsigned char*)&_settingsDataStore + HOUSEKEEPINGSIZE, sizeof(_settingsDataStoreStructure) - HOUSEKEEPINGSIZE);

	_settingsDataStore.checksum = newChecksum;
}

void Settings::save()
{
	this->_recalculateChecksum();
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
	
	serialcoms.shell()->registerKeyword("listPIDs", "listPIDs", _listPIDs );
	serialcoms.shell()->registerKeyword("setPID", "setPID", _setPID );
	serialcoms.shell()->registerKeyword("getPID", "getPID", _getPID );
	serialcoms.shell()->registerKeyword("saveParameters", "saveParameters", _saveParameters );
}

void Settings::setupDefaults()
{
	//Add settings defaults below
	int maxDefaults[] = {2000,2000,2000,2000,2000,2000,2000,2000};
	SETTING_TYPE_INT_ARRAY_DEFAULT(ReceiverCalibrationMax, maxDefaults);
	
	int minDefaults[] = {1000,1000,1000,1000,1000,1000,1000,1000};
	SETTING_TYPE_INT_ARRAY_DEFAULT(ReceiverCalibrationMin, minDefaults);
	
	PIDParameters params = {1.0, 0.0, 0.0, 2000};
	SETTING_TYPE_PID_DEFAULT(RollRatePIDParameters, params);
	SETTING_TYPE_PID_DEFAULT(PitchRatePIDParameters, params);
	SETTING_TYPE_PID_DEFAULT(YawRatePIDParameters, params);
	SETTING_TYPE_PID_DEFAULT(ThrottleRatePIDParameters, params);
	
	SETTING_TYPE_PID_DEFAULT(RollAnglePIDParameters, params);
	SETTING_TYPE_PID_DEFAULT(PitchAnglePIDParameters, params);
	SETTING_TYPE_PID_DEFAULT(HeadingPIDParameters, params);
	SETTING_TYPE_PID_DEFAULT(AltitudePIDParameters, params);
	
	_recalculateChecksum();
	save();
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

const ArduinoShellCallback::callbackReturn _listPIDs(ArduinoShell &shell, const int argc, const char* argv[])
{
	shell << "PIDs" << endl;
	shell << "---" << endl;
	
	shell << " - RollRate" << endl;
	shell << " - PitchRate" << endl;
	shell << " - YawRate" << endl;
	shell << " - ThrottleRate" << endl;
	
	shell << " - RollAngle" << endl;
	shell << " - PitchAngle" << endl;
	shell << " - Heading" << endl;
	shell << " - Altitude" << endl;
	
	return ArduinoShellCallback::Success;	
}

const ArduinoShellCallback::callbackReturn _getPID(ArduinoShell &shell, const int argc, const char* argv[])
{
	if (argc != 1)
	{		
		return ArduinoShellCallback::InvalidParameterCount;		
	}
	else
	{
		const Settings::PIDParameters *pidSettings = NULL;
		if (strcasecmp(argv[0], "RollRate") == 0)
		{
			pidSettings = settings.getRollRatePIDParameters();			
		}
		else if (strcasecmp(argv[0], "PitchRate") == 0)
		{
			pidSettings = settings.getPitchRatePIDParameters();			
		}
		else if (strcasecmp(argv[0], "YawRate") == 0)
		{
			pidSettings = settings.getYawRatePIDParameters();			
		}
		else if (strcasecmp(argv[0], "ThrottleRate") == 0)
		{
			pidSettings = settings.getThrottleRatePIDParameters();			
		}
		
		else if (strcasecmp(argv[0], "RollAngle") == 0)
		{
			pidSettings = settings.getRollAnglePIDParameters();			
		}
		else if (strcasecmp(argv[0], "PitchAngle") == 0)
		{
			pidSettings = settings.getPitchAnglePIDParameters();			
		}
		else if (strcasecmp(argv[0], "Heading") == 0)
		{
			pidSettings = settings.getHeadingPIDParameters();			
		}
		else if (strcasecmp(argv[0], "Altitude") == 0)
		{
			pidSettings = settings.getAltitudePIDParameters();			
		}
		
		if (pidSettings)
		{
			shell << "P: " << pidSettings->P << ", I: " << 	pidSettings->I << ", D: " << pidSettings->D << ", Windup Guard: " << pidSettings->windupGuard << endl;	
		}
		else
		{
			shell << "ERROR: Unknown parameter" << endl;		
		}
		
		return ArduinoShellCallback::Success;	
	}
}

const ArduinoShellCallback::callbackReturn _setPID(ArduinoShell &shell, const int argc, const char* argv[])
{
	if (argc != 5)
	{
		return ArduinoShellCallback::InvalidParameterCount;	
	}
	else
	{
		Settings::PIDParameters pidSettings;
		
		pidSettings.P = shell.getArgumentAsFloat(argv[1]);
		pidSettings.I = shell.getArgumentAsFloat(argv[2]);
		pidSettings.D = shell.getArgumentAsFloat(argv[3]);
		pidSettings.windupGuard = shell.getArgumentAsFloat(argv[4]);
		
		if (strcasecmp(argv[0], "RollRate") == 0)
		{
			settings.setRollRatePIDParameters(&pidSettings);	
		}
		else if (strcasecmp(argv[0], "PitchRate") == 0)
		{
			settings.setPitchRatePIDParameters(&pidSettings);			
		}
		else if (strcasecmp(argv[0], "YawRate") == 0)
		{
			settings.setYawRatePIDParameters(&pidSettings);			
		}
		else if (strcasecmp(argv[0], "ThrottleRate") == 0)
		{
			settings.setThrottleRatePIDParameters(&pidSettings);			
		}
		
		else if (strcasecmp(argv[0], "RollAngle") == 0)
		{
			settings.setRollAnglePIDParameters(&pidSettings);			
		}
		else if (strcasecmp(argv[0], "PitchAngle") == 0)
		{
			settings.setPitchAnglePIDParameters(&pidSettings);			
		}
		else if (strcasecmp(argv[0], "Heading") == 0)
		{
			settings.setHeadingPIDParameters(&pidSettings);			
		}
		else if (strcasecmp(argv[0], "Altitude") == 0)
		{
			settings.setAltitudePIDParameters(&pidSettings);			
		}
		else
		{
				shell << "ERROR: Unknown parameter" << endl;	
				return ArduinoShellCallback::Success;				
		}
				
		shell << argv[0] << " PID set to P: " << pidSettings.P << ", I: " << pidSettings.I << ", D: " << pidSettings.D << ", Windup Guard: " << pidSettings.windupGuard << endl;	
	}

	return ArduinoShellCallback::Success;	
}

const ArduinoShellCallback::callbackReturn _saveParameters(ArduinoShell &shell, const int argc, const char* argv[])
{
	if (motors.armed())
	{
		shell << "ERROR: Unable to calibrate the IMU while motors are armed." << endl;		
	}
	else
	}
	settings.save();	
	return ArduinoShellCallback::Success;
}
