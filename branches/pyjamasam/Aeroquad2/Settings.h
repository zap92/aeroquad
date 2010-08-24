#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include <avr/eeprom.h>

//Some fancy macros to make it easier to read and write settings
//And it saves some typing when defining new settings.
#define SETTING_TYPE_FLOAT_STRUCT_DEFINATION(name, size) float _##name

#define SETTING_TYPE_FLOAT_DEFAULT(name, defaultData) {\
		_settingsDataStore._##name = defaultData;\
	}\
}

#define SETTING_TYPE_FLOAT_DEFINATION(name) void set##name(const float newData); const float get##name();

#define SETTING_TYPE_FLOAT_IMPLIMENTATION(name) void Settings::set##name(const float newData)\
{ \
	_settingsDataStore._##name = newData;\
	_reCalculateCheckSum();\
	_save();\
}\
const float Settings::get##name()\
{\
	return _settingsDataStore._##name;\
}




#define SETTING_TYPE_PID_STRUCT_DEFINATION(name) PIDParameters _##name

#define SETTING_TYPE_PID_DEFAULT(name, defaultData) {\
		_settingsDataStore._##name.P = defaultData.P;\
		_settingsDataStore._##name.I = defaultData.I;\
		_settingsDataStore._##name.D = defaultData.D;\
		_settingsDataStore._##name.windupGuard = defaultData.windupGuard;\
}

#define SETTING_TYPE_PID_DEFINATION(name) void set##name(const PIDParameters *newData); const Settings::PIDParameters *get##name();

#define SETTING_TYPE_PID_IMPLIMENTATION(name) void Settings::set##name(const Settings::PIDParameters *newData)\
{ \
	_settingsDataStore._##name.P = newData->P;\
	_settingsDataStore._##name.I = newData->I;\
	_settingsDataStore._##name.D = newData->D;\
	_settingsDataStore._##name.windupGuard = newData->windupGuard;\
	\
	_reCalculateCheckSum();\
	_save();\
}\
const Settings::PIDParameters *Settings::get##name()\
{\
	return &_settingsDataStore._##name;\
}




#define SETTING_TYPE_INT_ARRAY_STRUCT_DEFINATION(name, size) int _##name[size]

#define SETTING_TYPE_INT_ARRAY_DEFAULT(name, defaultData) {\
	unsigned int arraySize = sizeof(_settingsDataStore._##name) / sizeof(int);\
	for (int i = 0; i < arraySize; i++) \
	{\
		_settingsDataStore._##name[i] = defaultData[i];\
	}\
}

#define SETTING_TYPE_INT_ARRAY_DEFINATION(name) void set##name(const int *newData); const int *get##name();

#define SETTING_TYPE_INT_ARRAY_IMPLIMENTATION(name) void Settings::set##name(const int *newData)\
{ \
   	unsigned int arraySize = sizeof(_settingsDataStore._##name) / sizeof(int);\
	for (int i = 0; i < arraySize; i++) \
	{\
		_settingsDataStore._##name[i] = newData[i];\
	}\
	_reCalculateCheckSum();\
	_save();\
}\
const int *Settings::get##name()\
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
			SETTING_TYPE_INT_ARRAY_STRUCT_DEFINATION(ReceiverCalibrationMax, 8);
			SETTING_TYPE_INT_ARRAY_STRUCT_DEFINATION(ReceiverCalibrationMin, 8);
			
			SETTING_TYPE_PID_STRUCT_DEFINATION(RollRatePIDParameters);
			SETTING_TYPE_PID_STRUCT_DEFINATION(PitchRatePIDParameters);
			SETTING_TYPE_PID_STRUCT_DEFINATION(YawRatePIDParameters);
			SETTING_TYPE_PID_STRUCT_DEFINATION(ThrottleRatePIDParameters);
			
			SETTING_TYPE_PID_STRUCT_DEFINATION(RollAnglePIDParameters);
			SETTING_TYPE_PID_STRUCT_DEFINATION(PitchAnglePIDParameters);
			SETTING_TYPE_PID_STRUCT_DEFINATION(HeadingPIDParameters);
			SETTING_TYPE_PID_STRUCT_DEFINATION(AltitudePIDParameters);
		};

  		_settingsDataStoreStructure _settingsDataStore;

		unsigned int _KIRSP_cal_CRC16(unsigned char *CRC16_Data_Array, unsigned int CRC16_Data_Array_Len);

		void _reCalculateCheckSum();
  
		void _save();

	public:	
  		Settings(unsigned int spaceAllowedToUse = 1024);

		void initialize();

		void setupDefaults();

		SETTING_TYPE_INT_ARRAY_DEFINATION(ReceiverCalibrationMax);
		SETTING_TYPE_INT_ARRAY_DEFINATION(ReceiverCalibrationMin);
		
		SETTING_TYPE_PID_DEFINATION(RollRatePIDParameters);
		SETTING_TYPE_PID_DEFINATION(PitchRatePIDParameters);
		SETTING_TYPE_PID_DEFINATION(YawRatePIDParameters);
		SETTING_TYPE_PID_DEFINATION(ThrottleRatePIDParameters);
		
		SETTING_TYPE_PID_DEFINATION(RollAnglePIDParameters);
		SETTING_TYPE_PID_DEFINATION(PitchAnglePIDParameters);
		SETTING_TYPE_PID_DEFINATION(HeadingPIDParameters);
		SETTING_TYPE_PID_DEFINATION(AltitudePIDParameters);
};

extern Settings settings;

#endif //#ifndef __SETTINGS_H__