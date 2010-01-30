/*
  AeroQuad v2.0 - January 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef EEPROM_H
#define EEPROM_H

#include "SubSystem.h"
#include <EEPROM.h>

class Eeprom {
  private:
    // EEPROM storage addresses
    #define PGAIN_ADR 0
    #define IGAIN_ADR 4
    #define DGAIN_ADR 8
    #define LEVEL_PGAIN_ADR 12
    #define LEVEL_IGAIN_ADR 16
    #define LEVEL_DGAIN_ADR 20
    #define YAW_PGAIN_ADR 24
    #define YAW_IGAIN_ADR 28
    #define YAW_DGAIN_ADR 32
    #define WINDUPGUARD_ADR 36
    #define LEVELLIMIT_ADR 40
    #define LEVELOFF_ADR 44
    #define XMITFACTOR_ADR 48
    #define GYROSMOOTH_ADR 52
    #define ACCSMOOTH_ADR 56
    #define LEVELPITCHCAL_ADR 60
    #define LEVELROLLCAL_ADR 64
    #define LEVELZCAL_ADR 68
    #define FILTERTERM_ADR 72
    #define MODESMOOTH_ADR 76
    #define ROLLSMOOTH_ADR 80
    #define PITCHSMOOTH_ADR 84
    #define YAWSMOOTH_ADR 88
    #define THROTTLESMOOTH_ADR 92
    #define GYRO_ROLL_ZERO_ADR 96
    #define GYRO_PITCH_ZERO_ADR 100
    #define GYRO_YAW_ZERO_ADR 104
    #define PITCH_PGAIN_ADR 124
    #define PITCH_IGAIN_ADR 128
    #define PITCH_DGAIN_ADR 132
    #define LEVEL_PITCH_PGAIN_ADR 136
    #define LEVEL_PITCH_IGAIN_ADR 140
    #define LEVEL_PITCH_DGAIN_ADR 144
    #define THROTTLESCALE_ADR 148
    #define THROTTLEOFFSET_ADR 152
    #define ROLLSCALE_ADR 156
    #define ROLLOFFSET_ADR 160
    #define PITCHSCALE_ADR 164
    #define PITCHOFFSET_ADR 168
    #define YAWSCALE_ADR 172
    #define YAWOFFSET_ADR 176
    #define MODESCALE_ADR 180
    #define MODEOFFSET_ADR 184
    #define AUXSCALE_ADR 188
    #define AUXOFFSET_ADR 192
    #define AUXSMOOTH_ADR 196
    #define HEADINGSMOOTH_ADR 200
    #define HEADING_PGAIN_ADR 204
    #define HEADING_IGAIN_ADR 208
    #define HEADING_DGAIN_ADR 212
    
    union floatStore {
        byte floatByte[4];
        float floatVal;
    } floatData;
    
    int axis;
  
  public:
    Eeprom() {
      floatData.floatVal = 0.0;
    }
    
    float read(int address) {
      for (int i = 0; i < 4; i++) floatData.floatByte[i] = EEPROM.read(address + i);
      return floatData.floatVal;
    }

    void write(float value, int address) {
      floatData.floatVal = value;
      for (int i = 0; i < 4; i++) EEPROM.write(address + i, floatData.floatByte[i]);
    }

    void readEEPROM() {
      PID[ROLL].P = read(PGAIN_ADR);
      PID[ROLL].I = read(IGAIN_ADR);
      PID[ROLL].D = read(DGAIN_ADR);
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
  
      PID[PITCH].P = read(PITCH_PGAIN_ADR);
      PID[PITCH].I = read(PITCH_IGAIN_ADR);
      PID[PITCH].D = read(PITCH_DGAIN_ADR);
      PID[PITCH].lastPosition = 0;
      PID[PITCH].integratedError = 0;
  
      PID[YAW].P = read(YAW_PGAIN_ADR);
      PID[YAW].I = read(YAW_IGAIN_ADR);
      PID[YAW].D = read(YAW_DGAIN_ADR);
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
  
      PID[LEVELROLL].P = read(LEVEL_PGAIN_ADR);
      PID[LEVELROLL].I = read(LEVEL_IGAIN_ADR);
      PID[LEVELROLL].D = read(LEVEL_DGAIN_ADR);
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;  
  
      PID[LEVELPITCH].P = read(LEVEL_PITCH_PGAIN_ADR);
      PID[LEVELPITCH].I = read(LEVEL_PITCH_IGAIN_ADR);
      PID[LEVELPITCH].D = read(LEVEL_PITCH_DGAIN_ADR);
      PID[LEVELPITCH].lastPosition = 0;
      PID[LEVELPITCH].integratedError = 0;
  
      PID[HEADING].P = read(HEADING_PGAIN_ADR);
      PID[HEADING].I = read(HEADING_IGAIN_ADR);
      PID[HEADING].D = read(HEADING_DGAIN_ADR);
      PID[HEADING].lastPosition = 0;
      PID[HEADING].integratedError = 0;
  
      mTransmitter[THROTTLE] = read(THROTTLESCALE_ADR);
      bTransmitter[THROTTLE] = read(THROTTLEOFFSET_ADR);
      mTransmitter[ROLL] = read(ROLLSCALE_ADR);
      bTransmitter[ROLL] = read(ROLLOFFSET_ADR);
      mTransmitter[PITCH] = read(PITCHSCALE_ADR);
      bTransmitter[PITCH] = read(PITCHOFFSET_ADR);
      mTransmitter[YAW] = read(YAWSCALE_ADR);
      bTransmitter[YAW] = read(YAWOFFSET_ADR);
      mTransmitter[MODE] = read(MODESCALE_ADR);
      bTransmitter[MODE] = read(MODEOFFSET_ADR);
      mTransmitter[AUX] = read(AUXSCALE_ADR);
      bTransmitter[AUX] = read(AUXOFFSET_ADR);

      windupGuard = read(WINDUPGUARD_ADR);
      levelLimit = read(LEVELLIMIT_ADR);
      levelOff = read(LEVELOFF_ADR);
      xmitFactor = read(XMITFACTOR_ADR);
      for (axis = ROLL; axis < LASTAXIS; i++)
        gyro[axis].initalize(read(GYROSMOOTH_ADR));
      for (int axis = ROLL; axis < LASTAXIS; i++)
        accel[axis].initalize(read(ACCSMOOTH_ADR));

      smoothTransmitter[THROTTLE] = read(THROTTLESMOOTH_ADR);
      smoothTransmitter[ROLL] = read(ROLLSMOOTH_ADR);
      smoothTransmitter[PITCH] = read(PITCHSMOOTH_ADR);
      smoothTransmitter[YAW] = read(YAWSMOOTH_ADR);
      smoothTransmitter[MODE] = read(MODESMOOTH_ADR);
      smoothTransmitter[AUX] = read(AUXSMOOTH_ADR);
      accelZero[ROLL] = read(LEVELROLLCAL_ADR);
      accelZero[PITCH] = read(LEVELPITCHCAL_ADR);
      accelZero[ZAXIS] = read(LEVELZCAL_ADR);
      gyroZero[ROLL] = read(GYRO_ROLL_ZERO_ADR);
      gyroZero[PITCH] = read(GYRO_PITCH_ZERO_ADR);
      gyroZero[YAW] = read(GYRO_YAW_ZERO_ADR);
      timeConstant = read(FILTERTERM_ADR);
      smoothHeading = read(HEADINGSMOOTH_ADR);
    }
};

#endif
