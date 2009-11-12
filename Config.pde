// EEPROM storage addresses
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define LEVEL_ROLL_PGAIN_ADR 12
#define LEVEL_ROLL_IGAIN_ADR 16
#define LEVEL_ROLL_DGAIN_ADR 20
#define YAW_PGAIN_ADR 24
#define YAW_IGAIN_ADR 28
#define YAW_DGAIN_ADR 32
#define WINDUPGUARD_ADR 36
#define LEVEL_LIMIT_ADR 40
#define LEVEL_ENABLERANGE_ADR 44
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define ACCEL_PITCH_ZERO_ADR 60
#define ACCEL_ROLL_ZERO_ADR 64
#define ACCEL_ZAXIS_ZERO_ADR 68
#define FILTERTERM_ADR 72
#define GEARSMOOTH_ADR 76
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
#define AUXSMOOTH_ADR 196
#define HEADINGSMOOTH_ADR 200
#define HEADING_PGAIN_ADR 204
#define HEADING_IGAIN_ADR 208
#define HEADING_DGAIN_ADR 212

void configRead() {
  accel.zero[SENSOR_ROLL] = readInt(ACCEL_ROLL_ZERO_ADR);
  accel.zero[SENSOR_PITCH] = readInt(ACCEL_PITCH_ZERO_ADR);
  accel.zero[SENSOR_ZAXIS] = readInt(ACCEL_ZAXIS_ZERO_ADR);
  gyro.zero[SENSOR_ROLL] = readInt(GYRO_ROLL_ZERO_ADR);
  gyro.zero[SENSOR_PITCH] = readInt(GYRO_PITCH_ZERO_ADR);
  gyro.zero[SENSOR_YAW] = readInt(GYRO_YAW_ZERO_ADR);
 
  pd[RX_ROLL].P = readInt(PGAIN_ADR);
  readInt(IGAIN_ADR);
  pd[RX_ROLL].D = readInt(DGAIN_ADR);
  pd[RX_PITCH].P = readInt(PITCH_PGAIN_ADR);
  readInt(PITCH_IGAIN_ADR);
  pd[RX_PITCH].D = readInt(PITCH_DGAIN_ADR);
  pd[RX_YAW].P = readInt(YAW_PGAIN_ADR);
  readInt(YAW_IGAIN_ADR);
  pd[RX_YAW].D = readInt(YAW_DGAIN_ADR);
  // Auto Level PID values
  pd[LEVEL_ROLL].P = readInt(LEVEL_ROLL_PGAIN_ADR);
  readInt(LEVEL_ROLL_IGAIN_ADR);
  pd[LEVEL_ROLL].D = readInt(LEVEL_ROLL_DGAIN_ADR);
  pd[LEVEL_PITCH].P = readInt(LEVEL_PITCH_PGAIN_ADR);
  readInt(LEVEL_PITCH_IGAIN_ADR);
  pd[LEVEL_PITCH].D = readInt(LEVEL_PITCH_DGAIN_ADR);
  pd[LEVEL_HEADING].P = readInt(HEADING_PGAIN_ADR);
  readInt(HEADING_IGAIN_ADR);
  pd[LEVEL_HEADING].D = readInt(HEADING_DGAIN_ADR);
    
  readInt(WINDUPGUARD_ADR);
  level.limit = readInt(LEVEL_LIMIT_ADR);
  level.enableRange = readInt(LEVEL_ENABLERANGE_ADR);
  rx.speccyFactor = readInt(XMITFACTOR_ADR);
  gyro.factor = 4; //readInt(GYROSMOOTH_ADR);
  accel.factor = 1; //readInt(ACCSMOOTH_ADR);
  
  rx.factor[RX_ROLL] = readInt(YAWSMOOTH_ADR);
  rx.factor[RX_PITCH] = readInt(PITCHSMOOTH_ADR);
  rx.factor[RX_YAW] = readInt(YAWSMOOTH_ADR);
  rx.factor[RX_THROTTLE] = readInt(THROTTLESMOOTH_ADR);
  rx.factor[RX_GEAR] = readInt(GEARSMOOTH_ADR);
  rx.factor[RX_AUX] = readInt(AUXSMOOTH_ADR);
  level.timeConstant = readInt(FILTERTERM_ADR);
}

// Utilities for writing and reading from the EEPROM
int readInt(int address) {
  union intStore { byte intByte[2]; int intVal; } intOut;
  for(int i=0;i<2;i++) {
    intOut.intByte[i] = EEPROM.read(address+i);
  }
  return intOut.intVal;
}

void writeInt(int value, int address) {
  union intStore { byte intByte[2]; int intVal; } intIn;

  intIn.intVal = value;
  for(int i=0;i<2;i++) {
    EEPROM.write(address+i, intIn.intByte[i]);
  }
}
