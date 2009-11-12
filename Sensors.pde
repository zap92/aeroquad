// Radians to Degrees
#define R2D 57.2957 //360.0/(2*PI)

// Calibration parameters
#define FINDZERO 150
long findZero[FINDZERO];

// Allows user to zero gyros on command
void zeroGyros() {
  for(int n=0;n<3;n++) {
    for(int i=0;i<FINDZERO;i++) {
      delayMicroseconds(50);
      findZero[i] = gyroRead(n);
    }
    gyro.zero[n] = findMode(findZero, FINDZERO);
    writeInt(gyro.zero[SENSOR_ROLL], GYRO_ROLL_ZERO_ADR);
    writeInt(gyro.zero[SENSOR_PITCH], GYRO_PITCH_ZERO_ADR);
    writeInt(gyro.zero[SENSOR_YAW], GYRO_YAW_ZERO_ADR);
  }
}

// Allows user to zero accelerometers on command
void zeroAccelerometers() {
  for(int n=0;n<3;n++) {
    for(int i=0;i<FINDZERO; i++) {
      LEDOFF;
      delayMicroseconds(50);
      findZero[i] = accelRead(n, false);
      LEDON;
    }
    accel.zero[n] = findMode(findZero, FINDZERO);
  }
  writeInt(accel.zero[SENSOR_ROLL], ACCEL_ROLL_ZERO_ADR);
  writeInt(accel.zero[SENSOR_PITCH], ACCEL_PITCH_ZERO_ADR);
  writeInt(accel.zero[SENSOR_ZAXIS], ACCEL_ZAXIS_ZERO_ADR);
}

void updateSensors(float delta) { // Delta in ms
  for(int n=0;n<3;n++) {
    gyro.raw[n] = gyroRead(n) - gyro.zero[n];
    gyro.value[n] = smooth(gyro.raw[n], gyro.value[n], gyro.factor);
    if(n<2) {
      // Complementary filter - fusing gyro integrated angle with accelerometer derived angle
      level.angle[n] = ((level.angle[n] + (((gyro.value[n] * 1.611328125) * delta) / 1000.0)) * level.coefficient) + (level.accelAngle[n] * (1.0-level.coefficient));
    } else {
      level.angle[n] += ((gyro.value[n] * 1.611328125*3) * delta) / 1000.0; // TODO: Correct with compass
    }
    accel.raw[n] = accelRead(n, true) - accel.zero[n];
    accel.value[n] = smooth(accel.raw[n], accel.value[n], accel.factor);
  }
}

void updateLevel() {
  level.accelAngle[SENSOR_PITCH] = atan2(accel.value[SENSOR_PITCH], sqrt(sq((long)accel.value[SENSOR_ROLL])+sq((long)accel.value[SENSOR_ZAXIS]))) * R2D;
  level.accelAngle[SENSOR_ROLL] = atan2(accel.value[SENSOR_ROLL], sqrt(sq((long)accel.value[SENSOR_PITCH])+sq((long)accel.value[SENSOR_ZAXIS]))) * R2D;
}

int findMode(long *data, int arraySize) {
  int currentFrequency = 0;
  int maxNumber = 0;
  int maxFrequency = 0;

  for(int n=0;n<arraySize;n++) {
    currentFrequency = 0;
    for(int i=0;i<arraySize;i++) {
      if(data[i] == data[n]) {
        currentFrequency++; 
      }
    }
    if(currentFrequency>maxFrequency) {
      maxFrequency = currentFrequency;
      maxNumber = data[n];
    }
  }
  return maxNumber;
}

int gyroRead(int axis) {
  switch(axis) {
    case 0: return analogRead(3);
    case 1: return analogRead(4);
    case 2: return analogRead(5);
  } 
}

int accelRead(int axis, boolean compensate) {
  switch(axis) {
    case 0: return analogRead(0);
    case 1: return analogRead(1);
    case 2: return analogRead(2)+(compensate?85:0);
  } 
}
