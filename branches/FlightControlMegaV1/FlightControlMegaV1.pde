// =================================================
// =             Q  U  A  D  U  I  N  O            =
// =     An Arduino based Quadcopter Controller    =
// =  Copyright (c) 2008-2009 Paul René Jørgensen  =
// =================================================
// =    http://quaduino.org | http://paulrene.no   =
// =================================================
// = 2009-09                                       = 
//
#include <ctype.h>
#include <EEPROM.h>
#include <SoftwareServo.h>

// GLOBAL DEFINES
#define LEDPIN 13
#define LEDON digitalWrite(LEDPIN, HIGH)
#define LEDOFF digitalWrite(LEDPIN, LOW)
#define RX_ROLL 0
#define RX_PITCH 1
#define RX_YAW 2
#define RX_THROTTLE 3
#define RX_GEAR 4
#define RX_AUX 5
#define LEVEL_ROLL 3
#define LEVEL_PITCH 4
#define LEVEL_HEADING 5
#define SENSOR_PITCH 0
#define SENSOR_ROLL 1
#define SENSOR_YAW 2
#define SENSOR_ZAXIS 2
#define MOTOR_FRONT 0
#define MOTOR_REAR 1
#define MOTOR_LEFT 2
#define MOTOR_RIGHT 3
#define MINDELTA 200
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100

boolean fastTransfer = false;

// GLOBAL STRUCTS
struct Timing {
  unsigned long current;
//  unsigned long previous;
  unsigned long serial;
  unsigned long control;
  unsigned long sensor;
  unsigned long sensorMicros;
  unsigned long level;
  unsigned long rx;
//  int delta;
  unsigned long currentMicros;
  unsigned long previousMicros;
  unsigned long deltaMicros;
  unsigned long deltaMicrosAvg;
} time = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

struct RX {
  int realRaw[10];
  int raw[6];
  int value[6]; // Smoothed
  int command[6]; // speccyFactor applied to ROLL, PITCH and YAW
  int factor[6];
  int center[2];
  int speccyFactor;
  int currentChannel;
  unsigned long last;
  boolean sync;
} rx = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 },
  { 16, 16, 12, 16, 16, 16 }, // Smooth factors in 1/16th (int) = 0-16 maps to 0-100%
  { 1500, 1500 },
  3, // Speccy factor to dumb down the sticks 3/16th = 0.18
  0, 0, false
};

struct Motor {
  int command[4];
  int axisCommand[3];
  boolean armed;
  int minimum;
} motor = {
  { MINCOMMAND, MINCOMMAND, MINCOMMAND, MINCOMMAND },
  { 0, 0, 0 },
  false,
  1000
};

struct GyroData {
  int zero[3];
  int raw[3];
  int value[3];
  int factor;
} gyro = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 8 };

struct AccelData {
  int zero[3];
  int raw[3];
  int value[3];
  int factor;
} accel = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 4 };

struct PD {
  int P, D;
  int lastPosition;
} pd[5];

struct Level {  
  int adjustment[2];
  float accelAngle[2];
  float angle[3];
  float coefficient;
  int timeConstant;
  int limit;
  int enableRange;
  boolean enabled;
} level = { {0, 0}, {0.0, 0.0}, {0.0, 0.0, 0.0}, 0.98, 245, 250, 50, false }; // 0.98 = 245/(245+5) = timeConstant / (timeConstant + deltaTime)

void setup() {
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
  LEDOFF;

  configRead();
  setupRX();
  setupMotors();
  LEDON;
  delay(250);
  zeroGyros();
  LEDOFF;
  delay(250);
//  zeroAccelerometers();

  time.previousMicros = micros();
  LEDON;
}


void loop() {
  // Calculate timing
  time.current = millis();
  time.currentMicros = micros();
//  time.delta = (int) (time.current - time.previous);
//  time.previous = time.current;
  time.deltaMicros = (time.currentMicros - time.previousMicros);
  time.deltaMicrosAvg = (time.deltaMicrosAvg * 99.0 + time.deltaMicros) / 100.0;
  time.previousMicros = time.currentMicros;
  
  // RX LOOP
  if(time.current > (time.rx + 100)) { // 10Hz
    updateRX();
    time.rx = time.current;
  }

  // SENSOR LOOP
  if(time.current > (time.sensor + 2)) {
    updateSensors((float) ((time.currentMicros - time.sensorMicros) / 1000.0));
    time.sensorMicros = time.currentMicros;
    time.sensor = time.current;
  }
  
  // LEVEL LOOP
  if(time.current > (time.level + 10)) {
    updateLevel();
    time.level = time.current;
  }
  
  // CONTROL LOOP
  if(time.current > time.control + 2) { // 500Hz
    // Auto level on?
    level.enabled = false;
    if(rx.raw[RX_GEAR] > MIDCOMMAND) {
//      if((abs(rx.value[RX_PITCH] - rx.center[RX_PITCH]) <= level.enableRange)) { // 50 = levelOff
//        if((abs(rx.value[RX_ROLL] - rx.center[RX_ROLL]) <= level.enableRange)) {
          level.enabled = true;
//        }
//      }
    }
    // Calculate auto level
    if(level.enabled) {
//      level.adjustment[SENSOR_ROLL] = constrain(updatePD(0, level.angle[SENSOR_ROLL], &pd[LEVEL_ROLL]), -level.limit, level.limit);
//      level.adjustment[SENSOR_PITCH] = constrain(updatePD(0, level.angle[SENSOR_PITCH], &pd[LEVEL_PITCH]), -level.limit, level.limit);
      level.adjustment[SENSOR_ROLL] = constrain(updatePD(0, accel.value[SENSOR_ROLL], &pd[LEVEL_ROLL]), -level.limit, level.limit);
      level.adjustment[SENSOR_PITCH] = constrain(updatePD(0, accel.value[SENSOR_PITCH], &pd[LEVEL_PITCH]), -level.limit, level.limit);
    } else {
      level.adjustment[SENSOR_ROLL] = 0;
      level.adjustment[SENSOR_PITCH] = 0;
    }

    motor.axisCommand[RX_ROLL] = updatePD(rx.command[RX_ROLL] + level.adjustment[SENSOR_ROLL], gyro.value[SENSOR_ROLL] + MIDCOMMAND, &pd[RX_ROLL]);
    motor.axisCommand[RX_PITCH] = updatePD(rx.command[RX_PITCH] - level.adjustment[SENSOR_PITCH], gyro.value[SENSOR_PITCH] + MIDCOMMAND, &pd[RX_PITCH]);
    motor.axisCommand[RX_YAW] = updatePD(rx.command[RX_YAW], gyro.value[SENSOR_YAW] + MIDCOMMAND, &pd[RX_YAW]);

    if(motor.armed) {
      // Multiply with 0.95 and 1.05 to increase throttle by 5% during yawing to ensure we don't loose altitude
      motor.command[MOTOR_FRONT] = constrain(rx.command[RX_THROTTLE] - motor.axisCommand[RX_PITCH] - motor.axisCommand[RX_YAW]*0.95, motor.minimum, MAXCOMMAND);
      motor.command[MOTOR_REAR] = constrain(rx.command[RX_THROTTLE] + motor.axisCommand[RX_PITCH]  - motor.axisCommand[RX_YAW]*0.95, motor.minimum, MAXCOMMAND);
      motor.command[MOTOR_RIGHT] = constrain(rx.command[RX_THROTTLE] + motor.axisCommand[RX_ROLL] + motor.axisCommand[RX_YAW]*1.05, motor.minimum, MAXCOMMAND);
      motor.command[MOTOR_LEFT] = constrain(rx.command[RX_THROTTLE] - motor.axisCommand[RX_ROLL] + motor.axisCommand[RX_YAW]*1.05, motor.minimum, MAXCOMMAND);
      
      // If throttle in minimum position, don't apply yaw
      if(rx.command[RX_THROTTLE] < MINCHECK) {
        setAllMotors(motor.minimum); 
      }
      
    } else {
      setAllMotors(MINCOMMAND);
    }
    updateMotors();
    time.control = time.current;
  }

  // SERIAL PORT LOOP
  if(time.current > (time.serial + 100)) { // 10Hz
    updateSerial();
    time.serial = time.current;
  }
  
  // Update Motor PWM
  SoftwareServo::refresh();
}
