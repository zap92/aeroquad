//
//  RX channel order
// ==================
//  1: Roll
//  2: Pitch
//  3: Throttle
//  4: Yaw
//  5: Gear
//  6: Aux
//

void setupRX() {
  pinMode(2, INPUT);
  attachInterrupt(0, rxInt, RISING);
  while(!rx.sync) {
    LEDON;
    delay(50);
    LEDOFF;
    delay(50);
  }
}

void updateRX() {
  for(int n=0;n<6;n++) {
    rx.raw[n] = readRX(n);
    rx.value[n] = smooth(rx.raw[n], rx.value[n], rx.factor[n]);
    // Applying speccy on ROLL, PITCH and YAW to dumb down the sticks a bit
    if(n<3) {
      rx.command[n] = (((rx.value[n] - MIDCOMMAND) * rx.speccyFactor) / 32) + MIDCOMMAND;
    } else {
      rx.command[n] = rx.value[n];
    }
  }

  // Only process RX commands when throttle is off
  if(rx.raw[RX_THROTTLE] < MINCHECK) {
    processRXCommands();
  }
  
  // Prevents too little power applied to motors during hard manuevers
  if(rx.value[RX_THROTTLE] > (MIDCOMMAND - MINDELTA)) {
    motor.minimum = rx.value[RX_THROTTLE] - MINDELTA;
  }
  if(rx.value[RX_THROTTLE] < MINTHROTTLE) {
    motor.minimum = MINTHROTTLE;
  }
  // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
  //if((receiverData[ROLL] < MINCHECK) || (receiverData[ROLL] > MAXCHECK) || (receiverData[PITCH] < MINCHECK) || (receiverData[PITCH] > MAXCHECK))
  //  minCommand = MINTHROTTLE;
}

void processRXCommands() {
  // Arm motors
  if(rx.raw[RX_YAW] > MAXCHECK && !motor.armed) {
    motor.armed = true;
    motor.minimum = MINTHROTTLE;
    rx.center[RX_ROLL] = rx.value[RX_ROLL];
    rx.center[RX_PITCH] = rx.value[RX_PITCH];
  }
  
  if(rx.raw[RX_YAW] < MINCHECK && motor.armed) {
    motor.armed = false;
    setAllMotors(MINCOMMAND);
    updateMotors();
  }
  
  // Zero sensors (left stick lower left, right stick lower right corner)
  if(rx.raw[RX_YAW] < MINCHECK && rx.raw[RX_ROLL] > MAXCHECK && rx.raw[RX_PITCH] < MINCHECK) {
    zeroGyros();
    zeroAccelerometers();
  }
}

int readRX(int channel) {
  // Swap YAW and THROTTLE
  if(channel==3) {
    channel = 2;
  } else if(channel==2) {
    channel = 3;
  }
  return rx.realRaw[channel];
}

void rxInt() {
  long now = micros();
  if(rx.last>0) {
    unsigned long diff = now - rx.last;
    if(diff>5000) {
      rx.sync = true;
      rx.currentChannel = 0;
    } else {
      if(rx.sync) {
        if(diff<=2000 && diff>=1000) {
            rx.realRaw[rx.currentChannel] = diff;
        }
        rx.currentChannel++;
      }
    }
  }
  rx.last = now;
}
