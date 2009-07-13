#include "pins_arduino.h"
/*
 * an extension to the interrupt support for arduino.
 * add pin change interrupts to the external interrupts, giving a way
 * for users to have interrupts drive off of any pin.
 * Refer to avr-gcc header files, arduino source and atmega datasheet.
 */

/*
 * Theory: all IO pins on Atmega168 are covered by Pin Change Interrupts.
 * The PCINT corresponding to the pin must be enabled and masked, and
 * an ISR routine provided.  Since PCINTs are per port, not per pin, the ISR
 * must use some logic to actually implement a per-pin interrupt service.
 */

/* Pin to interrupt map:
 * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
 */

#include "WProgram.h"
void PCattachInterrupt(uint8_t pin);
void PCdetachInterrupt(uint8_t pin);
static void PCint(uint8_t port);
void setup();
void loop();
unsigned int readReceiver(byte receiverPin);
volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

volatile static uint8_t PCintLast[3];

// Channel data 
typedef struct {   
  unsigned long riseTime;    
  unsigned long fallTime; 
  unsigned long lastGoodWidth;
} pinTimingData;  

volatile static pinTimingData pinData[24]; 

/*
 * attach an interrupt to a specific pin using pin change interrupts.
 * First version only supports CHANGE mode.
 */
 void PCattachInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

void PCdetachInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  // disable the mask.
  *pcmask &= ~bit;
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  currentTime = micros();
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = port * 8 + i;
      //Serial.print(pin, DEC); Serial.print(", ");
      if (bit & PCintLast[port]) {
        pinData[pin].riseTime = currentTime;
        //Serial.print("riseTime = ");
        //Serial.println(pinData[pin].riseTime, DEC);
      }
      else {
        pinData[pin].fallTime = currentTime;
        //Serial.print("fallTime = ");
        //Serial.println(pinData[pin].fallTime, DEC);
      }
    }
  }
  //Serial.println();
}

SIGNAL(PCINT0_vect) {
  PCint(0);
}
SIGNAL(PCINT1_vect) {
  PCint(1);
}
SIGNAL(PCINT2_vect) {
  PCint(2);
}

#define ROLLPIN 2 
#define THROTTLEPIN 4 
#define PITCHPIN 5 
#define YAWPIN 6 
#define MODEPIN 7 
#define AUXPIN 8 
#define ROLL 0 
#define PITCH 1 
#define YAW 2 
#define THROTTLE 3 
#define MODE 4 
#define AUX 5 
#define LASTCHANNEL 6 
#define MINWIDTH 950
#define MAXWIDTH 2050
int receiverChannel[6] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, MODEPIN, AUXPIN};
int receiverPin[6] = {18, 21, 22, 20, 23, 0};
volatile byte nextChannel = ROLL;

unsigned long currentTime;
unsigned long previousTime;
byte channel = 0;

void setup()
{
  Serial.begin(115200);
  for (channel = ROLL; channel < LASTCHANNEL; channel++) {
    pinMode(channel, INPUT);
    PCattachInterrupt(receiverChannel[channel]);
  }
  previousTime = millis();
}

void loop() {
  currentTime = millis();
  if (currentTime > (previousTime + 20)) {
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(readReceiver(receiverPin[channel]));
      Serial.print(", ");
    }
    Serial.println(readReceiver(receiverPin[AUX]));
  }
}

unsigned int readReceiver(byte receiverPin) {
  unsigned int time;
  
  time = pinData[receiverPin].fallTime - pinData[receiverPin].riseTime;
  if ((time > MINWIDTH) && (time < MAXWIDTH))
    pinData[receiverPin].lastGoodWidth = time;

  return pinData[receiverPin].lastGoodWidth;
}
    
    
int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

