#include <ServoDecode.h>

// ServoDecodeTest

#include "WProgram.h"
void setup();
void loop();
char * stateStrings[] = {
  "NOT_SYNCHED", "ACQUIRING", "READY", "in Failsafe"};

void setup()			  // run once, when the sketch starts
{
  Serial.begin(38400);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  ServoDecode.begin();
  ServoDecode.setFailsafe(3,1234); // set channel 3 failsafe pulse  width
}

void loop()			   // run over and over again
{
  int pulsewidth;

  // print the decoder state
  if( ServoDecode.getState()!= READY_state) {
    Serial.print("The decoder is ");
    Serial.println(stateStrings[ServoDecode.getState()]);
    for ( int i =0; i <=MAX_CHANNELS; i++ ){ // print the status of the first four channels
	Serial.print("Cx"); // if you see this, the decoder does not have a valid signal
	Serial.print(i);
	Serial.print("= ");
	pulsewidth = ServoDecode.GetChannelPulseWidth(i);
	Serial.print(pulsewidth);
	Serial.print("  ");
    }
    Serial.println("");
  }
  else {
    // decoder is ready, print the channel pulse widths
    for ( int i =1; i <=MAX_CHANNELS; i++ ){ // print the status of the first four channels
	Serial.print("Ch");
	Serial.print(i);
	Serial.print("= ");
	pulsewidth = ServoDecode.GetChannelPulseWidth(i);
	Serial.print(pulsewidth);
	Serial.print("  ");
    }
    Serial.println("");
    digitalWrite(12,LOW);
    digitalWrite(13,LOW);
  }
  delay(500); // update 2 times a second
} 



int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

