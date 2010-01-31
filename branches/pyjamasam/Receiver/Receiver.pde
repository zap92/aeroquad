#include <EEPROM.h>
#include <Wire.h>
#include "Streaming.h"

#define VERSION "0.1"

#define I2CAddress 0x63

//Configurable Part
#define NUMBEROFCHANNELS 6

#define LEDPIN 13

int channelPins [NUMBEROFCHANNELS] = {
  2, 3, 4, 5, 6, 7};

//End Configurable Part

#include "pins_arduino.h"

unsigned int _calibratedChannelMin[NUMBEROFCHANNELS];
unsigned int _calibratedChannelMax[NUMBEROFCHANNELS];
byte _calibrating = 0;

float _channelSmoothFactor[NUMBEROFCHANNELS];
unsigned long _channelSmoothedValue[NUMBEROFCHANNELS];

// Channel data 
typedef struct {
  byte edge;
  unsigned long riseTime;    
  unsigned long fallTime; 
  unsigned long lastGoodWidth;
  unsigned long lastGoodTime;
} 
pinTimingData;

//a container for all possible chunks of pin data (indexed on the raw pin number)
volatile static pinTimingData pinData[24];

//map the port number to the PCMSK register
volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

//Holder for the last pin change data.  Used to check for a change when the interrupt fires again
volatile static uint8_t PCintLast[3];

uint8_t PCIntToChannelMap[NUMBEROFCHANNELS];

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

uint8_t portAndPinToPCINT(uint8_t port, uint8_t pin)
{
  uint8_t pcintNumber = port * 8;

  switch (pin >> 1)
  {
  case 0:
    {
      pcintNumber += 0;
      break;
    }
  case 1:
    {
      pcintNumber += 1;
      break;
    }
  case 2:
    {
      pcintNumber += 2;
      break;
    }
  case 4:
    {
      pcintNumber += 3;
      break;
    }
  case 8:
    {
      pcintNumber += 4;
      break;
    }
  case 16:
    {
      pcintNumber += 5;
      break;
    }
  case 32:
    {
      pcintNumber += 6;
      break;
    }
  case 64:
    {
      pcintNumber += 7;
      break;
    }
  }

  return pcintNumber; 
}

// Attaches PCINT to Arduino Pin
void setupChannel(uint8_t channelIndex) 
{
  uint8_t pin = channelPins[channelIndex];

  //set each pin up as an input pin
  pinMode(channelPins[channelIndex], INPUT);

  //Initialize the pindata structure
  pinData[channelPins[channelIndex]].edge == FALLING_EDGE;

  //map the pin to the internal registers and pins needed to setup the interrupts
  uint8_t pinBit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);

  //Move the port down to where it matches the data sheet for the PCMSK registers
  port -= 2;

  //sort out the "internal pin number" and store it in an array indexed to the channel index.
  //This is used later to read the channel data out
  PCIntToChannelMap[channelIndex] = portAndPinToPCINT(port, pinBit);

  //turn on the bit for this pin in the mask
  volatile uint8_t *pcmask = port_to_pcmask[port];
  *pcmask |= pinBit;

  //enable interrups on this port
  PCICR |= 0x01 << port;  
}

static void measurePulseWidthISR(uint8_t port) 
{
  uint8_t bit;
  uint8_t pin;
  uint32_t time;

  // get the pin states for the indicated port.
  uint8_t curr = *portInputRegister(port+2);
  uint8_t mask = curr ^ PCintLast[port];

  //update the last seen pin change data with the new data
  PCintLast[port] = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) 
  {
    return;
  }

  uint32_t currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) 
  {
    bit = 0x01 << i;
    if (bit & mask) 
    {
      pin = port * 8 + i;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) 
      {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;        
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
        {
          pinData[pin].edge = RISING_EDGE;
        }
        else
        {
          pinData[pin].edge == FALLING_EDGE; // invalid rising edge detected
        }
      }
      else 
      {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) 
        {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].lastGoodTime = currentTime / 1000;

          pinData[pin].edge = FALLING_EDGE;
        } 
      }
    }
  }
}


//interrupt vectors used to measure the pulse width on the pins
SIGNAL(PCINT0_vect) 
{
  measurePulseWidthISR(0);
}

SIGNAL(PCINT1_vect) 
{
  measurePulseWidthISR(1);
}

SIGNAL(PCINT2_vect) 
{
  measurePulseWidthISR(2);
}


int smooth(int currentData, int previousData, float smoothFactor) 
{
  return (previousData * (1 - smoothFactor) + (currentData * smoothFactor));
}

uint16_t getChannelValue(uint8_t channelIndex, unsigned long currentTime) 
{
  uint8_t pcintNumber = PCIntToChannelMap[channelIndex];
  uint8_t oldSREG = SREG;
  cli();
  uint16_t lastGoodWidth = pinData[pcintNumber].lastGoodWidth;
  uint16_t lastGoodTime = pinData[pcintNumber].lastGoodTime;
  SREG = oldSREG;

  if (!_calibrating)
  {
    uint16_t latestReading = constrain((uint32_t)1000 * (lastGoodWidth - _calibratedChannelMin[channelIndex]) / (_calibratedChannelMax[channelIndex] - _calibratedChannelMin[channelIndex]) + 1000, 1000, 2000);
    _channelSmoothedValue[channelIndex] = smooth(latestReading, _channelSmoothedValue[channelIndex],_channelSmoothFactor[channelIndex]);

    return _channelSmoothedValue[channelIndex];
  }
  else
  {
    if (lastGoodWidth < _calibratedChannelMin[channelIndex])
    {
      _calibratedChannelMin[channelIndex] = lastGoodWidth;
    }

    if (lastGoodWidth > _calibratedChannelMax[channelIndex])
    {
      _calibratedChannelMax[channelIndex] = lastGoodWidth;
    }

    return lastGoodWidth;
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
static uint16_t outputData[NUMBEROFCHANNELS];
void requestEvent()
{
  if (!_calibrating)
  {
    unsigned long currentTime = millis();
    for (int i = 0; i< NUMBEROFCHANNELS; i++) 
    {
      outputData[i] = getChannelValue(i, currentTime);
    }
  }
  else
  {
    for (int i = 0; i< NUMBEROFCHANNELS; i++) 
    {
      outputData[i] = 0;
    }
  }

  Wire.send((byte*)outputData, sizeof(outputData));
}

static int epromChannelCalibrationMinData = 0;
static int epromChannelCalibrationMaxData = epromChannelCalibrationMinData + (NUMBEROFCHANNELS * 2);
static int epromChannelSmoothFactorData = epromChannelCalibrationMaxData + (NUMBEROFCHANNELS * 2);
static int epromChecksumValue = epromChannelSmoothFactorData + (NUMBEROFCHANNELS * 4);

unsigned int calcEEPromChecksumValue()
{
  byte eepromSettingsData[epromChecksumValue - epromChannelCalibrationMinData];

  for (int i = epromChannelCalibrationMinData; i < epromChecksumValue; i++) 
  {
    eepromSettingsData[i] = EEPROM.read(i);
  }

  return KIRSP_cal_CRC16(eepromSettingsData, sizeof(eepromSettingsData));
}

void loadSettings()
{
  //Load eeprom checksum
  unsigned int eepromChecksumValue = calcEEPromChecksumValue();
  unsigned int storedEEPromChecksumValue = EEPROM_readInt(epromChecksumValue);

  if (eepromChecksumValue != storedEEPromChecksumValue)
  {
    //eeprom failed the checksum.  Use default values
    for (int i = 0; i< NUMBEROFCHANNELS; i++) 
    {
      _calibratedChannelMin[i] = 1000;
      _calibratedChannelMax[i] = 2000;
      _channelSmoothFactor[i] = 1.0;
    }
  }
  else
  {
    for (int i = 0; i< NUMBEROFCHANNELS; i++) 
    {
      _calibratedChannelMin[i] = EEPROM_readInt(epromChannelCalibrationMinData + (sizeof(int) * i));
      _calibratedChannelMax[i] = EEPROM_readInt(epromChannelCalibrationMaxData + (sizeof(int) * i));
      _channelSmoothFactor[i] = EEPROM_readFloat(epromChannelSmoothFactorData + (sizeof(float) * i));
    }
  }
}

void saveSettings()
{
  for (int i = 0; i< NUMBEROFCHANNELS; i++) 
  {
    EEPROM_writeInt(epromChannelCalibrationMinData + (sizeof(int) * i), _calibratedChannelMin[i]);
    EEPROM_writeInt(epromChannelCalibrationMaxData + (sizeof(int) * i), _calibratedChannelMax[i]);

    EEPROM_writeFloat(epromChannelSmoothFactorData + (sizeof(float) * i), _channelSmoothFactor[i]);
  }

  //store a new checksum
  unsigned int eepromChecksumValue = calcEEPromChecksumValue();
  EEPROM_writeInt(epromChecksumValue, eepromChecksumValue);
}


void setup()
{
  Serial.begin(115200);

  pinMode(LEDPIN, OUTPUT);

  //Load settings from EEPROM
  loadSettings();

  for (int i = 0; i< NUMBEROFCHANNELS; i++) 
  {
    //attach the interrupt handler to each pin
    setupChannel(i);
  }

  Wire.begin(I2CAddress);
  Wire.onRequest(requestEvent);
  
  Serial << "v" << VERSION << " - 0x" << _HEX(I2CAddress) << endl;
}

unsigned long lastReadTime = 0;
unsigned int serialReadInterval = 200;

#define DEFAULTWRITEINTERVAL 200
unsigned long lastWriteTime = 0;
unsigned int serialWriteInterval = DEFAULTWRITEINTERVAL;

#define DEFAULTLEDBLINKINTERVAL 1000
unsigned long lastBlinkTime = 0;
unsigned int ledBlinkInterval = DEFAULTLEDBLINKINTERVAL;

unsigned int ledState = 1;

byte outputType = 'v';

void loop()
{    
  unsigned long currentTime = millis();

  if (lastReadTime + serialReadInterval <= currentTime)
  {
    if (Serial.available())
    {
      byte serialByte = Serial.read();

      switch (serialByte)
      {
      case 'd':
        {
          //Dump channel readings
          outputType = 'd';
          serialWriteInterval = 100;
          break;
        }

      case 's':
        {
          for (int i = 0; i< NUMBEROFCHANNELS; i++) 
          {
            if (i > 0)
            {
              Serial << ",";
            }
            Serial << _calibratedChannelMin[i] << ":" << _calibratedChannelMax[i] << ":" << _channelSmoothFactor[i];
          }
          Serial << endl;
          break;
        }
      case 'c':
        {
          //calibrate
          if (_calibrating)
          {
            _calibrating = 0;
            saveSettings();
            Serial << "Done calibrating." << endl;
            ledBlinkInterval = DEFAULTLEDBLINKINTERVAL;
            serialWriteInterval = DEFAULTWRITEINTERVAL;
            outputType = 0;
          }
          else
          {
            //set the max and min points to be out of bounds so we find the real ones
            for (int i = 0; i< NUMBEROFCHANNELS; i++) 
            {
              _calibratedChannelMin[i] = 3000;
              _calibratedChannelMax[i] = 0;
            }
            _calibrating = 1;
            ledBlinkInterval = 40;
            serialWriteInterval = 100;
            Serial << "Calibrating now.  Type c again when all sticks have been moved to their extremes" << endl;
            outputType = 'd';
          }

          break;
        }
      case 'r':
        {
          //reset
          for (int i = 0; i< NUMBEROFCHANNELS; i++) 
          {
            _calibratedChannelMin[i] = 1000;
            _calibratedChannelMax[i] = 2000;
            _channelSmoothFactor[i] = 1.0;
          }
          saveSettings();

          Serial << "Settings reset" << endl;
          break;
        }

      case 'v':
        {
          Serial << "v" << VERSION << endl;
          break;
        }
        
       case 'h':
       {
         Serial << "I2C Reciever Decoder - v" << VERSION << endl
         << "By Chris Whiteford - Copyright 2010" << endl
         << "-----------------------------------" << endl
         << "Running on I2C Address: 0x" << _HEX(I2CAddress) << " (" << _DEC(I2CAddress) << ")" << endl
         << "-----------------------------------" << endl
         << "d        " << "Dump current channel readings" << endl
         << "x        " << "Stop dumping channel readings" << endl
         << "s        " << "Dump current channel settings (min:max:smooth factor)" << endl
         << "c        " << "Calibrate channels" << endl
         << "r        " << "Reset channel calibration" << endl
         << "v        " << "Print version information" << endl
         << "h        " << "Display this help message" << endl;
         break;
       }
      case 'x':
        {
          outputType = 0;
          serialWriteInterval = DEFAULTWRITEINTERVAL;
          break;
        }
      }
    }

    lastReadTime = currentTime;
  }

  if (lastWriteTime + serialWriteInterval <= currentTime)
  {
    switch (outputType)
    {
    case 'd':
      {
        for (int i = 0; i< NUMBEROFCHANNELS; i++) 
        {
          uint16_t currentValue = getChannelValue(i, currentTime);
          if (i > 0)
          {
            Serial << ",";
          }
          Serial << currentValue;
        }
        Serial << endl;
        break;
      }
    case 0:
      {
        //No output
        break;
      }
    }

    lastWriteTime = currentTime;
  }

  if (lastBlinkTime + ledBlinkInterval <= currentTime)
  {
    digitalWrite(LEDPIN,ledState);
    ledState = !ledState;

    lastBlinkTime = currentTime;
  } 
}








