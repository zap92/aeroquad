
#include "SubSystem.h"

class Blinkie: 
public SubSystem
{
private:

  bool _blinkState;
  int _blinkPin;

public:

  //Required methods to impliment for a SubSystem
  Blinkie():
  SubSystem()
  {
    //Perform any initalization of varialbes you need in the constructor of this SubSystem
    _blinkState = false;
    _blinkPin = 13;
  }

  void initialize(unsigned int frequency, unsigned int offset = 0) 
  { 
    //Call the parent class' _initialize to setup all the frequency and offset related settings
    this->_initialize(frequency, offset);   
    
    //Perform any custom initalization you need for this SubSystem    
    pinMode(_blinkPin, OUTPUT);
    digitalWrite(_blinkPin, LOW);        
  }

  void process(unsigned long currentTime)
  {
    //Check to see if this SubSystem is allowed to run
    //The code in _canProcess checks to see if this SubSystem is enabled and its been long enough since the last time it ran
    //_canProcess also records the time that this SubSystem ran to use in future timing checks.
    if (this->_canProcess(currentTime))
    {   
      //If the code reaches this point the SubSystem is allowed to run.
      
      if (_blinkState)
      {
        digitalWrite(_blinkPin, HIGH);
      }
      else
      {
        digitalWrite(_blinkPin, LOW);
      }
      
      _blinkPin = !_blinkPin;
    }
  }

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionaly externally  
  bool getBlinkState()
  {
    return _blinkState;
  }
  
  void setBlinkState(bool newBlinkState)
  {
    _blinkState = newBlinkState;
  }
};




