/*
 AeroQuad v1.6 - March 2010
 www.AeroQuad.info
 Copyright (c) 2009 Chris Whiteford.  All rights reserved.
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

// This class is used as an example of how to implement a subclass for SubSystem
// This is the main example used for creating the Sensor, Receiver, Attitude, FlightControl, Motor and SerialComs classes

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
    //Perform any initalization of variables you need in the constructor of this SubSystem
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

      _blinkState = !_blinkState;
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




