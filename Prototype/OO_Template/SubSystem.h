#ifndef __SUBSYSTEM_H__
#define __SUBSYSTEM_H__

/*
 AeroQuad v2.0 - January 2009
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


/* 
This is the base SubSystem class.  Its designed purpose is to abstract all the "scheduling" for an AeroQuad component.
To use this class you would create a subclass of it and impliment the specific logic for what your building.
*/

class SubSystem
{
private:
  //Private properties used to store the state of a SubSystem
  unsigned int _enabled;              //Is this SubSystem enabled
  
  unsigned long _lastRunTime;         //When was the last time this SubSystem ran (in ms)
  unsigned int _frequency;            //How often should this SubSystem run (in ms)

public:
  SubSystem() 
  {
    //Init all the private properties
    _enabled = 0;
    _lastRunTime = 0;
    _frequency = 0;
  }

  void _initialize(unsigned int frequency, unsigned int offset = 0)
  {
    //Assign the properties to for this SubSystem
    _frequency = frequency;
    _lastRunTime = offset;
    
    //Ensure that this SubSystem is marked as enabled
    _enabled = 1;
  }
  
  //This method can be overloaded by subclasses to perform any necessary initalization.
  //It is imporatnt for any overloaded methods to call the _initialize method to ensure correct setup
  //of a SubSystem
  virtual void initialize(unsigned int frequency, unsigned int offset) 
  { 
    this->_initialize(frequency, offset); 
  }

  //These two methods allow a SubSystem to be enabled or disabled at runtime.  
  void enable()  
  {
    _enabled = 1;
  }

  void disable()
  {
    _enabled = 0;
  }

  //This method lets the current state of this SubSystem be queried.
  unsigned int enabled()
  {
    return _enabled;
  }

  //This method should be called from the overloaded process method listed below.
  //Based on it's return value the subclassed SubSystem can decided to continue processing or stop and wait for the next cycle.
  //It's generally called inside an if and is the first thing done inside the process method.
  unsigned int _canProcess(unsigned long currentTime)
  {
    if (_enabled == 1)
    {
      if (currentTime > (_lastRunTime + _frequency))
      {
        _lastRunTime = currentTime;
        return 1;
      }
    }   

    return 0;
  }

  //This method is ment to be overloaded in the subclass.  It will perform all processing related to the SubSystem
  //it can be though of as the main loop for the SubSystem
  //It should call _canProcess to check to see if the SubSystem is enabled and enough time has elapsed since the last processing cycle
  virtual void process(unsigned long currentTime) {}
};

#endif



