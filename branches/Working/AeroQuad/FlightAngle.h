/*
  AeroQuad v1.6 - March 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

// This class is responsible for calculating vehicle attitude

class FlightAngle_CompFilter {
private:
  float dt;
  float previousAngle;
  float newAngle;
  float newRate;
  float filterTerm0;
  float filterTerm1;
  float filterTerm2;
  float timeConstant; // Read in from EEPROM

public:
  //Required methods to impliment for a SubSystem
  FlightAngle_CompFilter() {
    //Perform any initalization of variables you need in the constructor of this SubSystem
    timeConstant = eeprom.read(FILTERTERM_ADR);
    dt = 0;
    filterTerm0 = 0;
    filterTerm1 = 0;
  }
  
  void intialize(float angle, float rate, float ms) {
    previousAngle = angle;
    filterTerm2 = rate;
    dt = ms / 1000.0;
  }
  
  float read(float newAngle, float newRate) {
    // Written by RoyLB at http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286    
    filterTerm0 = (newAngle - previousAngle) * timeConstant * timeConstant;
    filterTerm2 += filterTerm0 * dt;
    filterTerm1 = filterTerm2 + (newAngle - previousAngle) * 2 * timeConstant + newRate;
    previousAngle = (filterTerm1 * dt) + previousAngle;
    return previousAngle; // This is actually the current angle, but is stored for the next iteration
  }

  void setTimeConstant(float value) {timeConstant = value;}
  float getTimeConstant(void) {return timeConstant;}
  void setDeltaT(float ms) {dt = ms / 1000.0;}
};
