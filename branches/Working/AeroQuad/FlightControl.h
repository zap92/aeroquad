#include "SubSystem.h"

class FlightControl:
public SubSystem
{
private:

  bool _blinkState;
  int _blinkPin;

public:

  //Required methods to impliment for a SubSystem
  FlightControl():
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
// ********************************************************************
// *********************** Flight Control Loop ************************
// ********************************************************************
  if ((currentTime > controlLoopTime + CONTROLLOOPTIME) && (controlLoop == ON)) { // 500Hz

  // ********************* Check Flight Mode *********************
    #ifdef AutoLevel
      if (transmitterCommandSmooth[MODE] < 1500) {
        // Acrobatic Mode
        levelAdjust[ROLL] = 0;
        levelAdjust[PITCH] = 0;
      }
      else {
        // Stable Mode
        for (axis = ROLL; axis < YAW; axis++)
          levelAdjust[axis] = limitRange(updatePID(0, flightAngle[axis], &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
        // Turn off Stable Mode if transmitter stick applied
        if ((abs(receiverData[ROLL] - transmitterCenter[ROLL]) > levelOff)) {
          levelAdjust[ROLL] = 0;
          PID[axis].integratedError = 0;
        }
        if ((abs(receiverData[PITCH] - transmitterCenter[PITCH]) > levelOff)) {
          levelAdjust[PITCH] = 0;
          PID[PITCH].integratedError = 0;
        }
      }
    #endif
    
    // ************************** Update Roll/Pitch ***********************
    // updatedPID(target, measured, PIDsettings);
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
    motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL] + levelAdjust[ROLL], (gyroData[ROLL] * mMotorRate) + bMotorRate, &PID[ROLL]);
    motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH] - levelAdjust[PITCH], (gyroData[PITCH] * mMotorRate) + bMotorRate, &PID[PITCH]);

    // ***************************** Update Yaw ***************************
    // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
    // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
    #ifdef HeadingHold
      currentHeading += gyroData[YAW] * headingScaleFactor * controldT;
      if (transmitterCommand[THROTTLE] > MINCHECK ) { // apply heading hold only when throttle high enough to start flight
        if ((transmitterCommand[YAW] > (MIDCOMMAND + 25)) || (transmitterCommand[YAW] < (MIDCOMMAND - 25))) { // if commanding yaw, turn off heading hold
          headingHold = 0;
          heading = currentHeading;
        }
        else // no yaw input, calculate current heading vs. desired heading heading hold
          headingHold = updatePID(heading, currentHeading, &PID[HEADING]);
      }
      else {
        heading = 0;
        currentHeading = 0;
        headingHold = 0;
        PID[HEADING].integratedError = 0;
      }
      motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW] + headingHold, (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    #endif
    
    #ifndef HeadingHold
      motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW], (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    #endif
////////////////////    
    controlLoopTime = currentTime;
  } 
/////////////////////////
// End of control loop //
/////////////////////////
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




