/*
  ServoMotorLeg.h - Library for raising and lowering desk legs
  Created by John P. Black 9/8/21
  Released into the public domain.
*/
#ifndef ServoMotorLeg_h
#define ServoMotorLeg_h

#include "Arduino.h"

class ServoMotorLeg
{
  public:
    ServoMotorLeg(int upRelayPin, int downRelayPin);
    void Raise();
    void Lower();
  private:
    int _upRelayPin
    int _downRelayPin;
};


#endif
