/*
  Test.h - Test library for Wiring - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef ServoMotorLeg_h
#define ServoMotorLeg_h

// library interface description
class ServoMotorLeg
{
  // user-accessible "public" interface
  public:
    ServoMotorLeg(int,int);
    void raiseLeg();
    void lowerLeg();

  // library-accessible "private" interface
  private:
      int _upRelayPin;
      int _downRelayPin;
};

#endif

