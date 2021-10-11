/*
  Test.h - Test library for Wiring - implementation
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// include core Wiring API
#include "ServoMotorLeg.h"

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

ServoMotorLeg::ServoMotorLeg(int upRelayPin, int downRelayPin)
{
  // initialize this instance's variables
  _upRelayPin = upRelayPin;
  _downRelayPin = downRelayPin;

  // do whatever is required to initialize the library
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

void ServoMotorLeg::raiseLeg(void)
{
  Serial.print("Raising leg at pin ");
  Serial.println(_upRelayPin);
}

void ServoMotorLeg::lowerLeg(void)
{
	Serial.print("Lowering leg at pin ");
	Serial.println(_downRelayPin);
}