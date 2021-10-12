/***************************************************************************************************
 * This commit is for testing the buttons and relays ONLY, there is no gyroscope implementation yet
 ***************************************************************************************************/

#include "Arduino.h"
#include "Wire.h"

// Declaring Buttons Pins:
const int UP_BUTTON = 13;
const int DOWN_BUTTON = 12;

// Declaring Button States:
int upButtonState = 0;
int downButtonState = 0;

// Declaring Servo Pins:
const int FRONT_LEFT_UP = 9;
const int FRONT_LEFT_DOWN = 8;
const int BACK_LEFT_UP = 7;
const int BACK_LEFT_DOWN = 6;
const int BACK_RIGHT_UP = 5;
const int BACK_RIGHT_DOWN = 4;
const int FRONT_RIGHT_UP = 3;
const int FRONT_RIGHT_DOWN = 2;

// Declaring Leg States:
int frontLeft = 0;
int backLeft = 0;
int backRight = 0;
int frontRight = 0;

void setup() {
  // Initializing Buttons as Input:
  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);
  
  //Initializing Servo Pins as Output
  pinMode(FRONT_LEFT_UP, OUTPUT);
  pinMode(FRONT_LEFT_DOWN, OUTPUT);
  pinMode(BACK_LEFT_UP, OUTPUT);
  pinMode(BACK_LEFT_DOWN, OUTPUT);
  pinMode(BACK_RIGHT_UP, OUTPUT);
  pinMode(BACK_RIGHT_DOWN, OUTPUT);
  pinMode(FRONT_RIGHT_UP, OUTPUT);
  pinMode(FRONT_RIGHT_DOWN, OUTPUT);
}

void loop() {
    // Up Functionality
    upButtonState = digitalRead(UP_BUTTON);
    downButtonState = digitalRead(DOWN_BUTTON);
    
    if (upButtonState == HIGH && downButtonState == LOW) {
        // Up
        digitalWrite(FRONT_LEFT_UP, LOW);
        digitalWrite(FRONT_RIGHT_UP, LOW);
        digitalWrite(BACK_LEFT_UP, LOW);
        digitalWrite(BACK_RIGHT_UP, LOW);
        digitalWrite(FRONT_LEFT_DOWN, HIGH);
        digitalWrite(FRONT_RIGHT_DOWN, HIGH);
        digitalWrite(BACK_LEFT_DOWN, HIGH);
        digitalWrite(BACK_RIGHT_DOWN, HIGH);

    } else if (upButtonState == LOW && downButtonState == HIGH) {
        //  Down
        digitalWrite(FRONT_LEFT_UP, HIGH);
        digitalWrite(FRONT_RIGHT_UP, HIGH);
        digitalWrite(BACK_LEFT_UP, HIGH);
        digitalWrite(BACK_RIGHT_UP, HIGH);
        digitalWrite(FRONT_LEFT_DOWN, LOW);
        digitalWrite(FRONT_RIGHT_DOWN, LOW);
        digitalWrite(BACK_LEFT_DOWN, LOW);
        digitalWrite(BACK_RIGHT_DOWN, LOW);
        
    } else if (upButtonState == LOW && downButtonState == LOW) {
        // Level
        
        
    } else {
        // Stop 
        digitalWrite(FRONT_LEFT_UP, HIGH);
        digitalWrite(FRONT_LEFT_DOWN, HIGH);
        digitalWrite(BACK_LEFT_UP, HIGH);
        digitalWrite(BACK_LEFT_DOWN, HIGH);
        digitalWrite(BACK_RIGHT_UP, HIGH);
        digitalWrite(BACK_RIGHT_DOWN, HIGH);
        digitalWrite(FRONT_RIGHT_UP, HIGH);
        digitalWrite(FRONT_RIGHT_DOWN, HIGH);
    }
  }   
