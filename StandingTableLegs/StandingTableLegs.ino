  /* 
Relay Board:
****************
Blue(x4) - Up Relay/Up Leg
Green(x4) - Down Relay/Down Leg
Blue(x4) - Up Relay Controller Input/Up Pin
Green(x4) - Down Relay Controller Input/Down Pin
Red - VCC/5V
Black - GND/Ground
Yellow(x4) - 12V/Input of Relay
White(x4) - Grount/Output of Relay

Buttons:
********
Blue - 13/Up
Green - 12/Down
Red(x2) - 5V/10k Resistor/Button(Same Side as Pin)
Black(x2) - Ground/Button(Opposite Side as Pin)

MPU 6050 Board:
***************
Red - VCC/3.3
Black - GND/Ground
Yellow - SCL/A5
Green - SDA/A4
*/
 
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Arduino.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// Initializing mpu6050 Values
int16_t gx, gy, gz;
int16_t igx, igy, igz;

// Creating float to be used to calculate rotation
float rotX, rotY, rotZ;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
// #define OUTPUT_BINARY_ACCELGYRO

// Declaring Buttons Pins:
const int UP_BUTTON = 12;
const int DOWN_BUTTON = 11;

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
/************** Leg Tolerance ******************/
int legTolerance = 0;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
  // Initializing mpu6050 & getting defaults
  accelgyro.initialize();
  accelgyro.getAcceleration(&gx, &gy, &gz);
  
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

  Serial.begin(38400);
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
        // Reading Gyroscope Data
        accelgyro.getAcceleration(&gx, &gy, &gz);
        processGyroData();
        frontLeft = rotX + rotY;
        backLeft = rotX - rotY;
        backRight = -(rotX) - rotY;
        frontRight = -(rotX) + rotY;

        Serial.print("frontLeft: " + frontLeft );
        Serial.print("backLeft: " + backLeft);
        Serial.print("backRight: " + backRight);
        Serial.println("frontRight: " + frontRight);
        
        // Level Front Left
        if(frontLeft > legTolerance) {
          digitalWrite(FRONT_LEFT_UP, HIGH);
          digitalWrite(FRONT_LEFT_DOWN, LOW);
        } else if (frontLeft < -legTolerance) {
          digitalWrite(FRONT_LEFT_UP, LOW);
          digitalWrite(FRONT_LEFT_DOWN, HIGH);
        } else {
          digitalWrite(FRONT_LEFT_UP, HIGH);
          digitalWrite(FRONT_LEFT_DOWN, HIGH);
        }

        // Level Back Left
        if(backLeft > legTolerance) {
          digitalWrite(BACK_LEFT_UP, HIGH);
          digitalWrite(BACK_LEFT_DOWN, LOW);
        } else if (frontLeft < -legTolerance) {
          digitalWrite(BACK_LEFT_UP, LOW);
          digitalWrite(BACK_LEFT_DOWN, HIGH);
        } else {
          digitalWrite(BACK_LEFT_UP, HIGH);
          digitalWrite(BACK_LEFT_DOWN, HIGH);
        }
        
        // Level Back Right
        if(backRight > legTolerance) {
          digitalWrite(BACK_RIGHT_UP, HIGH);
          digitalWrite(BACK_RIGHT_DOWN, LOW);
        } else if (backRight < -legTolerance) {
          digitalWrite(BACK_RIGHT_UP, LOW);
          digitalWrite(BACK_RIGHT_DOWN, HIGH);
        } else {
          digitalWrite(BACK_RIGHT_UP, HIGH);
          digitalWrite(BACK_RIGHT_DOWN, HIGH);
        }

        // Level Front Right
        if(frontRight > legTolerance) {
          digitalWrite(FRONT_RIGHT_UP, HIGH);
          digitalWrite(FRONT_RIGHT_DOWN, LOW);
        } else if (frontRight < -legTolerance) {
          digitalWrite(FRONT_RIGHT_UP, LOW);
          digitalWrite(FRONT_RIGHT_DOWN, HIGH);
        } else {
          digitalWrite(FRONT_RIGHT_UP, HIGH);
          digitalWrite(FRONT_RIGHT_DOWN, HIGH);
        }
        
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

void processGyroData() {
  // Converting the values received into g's based off unit's LSB sensitivity
  rotX = (gx - igx) / 131.0;
  rotY = (gy - igy) / 131.0;
  rotZ = (gz - igz) / 131.0;
}
