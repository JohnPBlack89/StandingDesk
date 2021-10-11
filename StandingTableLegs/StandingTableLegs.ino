#include "Arduino.h"
#include "Wire.h"

// Declaring Buttons Pins:
const int UP_BUTTON = 12;
const int DOWN_BUTTON = 13;
// & Button States:
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

    } else if (upButtonState == LOW && downButtonState == HIGH) {
        //  Down
        digitalWrite(FRONT_LEFT_DOWN, LOW);
        digitalWrite(FRONT_RIGHT_DOWN, LOW);
        digitalWrite(BACK_LEFT_DOWN, LOW);
        digitalWrite(BACK_RIGHT_DOWN, LOW);
    } else if (upButtonState == HIGH && downButtonState == HIGH) {
        // Level
        digitalWrite(FRONT_LEFT_UP, LOW);
        digitalWrite(FRONT_RIGHT_UP, LOW);
        digitalWrite(BACK_LEFT_UP, LOW);
        digitalWrite(BACK_RIGHT_UP, LOW);
        digitalWrite(FRONT_LEFT_DOWN, LOW);
        digitalWrite(FRONT_RIGHT_DOWN, LOW);
        digitalWrite(BACK_LEFT_DOWN, LOW);
        digitalWrite(BACK_RIGHT_DOWN, LOW);
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

/**** Button ****
  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.
  The circuit:
  - LED attached from pin 13 to ground through 220 ohm resistor
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 3;     // the number of the pushbutton pin
// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
void setup() {
  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}
void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    // turn LED off:
    digitalWrite(LED_BUILTIN, LOW);
  }
} */

/**** Gyroscope ****
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */
/*

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
/*
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
/*
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
} */


/***** StandingDesk Class *****
 *  class StandingDesk {
  public:
    StandingDesk(int, int, ServoLeg, ServoLeg, ServoLeg, ServoLeg);
    void Activate();
    void Raise();
    void Lower();
    void Level();
    int upButton;
    int downButton;
    int upButtonState;
    int downButtonState;
    ServoLeg frontLeft(int,int);
    ServoLeg backLeft(int,int);
    ServoLeg backRight(int,int);
    ServoLeg frontRight(int,int);
};

StandingDesk::StandingDesk(int upButtonPin, int downButtonPin, ServoLeg frontLeftSL, ServoLeg backLeftSL, ServoLeg backRightSL, ServoLeg frontRightSL){
  // Declaring Button Pins and States:
  upButton = upButtonPin;
  downButton = downButtonPin;
  upButtonState = 0;
  downButtonState = 0;

  // Getting Servo Motors
  frontLeft = frontLeft(frontLeftSL.upRelay, frontLeftSL.downRelay); 
  backLeft(backLeftSL.upRelay, backLeftSL.downRelay);
  backRight(backRightSL.upRelay, backRightSL.downRelay);
  frontRight(frontRightSL.upRelay, frontRightSL.downRelay);
}

void StandingDesk::Activate() {
  // Declaring Buttons as inputs:
  pinMode(upButton, INPUT);
  pinMode(downButton, INPUT);
  /*
  // Declaring servo leg relay pins as outputs:
  pinMode(frontLeft.upRelay, OUTPUT);
  pinMode(frontLeft.downRelay, OUTPUT);
} */

/***** ServoLeg Classes *****
ServoLeg::ServoLeg(int upRelayPin, int downRelayPin) {
  upRelay = upRelayPin;
  downRelay = downRelayPin;
}

void ServoLeg::Up(){
  digitalWrite(upRelay, LOW);
}

void ServoLeg::Down(){
  digitalWrite(downRelay, LOW);;


class ServoLeg {
  public:
    int upRelay;
    int downRelay;
    ServoLeg(int, int);
    void Up();
    void Down();
};

// Declaring servo legs:
ServoLeg servoLeg(11,11);

} */
