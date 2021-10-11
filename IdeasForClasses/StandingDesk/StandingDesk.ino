/***** StandingDesk Class *****/
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

  // Declaring servo leg relay pins as outputs:
  pinMode(frontLeft.upRelay, OUTPUT);
  pinMode(frontLeft.downRelay, OUTPUT);
}
