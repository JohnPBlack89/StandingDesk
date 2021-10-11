/***** ServoLeg Classes *****/
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

}
