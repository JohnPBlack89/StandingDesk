// Wiring for this sketch:
// Arduino  ->     MPU
// *******       *******
//    5V           VCC
//   GND           GND
//    A4           SCL
//    A5           SDA
//                 AD0 -> MPU GND


// The Wire.h library is used for the I2C communication
#include <Wire.h>

// While GPU6050 Libraries exist, they were not used to
// demonstrate how to access the registers on thier own

// Raw acceleration data read from the MPU
long accelX, accelY, accelZ;
// Creating floats used to calculate g-forces
float gForceX, gForceY, gForceZ;

// Raw gyroscopic data read from the MPU
long gyroX, gyroY, gyroZ;
// Creating float to be used to calculate rotation
float rotX, rotY, rotZ;

void setup() {
  // Serial.begin used for troubleshooting data:
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  delay(100);
}

// The purpose of the setupMPU() function is two-fold:
//  1. Establish communication with the MPU
//  2. Set up all of the registers that we will use to read data back from the MPU back to the Arduino
void setupMPU() {
  // Setting up transmission with the MPU:
  Wire.beginTransmission(0b1101000); // This is the I2C address of the MPU (b1101000/b1101001 for the AC0 low/high data)
  Wire.write(0x6B); // Accessing the register 6B - Power Management (Sec. 4.28 of the Datasheet)
  Wire.write(0b00000000); // Setting the SLEEP register to 0. (Required; see Note on pg. 9, device comes up in sleep mode on power up
                          // Probably need to adjust this to conserve power, as the desk will only need to access these registers when moving up or down
                          // First you set the register you wish to write to, then what you wish to write (In this case 0x6B is our register, 0b00000000 is what were writing)
  Wire.endTransmission();

  // Implementation of the Accelerometer
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x1C); // Accessing the register 1C -Accelerometer Configuration
  Wire.write(0b00000000); // Setting the accel to +/- 2g, the bits are counted from right to left
  Wire.endTransmission();

  // Implementation of the Gyroscope
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x1B); // Accessing the register  1B - Gyroscope Configuration
  Wire.write(0b00000000); // Setting the gyro to full scale +/- 250deg
  Wire.endTransmission();
}


void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x3B); // Starting Register for Accel Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0b1101000,6); // Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();
  accelY = Wire.read()<<8|Wire.read();
  accelZ = Wire.read()<<8|Wire.read();

  processAccelData();
}

void processAccelData(){
  // Converting the values received into g's based off unit's LSB sensitivity
  gForceX = accelY /16384.0;
  gForceX = accelY /16384.0;
  gForceX = accelY /16384.0;
}


void recordGyroRegisters(){
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x43); // Starting Register for Accel Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0b1101000,6); // Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read(); 

  processGyroData();
}

void processGyroData() {
  // Converting the values received into g's based off unit's LSB sensitivity
  rotX = gyroX/131.0;
  rotY = gyroY/131.0;
  rotZ = gyroZ/131.0;
}

void printData(){
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(gyroX);
  Serial.print(" Y=");
  Serial.print(gyroY);
  Serial.print(" Z=");
  Serial.print(gyroZ);
  Serial.print(" Accel(g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.print(gForceZ);
}
