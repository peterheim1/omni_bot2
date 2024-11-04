#include <Arduino.h>
#include <Messenger.h>
#include <MobaTools.h>
#include <Wire.h>
#include <TCA9548A.h>
#include <AS5600.h>

// Define stepper motor connections and parameters
const byte stepPinS0 = 3;
const byte dirPinS0 = 6;
const byte stepPinS1 = 2;
const byte dirPinS1 = 5;
const byte stepPinS2 = 4;
const byte dirPinS2 = 7;
const byte stepPinS3 = 12;
const byte dirPinS3 = 13;
const int stepsPerRev = 12835;
const int stepsPerDegree = stepsPerRev / 360;
const int maxDegrees = 135;
const int DockPin = 10;
int DockState = 0;


// Define AS5600 objects
AS5600 stpA;
AS5600 stpB;
AS5600 stpC;
AS5600 stpD;

// I2C ports
const int I2CA = 2;
const int I2CB = 0;
const int I2CC = 1;
const int I2CD = 3;

MoToStepper stepper0(stepsPerRev, STEPDIR);
MoToStepper stepper1(stepsPerRev, STEPDIR);
MoToStepper stepper2(stepsPerRev, STEPDIR);
MoToStepper stepper3(stepsPerRev, STEPDIR);

int S0, S1, S2, S3;  // Target positions
int md0, md1, md2, md3;  // Steps to move
int e0, e1, e2, e3;  // E-stop values
int currentPos0 = 0, currentPos1 = 0, currentPos2 = 0, currentPos3 = 0;
int v0,v1,v2,v3;
Messenger _Messenger = Messenger();
unsigned long previousMillis = 0;
const long interval = 100;
unsigned long lastActiveMillis = 0;
const long idleTime = 2000; // 2 seconds of idle time
float voltage = 0;

// I2C transmission variables
byte a1 = 0;
byte b1 = 0;

// Function prototypes
void TCA9548(uint8_t bus);
void SetTarget(int address, int16_t target);
void ReadSerial();
void OnMessageCompleted();
void ser_print();
void calibrateMotors();
int RequestData(int address);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(DockPin, INPUT_PULLUP);

  // Initialize TCA9548 and AS5600 sensors
  TCA9548(I2CB);
  stpB.begin(4);
  stpB.setDirection(AS5600_CLOCK_WISE);
  
  TCA9548(I2CD);
  stpD.begin(4);
  stpD.setDirection(AS5600_CLOCK_WISE);
  
  TCA9548(I2CA);
  stpA.begin(4);
  stpA.setDirection(AS5600_CLOCK_WISE);
  
  TCA9548(I2CC);
  stpC.begin(4);
  stpC.setDirection(AS5600_CLOCK_WISE);

  _Messenger.attach(OnMessageCompleted);
  Serial.println("moba starting");



  // Initialize steppers
  stepper0.attach(stepPinS0, dirPinS0);
  stepper0.setSpeed(200);
  stepper1.attach(stepPinS1, dirPinS1);
  stepper1.setSpeed(200);
  stepper2.attach(stepPinS2, dirPinS2);
  stepper2.setSpeed(200);
  stepper3.attach(stepPinS3, dirPinS3);
  stepper3.setSpeed(200);

  stepper0.setRampLen(50);
  stepper1.setRampLen(50);
  stepper2.setRampLen(50);
  stepper3.setRampLen(50);
  //analogReference(AR_INTERNAL);
  
}

void loop() {
  ReadSerial();
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // Read current positions from AS5600 sensors
    TCA9548(I2CB);
    currentPos0 = (stpB.rawAngle() * AS5600_RAW_TO_DEGREES) - (180 - 53);
    TCA9548(I2CD);
    currentPos1 = (stpD.rawAngle() * AS5600_RAW_TO_DEGREES) - (180 - 97);
    TCA9548(I2CA);
    currentPos2 = (stpA.rawAngle() * AS5600_RAW_TO_DEGREES) - (180 + 13);
    currentPos2 = -currentPos2;
    TCA9548(I2CC);
    currentPos3 = (stpC.rawAngle() * AS5600_RAW_TO_DEGREES) - (180 - 1);
    voltage = analogRead(A0);
    
    v0 = RequestData(5);
    v1 = RequestData(7);
    v2 = RequestData(8);
    v3 = RequestData(9);
    // Check for motor activity
   // if (abs(stepper0.currentPosition() - currentPos0) > 1 || abs(stepper1.currentPosition() - currentPos1) > 1 || abs(stepper2.currentPosition() - currentPos2) > 1 || abs(stepper3.currentPosition() - currentPos3) > 1) {
      //lastActiveMillis = currentMillis;
    //}

    // Update stepper positions
    //stepper0.write(S0);
    //stepper1.write(S1);
    //stepper2.write(S2);
    //stepper3.write(S3);

    ser_print();
    DockState = digitalRead(DockPin);
  }
  //if (currentMillis - lastActiveMillis > idleTime) {
    //calibrateMotors();
    //lastActiveMillis = currentMillis; // Reset the idle timer after calibration
 // }
}

void TCA9548(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

void SetTarget(int address, int16_t target) {
  a1 = lowByte(target);
  b1 = highByte(target);

  Wire.beginTransmission(address);
  Wire.write(a1);
  Wire.write(b1);
  Wire.endTransmission();
}

int RequestData(int address) {
  int16_t val = 0;
  Wire.requestFrom(address, 2);    // request 2 bytes from slave device

  if (Wire.available() >= 2) {     // Check if two bytes were received
    val = Wire.read();             // Read the first byte
    val <<= 8;                     // Shift it left by 8 bits
    val |= Wire.read();            // Read the second byte and combine
  }

  return val;
}

void ReadSerial() {
  while (Serial.available()) {
    _Messenger.process(Serial.read());
  }
}

void OnMessageCompleted() {
  if (_Messenger.checkString("a") || _Messenger.checkString("b")) {
    S0 = _Messenger.readInt();
    S1 = _Messenger.readInt();
    S2 = _Messenger.readInt();
    S3 = _Messenger.readInt();
    e0 = _Messenger.readInt();
    e1 = _Messenger.readInt();
    e2 = _Messenger.readInt();
    e3 = _Messenger.readInt();

    md0 = (S0 - currentPos0) * stepsPerDegree;
    md1 = (S1 - currentPos1) * stepsPerDegree;
    md2 = (S2 - currentPos2) * stepsPerDegree;
    md3 = (S3 - currentPos3) * stepsPerDegree;

    //e0 = constrain(e0, -255, 255);
    //e1 = constrain(e1, -255, 255);
    //e2 = constrain(e2, -255, 255);
    //e3 = constrain(e3, -255, 255);

    stepper0.write(S0);
    stepper1.write(S1);
    stepper2.write(S2);
    stepper3.write(S3);

    SetTarget(5, e0);
    SetTarget(7, e1);
    SetTarget(8, e2);
    SetTarget(9, e3);

    delay(10);
    lastActiveMillis = millis(); // Reset the idle timer on activity
  } else if (_Messenger.checkString("c")) {
    stepper0.setZero(currentPos0 * stepsPerDegree * -1);
    stepper1.setZero(currentPos1 * stepsPerDegree * -1);
    stepper2.setZero(currentPos2 * stepsPerDegree * -1);
    stepper3.setZero(currentPos3 * stepsPerDegree * -1);
    stepper0.write(0);
    stepper1.write(0);
    stepper2.write(0);
    stepper3.write(0);
  } else if (_Messenger.checkString("d")) {
    S0 = _Messenger.readInt();
  S1 = _Messenger.readInt();
  S2 = _Messenger.readInt();
  S3 = _Messenger.readInt();
  e0 = _Messenger.readInt();
  e1 = _Messenger.readInt();
  e2 = _Messenger.readInt();
  e3 = _Messenger.readInt();
  md0 = (S0 - currentPos0) * stepsPerDegree;
  md1 = (S1 - currentPos1) * stepsPerDegree;
  md2 = (S2 - currentPos2) * stepsPerDegree;
  md3 = (S3 - currentPos3) * stepsPerDegree;
   e0 = constrain(e0, -255, 255);
  e1 = constrain(e1, -255, 255);
  e2 = constrain(e2, -255, 255);
  e3 = constrain(e3, -255, 255);
    
    stepper0.move(md0); 
    stepper1.move(md1);
    stepper2.move(md2); 
    stepper3.move(md3);
    SetTarget(5, e0);
   SetTarget(7, e1);
   SetTarget(8, e2);
   SetTarget(9, e3);
    delay(10);
  } else if (_Messenger.checkString("s")) {
    S0 = _Messenger.readInt();
    S1 = _Messenger.readInt();
    S2 = _Messenger.readInt();
    S3 = _Messenger.readInt();

    stepper0.write(S0);
    stepper1.write(S1);
    stepper2.write(S2);
    stepper3.write(S3);
  }

  while (_Messenger.available()) {
    _Messenger.readInt();
  }
}

void ser_print() {
  Serial.print("a");
  Serial.print("\t");
  Serial.print(currentPos0);
  Serial.print("\t");
  Serial.print(currentPos1);
  Serial.print("\t");
  Serial.print(currentPos2);
  Serial.print("\t");
  Serial.print(currentPos3);
  Serial.print("\t");
  Serial.print(v0);
  Serial.print("\t");
  Serial.print(v1);
  Serial.print("\t");
  Serial.print(v2);
  Serial.print("\t");
  Serial.print(v3);
  Serial.print("\t");
  Serial.print(stepper0.currentPosition());
  Serial.print("\t");
  Serial.print(stepper1.currentPosition());
  Serial.print("\t");
  Serial.print(stepper2.currentPosition());
  Serial.print("\t");
  Serial.print(stepper3.currentPosition());
  Serial.print('\t');
  Serial.print("\n");
  
  Serial.print("b");
  Serial.print("\t");
  Serial.print(voltage);
  Serial.print("\t");
  Serial.print(DockState);
  Serial.print('\t');
  Serial.print("\n");
}
void calibrateMotors() {
  // Calibration function to reset stepper positions based on AS5600 sensors
  Serial.println("Calibrating motors...");
  if (abs(stepper0.currentPosition() - currentPos0) > 1 || abs(stepper1.currentPosition() - currentPos1) > 1 || abs(stepper2.currentPosition() - currentPos2) > 1 || abs(stepper3.currentPosition() - currentPos3) > 1) {
      Serial.println(" i need to cal");
      stepper0.setZero(currentPos0 * stepsPerDegree * -1);
    stepper1.setZero(currentPos1 * stepsPerDegree * -1);
    stepper2.setZero(currentPos2 * stepsPerDegree * -1);
    stepper3.setZero(currentPos3 * stepsPerDegree * -1);
    stepper0.write(0);
    stepper1.write(0);
    stepper2.write(0);
    stepper3.write(0);
      }

}//
