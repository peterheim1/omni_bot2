
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
const int stepsPerRev = 12835; //12800; // Steps per revolution
const int stepsPerDegree = stepsPerRev / 360; // Steps per degree
const int maxDegrees = 135; // Maximum allowed angle

// Define three as5600 objects as X, Y and Z for further use
AS5600 stpA;
AS5600 stpB;
AS5600 stpC;
AS5600 stpD;
//i2c port
int I2CA=2;
int I2CB=0;
int I2CC=1;
int I2CD=3;

MoToStepper stepper0(stepsPerRev, STEPDIR); // Create a stepper instance
MoToStepper stepper1(stepsPerRev, STEPDIR); // Create a stepper instance
MoToStepper stepper2(stepsPerRev, STEPDIR); // Create a stepper instance
MoToStepper stepper3(stepsPerRev, STEPDIR); // Create a stepper instance
int S0, S1, S2, S3; // Variables to store the received int values

int m0,m1,m2,m3;
// steppes to move
int md0, md1, md2,md3;
// e stops
int e0, e1, e2, e3;
//position off joint 
int p0, p1, p2;
int currentPos0 = 0;
int currentPos1 = 0;
int currentPos2 = 0;
int currentPos3 = 0;
//MovingAveragePlus<unsigned> p00(10);
//MovingAveragePlus<unsigned> p01(10);
//MovingAveragePlus<unsigned> p02(10);
//MovingAveragePlus<unsigned> p03(10);

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

unsigned long previousMillis = 0; 
long interval = 100; 

// i2c dtuff

byte a1 = 0;
byte b1 = 0;

void TCA9548(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}
 
//void displayPosition(float position){
 
//}


void ser_print();

void SetTarget(int address, int target)
{
  a1 = lowByte(target);// mapped_Right_Lift change to *10
  b1 = highByte(target);

   double foo = target;
   Wire.beginTransmission (address);
    Wire.write (a1);
    Wire.write (b1);
   //I2C_writeAnything (target);
    Wire.endTransmission ();
  //
//  Wire.write(address);
//  Wire.write(a1); // respond with message of 6 bytes
  //Wire.write(b1);
       
}


void ReadSerial()
{
  while (Serial.available())
  {
    _Messenger.process(Serial.read());
  }
  
}

// Define messenger function
void OnMssageCompleted()
{

  

 
 if (_Messenger.checkString("a"))// steering joints
  {
    
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
    //Serial.print(" S0 ");
    //Serial.print(m0);
    //Serial.print(" S1 ");
    //Serial.print(m1);
    //Serial.print(" S2 ");
    //Serial.print(m2);
    //Serial.print(" S3 ");
    //Serial.println(m3);
    // Calculate the target positions in steps
    //int targetPos0 = m0 - currentPos0 ;
    //int targetPos1 = m1 - currentPos1 ;
    //int targetPos2 = m2 - currentPos2 ;
    //int targetPos3 = m3 - currentPos3 ;
    //Serial.print("Target positions (degrees): ");
     //Serial.print(", S0: "); Serial.print(targetPos0);
     //Serial.print(", S1: "); Serial.print(targetPos1);
     //Serial.print(", S2: "); Serial.print(targetPos2);
     //Serial.print(", S3: "); Serial.print(targetPos3);

    // Move the steppers to the target positions
    //stepper0.setZero(currentPos0);
    stepper0.move(md0); 
    stepper1.move(md1);
    stepper2.move(md2); 
    stepper3.move(md3);
    SetTarget(5, e0);
   SetTarget(7, e1);
   SetTarget(8, e2);
   SetTarget(9, e3);
    delay(10);
  }


if (_Messenger.checkString("b"))// steering joints
  {
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
  //S0 = S0 * stepsPerDegree
  /*stepper0.setZero(currentPos0);
  int gg = (stepper0.read() - currentPos0) * stepsPerDegree;

  int hh = S0 - currentPos0;
  Serial.print(currentPos0);
  Serial.print("\t"); 
  Serial.print(stepper0.read());
  Serial.print("\t");
  Serial.print(stepper0.readSteps());
  Serial.print("\t"); 
  Serial.print(gg);
  Serial.print("\t"); 
  Serial.print(hh);
  Serial.print("\n");
*/
  //stepper0.write(S0); 
  //stepper1.write(S1); 
  //stepper2.write(S2); 
  //stepper3.write(S3);
  //delay(10);
  SetTarget(5, e0);
   SetTarget(7, e1);
   SetTarget(8, e2);
   SetTarget(9, e3);
   

  }

  if (_Messenger.checkString("c"))// steering joints
  {
    stepper0.setZero(currentPos0 * stepsPerDegree * -1);
    stepper1.setZero(currentPos1 * stepsPerDegree * -1);
    stepper2.setZero(currentPos2 * stepsPerDegree * -1);
    stepper3.setZero(currentPos3 * stepsPerDegree * -1);

  }
  if (_Messenger.checkString("d"))// steering joints
  {
    e0 = _Messenger.readInt();
  e1 = _Messenger.readInt();
  e2 = _Messenger.readInt();
  e3 = _Messenger.readInt();

  e0 = constrain(e0, -255, 255);
  e1 = constrain(e1, -255, 255);
  e2 = constrain(e2, -255, 255);
  e3 = constrain(e3, -255, 255);

  SetTarget(5, e0);
   SetTarget(7, e1);
   SetTarget(8, e2);
   SetTarget(9, e3);

  }

  if (_Messenger.checkString("s"))// steering joints
  {
    S0 = _Messenger.readInt();
  S1 = _Messenger.readInt();
  S2 = _Messenger.readInt();
  S3 = _Messenger.readInt();

  stepper0.write(S0); 
  stepper1.write(S1); 
  stepper2.write(S2); 
  stepper3.write(S3);

  }

   

  // clear out unrecognized content
  while(_Messenger.available())
  {
    _Messenger.readInt();
  }
}


void setup(){

  Serial.begin(115200);
  Wire.begin();
  


    //S0
    TCA9548(I2CB);
    stpB.begin(4);
    stpB.setDirection(AS5600_CLOCK_WISE);

// S1
    TCA9548(I2CD);
    stpD.begin(4);
    stpD.setDirection(AS5600_CLOCK_WISE);

    // S2
  TCA9548(I2CA);
    stpA.begin(4);
    stpA.setDirection(AS5600_CLOCK_WISE);

    //S3
     TCA9548(I2CC);
    stpC.begin(4);
    stpC.setDirection(AS5600_CLOCK_WISE);
    




  _Messenger.attach(OnMssageCompleted);
  Serial.println("moba starting");
  
  stepper0.attach(stepPinS0, dirPinS0);
  stepper0.setSpeed(300);   
  stepper1.attach(stepPinS1, dirPinS1);
  stepper1.setSpeed(300);  
  stepper2.attach(stepPinS2, dirPinS2);
  stepper2.setSpeed(300);   
  stepper3.attach(stepPinS3, dirPinS3);
  stepper3.setSpeed(300);  

  stepper0.setRampLen(0);
  stepper1.setRampLen(0);
  stepper2.setRampLen(0);
  stepper3.setRampLen(0);   
 
}
void loop(){
  
ReadSerial();
unsigned long currentMillis = millis();
  
    if(currentMillis - previousMillis > interval) {
   
    previousMillis = currentMillis; 
    stepper0.write(S0); 
  stepper1.write(S1); 
  stepper2.write(S2); 
  stepper3.write(S3);
    
  //currentPos0 = map(analogRead(A2), 0, 1023, -maxDegrees , maxDegrees );  
  //currentPos1 = map(analogRead(A1), 0, 1023, -maxDegrees , maxDegrees );  
  //currentPos2 = map(analogRead(A0), 0, 1023, -maxDegrees , maxDegrees );  
  //currentPos3 = map(analogRead(A3), 0, 1023, -maxDegrees , maxDegrees );
  //S0
  TCA9548(I2CB);
    currentPos0 =(stpB.rawAngle() * AS5600_RAW_TO_DEGREES)- (180 - 53);
  //S1
  TCA9548(I2CD);
    currentPos1 =(stpD.rawAngle() * AS5600_RAW_TO_DEGREES)- (180 - 97);
  //S2
  TCA9548(I2CA);
   currentPos2 =(stpA.rawAngle() * AS5600_RAW_TO_DEGREES)- (180 + 13);
   currentPos2 = currentPos2 * -1;

  //S3
  TCA9548(I2CC);
    currentPos3 =(stpC.rawAngle() * AS5600_RAW_TO_DEGREES)- (180 - 1);

  
 

  ser_print();
  //Serial.print(currentPos0);

//Serial.print("\t"); 
  //Serial.println(stepper0.read());
    
  }

  
}
void ser_print(){
  
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

Serial.print(stepper0.currentPosition());
Serial.print("\t");
Serial.print(stepper1.currentPosition());
Serial.print("\t");
Serial.print(stepper2.currentPosition());
Serial.print("\t");
Serial.print(stepper3.currentPosition());
Serial.print("\t");
  Serial.print("\n");

}