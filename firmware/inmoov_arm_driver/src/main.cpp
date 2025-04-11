#include <Arduino.h>
/*
 * PCA9685_Expander.cpp / OneServo.cpp
 *
 * This example is exact the OneServo example but with #define USE_PCA9685_SERVO_EXPANDER enabled to figure out that there is almost no difference between using the PCA9685 expander or the default Arduino Servo interface.
 * The PCA9685 library was successfully tested with 3 expander boards :-)
 *
 *  Shows smooth linear movement from one servo position to another.
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  ServoEasing is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

// Must specify this before the include of "ServoEasing.hpp"
#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define PCA9685_ACTUAL_CLOCK_FREQUENCY 26000000L // Change it, if your PCA9685 has another than the default 25 MHz internal clock.
//#define USE_SOFT_I2C_MASTER           // Saves 1756 bytes program memory and 218 bytes RAM compared with Arduino Wire
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
//#define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
#define MAX_EASING_SERVOS 32
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables constraints. Saves 4 bytes RAM per servo but strangely enough no program memory.
//#define DEBUG   
// Activating this enables generate lots of lovely debug output for this library.
#include <Wire.h>
#include <I2C_Anything.h>
/*
 * Specify which easings types should be available.
 * If no easing is defined, all easings are active.
 * This must be done before the #include "ServoEasing.hpp"
 */
//#define ENABLE_EASE_QUADRATIC
#define ENABLE_EASE_CUBIC
//#define ENABLE_EASE_QUARTIC
//#define ENABLE_EASE_SINE
//#define ENABLE_EASE_CIRCULAR
//#define ENABLE_EASE_BACK
//#define ENABLE_EASE_ELASTIC
//#define ENABLE_EASE_BOUNCE
//#define ENABLE_EASE_PRECISION
//#define ENABLE_EASE_USER
#include "ServoEasing.hpp"

#include "PinDefinitionsAndMore.h"
#include <Messenger.h>
// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

int j0 = 500;
int j1 = 500;
int j2 = 500;
int j3 = 500;
int j4 = 500;
int j5 = 500;
int j6 = 500;
int j7 = 50;


int s0 = 500;
int s1 = 500;
int s2 = 500;
int s3 = 500;
int s4 = 500;
int s5 = 500;
int s6 = 500;
int s7 = 500;

int rl = 0; //right arm lift
int rr = 0; //right arm rotate
int re =0; //right arm elbow
// upper board
ServoEasing right_ear(0x40); // 
ServoEasing left_ear(0x40); // 
ServoEasing head_yaw(0x40); // 

//lower board
ServoEasing Right_lift(0x60); //
ServoEasing Right_rotate(0x60);//
ServoEasing Right_elbow(0x60);//



byte a1 = 0;
byte b1 = 0;

// set servo target on slave servo
void SetTarget(int address, long target)
{
    //double foo = target;
    b1 = lowByte(target);
    a1 = highByte(target);
    
    Wire.beginTransmission (address);
    Wire.write(b1); // respond with message of 2 bytes
    Wire.write(a1);
    Wire.endTransmission ();
    Serial.println("sent angle");
    return;


       
}

// request position from slave
int RequestData(int address)
{
  int val;
  Wire.requestFrom(address, 2);    // request 2 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
  val = Wire.read ();
  val <<= 8;
  val |= Wire.read ();
  int result = val;
  return result;
  }
}


#define START_DEGREE_VALUE  90 // The degree value written to the servo at time of attach.
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

  if (_Messenger.checkString("m"))//rc joints
  {    
    j0 = _Messenger.readInt();
    //j1 = _Messenger.readInt();
    //j2 = _Messenger.readInt();
    //j4 = _Messenger.readInt();
    s0 = map(j0, -135, 135, 2500, 500);   
    s1 = map(j0, -135, 135, 500, 2500);
    //s2 = map(j2, -135, 135, 500, 2500);
    Serial.println(j0);
    right_ear.startEaseTo(s0,30);//head ears
    left_ear.startEaseTo(s1,30);// hear ears
    
   return;       
  }

   if (_Messenger.checkString("h"))//rc joints
  {    
    j0 = _Messenger.readInt(); //yaw
    j1 = _Messenger.readInt(); //pitch
    j2 = _Messenger.readInt(); //ears
    //ears
    s3 = map(j2, -135, 135, 2500, 500);   
    s2 = map(j2, -135, 135, 500, 2500);
    //pitch
    j1 = j1 * 10;
    //if (j1 > 200) {j1 = 200;}
    //else if (j1 <160) {j1 = 160;}
    //Serial.println(j1);
    s1 = map(j0, -135, 135, 500, 2500); //yaw
    SetTarget(3, j1);//pitch
    //Serial.println(j0);
    head_yaw.startEaseTo(s1,30);//head roll
    right_ear.startEaseTo(s2,30);//head ears
    left_ear.startEaseTo(s3,30);// hear ears
    
   return;       
  }

  
  if (_Messenger.checkString("s"))//steppers
  {
    //joint angle i2c
   int a = _Messenger.readInt() + 180;
   int b = _Messenger.readInt() + 180;
   //int b = _Messenger.readInt();
   Serial.print("sending angle..");
   Serial.println(a);
   Serial.print("sending angle..");
   Serial.println(b);
   SetTarget(3, a);// waist
   SetTarget(9, b);// head tilt
   
   
   
    return;

  }

  if (_Messenger.checkString("i"))//steppers
  {
    //joint angle i2c
   int a = _Messenger.readInt() + 180;
   int b = _Messenger.readInt();
   //int b = _Messenger.readInt();
   Serial.print("sending angle..");
   Serial.println(a);
   Serial.print("to joint..");
   Serial.println(b);
   SetTarget(b, a);// waist  
    return;

  }

  if (_Messenger.checkString("r"))//right arm
  {
    //joint angle i2c
   int a = _Messenger.readInt() *10 ;// j0 waist
   int b = _Messenger.readInt() *10 ;// rj1 tilt
   int c = _Messenger.readInt(); // rj2 lift
   int d = _Messenger.readInt();//rj3 rotate
   int e = _Messenger.readInt();//rj4 elbow 

   a = a + 1500;
   SetTarget(6, a);// waist  
   //SetTarget(4, b);//right tilt
  rl = map(c, -90, 90, 2500, 500); 
  rr = map(d, -135, 135, 2500, 500);  
  re = map(e, -90, 90, 2500, 500); 
  
   Right_lift.startEaseTo(rl,30);//right_lift
   Right_rotate.startEaseTo(rr,30);//right_rotate
   Right_elbow.startEaseTo(re,30);//right_rotate
    return;

  }
  
  if (_Messenger.checkString("a"))//joint angles
  {
    Serial.print("waist ");
    Serial.print(RequestData(6));//waist
//Serial.print(" lt ");
//Serial.print(RequestData(2));//left
Serial.print(" right tilt ");
Serial.print(RequestData(4));//right
Serial.print(" head tilt  ");
Serial.print(RequestData(3));//right
//Serial.print("\t");
//Serial.print(RequestData(5));//right
//Serial.print(" h p ");
Serial.print("\n");
    }
if (_Messenger.checkString("c"))//rc joints
  {
    Serial.println("Commands inmoov robot");
    Serial.println("a read Joint angles");
    Serial.println("s send  Joint angles");
    Serial.println("m  send RC Joint angles");
    Serial.println("i  send Joint angles to motor b");
    return;
    }
    
  }

void setup() {
    
    Serial.begin(57600);
    Serial.println("servo easing inmoov");
    Wire.begin ();
    _Messenger.attach(OnMssageCompleted);
   right_ear.attach(15, START_DEGREE_VALUE);
   left_ear.attach(0, START_DEGREE_VALUE);
   head_yaw.attach(8, START_DEGREE_VALUE);
   
   Right_lift.attach(15,0);  //lower board
   Right_rotate.attach(14,90);  //lower board
   Right_elbow.attach(13,90);  //lower board
   //Servo2.attach(SERVO2_PIN, START_Right_rotate.attach(14,90);  //lower boardDEGREE_VALUE);
   right_ear.setSpeed(10);  
   left_ear.setSpeed(10);
   head_yaw.setSpeed(10);  
   Right_rotate.setSpeed(10);
    
   Right_lift.setSpeed(10);
}

void loop() {
    ReadSerial();
    Serial.print("j");
    Serial.print('\t');
    //Serial.print(RequestData(6));//waist
    Serial.print('\t');
    Serial.print(RequestData(4));//right tilt
    Serial.print('\t');
    Serial.print(RequestData(3));//right
    Serial.print('\n');

      // This speed is taken if no further speed argument is given.
   
    //Serial.println(j0);
//    Servo1.easeTo(DEFAULT_MICROSECONDS_FOR_90_DEGREE); // Alternatively you can specify the target as microsecond value

  
 

    
}
