#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>    // RobTillaart AS5600 library
#include <MobaTools.h> // MobaTools library for stepper motor control
// AS5600 sensor instance
AS5600 sensor;

// Stepper motor definitions
const byte dirPinS0 = 19;
const byte stepPinS0 = 18;
const int stepsPerRev = 241560;         // Steps per revolution
const int stepsPerDegree = 671; //stepsPerRev / 360;  // Steps per degree

// Create stepper instance in STEPDIR mode
MoToStepper stepper0(stepsPerRev, STEPDIR);

// Must specify this before the include of "ServoEasing.hpp"
#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
#define MAX_EASING_SERVOS 32
#define ENABLE_EASE_CUBIC
#include "ServoEasing.hpp"
#include "PinDefinitionsAndMore.h"
// upper board
ServoEasing right_ear(0x40); // 
ServoEasing left_ear(0x40); // 
ServoEasing head_yaw(0x40); // 

//lower board
ServoEasing right_j1_joint(0x60); //
ServoEasing right_j2_joint(0x60);//
ServoEasing right_j3_joint(0x60); //
ServoEasing right_j4_joint(0x60);//
ServoEasing right_j5_joint(0x60);//
ServoEasing right_j6_joint(0x60);//

ServoEasing left_j1_joint(0x60); //
ServoEasing left_j2_joint(0x60);//
ServoEasing left_j3_joint(0x60); //
ServoEasing left_j4_joint(0x60);//
ServoEasing left_j5_joint(0x60);//
ServoEasing left_j6_joint(0x60);//


int h1,h2,h3,h4;
int r1,r2,r3,r4,r5,r6;
int l1,l2,l3,l4,l5,l6;
int s0,s1,s2,s3,s4,s5,s6;

// Function prototypes for command handling
void processCommandH(int angles[], int count);
void processCommandL(int angles[], int count);
void processCommandC(int angles[], int count);
void processCommandD(int angles[], int count);
void processCommandR(int angles[], int count);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port connection (if needed)

  }

Wire.begin(); // Initialize I2C

  // Initialize the AS5600 sensor; if not found, halt execution.
  if (!sensor.begin()) {
    Serial.println("AS5600 sensor not found!");
    while (1);
  }
   right_ear.attach(15, 90);
   left_ear.attach(0, 90);
   head_yaw.attach(8, 90);
   
   left_j1_joint.attach(15,160); //
   left_j2_joint.attach(14,90);//
   left_j3_joint.attach(13,90); //
   left_j4_joint.attach(12,90);//
   left_j5_joint.attach(11,90);//
   left_j6_joint.attach(10,90);//

   right_j1_joint.attach(0,30); //
   right_j2_joint.attach(1,90);//
   right_j3_joint.attach(2,90); //
   right_j4_joint.attach(3,90);//
   right_j5_joint.attach(4,90);//
   right_j6_joint.attach(5,90);//

   left_j1_joint.setSpeed(30);
   left_j2_joint.setSpeed(30);
   left_j3_joint.setSpeed(30);
   left_j4_joint.setSpeed(30);
   left_j5_joint.setSpeed(30);
   left_j6_joint.setSpeed(30);

   right_j1_joint.setSpeed(30);
   right_j2_joint.setSpeed(30);
   right_j3_joint.setSpeed(30);
   right_j4_joint.setSpeed(30);
   right_j5_joint.setSpeed(30);
   right_j6_joint.setSpeed(30);

   right_ear.setSpeed(10);  
   left_ear.setSpeed(10);
   head_yaw.setSpeed(10);  
 
 // Initialize stepper motor pins and attach the motor.
  pinMode(stepPinS0, OUTPUT);
  pinMode(dirPinS0, OUTPUT);
  stepper0.attach(stepPinS0, dirPinS0);
  // Set initial stepper speed and ramp length.
  stepper0.setSpeed(10);
  stepper0.setRampLen(50);


  

 
  Serial.println("ESP32 Serial Parser with servo easing Ready");
}

void loop() {
  // Process incoming serial data
  if (Serial.available() > 0) {
    // Read a full line from the serial monitor (ending with '\n')
    String line = Serial.readStringUntil('\n');
    line.trim();  // Remove extra whitespace

    if (line.length() > 0) {
      // Convert the first character (command) to lower case
      char cmd = tolower(line.charAt(0));
      
      // Extract the rest of the line as parameters (angle values)
      String paramsString = line.substring(1);
      paramsString.trim();
      
      // Array to hold up to 6 angle values
      int angles[6];
      int count = 0;
      
      // Tokenize the parameters using spaces as delimiters
      while (paramsString.length() > 0 && count < 6) {
        int spaceIndex = paramsString.indexOf(' ');
        String token;
        if (spaceIndex == -1) {
          token = paramsString;
          paramsString = "";
        } else {
          token = paramsString.substring(0, spaceIndex);
          paramsString = paramsString.substring(spaceIndex + 1);
        }
        token.trim();
        if (token.length() > 0) {
          int value = token.toInt();
          // Validate that the angle value is within -256 to +256.
          if (value < -256 || value > 256) {
            Serial.print("Angle out of range: ");
            Serial.println(value);
          }
          angles[count++] = value;
        }
      }
      
      // Dispatch command based on the lower-case command letter.
      switch (cmd) {
        case 'h':
          processCommandH(angles, count);
          break;
        case 'c':
          processCommandC(angles, count);
          break;
        case 'l':
          processCommandL(angles, count);
          break;
        case 'd':
          processCommandD(angles, count);
          break;
        case 'r':
          processCommandR(angles, count);
          break;  
        default:
          Serial.print("Unknown command: ");
          Serial.println(cmd);
          break;
      }
    }
  }
  
  // Read and print the encoder angle every second.
  //static unsigned long lastPrint = 0;
  //if (millis() - lastPrint > 1000) {
    
  //}
}

// --- Option h head ---
// Take the first value as the required (absolute) angle, then subtract the current encoder angle,
// multiply the difference by stepsPerDegree, and move the stepper accordingly.
void processCommandH(int angles[], int count) {
  Serial.print("Function A called with ");
  Serial.print(count);
  Serial.println(" angle value(s):");
  for (int i = 0; i < count; i++) {
    Serial.print("Angle ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(angles[i]);
int raw = sensor.rawAngle();
    float sensorAngle = (raw * 360.0 / 4096.0) - 56.0;  // Convert raw (0-4095) to degrees
    //h1 = angles[0];
    h1 = map(angles[0], -90, 90 , 500, 2500);// yaw
    h2 = angles[1];
    h3 = angles[2];//  ears
    h4 = angles[3];// ears
    h2 = constrain(h2, -30, 30);


    
  }
  
  if (count > 0) {
    Serial.print(h1);//pan
    Serial.print(" : ");
    Serial.print(h2);//tilt
    Serial.print(" : ");
     Serial.println(h3);//ears
    //Serial.print(" : ");
    //Serial.println(h4);
  
    
    int requiredAngle = h2;
    float currentAngle = sensor.rawAngle() * (360.0 / 4096.0)-146;
    int diffAngle = requiredAngle - currentAngle; // Difference in degrees
    int steps = diffAngle * stepsPerDegree;         // Convert difference to steps
    s1 = map(h3, -90, 90, 500, 2500);   
    s2 = map(h4, -90, 90, 2500, 500);// right ear
    stepper0.move(steps);
    head_yaw.startEaseTo(h1,30);//head yaw
    //tilt
    right_ear.startEaseTo(s2,30);//head ears
    left_ear.startEaseTo(s1,30);// hear ears
    //delay(2000);
    
  }
}

// --- Option B ---
// Use the first parameter as the number of steps to move the stepper.
// Print the current encoder angle before the move and the angle after the move.
// This can be used to calibrate the steps per degree.
void processCommandC(int angles[], int count) {
  Serial.print("Function home called with ");

    float currentAngle = sensor.rawAngle() * (360.0 / 4096.0)-146;
    int diffAngle = 0 - currentAngle; // Difference in degrees
    int steps = diffAngle * stepsPerDegree;         // Convert difference to steps

    stepper0.move(steps);
    head_yaw.startEaseTo(90,30);//head yaw
    //tilt
    right_ear.startEaseTo(90,30);//head ears
    left_ear.startEaseTo(90,30);// hear ears
    right_j1_joint.startEaseTo(180 ,30);
    right_j2_joint.startEaseTo(160,30);
    right_j3_joint.startEaseTo(90,10);
    right_j4_joint.startEaseTo(90,10);
    left_j1_joint.startEaseTo(180,30);
   delay(2000);

  
}

// --- Option C ---  left arm
// Currently, Option C simply prints the received angle values.
void processCommandL(int angles[], int count) {
  Serial.print("Function C left arm count ");
  Serial.print(count);
  Serial.println(" parameter(s) for servo control:");
  for (int i = 0; i < count; i++) {
    Serial.print("Servo value ");
    Serial.print(i);
    Serial.print(": ");
    
  }
  
  // Ensure at least three parameters are provided.
  if (count >= 0) {
    
    l1 = map(angles[0], -135, 135 , 500, 2500);// l1
    l2 = map(angles[1], -90, 90 , 500, 2500);// l2
    l3 = map(angles[2], -90, 90 , 500, 2500);// l3
    int cl = constrain(angles[3], 0 ,90);
    l4 = map(cl, -90, 90 , 2500, 500);// l4
    l5 = map(angles[4], -90, 90 , 500, 2500);// left_j5_joint
    l6 = map(angles[5], -90, 90 , 500, 2500);// l6  
     left_j1_joint.startEaseTo(l1,30);
     left_j2_joint.startEaseTo(l2,30);
     left_j3_joint.startEaseTo(l3,30);
     left_j4_joint.startEaseTo(l4,30);
     left_j5_joint.startEaseTo(l5,30);
     left_j6_joint.startEaseTo(l6,30);
    
    Serial.println("left Servos moving:");
   
    
    // Optionally, if a fourth parameter is provided, control head_laser.
    if (count >= 4) {
      //int s4 = angles[3];
      //head_laser.startEaseTo(s4, 30);
      Serial.print("  head_laser -> ");
      //Serial.println(s4);
    }
  } else {
    Serial.println("Insufficient parameters for servo control (need at least 3).");
  }
}

// --- Option D ---
// Currently, Option D simply prints the received angle values.
void processCommandD(int angles[], int count) {
  Serial.println("Option D selected: Initiating head_laser sweep sequence.");

  
 
int raw = sensor.rawAngle();
    float sensorAngle = (raw * 360.0 / 4096.0) - 146.0;  // Convert raw (0-4095) to degrees
    Serial.println(sensorAngle);

}
// right arm
void processCommandR(int angles[], int count) {
  Serial.print("Function R called with ");
  Serial.print(count);
  Serial.println(" parameter(s) for calibration:");
  if (count > 0) {
    
    Serial.print("right arm Angle: ");
    r1 = map(angles[0], -135, 135 , 2500, 500);// r1
    r2 = map(angles[1], -135, 135, 500, 2500);// r2
    r3 = map(angles[2], -90, 90 , 500, 2500);// r3
    int cr = constrain(angles[3], -90 ,0);
    r4 = map(cr, -90, 90 , 550 , 2500);// r4
    r5 = map(angles[4], -90, 90 , 500, 2500);// right_j4_joint
    r6 = map(angles[5], -90, 90 , 500, 2500);// right_j5_joint
    //Serial.println(currentAngle);
    
    right_j1_joint.startEaseTo(r1,30);
    right_j2_joint.startEaseTo(r2,30);
    right_j3_joint.startEaseTo(r3,30);
    right_j4_joint.startEaseTo(r4,30);
    right_j5_joint.startEaseTo(r5,30);
    right_j6_joint.startEaseTo(r6,30);
  }
}
