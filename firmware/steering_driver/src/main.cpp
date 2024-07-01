#include <Arduino.h>
#include <MobaTools.h>

// Define stepper motor connections and parameters
const byte stepPinS0 = 3;
const byte dirPinS0 = 6;
const byte stepPinS1 = 2;
const byte dirPinS1 = 5;
const byte stepPinS2 = 4;
const byte dirPinS2 = 7;
const byte stepPinS3 = 12;
const byte dirPinS3 = 13;
const int stepsPerRev = 12800; // Steps per revolution
const int stepsPerDegree = stepsPerRev / 360; // Steps per degree
const int maxDegrees = 135; // Maximum allowed angle

MoToStepper stepper0(stepsPerRev, STEPDIR); // Create a stepper instance
MoToStepper stepper1(stepsPerRev, STEPDIR); // Create a stepper instance
MoToStepper stepper2(stepsPerRev, STEPDIR); // Create a stepper instance
MoToStepper stepper3(stepsPerRev, STEPDIR); // Create a stepper instance
int S0, S1, S2, S3; // Variables to store the received int values

// Function to parse a string of ints and store them in variables
void parseIntArray(const String& data, int& S0, int& S1, int& S2, int& S3) {
  int startIndex = 0;
  for (int i = 0; i < 4; i++) {
    int endIndex = data.indexOf(' ', startIndex);
    if (endIndex == -1) {
      endIndex = data.length();
    }
    switch (i) {
      case 0: S0 = data.substring(startIndex, endIndex).toInt(); break;
      case 1: S1 = data.substring(startIndex, endIndex).toInt(); break;
      case 2: S2 = data.substring(startIndex, endIndex).toInt(); break;
      case 3: S3 = data.substring(startIndex, endIndex).toInt(); break;
    }
    startIndex = endIndex + 1;
  }
}

void setup() {
  // Start the default serial port
  Serial.begin(115200);

  // Initialize the steppers
  stepper0.attach(stepPinS0, dirPinS0);
  stepper0.setSpeed(10000);   
  stepper1.attach(stepPinS1, dirPinS1);
  stepper1.setSpeed(10000);  
  stepper2.attach(stepPinS2, dirPinS2);
  stepper2.setSpeed(10000);   
  stepper3.attach(stepPinS3, dirPinS3);
  stepper3.setSpeed(10000);  

  stepper0.setRampLen(50);
  stepper1.setRampLen(50);
  stepper2.setRampLen(50);
  stepper3.setRampLen(50);
}

void loop() {
  // Read current positions from analog inputs
  int currentSteps0 = map(analogRead(A2), 0, 1023, -maxDegrees * stepsPerDegree, maxDegrees * stepsPerDegree);  
  int currentSteps1 = map(analogRead(A1), 0, 1023, -maxDegrees * stepsPerDegree, maxDegrees * stepsPerDegree);  
  int currentSteps2 = map(analogRead(A0), 0, 1023, -maxDegrees * stepsPerDegree, maxDegrees * stepsPerDegree);  
  int currentSteps3 = map(analogRead(A3), 0, 1023, -maxDegrees * stepsPerDegree, maxDegrees * stepsPerDegree);

  // Convert steps to degrees for printing
  float currentPos0 = currentSteps0 / (float)stepsPerDegree;
  float currentPos1 = currentSteps1 / (float)stepsPerDegree;
  float currentPos2 = currentSteps2 / (float)stepsPerDegree;
  float currentPos3 = currentSteps3 / (float)stepsPerDegree;

  // Print the current positions in degrees
  Serial.print("Current positions (degrees): ");
  Serial.print("S0: "); Serial.print(currentPos0);
  Serial.print(", S1: "); Serial.print(currentPos1);
  Serial.print(", S2: "); Serial.print(currentPos2);
  Serial.print(", S3: "); Serial.println(currentPos3);

  if (Serial.available() > 0) {
    // Read incoming data until a newline character is found
    String data = Serial.readStringUntil('\n');
    
    // Print the received data
    Serial.print("Received data: ");
    Serial.println(data);

    // Parse the received string into variables
    parseIntArray(data, S0, S1, S2, S3);

    // Constrain the target positions to be within -maxDegrees to +maxDegrees
    S0 = constrain(S0, -maxDegrees, maxDegrees);
    S1 = constrain(S1, -maxDegrees, maxDegrees);
    S2 = constrain(S2, -maxDegrees, maxDegrees);
    S3 = constrain(S3, -maxDegrees, maxDegrees);

    // Print the target positions in degrees
    Serial.print("Target positions (degrees): ");
    Serial.print("S0: "); Serial.print(S0);
    Serial.print(", S1: "); Serial.print(S1);
    Serial.print(", S2: "); Serial.print(S2);
    Serial.print(", S3: "); Serial.println(S3);

    // Calculate the target positions in steps
    int targetPos0 = S0 * stepsPerDegree;
    int targetPos1 = S1 * stepsPerDegree;
    int targetPos2 = S2 * stepsPerDegree;
    int targetPos3 = S3 * stepsPerDegree;

    // Move the steppers to the target positions
    stepper0.move(targetPos0 - currentSteps0); 
    stepper1.move(targetPos1 - currentSteps1);
    stepper2.move(targetPos2 - currentSteps2); 
    stepper3.move(targetPos3 - currentSteps3);
  }

  delay(1);  // Small delay to allow time for serial communication and other operations
}
