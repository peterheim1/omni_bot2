#include <Arduino.h>

//Encoder front_Left_Encoder(34, 33); //en1
//Encoder front_Right_Encoder(36, 35);  //en2
//Encoder rear_Left_Encoder(37, 38);  //en3
//Encoder rear_Right_Encoder(39, 32);   //en4

//Front Left Drive motor 0
#define F_L_drive_InA            30                       // INA motor pin
#define F_L_drive_InB            31                       // INB motor pin
#define F_L_drive_PWM            14                      // PWM motor pin m

//Front Right Drive motor  3
#define F_R_drive_InA            26                       // INA motor pin
#define F_R_drive_InB            27                       // INB motor pin
#define F_R_drive_PWM            7                        // PWM motor pin m

//rear left Drive motor  1
#define R_L_drive_InA            28                       // INA motor pin
#define R_L_drive_InB            29                       // INB motor pin
#define R_L_drive_PWM            8                        // PWM motor pin m

//rear Right Drive motor  2
#define R_R_drive_InA            24                       // INA motor pin
#define R_R_drive_InB            25                       // INB motor pin
#define R_R_drive_PWM            2                        // PWM motor pin m
#define battery_pin 	        	A6
#define relay_pin               23

// motor variables
void front_right_motor();
void front_left_motor();
void rear_right_motor();
void rear_left_motor();


// pid output
double Output_F_L = 0;
double Output_F_R = 0;
double Output_R_L = 0;
double Output_R_R = 0;


// Function to parse a string of ints and store them in variables
void parseIntArray(String data, int& S0, int& S1, int& S2, int& S3) {
  int values[4];
  int startIndex = 0;
  for (int i = 0; i < 4; i++) {
    int endIndex = data.indexOf(' ', startIndex);
    if (endIndex == -1) {
      endIndex = data.length();
    }
    values[i] = data.substring(startIndex, endIndex).toInt();
    startIndex = endIndex + 1;
  }
  S0 = values[0];
  S1 = values[1];
  S2 = values[2];
  S3 = values[3];
}
int S0, S1, S2, S3;  // Variables to store the received int values
//position off joint from pot
int p0, p1, p2, p3;

const unsigned long loopInterval = 20;  // Interval in milliseconds for a 50 Hz loop
unsigned long lastUpdateTime = 0;

void setup() {
  // Start the default serial port
  Serial.begin(115200);

  // Wait for the serial port to connect - useful in case you are using the Serial Monitor
  while (!Serial) {
    delay(10);
  }

   Serial.println("teensy starting");
  pinMode(LED_BUILTIN, OUTPUT);

//init front left
  pinMode(F_L_drive_InA, OUTPUT);
  pinMode(F_L_drive_InB, OUTPUT);
  pinMode(F_L_drive_PWM, OUTPUT);
  //init front right
  pinMode(F_R_drive_InA, OUTPUT);
  pinMode(F_R_drive_InB, OUTPUT);
  pinMode(F_R_drive_PWM, OUTPUT);
   //init rear right
  pinMode(R_R_drive_InA, OUTPUT);
  pinMode(R_R_drive_InB, OUTPUT);
  pinMode(R_R_drive_PWM, OUTPUT);
                     
  //init rear left
  pinMode(R_L_drive_InA, OUTPUT);
  pinMode(R_L_drive_InB, OUTPUT);
  pinMode(R_L_drive_PWM, OUTPUT);

  //analogWriteFrequency(F_L_drive_PWM, 46875);
  //analogWriteFrequency(F_R_drive_PWM, 46875);
  //analogWriteFrequency(R_L_drive_PWM, 46875);
  //analogWriteFrequency(R_R_drive_PWM, 46875);
  
  Serial.println("uno r4 Serial Int Array Read Example");

  
  
}

void loop() {
  if (Serial.available()) {
    // Read incoming data until a newline character is found
    String data = Serial.readStringUntil('\n');
    
    // Print the received data
    Serial.print("Received data: ");
    Serial.println(data);

    // Parse the received string into variables
    parseIntArray(data, S0, S1, S2, S3);

    // Print the int values
    //Serial.print("S0: ");
    //Serial.println(S0);
    //Serial.print("S1: ");
   // Serial.println(S1);
    //Serial.print("S2: ");
    //Serial.println(S2);
    //Serial.print("S3: ");
    //Serial.println(S3);
  }
  Serial.print("S0: ");
    Serial.println(S0);
    Serial.print("S1: ");
    Serial.println(S1);
    Serial.print("S2: ");
    Serial.println(S2);
    Serial.print("S3: ");
    Serial.println(S3);
  
  delay(100);  // Adjust the delay as necessary

  p0 = map(analogRead(A1), 0, 1023, -135, 135);  
  p1 = map(analogRead(A2), 0, 1023, -135, 135);  
  p2 = map(analogRead(A3), 0, 1023, -135, 135);  
  p3 = map(analogRead(A4), 0, 1023, -135, 135); 
 
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
  
  //delay(50);
  
        Serial.print("s"); // s indicates steering position
        Serial.print("\t");
        Serial.print(p0); // front left
        Serial.print("\t");
        Serial.print(p1); // rear left
        Serial.print("\t");
        Serial.print(p2); // rear right
        Serial.print("\t");
        Serial.print(p2); // front right
        Serial.print("\n");

 
       Output_F_L = S0;
       Output_F_R = S3;
       Output_R_L = S1;
       Output_R_R = S2;

front_left_motor();
front_right_motor();
rear_left_motor();
rear_right_motor();
}


void front_right_motor() {
  if (Output_F_R > 1) {
    digitalWrite(F_R_drive_InA, 0);
    digitalWrite(F_R_drive_InB, 1);
    analogWrite(F_R_drive_PWM, abs(Output_F_R));
    
  }

  if (Output_F_R < -1) {
    digitalWrite(F_R_drive_InA, 1);
    digitalWrite(F_R_drive_InB, 0);
    analogWrite(F_R_drive_PWM, abs(Output_F_R));
    
  }

  if (Output_F_R == 0) {
    digitalWrite(F_R_drive_InA, 0);
    digitalWrite(F_R_drive_InB, 0);
    analogWrite(F_R_drive_PWM, abs(0));
    
  }
}

void front_left_motor() {
  if (Output_F_L > 1) {
    digitalWrite(F_L_drive_InA, 1);
    digitalWrite(F_L_drive_InB, 0);
    analogWrite(F_L_drive_PWM, abs(Output_F_L));
    
  }

  if (Output_F_L < -1) {
    digitalWrite(F_L_drive_InA, 0);
    digitalWrite(F_L_drive_InB, 1);
    analogWrite(F_L_drive_PWM, abs(Output_F_L));
    
  }

  if (Output_F_L == 0) {
    digitalWrite(F_L_drive_InA, 0);
    digitalWrite(F_L_drive_InB, 0);
    analogWrite(F_L_drive_PWM, abs(0));
    
  }
}

void rear_right_motor() {
  if (Output_R_R > 1) {
    digitalWrite(R_R_drive_InA, 1);
    digitalWrite(R_R_drive_InB, 0);
    analogWrite(R_R_drive_PWM, abs(Output_R_R));
    
  }

  if (Output_R_R < -1) {
    digitalWrite(R_R_drive_InA, 0);
    digitalWrite(R_R_drive_InB, 1);
    analogWrite(R_R_drive_PWM, abs(Output_R_R));
    
  }

  if (Output_R_R == 0) {
    digitalWrite(R_R_drive_InA, 0);
    digitalWrite(R_R_drive_InB, 0);
    analogWrite(R_R_drive_PWM, abs(0));
    
  }
}

void rear_left_motor() {
  if (Output_R_L > 1) {
    digitalWrite(R_L_drive_InA, 1);
    digitalWrite(R_L_drive_InB, 0);
    analogWrite(R_L_drive_PWM, abs(Output_R_L));
    
  }

  if (Output_R_L < -1) {
    digitalWrite(R_L_drive_InA, 0);
    digitalWrite(R_L_drive_InB, 1);
    analogWrite(R_L_drive_PWM, abs(Output_R_L));
    
  }

  if (Output_R_L == 0) {
    digitalWrite(R_L_drive_InA, 0);
    digitalWrite(R_L_drive_InB, 0);
    analogWrite(R_L_drive_PWM, abs(0));
    
  }
}