#include <Arduino.h>
#include <TeensyStep.h>
#include <Servo.h>
#include <TeensyThreads.h>

// Define stepper motor connections and parameters
Stepper motor1(2, 3); // Step pin, Direction pin
Stepper motor2(4, 5);
Stepper motor3(6, 7);
Stepper motor4(8, 9);

// Define PWM motor connections
Servo pwmMotor1;
Servo pwmMotor2;
Servo pwmMotor3;
Servo pwmMotor4;

// Create stepper motor controllers
StepControl controller1;
StepControl controller2;
StepControl controller3;
StepControl controller4;

Threads::Mutex myMutex;

void TaskSerialReceive(void* arg);
void TaskSendPositionData(void* arg);
void handleSerialData(String data);

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate

  // Attach PWM motors to pins
  pwmMotor1.attach(10);
  pwmMotor2.attach(11);
  pwmMotor3.attach(12);
  pwmMotor4.attach(13);

  // Initialize stepper motors
  motor1.setMaxSpeed(10000);
  motor2.setMaxSpeed(10000);
  motor3.setMaxSpeed(10000);
  motor4.setMaxSpeed(10000);

  motor1.setAcceleration(1000);
  motor2.setAcceleration(1000);
  motor3.setAcceleration(1000);
  motor4.setAcceleration(1000);

  // Create threads for handling tasks
  threads.addThread(TaskSerialReceive, nullptr);
  threads.addThread(TaskSendPositionData, nullptr);
}

void loop() {
  // The loop is now managed by TeensyThreads tasks
}

void TaskSerialReceive(void* arg) {
  for (;;) {
    if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n');
      handleSerialData(data);
    }
    threads.delay(10);
  }
}

void TaskSendPositionData(void* arg) {
  for (;;) {
    myMutex.lock();
    String positionData = String(motor1.getPosition()) + " " + String(motor2.getPosition()) + " " + String(motor3.getPosition()) + " " + String(motor4.getPosition());
    Serial.println(positionData);
    myMutex.unlock();
    threads.delay(100); // Adjust delay as needed
  }
}

void handleSerialData(String data) {
  myMutex.lock();
  int positions[4];
  int pwmValues[4];

  sscanf(data.c_str(), "%d %d %d %d %d %d %d %d", &positions[0], &positions[1], &positions[2], &positions[3], &pwmValues[0], &pwmValues[1], &pwmValues[2], &pwmValues[3]);

  motor1.setTargetAbs(positions[0]);
  motor2.setTargetAbs(positions[1]);
  motor3.setTargetAbs(positions[2]);
  motor4.setTargetAbs(positions[3]);

  pwmMotor1.write(pwmValues[0]);
  pwmMotor2.write(pwmValues[1]);
  pwmMotor3.write(pwmValues[2]);
  pwmMotor4.write(pwmValues[3]);

  myMutex.unlock();
}