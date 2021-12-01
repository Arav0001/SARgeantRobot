// By: Arav Sonawane 2021
// Science Fair 2021-2022
// SARgeant Pathfinding Robot Project

// Library Inclusions
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <NewPing.h>
#include <SharpIR.h>
#include <Adafruit_MLX90614.h>

// Motor Initiaition
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

// Constant Declaring
#define TRIG_PIN A3
#define ECHO_PIN A2
#define PING_MAX 250

#define stoppingDist 12
#define MAXSPEED 125
#define NINETY 1675

#define switchPin 2
#define LED 4

// Sensor and Servo Initiaition
Servo servo;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
NewPing sonar(TRIG_PIN, ECHO_PIN, PING_MAX);
SharpIR sensor(SharpIR::GP2Y0A41SK0F, A0);

// Global Variable Declaring
long distance = 100;
int switchVal;
bool isHot;

// Setup function
void setup() {
  AFMS.begin();
  setMotorSpeed(MAXSPEED);
  
  if (!mlx.begin()) {
    while (1);
  };
  
  // Declaring pinmode of LED and switch
  pinMode(LED, OUTPUT);
  pinMode(switchPin, INPUT);
  delay(2000);
  switchVal = digitalRead(switchPin);
  
  // Positioning the servo to front
  servo.attach(10);
  servoAngle(90);
  delay(500);

  // Calibration
  distance = getDist();
  delay(100);
}

// Loop function
void loop() {
  // Initiaition of left/right distance variables
  long R = 0;
  long L = 0;

  // Checking for heat
  isHot = checkTemp(95.00);

  // Heat LED loop
  while (isHot) {
    stopMotion();
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    delay(300);
  }

  // Checking for obstacle
  if (distance <= stoppingDist) {
    // Motion when obstacle detected
    stopMotion();
    delay(100);
    
    backward();
    delay(7);
    stopMotion();
    
    // Getting right/left distances
    R = rightDistance();
    delay(200);
    L = leftDistance();
    delay(200);

    // Logic for rotations of robot
    if (R >= L) {
      if (R <= stoppingDist) {
        fullTurn();
      } else {
        turnRight(NINETY);
      }
    } else {
      if (L <= stoppingDist) {
        fullTurn();
      } else {
        turnLeft(NINETY);
      }
    }
  } else {
    forward();
  }
  // Re-checking for distance
  distance = getDist();
}

// Function for rotating servo
void servoAngle(int angle) {
  int pos = servo.read();
  
  if (pos > angle) {
    for (int newPos = pos; newPos >= angle; newPos -= 1) {
      servo.write(newPos);
      delay(15);
    }
  } else if (pos < angle) {
    for (int newPos = pos; newPos <= angle; newPos += 1) {
      servo.write(newPos);
      delay(15);
    }
  } else {
    servo.write(angle);
    delay(15);
  }
}

// Function for right distance
long rightDistance() {
    servoAngle(0);
    delay(500);
    
    long distance = getDist();
    
    servoAngle(90);
    delay(100);
    
    return distance;
}

// Function for left distance
long leftDistance() {
    servoAngle(180);
    delay(500);
    
    long distance = getDist();
    
    servoAngle(90);
    delay(100);
    
    return distance;
}

// Function to get distance
long getDist() {
  if (switchVal == 0) {
    long dist = sonar.ping_cm();

//    Uncomment following code if obstacle course if larger than 2.5 meters in any direction
//    if(dist == 0) {
//      dist = 250;
//    }
    return dist;
  } else if (switchVal == 1) {
    return (long) sensor.getDistance();
  } else {
    return -1;
  }
}

// Checking temperature function
bool checkTemp(long temp) {
  if (mlx.readObjectTempF() >= temp) {
    return true;
  }
  return false;
}

// To set motor speed
void setMotorSpeed(int speedNumber) {
  M1 -> setSpeed(speedNumber);
  M2 -> setSpeed(speedNumber);
  M3 -> setSpeed(speedNumber);
  M4 -> setSpeed(speedNumber);
}

// To stop motion of motors
void stopMotion() {
  M1 -> run(RELEASE);
  M2 -> run(RELEASE);
  M3 -> run(RELEASE);
  M4 -> run(RELEASE);
}

// Go forward with time
void forward() {
  M1 -> run(FORWARD);
  M2 -> run(FORWARD);
  M3 -> run(FORWARD);
  M4 -> run(FORWARD);
  
  for (int i = 0; i < MAXSPEED; i++) {
    setMotorSpeed(i);
    delay(5);
  }
}

// Go backward with time
void backward() {
  M1 -> run(BACKWARD);
  M2 -> run(BACKWARD);
  M3 -> run(BACKWARD);
  M4 -> run(BACKWARD);
  
  for (int i = 0; i < MAXSPEED; i++) {
    setMotorSpeed(i);
    delay(5);
  }
}

// Turn right function with time
void turnRight(int rotationTime) {
  M1 -> run(BACKWARD);
  M2 -> run(BACKWARD);
  M3 -> run(FORWARD);
  M4 -> run(FORWARD);
  
  delay(rotationTime);
  stopMotion();
}

// Turn left function with time
void turnLeft(int rotationTime) {
  M1 -> run(FORWARD);
  M2 -> run(FORWARD);
  M3 -> run(BACKWARD);
  M4 -> run(BACKWARD);
  
  delay(rotationTime);
  stopMotion();
}

// One-eighty degree rotation
void fullTurn() {
  turnRight(NINETY * 2);
}