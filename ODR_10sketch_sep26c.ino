#include "AFMotor.h"
#include <Servo.h>

#define echopin A4 // Echo pin
#define trigpin A5 // Trigger pin

Servo myservo;

const int MOTOR_1 = 1; 
const int MOTOR_2 = 2; 
const int MOTOR_3 = 3; 
const int MOTOR_4 = 4; 

AF_DCMotor motor1(MOTOR_1, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor2(MOTOR_2, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor3(MOTOR_3, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor4(MOTOR_4, MOTOR12_64KHZ); // create motor object, 64KHz pwm

//===============================================================================
//  Initialization
//===============================================================================

int distances[8];  // Array to store distances at each angle
long distance;
const int stopDistance = 30; // Stop distance in cm
int motorSpeed = 180; // Initial motor speed
int servoAngles[] = {0, 30, 60, 90, 120, 150, 180};  // Angles for scanning, including 0° and 180°

void setup() {
  Serial.begin(9600);           // Initialize serial port
  Serial.println("Start");

  myservo.attach(10);
  myservo.write(90); // Servo facing forward

  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);
  
  // Set the motor speed to 30% higher
  motorSpeed = motorSpeed * 1.3; // Increase motor speed by 30%
  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(motorSpeed);
  motor3.setSpeed(motorSpeed);
  motor4.setSpeed(motorSpeed);
}

//===============================================================================
//  Main
//=============================================================================== 
void loop() {
  // Measure the distance in front of the car
  long distance_F = getAverageDistance();
  
  // Print the distance
  Serial.print("Front Distance = ");
  Serial.println(distance_F);

  // If distance in front is greater than the stop threshold, move forward
  if (distance_F > stopDistance) {
    Serial.println("Moving Forward");
    moveForward();
  } else {
    // If object is detected within stop distance, stop the car and scan for available paths
    Serial.println("Object Detected - Stopping");
    stopCar();
    scanForClearPath();
  }

  delay(100); // Short delay for smooth operation
}

//===============================================================================
//  Distance Measurement Function (with Averaging)
//===============================================================================
long getDistance() {
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);

  long duration = pulseIn(echopin, HIGH);
  long distance = duration / 29 / 2;  // Convert time to distance in cm
  return distance;
}

// Function to get average distance for better accuracy
long getAverageDistance() {
  long totalDistance = 0;
  const int numReadings = 3; // Number of readings to average

  for (int i = 0; i < numReadings; i++) {
    totalDistance += getDistance();
    delay(10); // Small delay between readings
  }

  return totalDistance / numReadings;
}

//===============================================================================
//  Motor Control Functions
//===============================================================================
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void stopCar() {
  motor1.run(RELEASE);         // Stop motors
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

//===============================================================================
//  Servo Scanning for Clear Path (Scanning Diagonally and at Extremes)
//===============================================================================
void scanForClearPath() {
  // Loop through each servo angle for scanning
  for (int i = 0; i < 7; i++) {
    myservo.write(servoAngles[i]); // Set servo to angle
    delay(120); // Delay to allow servo to move and stabilize
    distances[i] = getAverageDistance(); // Store the measured distance
    Serial.print("Distance at ");
    Serial.print(servoAngles[i]);
    Serial.print("° = ");
    Serial.println(distances[i]);
  }

  // Return servo to center (90°)
  myservo.write(90);
  delay(120);

  // Determine the direction with the longest clear distance
  int maxDistanceIndex = 0;
  for (int i = 1; i < 7; i++) {
    if (distances[i] > distances[maxDistanceIndex]) {
      maxDistanceIndex = i;
    }
  }

  int maxDistance = distances[maxDistanceIndex];
  int chosenAngle = servoAngles[maxDistanceIndex];

  // Decide direction based on the longest clear distance
  if (maxDistance > stopDistance) {
    if (chosenAngle == 90) {
      Serial.println("Clear path ahead, moving forward");
      moveForward();
    } else if (chosenAngle < 90) {
      Serial.println("Clear diagonal path to the right, turning right");
      turnRight(chosenAngle);
    } else {
      Serial.println("Clear diagonal path to the left, turning left");
      turnLeft(chosenAngle);
    }
  } else {
    Serial.println("No clear path, moving backward to rescan.");
    moveBackward(); // Move backward if no clear path found
    stopCar();
    scanForClearPath(); // Rescan environment
  }
}

//===============================================================================
//  Turning Functions with Diagonal Angle
//===============================================================================
void turnRight(int angle) {
  // Adjust turn based on angle
  int turnTime = map(angle, 0, 90, 100, 400); // Adjust turn time based on angle (faster for smaller angles)
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(turnTime); // Adjust turning time
  moveForward(); // Continue moving forward after turning
}

void turnLeft(int angle) {
  // Adjust turn based on angle
  int turnTime = map(angle, 90, 180, 400, 600); // Adjust turn time based on angle
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(turnTime); // Adjust turning time
  moveForward(); // Continue moving forward after turning
}

//===============================================================================
//  New Function: Move Backward
//===============================================================================
void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(1000); // Move backward for 1 second
}
