#include "NewPing.h"

// Motor pins
const int dcMotorLeft = 2;
const int dcMotorRight = 3;
const int dcMotorBrush = 8;  


// Ultrasonic sensor pins
const int TRIGGER_PIN1 = 5;   // Trigger pin for Sensor 1 (left)
const int ECHO_PIN1 = 4;      // Echo pin for Sensor 1 (left)
const int TRIGGER_PIN2 = 6;   // Trigger pin for Sensor 2 (right)
const int ECHO_PIN2 = 7;      // Echo pin for Sensor 2 (right)
const int MAX_DISTANCE = 20;  // Maximum distance to measure in cm
bool objDetected = false;
int object = 0;

const int esp32Pin = A1;

// Create NewPing instances for each sensor
NewPing sonarLeft(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

void dcMotorBruh();

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(dcMotorLeft, OUTPUT);
  pinMode(dcMotorRight, OUTPUT);
  pinMode(esp32Pin, INPUT_PULLUP);
  pinMode(dcMotorBrush, OUTPUT);
}

void loop() {
  // Measure distance for left and right sensors
  int distanceLeft = sonarLeft.ping_cm();
  int distanceRight = sonarRight.ping_cm();
  Serial.println(object);
  Serial.println(objDetected);
  int hatdog = digitalRead(esp32Pin);

 // if (object % 2 == 0) {
 //   object++;
 //   objDetected = true;
 // }else if (object >= 10){
 //   object = 0;
 //   objDetected = false;
 // }
 // else{
 //   object++;
 //   objDetected = false;
 // }

 if (hatdog == LOW)
 {
  digitalWrite(dcMotorLeft, HIGH);
  digitalWrite(dcMotorRight, HIGH);
  delay(300);
  digitalWrite(dcMotorLeft, LOW);
  digitalWrite(dcMotorRight, LOW);
  dcMotorBruh();
  delay(300);
 }

  // Print distances
  Serial.print("Left Distance: ");
  Serial.print(distanceLeft);
  Serial.println(" cm");
  Serial.print("Right Distance: ");
  Serial.print(distanceRight);
  Serial.println(" cm");

  bool isLeft = distanceLeft > 0 && distanceLeft < MAX_DISTANCE;
  bool isRight = distanceRight > 0 && distanceRight < MAX_DISTANCE;

  // Decision-making logic
  if (isLeft && distanceRight == 0) {
    // Obstacle detected on the left
    Serial.println("Obstacle detected on the LEFT!");
    digitalWrite(dcMotorRight, HIGH);
    delay(300);
    digitalWrite(dcMotorRight, LOW);
    delay(300);
  } else if (isRight && distanceLeft == 0) {
    // Obstacle detected on the right
    Serial.println("Obstacle detected on the RIGHT!");
    digitalWrite(dcMotorLeft, HIGH);
    delay(300);
    digitalWrite(dcMotorLeft, LOW);
    delay(300);
  } else if (!isRight && !isLeft && objDetected) {
    // No obstacle nearby
    Serial.println("No obstacles detected.");
    digitalWrite(dcMotorRight, HIGH);
    digitalWrite(dcMotorLeft, HIGH);
    //dcMotorBruh();
    delay(200);
    digitalWrite(dcMotorRight, LOW);
    digitalWrite(dcMotorLeft, LOW);
    digitalWrite(dcMotorBrush, LOW);
    delay(300);
  } else if (isRight && isLeft && !objDetected) {
    digitalWrite(dcMotorRight, HIGH);
    digitalWrite(dcMotorLeft, HIGH);
    delay(300);
    digitalWrite(dcMotorRight, LOW);
    digitalWrite(dcMotorLeft, LOW);
    delay(50);
  } else {
    digitalWrite(dcMotorRight, HIGH);
    digitalWrite(dcMotorLeft, HIGH);
    delay(500);
    digitalWrite(dcMotorRight, LOW);
    digitalWrite(dcMotorLeft, LOW);
    delay(50);
  }


  // Stop the motors
  digitalWrite(dcMotorLeft, LOW);
  digitalWrite(dcMotorRight, LOW);


  delay(500);
}

void dcMotorBruh(){
  digitalWrite(dcMotorBrush, HIGH);
  delay(1000);
  digitalWrite(dcMotorBrush, LOW);
  delay(500);
}