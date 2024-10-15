#include <L298N.h>
#include "BluetoothSerial.h"  // Include the BluetoothSerial library for ESP32

// Pin definition for motor 1 (left motor)
#define Dir3 26  // IN1
#define Dir4 25  // IN2
#define PWM2 34  // ENA

// Pin definition for motor 2 (right motor)
#define Dir1 33  // IN3
#define Dir2 32  // IN4
#define PWM1 27  // ENB

// Pin definition for the IR sensors
#define LeftIR 14  // Left IR sensor
#define RightIR 12 // Right IR sensor

// Create two motor instances
L298N motor1(PWM1, Dir1, Dir2); //left
L298N motor2(PWM2, Dir3, Dir4); //right

// Variables to control speed and motor state
int motorSpeed2 = 255;    // Normal speed for left motor L
int motorSpeed1 = 255;    // Normal speed for right motor M 
int turnSpeed1 = 255;     // Speed for left motor during turns T  135
int turnSpeed2 = 255;     // Speed for right motor during turns U
bool motorsOn = true;
int iter = 0;
int DL=50;

// Variable to store the last turning direction
// -1 for left, 1 for right, 0 for straight
int lastTurnDirection = 0;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);


  // Initialize IR sensors as input
  pinMode(LeftIR, INPUT);
  pinMode(RightIR, INPUT);

  // Set initial speed for both motors
  motor1.setSpeed(motorSpeed1);
  motor2.setSpeed(motorSpeed2);
}

void goback(int lt){
  motor1.forward();
  motor2.forward();
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  delay(750);
  if (lt==1){
  motor1.backward();
  motor2.forward();
  }
  else if (lt==-1){
    motor2.backward();
    motor1.forward();
  }
  delay(200);
}

void loop() {
  // Check for Bluetooth commands
  // Read the IR sensor values only if motors are on
  if (motorsOn) {
    int leftIRValue = digitalRead(LeftIR);
    int rightIRValue = digitalRead(RightIR);

    Serial.print(leftIRValue);
    Serial. println();
    Serial.print(rightIRValue);
    Serial. println();
    // Invert the sensor values to match your setup
    leftIRValue = !leftIRValue;
    rightIRValue = !rightIRValue;
    Serial.print(leftIRValue);
    Serial. println();
    Serial.print(rightIRValue);
    Serial. println();
    Serial.print("******************");


    // Line-following logic based on IR sensor input
    if (leftIRValue == HIGH && rightIRValue == HIGH) {
      // Both sensors detect white (line is between the sensors)
      motor1.setSpeed(motorSpeed1);
      motor2.setSpeed(motorSpeed2);
      motor1.backward();
      motor2.backward();

    } 
    if (leftIRValue != HIGH && rightIRValue == HIGH) {
      // Left sensor detects black (line is on the left side)

      motor1.forward();  
      motor2.backward();       
      motor1.setSpeed(turnSpeed1); // Reduce left motor speed during the turn                   
      motor2.setSpeed(turnSpeed2);                
      if (lastTurnDirection != 1){
        iter= 0;
      }else{
        iter++;
      }
      lastTurnDirection = 1;       // Turning right
      delay(100);
    } 
    if (leftIRValue == HIGH && rightIRValue != HIGH) {
      
      motor2.forward();  
      motor1.backward();
      motor1.setSpeed(turnSpeed1);                
      motor2.setSpeed(turnSpeed2);
      if (lastTurnDirection != -1){
        iter= 0;
      }else{
        iter++;
      }  // Reduce right motor speed during the turn                      
      lastTurnDirection = -1;   
      delay(100);
      }
    if (leftIRValue != HIGH && rightIRValue != HIGH){
      goback(lastTurnDirection);

    }
    if (iter>100){
      goback(lastTurnDirection);
    }    // Turning left
    // } else if (leftIRValue == LOW && rightIRValue == LOW) {
    //   // Both sensors detect black (no line detected)
    //   if (lastTurnDirection == 1) {
    //     // Continue turning right if the last direction was right
    //     motor1.setSpeed(turnSpeed1);
    //     motor1.stop();
    //     motor2.setSpeed(turnSpeed2);
    //     motor2.backward();
    //   } else if (lastTurnDirection == -1) {
    //     // Continue turning left if the last direction was left
    //     motor1.setSpeed(turnSpeed1);
    //     motor1.backward();
    //     motor2.setSpeed(turnSpeed2);
    //     motor2.stop();
    //   }
    
  }
  delay(5);
}
