/*
 * Project motors
 * Description: stepper motor control for the nema8 stepper motors on each shoulder of wearable Take Heed
 * Author: Isabelle Charette
 * Date: Fall 2019
 */

#include "application.h"
#include "Particle.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);


unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

//pins for motor on left shoulder
int enableLeft = A2;
int stepLeft = A1;
int directionLeft = A0;

int rightShoulderMotors[3];
int leftShoulderMotors[3];

//pins for motor on right shoulder
int enableRight = A5;
int stepRight = A4;
int directionRight = A3;


// int stepperIndex; necessary?

void setupMotorLeft(){
  pinMode(enableLeft, OUTPUT); //Enable
  pinMode(stepLeft, OUTPUT); //Step
  pinMode(directionLeft, OUTPUT); //Direction

  digitalWrite(enableLeft,LOW);

  leftShoulderMotors[0]= directionLeft;
  leftShoulderMotors[1]= stepLeft;
  leftShoulderMotors[2]= enableLeft;
}

void setupMotorRight(){
  pinMode(enableRight, OUTPUT); //Enable
  pinMode(stepRight, OUTPUT); //Step
  pinMode(directionRight, OUTPUT); //Direction

  digitalWrite(enableRight,LOW);

  rightShoulderMotors[0]= directionRight;
  rightShoulderMotors[1]= stepRight;
  rightShoulderMotors[2]= enableRight;
}
// setup() runs once, when the device is first turned on.
void setup() {
   Serial.begin(9600);
   setupMotorLeft();
   setupMotorRight();
  

  //waiting for serial to correctly initialze and allocate memory
  //serial object
  while(!Serial);
  WiFi.connect();

  //wifi function
  while(!WiFi.ready());
  Serial.println("Setup");
  udp.begin(localPort);
  WiFi.setHostname("HQRouter_PUBLISH");
  Serial.println(WiFi.hostname());
  Serial.println(WiFi.localIP());

  }

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.

int pace = 500;
int wait = 1000;
int stepperIndexCap = 2000;

//spinStepperRightSolo(int motorPins[], int pace, int wait, int stepperIndexCap){
spinStepperRightSolo(rightShoulderMotors, pace, wait, stepperIndexCap);
spinStepperRightSolo(leftShoulderMotors, pace, wait, stepperIndexCap);
//spinStepperRightSolo(int motorPins[], int pace, int wait, int stepperIndexCap){
spinStepperLeftSolo(rightShoulderMotors, pace, wait, stepperIndexCap);
spinStepperLeftSolo(leftShoulderMotors, pace, wait, stepperIndexCap);
// spinStepperRightDuo(int motorPins[], int motorPins2[], int pace, int wait, int stepperIndexCap)
spinStepperRightDuo(rightShoulderMotors, leftShoulderMotors, pace, wait, stepperIndexCap);
// spinStepperLeftDuo(int motorPins[], int motorPins2[], int pace, int wait, int stepperIndexCap)
spinStepperLeftDuo(rightShoulderMotors, leftShoulderMotors, pace, wait, stepperIndexCap);

}

void spinStepperRightSolo(int motorPins[], int pace, int wait, int stepperIndexCap){
  Serial.println("STEP RIGHT SOLO");
  /*
  pin order in array:
  motorPins[0]= directionRight;
  motorPins[1]= stepRight;
  motorPins[2]= enableRight;

  */
  digitalWrite(motorPins[0],HIGH);

  for(int stepperIndex = 0; stepperIndex < stepperIndexCap; stepperIndex++)
  {
    digitalWrite(motorPins[1], HIGH);
    delayMicroseconds(pace);
    digitalWrite(motorPins[1],LOW);
    delayMicroseconds(pace);
  }
  delay(wait);
}

void spinStepperLeftSolo(int motorPins[], int pace, int wait, int stepperIndexCap){
  Serial.println("STEP LEFT SOLO");
  /*
  pin order in array:
  motorPins[0]= directionRight;
  motorPins[1]= stepRight;
  motorPins[2]= enableRight;

  */
  digitalWrite(motorPins[0],LOW);

  for(int stepperIndex = 0; stepperIndex < stepperIndexCap; stepperIndex++)
  {
    digitalWrite(motorPins[1], HIGH);
    delayMicroseconds(pace); //possibliy not necessary
    digitalWrite(motorPins[1],LOW);
    delayMicroseconds(pace); //how fast the motor turns because of time between successive step pulses
  }
  delay(wait);
}

//spin motors on both shoulders
void spinStepperRightDuo(int motorPins[], int motorPins2[], int pace, int wait, int stepperIndexCap){
  Serial.println("STEP RIGHT DUO");
  /*
  pin order in array:
  motorPins[0]= directionRight;
  motorPins[1]= stepRight;
  motorPins[2]= enableRight;

  */
  digitalWrite(motorPins[0],HIGH);
  digitalWrite(motorPins2[0],HIGH);

  for(int stepperIndex = 0; stepperIndex < stepperIndexCap; stepperIndex++)
  {
    digitalWrite(motorPins[1], HIGH);
    digitalWrite(motorPins2[1], HIGH);
    delayMicroseconds(pace);
    digitalWrite(motorPins[1],LOW);
    digitalWrite(motorPins[2],LOW);
    delayMicroseconds(pace);
  }
  delay(wait);
}

//spin motors on both shoulders
void spinStepperLeftDuo(int motorPins[], int motorPins2[], int pace, int wait, int stepperIndexCap){
  Serial.println("STEP LEFT DUO");
  /*
  pin order in array:
  motorPins[0]= directionRight;
  motorPins[1]= stepRight;
  motorPins[2]= enableRight;

  */
  digitalWrite(motorPins[0],LOW);
  digitalWrite(motorPins2[0],LOW);

  for(int stepperIndex = 0; stepperIndex < stepperIndexCap; stepperIndex++)
  {
    digitalWrite(motorPins[1], HIGH);
    digitalWrite(motorPins2[1], HIGH);
    delayMicroseconds(pace);
    digitalWrite(motorPins[1],LOW);
    digitalWrite(motorPins[2],LOW);
    delayMicroseconds(pace);
  }
  delay(wait);
}
