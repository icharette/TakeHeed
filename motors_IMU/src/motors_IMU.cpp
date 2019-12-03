/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/motors_IMU/src/motors_IMU.ino"
/*
 * Project motors
 * Description: stepper motor control for the nema8 stepper motors on each shoulder of wearable Take Heed
 * Author: Isabelle Charette
 * Date: Fall 2019
 */

// #include "application.h"
#include <Particle.h>

#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

#include "math.h"

#include "SparkCorePolledTimer.h"

void setupMotorLeft();
void setupMotorRight();
void setupMotor(int motorPinsArray[], int enable, int step, int direction);
void setupImu();
void calibrateSensor();
void setup();
void setValuesAccordingToState(char state);
void loop();
void motorTesting();
void spinStepperRightSolo(int motorPins[], int pace, int wait, int stepperIndexCap);
void spinStepperLeftSolo(int motorPins[], int pace, int wait, int stepperIndexCap);
void spinStepperRightDuo(int motorPins[], int motorPins2[], int pace, int wait, int stepperIndexCap);
void spinStepperLeftDuo(int motorPins[], int motorPins2[], int pace, int wait, int stepperIndexCap);
boolean checkSpeed();
void getMouvement();
void printMvmt();
#line 19 "/Users/ninjacat/Documents/Particle/TakeHeed/motors_IMU/src/motors_IMU.ino"
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

//pins for motor on right shoulder
int enableRight = A5;
int stepRight = A4;
int directionRight = A3;

int mosfetSwitch = D2;
//arrays to help set up the pins
int rightShoulderMotors[3];
int leftShoulderMotors[3];

//variables that set the stepping of the motors
int pace = 500;
int wait = 1000;
int stepperIndexCap = 2000;

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

void setupMotor(int motorPinsArray[], int enable, int step, int direction){
  pinMode(enable, OUTPUT); //Enable
  pinMode(step, OUTPUT); //Step
  pinMode(direction, OUTPUT); //Direction

  digitalWrite(enable, LOW);

  motorPinsArray[0]= direction;
  motorPinsArray[1]= step;
  motorPinsArray[2]= enable;
}


/////---------------------------------------------------------------- IMU


/***************************************************************** SOURCES
LSM9DS1_Basic_I2C.ino
SFE_LSM9DS1 Library Simple Example Code - I2C Interface
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 30, 2015
https://github.com/sparkfun/SparkFun_LSM9DS1_Particle_Library

*****************************************************************/

LSM9DS1 imu;

//sensor variables
float refX, refY, refZ;
float dX, dY, dZ;
float avMvmt;
int state;
float gainThreshold, lossThreshold;
 float fluxX = 0;
 float fluxY = 0;
 float fluxZ = 0;
int speedLimit = 200;
// float origin ;
// float originPitch, originRoll;
boolean calibrated = false;

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

// a declination to get a more accurate heading. 
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -14.17181// Declination (degrees) montreal

SparkCorePolledTimer updateTimer(1000);  //Create a timer object and set it's timeout in milliseconds
void OnTimer(void);   //Prototype for timer callback method

/* roll pitch and yaw angles computed by iecompass */
static int16_t iPhi, iThe, iPsi;
/* magnetic field readings corrected for hard iron effects and PCB orientation */
static int16_t iBfx, iBfy, iBfz;
/* hard iron estimate */
static int16_t iVx, iVy, iVz;
/* tilt-compensated e-Compass code */

const uint16_t MINDELTATRIG = 1; /* final step size for iTrig */
/* function to calculate ir = ix / sqrt(ix*ix+iy*iy) using binary division */

const uint16_t MINDELTADIV = 1; /* final step size for iDivide */

/* fifth order of polynomial approximation giving 0.05 deg max error */
const int16_t K1 = 5701;
const int16_t K2 = -1645;
const int16_t K3 = 446;

void setupImu(){
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  lossThreshold = 5;
  gainThreshold = 1;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }

     // imu.begin();
  Serial.println("calibration started");
  imu.calibrate(true);
  imu.calibrateMag(1);
  calibrateSensor();
  Serial.println("Calibration finished");
}

void calibrateSensor(){
  int count = 100;
  Serial.print("calibrating sensor. acc.");
  for(int i = 0; i < count; i++){
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
    refX += imu.calcAccel(imu.ax);
    refY += imu.calcAccel(imu.ay);
    refZ += imu.calcAccel(imu.az);
  }
  refX = refX / count;
  refY = refY / count;
  refZ = refZ / count; 
  Serial.println("done");
//  Serial.print("ref X: ");
//  Serial.print(refX);
//  Serial.print(" refY: ");
//  Serial.print(refY);
//  Serial.print(" refZ: ");
//  Serial.print(refZ);
//  Serial.println(" ");
}

/////---------------------------------------------------------------- IMU


// setup() runs once, when the device is first turned on.
void setup() {
  
  pinMode(mosfetSwitch, OUTPUT);
   //void setupMotor(int[] motorPinsArray, int enable, int step, int direction){
  setupMotor(rightShoulderMotors, enableRight, stepRight, directionRight);
  setupMotor(leftShoulderMotors, enableLeft, stepLeft, directionLeft);
  // setupMotorLeft();
  // setupMotorRight();
  

//   int enableLeft = A2;
// int stepLeft = A1;
// int directionLeft = A0;
 pinMode(6, OUTPUT); //Enable
  pinMode(5, OUTPUT); //Step
  pinMode(4, OUTPUT); //Direction

  digitalWrite(6,LOW);

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
   Serial.begin(9600);
    iVx = 0;
    iVy = 0;
    iVz = 0;

    // setupImu();
    
  // updateTimer.SetCallback(OnTimer);
  }

void setValuesAccordingToState(char state){
  switch(state){

    //life
    case 'L':
      pace = 500;
      wait = 1000;
      stepperIndexCap = 2000;
    break;

    //bleaching
    case 'B':
      pace = 2000;
      wait = 1000;
      stepperIndexCap = 2000;
    break;

    //dead
    case 'D':
    //not moving
      stepperIndexCap = 0;
    break;

    //symbiosis, coming back to life
    case 'S':
      pace = 2000;
      wait = 1000;
      stepperIndexCap = 2000;
    break;
  }
}

int Index;
// loop() runs over and over again, as quickly as it can execute.
void loop() {
// getMouvement();
// updateTimer.Update();


//if receiving message
//checking current state if having moved a lot during past few seconds
// checkSpeed();
//if both people are slow: regenerate
//if one is too fast: keep dying

 //should this be OnTimer too?
// printMvmt();
  // The core of your code will likely live here.
// analogWrite(mosfetSwitch, 255);
/*
L: life
B: bleaching
D: dead
S: symbiosis, coming back to life
*/
// setValuesAccordingToState('L');

// motorTesting();
 digitalWrite(4,HIGH);

  for(Index = 0; Index < 2000; Index++)
  {
    digitalWrite(5,HIGH);
    delayMicroseconds(500);
    digitalWrite(5,LOW);
    delayMicroseconds(500);
  }
  delay(1000);

  digitalWrite(4,LOW);

  for(Index = 0; Index < 2000; Index++)
  {
    digitalWrite(5,HIGH);
    delayMicroseconds(500);
    digitalWrite(5,LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}

void motorTesting(){
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


void OnTimer(void) {  //Handler for the timer, will be called automatically
 printMvmt();

     fluxX = 0;
     fluxY = 0;
     fluxZ = 0;

}

boolean checkSpeed(){
  if((fluxX + fluxY + fluxZ) <= speedLimit){
    return true;
  }else{
    return false;
  }
}
void getMouvement(){
//    reset values
    dX = 0;
    dY = 0;
    dZ = 0;
    avMvmt = 0;


    for (int i = 0; i < 100; i++){
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
    dX=imu.calcAccel(imu.ax);
    dY=imu.calcAccel(imu.ay);
    dZ=imu.calcAccel(imu.az);

    fluxX += abs(imu.calcAccel(imu.ax) - refX);
    fluxY += abs(imu.calcAccel(imu.ay) - refY);
    fluxZ += abs(imu.calcAccel(imu.az) - refZ);
    }


    // if (avMvmt < gainThreshold && pixelPointer <= NUM_LED){
    //  if (avMvmt < gainThreshold){
    // //   pixels[pixelPointer] = 1;
    //   pixelPointer++;
    // }
    // if (avMvmt > lossThreshold && pixelPointer >= 0){
    // //   pixels[pixelPointer] = 0;
    //   pixelPointer--;
    // }
    
}
void printMvmt(){
    // Serial.print("x: ");
    // Serial.print(dX);
    // Serial.print( " Y:");
    // Serial.print(dY);
    // Serial.print(" Z:");
    // Serial.print(dZ);
    // Serial.print(" av: ");
    // Serial.print(avMvmt);
    // Serial.println(" ");

        Serial.print("fluxX : ");
    Serial.println(fluxX);
    Serial.print("fluxY : ");
    Serial.println(fluxY);
    Serial.print("fluxZ : ");
    Serial.println(fluxZ);
}