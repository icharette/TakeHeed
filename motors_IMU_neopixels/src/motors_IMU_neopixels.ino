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
#include "neopixel.h"
#include "simple-OSC.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

/* ======================= headers =============================== */

uint32_t Wheel(byte WheelPos);
uint8_t red(uint32_t c);
uint8_t green(uint32_t c);
uint8_t blue(uint32_t c);
void colorWipe(uint32_t c, uint8_t wait);
void pulseWhite(uint8_t wait);
void healthyWave(uint8_t wait, int rainbowLoops, int whiteLoops);

//pins for motor on left shoulder
// int enableLeft = A2;
// int stepLeft = A1;
// int directionLeft = A0;

//pins for motor on right shoulder
int enableRight = A5;
int stepRight = A4;
int directionRight = A3;

//arrays to help set up the pins
int rightShoulderMotors[3];
int leftShoulderMotors[3];

//variables that set the stepping of the motors
int pace = 500;
int wait = 1000;
int stepperIndexCap = 2000;

//------NEOPIXEL
// IMPORTANT: Set pixel COUNT, PIN and TYPE
#define PIXEL_PIN D2
#define PIXEL_COUNT 80
#define PIXEL_TYPE SK6812RGBW

#define BRIGHTNESS 50 // 0 - 255
int troubleCount = 0;
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN);
uint32_t  colorArrSaved[PIXEL_COUNT];
bool pixels[PIXEL_COUNT];
int pixelPointer;
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
int speedLimit = 20;
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
    digitalWrite(D7, HIGH);
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
// LSM9DS1 --------- Photon
// SCL -------------- D1 (SCL)
// SDA -------------- D0 (SDA)
// VDD ------------- 3.3V
// GND ------------- GND

   //new driver motor motorTesting
#include "Stepper.h"
 
// change this to the number of steps on your motor
#define STEPS 300
 
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, A1, A2, A3, A4);
// Stepper stepperRgith(STEPS, D3, D4, D5, D6);
// setup() runs once, when the device is first turned on.
void setup() {
  pinMode(D7, OUTPUT);
  digitalWrite(D7, LOW);
  for(int i = 0; i < PIXEL_COUNT; i++){
    pixels[i] = true;
  }

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

    setupImu();
    
  
  updateTimer.SetCallback(OnTimer);

  // strip.setBrightness(BRIGHTNESS);
  // strip.begin();
  // strip.show();
    stepper.setSpeed(20);
  }

// void setValuesAccordingToState(char state){
//   switch(state){

//     //life
//     case 'L':
//       pace = 500;
//       wait = 1000;
//       stepperIndexCap = 2000;
//     break;

//     //bleaching
//     case 'B':
//       pace = 2000;
//       wait = 1000;
//       stepperIndexCap = 2000;
//     break;

//     //dead
//     case 'D':
//     //not moving
//       stepperIndexCap = 0;
//     break;

//     //symbiosis, coming back to life
//     case 'S':
//       pace = 2000;
//       wait = 1000;
//       stepperIndexCap = 2000;
//     break;
//   }
// }
// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // Serial.println("Forward");
  // stepper.step(STEPS);
  // Serial.println("Backward");

  // delay(100);
  // stepper.step(-STEPS);
  getMouvement();
updateTimer.Update();


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

//NEOPIXELS
//CORRECT CYCLE !!!!
  // healthyWave(10,10,1);
  // colorWipe(3000);
  // healthyWave(10,10,1);
}
//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXEL


// Set all pixels in the strip to a solid color, then wait (ms)
void colorAll(uint32_t c, uint8_t wait) {
  uint16_t i;

  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXEL

//-----------------------//-----------------------//-----------------------//-----------------------MOTORS


//-----------------------//-----------------------//-----------------------//-----------------------MOTORS
bool match = true;



void OnTimer(void) {  //Handler for the timer, will be called automatically
    int size = 0;
     OSCMessage inMessage;
        
       
  // Check if data has been received
      if ((size = udp.parsePacket()) > 0) {
        Serial.println("receiving message");
        // toggle=!toggle;
     
        char c;
        while(size--){
          Serial.println("---in while---");
          c=udp.read();
          Serial.println(c);
          inMessage.fill(c);
          
        }

        // Serial.print("alge: : ");
        // Serial.println(inMessage.getInt(0));
        //this works
  //  light(inMessage);
        if(inMessage.parse()){

Serial.println("PARSING");
          //this doesn't for some reason. 
          //is it reading/sending the right message?
          inMessage.route("still", STILL);
          inMessage.route("move", MOVE);
        }
        Serial.println();
      }
  // printMvmt();
  checkSpeed();
    //  fluxX = 0;
    //  fluxY = 0;
    //  fluxZ = 0;

}

//-----------------------//-----------------------//-----------------------//-----------------------IMU
boolean checkSpeed(){
  float total = fluxX + fluxY + fluxZ;
  Serial.print("Total movement: ");
  Serial.println(total);
  Serial.print("Speed limit : ");
  Serial.println(speedLimit);
  if((total) < speedLimit){
    //   fluxX = 0;
    //  fluxY = 0;
    //  fluxZ = 0;
    //  total = 0;
    return true;
  }else if(total >= speedLimit){
      fluxX = 0;
     fluxY = 0;
     fluxZ = 0;
     total = 0;
    return false;
  }

}

void STILL(OSCMessage &inMessag){
checkMatch(true);
}

void MOVE(OSCMessage &inMessag){
checkMatch(false);
}

void checkMatch(bool alge){
  if(alge && checkSpeed()){
    match =  true;
  }else{
    match = false;
  }

    Serial.print("IN CHECK MATCH :: ");
    Serial.println(match);
}
void getMouvement(){

    // for (int i = 0; i < 10; i++){
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
    // dX=imu.calcAccel(imu.ax);
    // dY=imu.calcAccel(imu.ay);
    // dZ=imu.calcAccel(imu.az);

    fluxX += abs(imu.calcAccel(imu.ax) - refX);
    fluxY += abs(imu.calcAccel(imu.ay) - refY);
    fluxZ += abs(imu.calcAccel(imu.az) - refZ);
    // 
    // printMvmt();
    // }


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

//-----------------------//-----------------------//-----------------------//-----------------------IMU


//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXELS

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3,0);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3,0);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0,0);
}

uint8_t red(uint32_t c) {
  return (c >> 8);
}
uint8_t green(uint32_t c) {
  return (c >> 16);
}
uint8_t blue(uint32_t c) {
  return (c);
}

void  healthyWave(uint8_t wait, int rainbowLoops, int whiteLoops) {
  bool toggle = true;
  float fadeMax = 100.0;
  int fadeVal = 0;
  uint32_t wheelVal;
  int redVal, greenVal, blueVal;

  for(int k = 0 ; k < rainbowLoops ; k ++) {
    // if(toggle){
    //   stepperLeft.step(STEPS);
    //   toggle = false;
    // }else{
    //   stepperLeft.step(-STEPS);
    //   toggle = true;
    // }
    
    // getMouvement();
  
     if(checkSpeed() == 0){
       
       for(int i = 0; i < strip.numPixels() ; i++){
         colorArrSaved[i] = strip.getPixelColor(i);
       }
       trouble();
     }
    for(int j=0; j<256; j++) { // 5 cycles of all colors on wheel
    getMouvement();
          
      for(int i=0; i< strip.numPixels(); i++) {
 
         wheelVal = Wheel(((i * 256 / strip.numPixels()) + j) & 255);

        redVal = red(wheelVal) * float(fadeVal/fadeMax);
        greenVal = green(wheelVal) * float(fadeVal/fadeMax);
        blueVal = blue(wheelVal) * float(fadeVal/fadeMax);


        strip.setPixelColor( i, strip.Color( redVal, 255, 0 ) );
      }

      // First loop, fade in!
      if(k == 0 && fadeVal < fadeMax-1) {
        fadeVal++;
      }
      // Last loop, fade out!
      else if(k == rainbowLoops - 1 && j > 255 - fadeMax ) {
        fadeVal--;
      }

      strip.show();
      delay(wait);
    }
  }

  delay(500);
}

void trouble(){
  
  bool complete = false;
  bool checkNum = true;
  int val = -1;

  int chunk = 0;
  

  int delayIn = 10;
  int delayOut = 0;

  
   wait = 10;
      delayIn = 3;
      delayOut = 3;

    if(troubleCount == 0){
      chunk = strip.numPixels()/4;
      // wait = 100;
      // delayIn = 3;
      // delayOut = 10;
      stepper.step(STEPS);
  }else if(troubleCount == 1){
    chunk = strip.numPixels()/3;
    // wait = 500;
    // delayIn = 6;
    //   delayOut = 8;
       stepper.step(-STEPS);
  }else if(troubleCount ==2){
      chunk = strip.numPixels()/2;
      // wait = 1000;
      // delayIn = 8;
      //  delayOut = 4;
        stepper.step(STEPS);
  }else if(troubleCount == 3){
      chunk = strip.numPixels();
      // wait = 2000;
      // delayIn = 10;
      // delayOut = 0;
       stepper.step(-STEPS);

  }

int randomNumList[chunk];
  while(!complete){

     //clean array
  for(int k = 0; k < chunk; k++){
    randomNumList[k] = -1;
  }

    Serial.print("TROUBLE COUNT :: ");
    Serial.println(troubleCount);

    for(uint16_t i=0; i<chunk; i++) {
      getMouvement();
       Serial.print("chunk :: ");
    Serial.println(chunk);
     while(checkNum){
        checkNum = false;
        //int nRandonNumber = rand()%((nMax+1)-nMin) + nMin;
        val = (rand() % ((strip.numPixels())));
        Serial.print("VAL:: ");
        Serial.println(val);
        for(int k = 0; k < chunk; k++){
          if(randomNumList[k] == val){
            checkNum = true;
          }
        }
      }
        checkNum = true;
      for(int k = 0; k <255 ; k++){
      
          // Serial.println((strip.getPixelColor(i)));
          strip.setPixelColor(val, strip.Color(k, 255, k));
          randomNumList[i] = val;
          delay(delayOut);
          pixels[val] = false;
          strip.show();
      }
    // delay(wait);

    Serial.print(i);
       Serial.print(" ::  ");
       Serial.println(val);
    }

if(troubleCount==3){
  delay(5000);
  troubleCount = 0;
}
    if(checkSpeed()==1){
      for(uint16_t i=0; i<chunk; i++) {
          for(int k = 255; k >=0 ; k--){
              strip.setPixelColor(randomNumList[i], strip.Color(k, 255, k));
              delay(10);
strip.show();
          }

//next while loop : https://arduino.stackexchange.com/questions/23174/how-to-get-neopixel-to-fade-colorwipe/23179
    // uint8_t curr_r, curr_g, curr_b;
    
    // curr_b = 0; 
    // curr_g = 255;
    // curr_r = 0;

    // uint8_t r = colorArrSaved[randomNumList[i]] & 0xFF; 
    // uint8_t g = (colorArrSaved[randomNumList[i]] >> 8) & 0xFF; 
    // uint8_t b = (colorArrSaved[randomNumList[i]] >> 16) & 0xFF; 
    //   while ((curr_r != r) || (curr_g != g) || (curr_b != b)){  // while the curr color is not yet the target color
    //     if (curr_r < r) curr_r++; else if (curr_r > r) curr_r--;  // increment or decrement the old color values
    //     if (curr_g < g) curr_g++; else if (curr_g > g) curr_g--;
    //     if (curr_b < b) curr_b++; else if (curr_b > b) curr_b--;
    //       // for(int k = 255; k >=0 ; k--){
    //           strip.setPixelColor(randomNumList[i], curr_r, curr_g, curr_b);
    //           delay(10);


    //   }
              pixels[val] = true;
              strip.show();
          }
        delay(delayIn);
      // }
    }
 
 
    for(int i = 0; i<PIXEL_COUNT; i++){
      if(pixels[i]==false){
        complete = false;
        break;
      }
    }

  complete= true;
  troubleCount++;
    

  }

//   if(complete){
// healthyWave(10,10,1);
//   }
}

void fadeIn(uint8_t wait, int red, int green, int blue){
int randomNumList[strip.numPixels()];

//clean array
for(int k = 0; k < strip.numPixels(); k++){
  randomNumList[k] = -1;
}

bool checkNum = false;
// bool full = false;
int val=0;
  Serial.println("lignthing");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
  
     while(checkNum){
        checkNum = false;
        //int nRandonNumber = rand()%((nMax+1)-nMin) + nMin;
        val = (rand() % ((strip.numPixels())));
        for(int k = 0; k < strip.numPixels(); k++){
          if(randomNumList[k] == val){
            checkNum = true;
          }
        }
        

      }
        checkNum = true;
       Serial.print(i);
       Serial.print(" ::  ");
       Serial.println(val);
    // int k = 255;
    for(int k = 255; k >=0 ; k--){
 strip.setPixelColor(val, strip.Color(k, 255, k));
        randomNumList[i] = val;
 delay(10);
 pixels[val] = true;
strip.show();
     
    }
     
    delay(wait);
  }

  // delay(2000);
 
}

void fadeOut(uint8_t wait, int red, int green, int blue){
  bool checkNum = false;
  int val=0;
 int randomNumList2[strip.numPixels()];
 //clean array
for(int k = 0; k < strip.numPixels(); k++){
  randomNumList2[k] = -1;
}
Serial.println("turning off");
   for(uint16_t i=0; i<strip.numPixels(); i++) {

     while(checkNum){
        checkNum = false;
        // val = (rand() % (strip.numPixels() - 0 + 1)+1);
        val = (rand() % ((strip.numPixels())));
            Serial.print("checknum");
        for(int k = 0; k < strip.numPixels(); k++){
          if(randomNumList2[k] == val){
            checkNum = true;
          }
        }
        

      }
        checkNum = true;
        Serial.print(i);
       Serial.print(" ::  ");
       Serial.println(val);
    
    
    // for(int j = 255; j >=0 ; j--){
      for(int j = 0; j <= 255 ; j++){
 strip.setPixelColor(val, strip.Color(j, 255, j));
        randomNumList2[i] = val;
 
    pixels[val] = false;
    strip.show();
     delay(10);
    }
 
    delay(wait);
  }

    delay(2000);

}
// Fill the dots one after the other with a color
void colorWipe(uint8_t wait) {
// fadeIn(wait, 255,128,0);
fadeOut(wait, 255,128,0);
fadeIn(wait, 255,128,0);
}
//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXELS