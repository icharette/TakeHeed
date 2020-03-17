/*
 * Project Take Heed _ Iteration 2
 * Description: This program tests each component of the circuit individually. See switch cases in loop()
 * Author: Isabelle Charette 
 * Date: Winter 2020
 * 
 * Sources:
 * 
 * https://github.com/sparkfun/SparkFun_LSM9DS1_Particle_Library //IMU SENSOR
 * https://github.com/msalaciak/senseNet/blob/master/REV1%20code.ino //WIFI SETUP + PARTICLE COMMUNICATION
 * http://www.ngdc.noaa.gov/geomag-web/#declination // CALCULATION OF RIGHT DECLINATION VALUE
 * https://arduino.stackexchange.com/questions/23174/how-to-get-neopixel-to-fade-colorwipe/23179 // LED STRIP
 * http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf // IMU SENSOR FOR COMPASS FUNCTION
 * https://learn.adafruit.com/adafruit-tb6612-h-bridge-dc-stepper-motor-driver-breakout/using-stepper-motors // nema8 stepper motors
 * Also from examples from following included libraries
 */

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

//pins for motor on right shoulder
int enableRight = A5;
int stepRight = A4;
int directionRight = A3;

//arrays to help set up the pins
int rightShoulderMotors[3];

//variables that set the stepping of the motors
int pace = 500;
int waitStep = 1000;
int stepperIndexCap = 2000;

//------NEOPIXEL
// IMPORTANT: Set pixel COUNT, PIN and TYPE
#define PIXEL_PIN D2
#define PIXEL_COUNT 24
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
boolean calibrated = true;

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

//COMPASS SOURCE COMPASS: //SOURCE : http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

// a declination to get a more accurate heading. 
#define DECLINATION -14.17181// Declination (degrees) montreal

SparkCorePolledTimer updateTimer(500);  //Create a timer object and set it's timeout in milliseconds
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
      imu.readAccel();

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


// LSM9DS1 --------- Photon
// SCL -------------- D1 (SCL)
// SDA -------------- D0 (SDA)
// VDD ------------- 3.3V
// GND ------------- GND
/////---------------------------------------------------------------- IMU

/////---------------------------------------------------------------- SETUP

   //new driver motor motorTesting
#include "Stepper.h"
 
#define STEPS 300
 
Stepper stepper(STEPS, A1, A2, A3, A4);

bool onlyMotor = true;

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

    if(!onlyMotor)setupImu();
    
  
  if(!onlyMotor)updateTimer.SetCallback(OnTimer);

  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show();
  // stepper.setSpeed(20);
  }

/////---------------------------------------------------------------- SETUP



//-----------------------//-----------------------//-----------------------//-----------------------LOOPING
void loop() {
// healthyWave(500,3,3);

// testCase(0); //motors
// testCase(1); //leds
theaterChase(strip.Color(255,0,0),1000);
// theaterChase(strip.Color(255,0,255),100);
//-->IMPORTANT: only set boolean onlyMotor to false when using next testcase
// testCase(2); //IMU
}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING

void testCase(int component){

  switch(component){
    case 0:
    //STEPPER motors--------------------------------------------------------------------------------------------------------------------------------------
    //PIN SETUP: Stepper stepper(STEPS, A1, A2, A3, A4);
    int steps = 3000;
    stepper.setSpeed(50);
    stepper.step(steps);
    delay(1000);
    stepper.setSpeed(80); 
    stepper.step(-steps);
    break;

    case 1:
    //LED STRIP--------------------------------------------------------------------------------------------------------------------------------------
    //examples from neopixels library
    // PIN SETUP: PIXEL_PIN D2
    // PIXEL_COUNT 20; can change at line 61
    // strip.Color(255,255,255);
    Serial.println("Red"); 
    colorWipe(50); // Red
    break;

    case 2:
    //IMU--------------------------------------------------------------------------------------------------------------------------------------
    // ***-->IMPORTANT: set onlyMotor boolean to false when uncommenting following functions
    // *** -->set back to TRUE when this is commented out. not needed.
    // **other wise you get a wierd error, the board flashes red and even if you flash the right
    // ***code afterwards, it still flashes red and won't work sometimes. If this happens, audit the environment, clean it and reset the particle into DFU mode (flashing yellow, then try flashing again)
    // *DFU mode: hold told *reset* and *setup*, only release *reset* and keep holding down *setup* until the led cycles to yellow light
    // *if D7 LED on particle board lights up; problem with IMU sensor connection
    // PIN setup:

    // 	LSM9DS1 --------- Photon
    // 	 SCL -------------- D1 (SCL)
    // 	 SDA -------------- D0 (SDA)
    // 	 VDD ------------- 3.3V
    // 	 GND ------------- GND
    getMouvement();
    printMvmt();
    break;

    default:
    break;
  }
}


//-----------------------//-----------------------//-----------------------//-----------------------COMMUNICATION


void OnTimer(void) {  //Handler for the timer, will be called automatically
    int size = 0;
     OSCMessage inMessage;
      
      Serial.println("LISTENING---------------");
       
  // Check if data has been received
      if ((size = udp.parsePacket()) > 0) {
        Serial.println("receiving message");

        char c;
        while(size--){
          Serial.println("---in while---");
          c=udp.read();
          Serial.println(c);
          inMessage.fill(c);
          
        }

        if(inMessage.parse()){

        Serial.println("PARSING");

          inMessage.route("still", STILL);
          inMessage.route("/move", MOVE);
        }
        Serial.println();
      }else{
        if(!checkSpeed()){
          trouble();
        }
      }
}

void STILL(OSCMessage &inMessag){
  Serial.println("STILL");
checkMatch(true);
}

void MOVE(OSCMessage &inMessag){
  Serial.println("MOVE");
checkMatch(false);
}

boolean match = true;

void checkMatch(bool alge){
  if(alge && checkSpeed()){
    match =  true;
  }else{
    match = false;
  }
    Serial.println(match);
}

//-----------------------//-----------------------//-----------------------//-----------------------COMMUNICATION


//-----------------------//-----------------------//-----------------------//-----------------------IMU
void getMouvement(){

    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }

    fluxX += abs(imu.calcAccel(imu.ax) - refX);
    fluxY += abs(imu.calcAccel(imu.ay) - refY);
    fluxZ += abs(imu.calcAccel(imu.az) - refZ);
}

void printMvmt(){
    Serial.print("fluxX : ");
    Serial.println(fluxX);
    Serial.print("fluxY : ");
    Serial.println(fluxY);
    Serial.print("fluxZ : ");
    Serial.println(fluxZ);
}

boolean checkSpeed(){
  float total = fluxX + fluxY + fluxZ;
  Serial.print("Total movement: ");
  Serial.println(total);
  Serial.print("Speed limit : ");
  Serial.println(speedLimit);
  if((total) < speedLimit){
      fluxX = 0;
     fluxY = 0;
     fluxZ = 0;
     total = 0;
    return true;
  }else if(total >= speedLimit){
      fluxX = 0;
     fluxY = 0;
     fluxZ = 0;
     total = 0;
    return false;
  }

}

//-----------------------//-----------------------//-----------------------//-----------------------IMU


//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXELS

//FROM EXAMPLES IN NEOPIXEL LIBRARY
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

//FROM EXAMPLES IN NEOPIXEL LIBRARY + MODIFICATIONS
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
    
  //   // getMouvement();
  // updateTimer.Update();
  //    if(!match){
       
  //      for(int i = 0; i < strip.numPixels() ; i++){
  //        colorArrSaved[i] = strip.getPixelColor(i);
  //      }
  //      trouble();
  //    }
    for(int j=0; j<256; j++) { // 5 cycles of all colors on wheel
    // getMouvement();
          int val =  strip.numPixels()/2;
          
      for(int i= strip.numPixels()/2; i>7; i--) {
        //for (; down > limit; down--)
 
         wheelVal = Wheel(((i * 256 / strip.numPixels()) + j) & 255);

        redVal = red(wheelVal) * float(fadeVal/fadeMax);
        greenVal = green(wheelVal) * float(fadeVal/fadeMax);
        blueVal = blue(wheelVal) * float(fadeVal/fadeMax);
        strip.clear();
        sectionSqueezeWave(7,i,val,strip.Color( 0, greenVal, blueVal ),strip.Color( redVal, 0, blueVal ));

        // strip.setPixelColor( i, strip.Color( 0, greenVal, blueVal ) );
        // strip.setPixelColor( val, strip.Color( 0, greenVal, blueVal ) );
        val++;
         strip.show();
      delay(wait);
      }

      // First loop, fade in!
      if(k == 0 && fadeVal < fadeMax-1) {
        fadeVal++;
      }
      // Last loop, fade out!
      else if(k == rainbowLoops - 1 && j > 255 - fadeMax ) {
        fadeVal--;
      }

     
    }
  }

  // delay(500);
}

//generates behavior state of coral in distress
void trouble(){
  
  bool complete = false;
  bool checkNum = true;
  int val = -1;

//how many LED to bleach in the LED strip
  int chunk = 0;
  

  int delayIn = 10;
  int delayOut = 0;

  int wait=10;

    if(troubleCount == 0){
      chunk = strip.numPixels()/4;
      wait = 50;
      delayIn = 5;
      delayOut = 5;
      stepper.setSpeed(50);
      stepper.step(STEPS);
  }else if(troubleCount == 1){
    chunk = strip.numPixels()/3;
         wait = 50;
      delayIn = 5;
      delayOut = 5;
      stepper.setSpeed(50);
       stepper.step(-STEPS);
  }else if(troubleCount ==2){
      chunk = strip.numPixels()/2;
           wait = 50;
      delayIn = 5;
      delayOut = 5;
      stepper.setSpeed(40);
        stepper.step(STEPS);
  }else if(troubleCount == 3){
      chunk = strip.numPixels();
           wait = 50;
      delayIn = 5;
      delayOut = 5;
      stepper.setSpeed(30);
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
          strip.setPixelColor(val, strip.Color(k, 255, k));
          randomNumList[i] = val;
          delay(delayOut);
          pixels[val] = false;
          strip.show();
      }
    delay(wait);

    Serial.print(i);
       Serial.print(" ::  ");
       Serial.println(val);
    }

if(troubleCount==3){
  delay(5000);
  troubleCount = 0; 
}

    if(checkSpeed()){
      stepper.setSpeed(50);
        stepper.step(STEPS);

      for(uint16_t i=0; i<chunk; i++) {
          for(int k = 255; k >=0 ; k--){
              strip.setPixelColor(randomNumList[i], strip.Color(k, 255, k));
              delay(10);
strip.show();
          }

//next while loop source: https://arduino.stackexchange.com/questions/23174/how-to-get-neopixel-to-fade-colorwipe/23179
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
void sectionSqueezeWave(int numLimit, int down, int up, uint32_t colorUP, uint32_t colorDOWN) {

  int valUP;
  int valDOWN;
  int numHalfPixels=strip.numPixels()/2;
   Serial.print("NUM_LIMIT--:");
  Serial.println(numLimit);
  for(int step = 0; step < numLimit ; step++){
    valUP=up+step;
    valDOWN=down-step;

  uint32_t colorOFF = strip.Color(0,0,0);
  Serial.print("UP--:");
  Serial.println(valUP);
  
  Serial.print("DOWN--:");
  Serial.println(valDOWN);
    // if(valUP==numHalfPixels){
    //   strip.setPixelColor(valUP, colorOFF); 
    // }else if(valDOWN==numHalfPixels){
    //   strip.setPixelColor(valDOWN, colorOFF); 
    // }else{

      strip.setPixelColor(valDOWN, colorDOWN); 
      strip.setPixelColor(valUP, colorUP);
    // }
   
  }
}
void sectionSqueeze(int numLimit, int down, int up) {

  int valUP;
  int valDOWN;
  int numHalfPixels=strip.numPixels()/2;
   Serial.print("NUM_LIMIT--:");
  Serial.println(numLimit);
   Serial.print("UP--:");
  Serial.println(up);
  
  Serial.print("DOWN--:");
  Serial.println(down);
  uint32_t colorBlue = strip.Color(0,0,255);
  uint32_t colorPink = strip.Color(255,0,255);
  uint32_t colorOFF = strip.Color(0,0,0);

  uint32_t colorStep;
  int colorStepRed;
  int colorStepGreen;
  int colorStepBlue;
int jump = 100;
  for(int step = 0; step < numLimit; step++){
    valUP=up+step;
    valDOWN=down-step;

  
 
    float pulseCounter = 0;
    int SCALE = 50;
    float pulseSpeed = 0.1;

    if(valUP<numHalfPixels){
      strip.setPixelColor(valUP, colorOFF); 
    }else if(valDOWN>numHalfPixels){
      strip.setPixelColor(valDOWN, colorOFF); 
    }else{
      // for(int i = 0; i < 255; i++){
        // colorBlue = strip.Color(0,0,i);
        // colorPink = strip.Color(i,0,i);

        if(step>numLimit/2){
          colorStep=colorBlue;
          colorStepRed=0;
          colorStepGreen=0;
          colorStepBlue=255;
        }else if(step>numLimit/4){
          // colorStep=colorPink;
          colorStepRed=0;
          colorStepGreen=255;
          colorStepBlue=255;
        }else{
          colorStep=colorPink;
          colorStepRed=255;
          colorStepGreen=0;
          colorStepBlue=255;
        }
        // delay(5);
       // strip.setPixelColor(valDOWN, colorStep); 
     //   strip.setPixelColor(valUP, colorStep);
     if(jump<255){
jump+=10;
     }else{
jump-=10;
     }
        strip.setColorDimmed(valUP, colorStepRed, colorStepGreen, colorStepBlue,jump );
        strip.setColorDimmed(valDOWN, colorStepRed, colorStepGreen, colorStepBlue, jump);
      // }

      //  delay(tan(pulseCounter)*SCALE);
      //       pulseCounter += pulseSpeed;
    }
   
  }
}

void colorFace(uint32_t color){
  for(int i = 0; i < 255; i++){
   
  }
}
void theaterChase(uint32_t color, int wait)
{
  
  int limit = 20;

  int numHalfPixels=strip.numPixels()/2;
  int numLimit = numHalfPixels-limit;
  int stretch = numHalfPixels;
  int up = numHalfPixels-limit;
  int down = numHalfPixels+limit;

  float pulseCounter = 0;
  int SCALE = 1000;
  float pulseSpeed = 0.05;

    for (; down > stretch; down--)
    {     
            strip.clear(); //   Set all pixels in RAM to 0 (off)
            sectionSqueeze(limit,down,up);
            /*
            strip.setPixelColor(down-1, strip.Color(0,0,255)); // Set pixel 'c' to value 'color'
            strip.setPixelColor(down-2, strip.Color(0,0,255));
            strip.setPixelColor(down-3, color);
            strip.setPixelColor(down, strip.Color(0,0,255));

            strip.setPixelColor(up, strip.Color(0,0,255));
            strip.setPixelColor(up+1, strip.Color(0,0,255));
            strip.setPixelColor(up+2, strip.Color(0,0,255));
            strip.setPixelColor(up+3, color);
            */


            up++;
            strip.show(); // Update strip with new contents
            delay(sin(pulseCounter)*SCALE);
            Serial.print("SIN-----");
            Serial.println(tan(pulseCounter)*SCALE);
            pulseCounter += pulseSpeed;
           // delay(wait*=0.8);  // Pause for a moment
        // }
    }
pulseCounter = 0;
    //wait = 1000;
      Serial.println("END FIRST LOOP");
      Serial.print("LIMIT--");
      Serial.println(limit);
        Serial.print("DOWN--");
      Serial.println(down);
    for (; down<numHalfPixels+limit; down++)
    { 
      Serial.println("SECOND LOOP");
            strip.clear(); //   Set all pixels in RAM to 0 (off)
            sectionSqueeze(limit,down,up);
            /*
            strip.setPixelColor(a-1, strip.Color(0,0,255)); // Set pixel 'c' to value 'color'
            strip.setPixelColor(a-2, color);
            strip.setPixelColor(a-3, color);
            strip.setPixelColor(a, strip.Color(0,0,255));

            strip.setPixelColor(j, strip.Color(0,0,255));
            strip.setPixelColor(j+1, strip.Color(0,0,255));
            strip.setPixelColor(j+2, color);
            strip.setPixelColor(j+3, color);
            */
            up--;
            strip.show(); // Update strip with new contents
            delay(sin(pulseCounter)*SCALE);
            pulseCounter += pulseSpeed;
           //delay(wait*=0.8); // Pause for a moment
    }

}
void colorWipe(uint8_t wait) {
  uint32_t c;
  uint8_t _wait = wait;
  _wait-=10;
  uint16_t j = strip.numPixels()/2;
  int increment=0;
  for(int round = 0; round < 3 ; round ++){
j = strip.numPixels()/2;
  
    for(uint16_t i=strip.numPixels()/2; i>0; i--) {
      if(round==0){
        c =strip.Color(255, 0, 0);
      }else if(round==1){
        c =strip.Color(0, 255, 0);
      }else if(round==2){
        c = strip.Color(255, 0, 255);
      }
      strip.setPixelColor(i, c);
      strip.show();
      strip.setPixelColor(j, c);
      strip.show();
      j++;

      delay(_wait);
    }
  }
}
//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXELS