/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/algue/src/algue.ino"
/*
 * Project: Take Heed
 * Description: communication of Algue movement
 * Author: Isabelle Charette
 * Date: Fall 2019
 */

#include <Particle.h>
#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include "simple-OSC.h"
#include "math.h"
#include "SparkCorePolledTimer.h"

void setupImu();
void calibrateSensor();
void setup();
void loop();
void send();
boolean checkSpeed();
void getMouvement();
void printMvmt();
#line 16 "/Users/ninjacat/Documents/Particle/TakeHeed/algue/src/algue.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

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
        //  digitalWrite(D7, LOW);
    }else{
        //  digitalWrite(D7, HIGH);
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
  pinMode(D7, OUTPUT);
  digitalWrite(D7, LOW);
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
  }


void loop() {

getMouvement();

updateTimer.Update();
send();
}

void send(){
  IPAddress ipAddress(192,168,0,100);
  unsigned int localPort = 8888;

///from 

  int speedInt = 0;
  if(checkSpeed()){
    speedInt = 1;
  }else{
    speedInt = 0;
  }
  String message = "";
if(speedInt){
message = "still";
}else{
  message = "/move";
}
    OSCMessage outMessage(message);
  outMessage.send(udp, ipAddress, localPort);
  Serial.println("in send method");
 
  Serial.println(speedInt);
}

void OnTimer(void) {  //Handler for the timer, will be called automatically
//  send();
   fluxX = 0;
     fluxY = 0;
     fluxZ = 0;
}

boolean checkSpeed(){
  if((fluxX + fluxY + fluxZ) <= speedLimit){
     int total= fluxX + fluxY + fluxZ;
     Serial.print("total: :");
  Serial.println(total);
    return true;
  }else{
     int total= fluxX + fluxY + fluxZ;
          Serial.print("total: :");
  Serial.println(total);
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
      // digitalWrite(D7, LOW);
    }else{
      // digitalWrite(D7, HIGH);
    }
    // dX=imu.calcAccel(imu.ax);
    // dY=imu.calcAccel(imu.ay);
    // dZ=imu.calcAccel(imu.az);

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