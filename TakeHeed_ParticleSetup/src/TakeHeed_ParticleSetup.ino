/*
AUTHOR: Isabelle Charette
PROJECT: Take Heed
DATE: November 2019

SOURCES:
*/

//including necessary libraries for neopixels, IMU sensor
#include <Particle.h>
#include "neopixel.h"
#include "SparkFunLSM9DS1/SparkFunLSM9DS1.h"
#include "math.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include "compass.h"

//set up to connect to Router instead of cloud
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);


unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;


//variables for IMU sensor
LSM9DS1 imu;
float refX, refY, refZ;
float origin ;
float originPitch, originRoll;
boolean calibrated = false;
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW


////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -14.17181// Declination (degrees) montreal
int testPin =D7;
#include "SparkCorePolledTimer.h"
SparkCorePolledTimer updateTimer(1000);  //Create a timer object and set it's timeout in milliseconds
void OnTimer(void);   //Prototype for timer callback method


void setup(){

  //setup to connect to router wireless network
  while(!Serial);
  WiFi.connect();

  //wifi function
  while(!WiFi.ready());
  Serial.println("Setup");
  udp.begin(localPort);
  WiFi.setHostname("HQRouter_PUBLISH");
  Serial.println(WiFi.hostname());
  Serial.println(WiFi.localIP());


  // settings for imu
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  //checking if imu is ready to communicate
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " 
                  "work for an out of the box LSM9DS1 " 
                  "Breakout, but may need to be modified " 
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void loop(){
  imu.readMag();
  imu.readAccel();

  // IMU_sensor compasCalc;
  // compasCalc.iecompass(-imu.my, -imu.mx, imu.mz, imu.ax, imu.ay, imu.az);
}