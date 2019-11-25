/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/IMU/src/IMU.ino"
#include <Particle.h>
#include <math.h>
// #include "/Users/ninjacat/Documents/Particle/TakeHeed/LSM9DS1_Breakout-master/Libraries/Particle/firmware/SparkFunLSM9DS1.h"

//  #include "/Users/ninjacat/Documents/Particle/TakeHeed/Particle-NeoPixel-master/src/neopixel.h"
// #include "libfixmath.h"


void setup();
void loop();
static void iecompass(int16_t iBpx, int16_t iBpy, int16_t iBpz, int16_t iGpx, int16_t iGpy, int16_t iGpz);
static int16_t iTrig(int16_t ix, int16_t iy);
static int16_t iHundredAtan2Deg(int16_t iy, int16_t ix);
static int16_t iHundredAtanDeg(int16_t iy, int16_t ix);
static int16_t iDivide(int16_t iy, int16_t ix);
#line 9 "/Users/ninjacat/Documents/Particle/TakeHeed/IMU/src/IMU.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);


unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;



/*****************************************************************
LSM9DS1_Basic_I2C.ino
SFE_LSM9DS1 Library Simple Example Code - I2C Interface
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 30, 2015
https://github.com/sparkfun/SparkFun_LSM9DS1_Particle_Library

The LSM9DS1 is a versatile 9DOF sensor. It has a built-in
accelerometer, gyroscope, and magnetometer. Very cool! Plus it
functions over either SPI or I2C.

This Photon sketch is a demo of the simple side of the
SFE_LSM9DS1 library. It'll demo the following:
* How to create a LSM9DS1 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS1 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and 
  the gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, 
  magnetic field strength using the calcAccel(), calcGyro() 
  and calcMag() functions.
* How to use the data from the LSM9DS1 to calculate 
  orientation and heading.

Hardware setup: This library supports communicating with the
LSM9DS1 over either I2C or SPI. This example demonstrates how
to use I2C. 

If you have the Photon IMU shield, no extra wiring is required.
If you're using a breakout, the pin-out is as follows:
	LSM9DS1 --------- Photon
	 SCL -------------- D1 (SCL)
	 SDA -------------- D0 (SDA)
	 VDD ------------- 3.3V
	 GND ------------- GND
(CSG, CSXM, SDOG, and SDOXM should all be pulled high. 
Jumpers on the breakout board will do this for you.)

Development environment specifics:
	IDE: Particle Build
	Hardware Platform: Particle Photon
	                   SparkFun Photon IMU Shield

This code is released under the MIT license.

Distributed as-is; no warranty is given.
*****************************************************************/
#include "SparkFunLSM9DS1/SparkFunLSM9DS1.h"
#include "math.h"

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;
float refX, refY, refZ;
    float origin ;
    float originPitch, originRoll;
    boolean calibrated = false;
///////////////////////
// Example I2C Setup //
///////////////////////
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

#include "application.h"
#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include "math.h"

// LSM9DS1 imu;

#define LSM9DS1_M    0x1E 
#define LSM9DS1_AG    0x6B 

//this code is derived from http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf

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

void setup() 
{
  
//  Serial1.begin(115200);  
//   updateTimer.SetCallback(OnTimer);
//   pinMode(testPin, OUTPUT);
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
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  // if (!imu.begin())
  // {
  //   Serial.println("Failed to communicate with LSM9DS1.");
  //   Serial.println("Double-check wiring.");
  //   Serial.println("Default settings in this sketch will " \
  //                 "work for an out of the box LSM9DS1 " \
  //                 "Breakout, but may need to be modified " \
  //                 "if the board jumpers are.");
  //   while (1)
  //     ;
  // }


  // imu.settings.device.commInterface = IMU_MODE_I2C;
  //   imu.settings.device.mAddress = LSM9DS1_M;
  //   imu.settings.device.agAddress = LSM9DS1_AG;
    
    iVx = 0;
    iVy = 0;
    iVz = 0;
    
    Serial.begin(115200);
    Serial.println("eCompass Test App");
    
    imu.begin();
  Serial.println("calibration started");
  imu.calibrate(true);
  imu.calibrateMag(1);
  
  Serial.println("Calibration fininshed");
}


// void loop()
// {

// updateTimer.Update();
//    imu.readGyro();
//    imu.readAccel();
//    imu.readMag();
//   printGyro();  // Print "G: gx, gy, gz"
//   printAccel(); // Print "A: ax, ay, az"
//   printMag();   // Print "M: mx, my, mz"
  
//   // Print the heading and orientation for fun!
//   // Call print attitude. The LSM9DS1's magnetometer x and y
//   // axes are opposite to the accelerometer, so my and mx are
//   // substituted for each other.
//   printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
//   Serial.println();
  
//   delay(PRINT_SPEED);
// }

// void calibrate(float _heading){
//     for(int i = 0; i < 100; i++){
//     origin += _heading;
//     }
//   origin = origin / 100;
//   Serial.println("==================HEADING CALIBRATED");
// }

// void calibratePitch(float _pitch){
//     for(int i = 0; i < 100; i++){
//     originPitch += _pitch;
//     }
//   originPitch = originPitch / 100;
//   Serial.println("==================HEADING CALIBRATED");
// }

// boolean toggle = true;
// void OnTimer(void) {  //Handler for the timer, will be called automatically
//   Serial1.println("Hello");
//   if(toggle){
//     digitalWrite(D7,HIGH);
//     toggle = false;
//   }else{
//     digitalWrite(D7,LOW);
//     toggle = true;
//   }
  
// }

// void calibrateRoll(float _roll){
//     for(int i = 0; i < 100; i++){
//     originRoll += _roll;
//     }
//   originRoll = originRoll / 100;
//   Serial.println("==================HEADING CALIBRATED");
// }
//  void calibrateSensor(){
//   Serial.print("calibrating sensor...");
//   for(int i = 0; i < 10; i++){
//     if ( imu.accelAvailable() )
//     {
//       imu.readAccel();
//     }
//     refX += imu.calcAccel(imu.ax);
//     refY += imu.calcAccel(imu.ay);
//     refZ += imu.calcAccel(imu.az);
//   }
//   refX = refX / 10;
//   refY = refY / 10;
//   refZ = refZ / 10; 

//   Serial.println("done");
//  Serial.print("ref X: ");
//  Serial.print(refX);
//  Serial.print(" refY: ");
//  Serial.print(refY);
//  Serial.print(" refZ: ");
//  Serial.print(refZ);
//  Serial.println(" ");
// }
// void printGyro()
// {
//   // To read from the gyroscope, you must first call the
//   // readGyro() function. When this exits, it'll update the
//   // gx, gy, and gz variables with the most current data.
//   imu.readGyro();
  
//   // Now we can use the gx, gy, and gz variables as we please.
//   // Either print them as raw ADC values, or calculated in DPS.
//   Serial.print("G: ");
// #ifdef PRINT_CALCULATED
//   // If you want to print calculated values, you can use the
//   // calcGyro helper function to convert a raw ADC value to
//   // DPS. Give the function the value that you want to convert.
//   Serial.print(imu.calcGyro(imu.gx), 2);
//   Serial.print(", ");
//   Serial.print(imu.calcGyro(imu.gy), 2);
//   Serial.print(", ");
//   Serial.print(imu.calcGyro(imu.gz), 2);
//   Serial.println(" deg/s");
// #elif defined PRINT_RAW
//   Serial.print(imu.gx);
//   Serial.print(", ");
//   Serial.print(imu.gy);
//   Serial.print(", ");
//   Serial.println(imu.gz);
// #endif
// }

// void printAccel()
// {
//   // To read from the accelerometer, you must first call the
//   // readAccel() function. When this exits, it'll update the
//   // ax, ay, and az variables with the most current data.
//   imu.readAccel();
  
//   // Now we can use the ax, ay, and az variables as we please.
//   // Either print them as raw ADC values, or calculated in g's.
//   Serial.print("A: ");
// #ifdef PRINT_CALCULATED
//   // If you want to print calculated values, you can use the
//   // calcAccel helper function to convert a raw ADC value to
//   // g's. Give the function the value that you want to convert.
//   Serial.print(imu.calcAccel(imu.ax), 2);
//   Serial.print(", ");
//   Serial.print(imu.calcAccel(imu.ay), 2);
//   Serial.print(", ");
//   Serial.print(imu.calcAccel(imu.az), 2);
//   Serial.println(" g");
// #elif defined PRINT_RAW 
//   Serial.print(imu.ax);
//   Serial.print(", ");
//   Serial.print(imu.ay);
//   Serial.print(", ");
//   Serial.println(imu.az);
// #endif

// }

// void printMag()
// {
//   // To read from the magnetometer, you must first call the
//   // readMag() function. When this exits, it'll update the
//   // mx, my, and mz variables with the most current data.
//   imu.readMag();
  
//   // Now we can use the mx, my, and mz variables as we please.
//   // Either print them as raw ADC values, or calculated in Gauss.
//   Serial.print("M: ");
// #ifdef PRINT_CALCULATED
//   // If you want to print calculated values, you can use the
//   // calcMag helper function to convert a raw ADC value to
//   // Gauss. Give the function the value that you want to convert.
//   Serial.print(imu.calcMag(imu.mx), 2);
//   Serial.print(", ");
//   Serial.print(imu.calcMag(imu.my), 2);
//   Serial.print(", ");
//   Serial.print(imu.calcMag(imu.mz), 2);
//   Serial.println(" gauss");
// #elif defined PRINT_RAW
//   Serial.print(imu.mx);
//   Serial.print(", ");
//   Serial.print(imu.my);
//   Serial.print(", ");
//   Serial.println(imu.mz);
// #endif
// }

// // Calculate pitch, roll, and heading.
// // Pitch/roll calculations take from this app note:
// // http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// // Heading calculations taken from this app note:
// // http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
// void printAttitude(
// float ax, float ay, float az, float mx, float my, float mz)
// {
//   float roll = atan2(ay, az);
//   float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
//   float heading;

// heading = atan2(mx, mz) * 180.0 / M_PI;

//     if(heading < 0.0)                         // Convert -180...+180 to 0...360
//       heading = (360.0 + heading);


//        heading -= DECLINATION * M_PI / 180;
//   // if (my == 0)
//   //   heading = (mx < 0) ? 180.0 : 0;
//   // else
//   //   heading = atan2(mx, my);
    
//   // heading -= DECLINATION * M_PI / 180;
  
//   // if (heading > M_PI) heading -= (2 * M_PI);
//   // else if (heading < -M_PI) heading += (2 * M_PI);
//   // else if (heading < 0) heading += 2 * M_PI;
  
//   // Convert everything from radians to degrees:
//   // heading *= 180.0 / M_PI;

//   pitch *= 180.0 / M_PI;
//   roll  *= 180.0 / M_PI;

//   if(!calibrated){
//     calibratePitch(pitch);
//     calibrateRoll(roll);
//     // calibrate(heading);
//     calibrated = true;
//   }
//   // heading-=origin;
//   pitch -= originPitch;
//   roll -= originRoll;
//   Serial.print("Pitch, Roll: ");
//   Serial.print(pitch, 2);
//   Serial.print(", ");
//   Serial.println(roll, 2);
//   Serial.println("Heading: "); Serial.println(heading, 2);
//   Serial.print("origin , pitchH, rollH: ");
//   Serial.print(origin);
//   Serial.print(", ");
//     Serial.print(originPitch);
//     Serial.print(", ");
//       Serial.println(originRoll);
// }




////////////COMPASS 


//SOURCE : http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf

void loop() {
    imu.readMag();
    imu.readAccel();
    
    iecompass(-imu.my, -imu.mx, imu.mz, imu.ax, imu.ay, imu.az);
    //iPhi, iThe, iPsi
    Serial.print("Pitch = ");
    Serial.println(iPhi / 100.0f);
    Serial.print("Roll  = ");
    Serial.println(iThe / 100.0f);
    float heading = iPsi / 100.0f;
    if(heading < 0){
      heading = heading + 360;
    }
    float offset = 20;

    Serial.print("Yaw   = ");
    Serial.println(heading);    
    Serial.println("");
    Serial.print("Accepted range: : ");
    float acceptedRange = heading + offset;
    Serial.print(acceptedRange);
    Serial.print(" , ");
    acceptedRange = heading - offset;
    Serial.println(acceptedRange);
    delay(1000);

}

static void iecompass(int16_t iBpx, int16_t iBpy, int16_t iBpz, int16_t iGpx, int16_t iGpy, int16_t iGpz)
{
    /* stack variables */
    /* iBpx, iBpy, iBpz: the three components of the magnetometer sensor */
    /* iGpx, iGpy, iGpz: the three components of the accelerometer sensor */
    /* local variables */
    int16_t iSin, iCos; /* sine and cosine */
    /* subtract the hard iron offset */
    iBpx -= iVx; /* see Eq 16 */
    iBpy -= iVy; /* see Eq 16 */
    iBpz -= iVz; /* see Eq 16 */
    /* calculate current roll angle Phi */
    iPhi = iHundredAtan2Deg(iGpy, iGpz);/* Eq 13 */
    /* calculate sin and cosine of roll angle Phi */
    iSin = iTrig(iGpy, iGpz); /* Eq 13: sin = opposite / hypotenuse */
    iCos = iTrig(iGpz, iGpy); /* Eq 13: cos = adjacent / hypotenuse */
    /* de-rotate by roll angle Phi */
    iBfy = (int16_t)((iBpy * iCos - iBpz * iSin) >> 15);/* Eq 19 y component */
    iBpz = (int16_t)((iBpy * iSin + iBpz * iCos) >> 15);/* Bpy*sin(Phi)+Bpz*cos(Phi)*/
    iGpz = (int16_t)((iGpy * iSin + iGpz * iCos) >> 15);/* Eq 15 denominator */
    /* calculate current pitch angle Theta */
    iThe = iHundredAtan2Deg((int16_t)-iGpx, iGpz);/* Eq 15 */
    /* restrict pitch angle to range -90 to 90 degrees */
    if (iThe > 9000) iThe = (int16_t) (18000 - iThe);
    if (iThe < -9000) iThe = (int16_t) (-18000 - iThe);
    /* calculate sin and cosine of pitch angle Theta */
    iSin = (int16_t)-iTrig(iGpx, iGpz); /* Eq 15: sin = opposite / hypotenuse */
    iCos = iTrig(iGpz, iGpx); /* Eq 15: cos = adjacent / hypotenuse */
    /* correct cosine if pitch not in range -90 to 90 degrees */
    if (iCos < 0) iCos = (int16_t)-iCos;
    /* de-rotate by pitch angle Theta */
    iBfx = (int16_t)((iBpx * iCos + iBpz * iSin) >> 15); /* Eq 19: x component */
    iBfz = (int16_t)((-iBpx * iSin + iBpz * iCos) >> 15);/* Eq 19: z component */
    /* calculate current yaw = e-compass angle Psi */
    iPsi = iHundredAtan2Deg((int16_t)-iBfy, iBfx); /* Eq 22 */
}

/*
int32_t tmpAngle; // temporary angle*100 deg: range -36000 to 36000 
static int16_t iLPPsi; // low pass filtered angle*100 deg: range -18000 to 18000 
static uint16_t ANGLE_LPF; // low pass filter: set to 32768 / N for N samples averaging 
// implement a modulo arithmetic exponential low pass filter on the yaw angle 
// compute the change in angle modulo 360 degrees 
tmpAngle = (int32_t)iPsi - (int32_t)iLPPsi;
if (tmpAngle > 18000) tmpAngle -= 36000;
if (tmpAngle < -18000) tmpAngle += 36000;
// calculate the new low pass filtered angle 
tmpAngle = (int32_t)iLPPsi + ((ANGLE_LPF * tmpAngle) >> 15);
// check that the angle remains in -180 to 180 deg bounds 
if (tmpAngle > 18000) tmpAngle -= 36000;
if (tmpAngle < -18000) tmpAngle += 36000;
// store the correctly bounded low pass filtered angle 
iLPPsi = (int16_t)tmpAngle;

//For the pitch angle θ, which is restricted to the range -90° to 90°, the final bounds check should be changed to:
if (tmpAngle > 9000) tmpAngle = (int16_t) (18000 - tmpAngle);
if (tmpAngle < -9000) tmpAngle = (int16_t) (-18000 - tmpAngle);


*/


static int16_t iTrig(int16_t ix, int16_t iy)
{
    uint32_t itmp; /* scratch */
    uint32_t ixsq; /* ix * ix */
    int16_t isignx; /* storage for sign of x. algorithm assumes x >= 0 then corrects later */
    uint32_t ihypsq; /* (ix * ix) + (iy * iy) */
    int16_t ir; /* result = ix / sqrt(ix*ix+iy*iy) range -1, 1 returned as signed Int16 */
    int16_t idelta; /* delta on candidate result dividing each stage by factor of 2 */
    /* stack variables */
    /* ix, iy: signed 16 bit integers representing sensor reading in range -32768 to 32767 */
    /* function returns signed Int16 as signed fraction (ie +32767=0.99997, -32768=-1.0000) */
    /* algorithm solves for ir*ir*(ix*ix+iy*iy)=ix*ix */
    /* correct for pathological case: ix==iy==0 */
    if ((ix == 0) && (iy == 0)) ix = iy = 1;
    /* check for -32768 which is not handled correctly */
    if (ix == -32768) ix = -32767;
    if (iy == -32768) iy = -32767;
    /* store the sign for later use. algorithm assumes x is positive for convenience */
    isignx = 1;
    if (ix < 0)
    {
        ix = (int16_t)-ix;
        isignx = -1;
    }
    /* for convenience in the boosting set iy to be positive as well as ix */
    iy = (int16_t)abs(iy);
    /* to reduce quantization effects, boost ix and iy but keep below maximum signed 16 bit */
    while ((ix < 16384) && (iy < 16384))
    {
        ix = (int16_t)(ix + ix);
        iy = (int16_t)(iy + iy);
    }
    /* calculate ix*ix and the hypotenuse squared */
    ixsq = (uint32_t)(ix * ix); /* ixsq=ix*ix: 0 to 32767^2 = 1073676289 */
    ihypsq = (uint32_t)(ixsq + iy * iy); /* ihypsq=(ix*ix+iy*iy) 0 to 2*32767*32767=2147352578 */
    /* set result r to zero and binary search step to 16384 = 0.5 */
    ir = 0;
    idelta = 16384; /* set as 2^14 = 0.5 */
    /* loop over binary sub-division algorithm */
    do
    {
    /* generate new candidate solution for ir and test if we are too high or too low */
    /* itmp=(ir+delta)^2, range 0 to 32767*32767 = 2^30 = 1073676289 */
        itmp = (uint32_t)((ir + idelta) * (ir + idelta));
        /* itmp=(ir+delta)^2*(ix*ix+iy*iy), range 0 to 2^31 = 2147221516 */
        itmp = (itmp >> 15) * (ihypsq >> 15);
        if (itmp <= ixsq) ir += idelta;
        idelta = (int16_t)(idelta >> 1); /* divide by 2 using right shift one bit */
    } while (idelta >= MINDELTATRIG); /* last loop is performed for idelta=MINDELTATRIG */
    /* correct the sign before returning */
    return (int16_t)(ir * isignx);
}


/* calculates 100*atan2(iy/ix)=100*atan2(iy,ix) in deg for ix, iy in range -32768 to 32767 */
static int16_t iHundredAtan2Deg(int16_t iy, int16_t ix)
{
    int16_t iResult; /* angle in degrees times 100 */
    /* check for -32768 which is not handled correctly */
    if (ix == -32768) ix = -32767;
    if (iy == -32768) iy = -32767;
    /* check for quadrants */
    if ((ix >= 0) && (iy >= 0)) /* range 0 to 90 degrees */
        iResult = iHundredAtanDeg(iy, ix);
    else if ((ix <= 0) && (iy >= 0)) /* range 90 to 180 degrees */
        iResult = (int16_t)(18000 - (int16_t)iHundredAtanDeg(iy, (int16_t)-ix));
    else if ((ix <= 0) && (iy <= 0)) /* range -180 to -90 degrees */
        iResult = (int16_t)((int16_t)-18000 + iHundredAtanDeg((int16_t)-iy, (int16_t)-ix));
    else /* ix >=0 and iy <= 0 giving range -90 to 0 degrees */
        iResult = (int16_t)(-iHundredAtanDeg((int16_t)-iy, ix));
    return (iResult);
}



/* calculates 100*atan(iy/ix) range 0 to 9000 for all ix, iy positive in range 0 to 32767 */
static int16_t iHundredAtanDeg(int16_t iy, int16_t ix)
{
    int32_t iAngle; /* angle in degrees times 100 */
    int16_t iRatio; /* ratio of iy / ix or vice versa */
    int32_t iTmp; /* temporary variable */
    /* check for pathological cases */
    if ((ix == 0) && (iy == 0)) return (0);
    if ((ix == 0) && (iy != 0)) return (9000);
    /* check for non-pathological cases */
    if (iy <= ix)
    iRatio = iDivide(iy, ix); /* return a fraction in range 0. to 32767 = 0. to 1. */
    else
    iRatio = iDivide(ix, iy); /* return a fraction in range 0. to 32767 = 0. to 1. */
    /* first, third and fifth order polynomial approximation */
    iAngle = (int32_t) K1 * (int32_t) iRatio;
    iTmp = ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5);
    iAngle += (iTmp >> 15) * (int32_t) K2;
    iTmp = (iTmp >> 20) * ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5);
    iAngle += (iTmp >> 15) * (int32_t) K3;
    iAngle = iAngle >> 15;
    /* check if above 45 degrees */
    if (iy > ix) iAngle = (int16_t)(9000 - iAngle);
    /* for tidiness, limit result to range 0 to 9000 equals 0.0 to 90.0 degrees */
    if (iAngle < 0) iAngle = 0;
    if (iAngle > 9000) iAngle = 9000;
    return ((int16_t) iAngle);
}



/* function to calculate ir = iy / ix with iy <= ix, and ix, iy both > 0 */
static int16_t iDivide(int16_t iy, int16_t ix)
{
    int16_t itmp; /* scratch */
    int16_t ir; /* result = iy / ix range 0., 1. returned in range 0 to 32767 */
    int16_t idelta; /* delta on candidate result dividing each stage by factor of 2 */
    /* set result r to zero and binary search step to 16384 = 0.5 */
    ir = 0;
    idelta = 16384; /* set as 2^14 = 0.5 */
    /* to reduce quantization effects, boost ix and iy to the maximum signed 16 bit value */
    while ((ix < 16384) && (iy < 16384))
    {
        ix = (int16_t)(ix + ix);
        iy = (int16_t)(iy + iy);
    }
    /* loop over binary sub-division algorithm solving for ir*ix = iy */
    do
    {
    /* generate new candidate solution for ir and test if we are too high or too low */
        itmp = (int16_t)(ir + idelta); /* itmp=ir+delta, the candidate solution */
        itmp = (int16_t)((itmp * ix) >> 15);
        if (itmp <= iy) ir += idelta;
            idelta = (int16_t)(idelta >> 1); /* divide by 2 using right shift one bit */
    } while (idelta >= MINDELTADIV); /* last loop is performed for idelta=MINDELTADIV */
    return (ir);
}
