/*
 * Project Corals
 * Description: IMU sensor data from alge program sending to corals
 * Author: Isabelle Charette & Nina Parenteau
 * Date: Fall 2019
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
#include "simple-OSC.h"
#include "math.h"
#include "SparkCorePolledTimer.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

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
}

void setup() {
  pinMode(D7, OUTPUT);
  digitalWrite(D7, LOW);

  while(!Serial);
  WiFi.connect();

  //wifi function
  while(!WiFi.ready());
  Serial.println("Setup");
  udp.begin(8888);
  //udp.begin(localPort);
  WiFi.setHostname("Photon_ALE");
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
   //my computer IP address: 132.205.229.249
 //	192.168.0.101
  //IPAddress ipAddress(172,31,13,183);
  IPAddress ipAddress(192,168,0,102);
  unsigned int localPort = 8888;
 unsigned int outPort = 7000;
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
// String message = "testing";
    OSCMessage outMessage(message);

  outMessage.send(udp, ipAddress, localPort);

//  udp.stop();
 
  Serial.println("in send method");
 
  Serial.println(speedInt);
}

void OnTimer(void) {  
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
    for (int i = 0; i < 100; i++){
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
    fluxX += abs(imu.calcAccel(imu.ax) - refX);
    fluxY += abs(imu.calcAccel(imu.ay) - refY);
    fluxZ += abs(imu.calcAccel(imu.az) - refZ);
    }
}
void printMvmt(){
        Serial.print("fluxX : ");
    Serial.println(fluxX);
    Serial.print("fluxY : ");
    Serial.println(fluxY);
    Serial.print("fluxZ : ");
    Serial.println(fluxZ);
}