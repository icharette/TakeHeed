/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/TakeHeed_ParticleSetup/src/TakeHeed_ParticleSetup.ino"
/*
AUTHOR: Isabelle Charette
PROJECT: Take Heed
DATE: November 2019

SOURCES:
*/

//including necessary libraries for neopixels, IMU sensor
#include <Particle.h>


//set up to connect to Router instead of cloud
void setup();
void loop();
#line 14 "/Users/ninjacat/Documents/Particle/TakeHeed/TakeHeed_ParticleSetup/src/TakeHeed_ParticleSetup.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);


unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;


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


}

void loop(){
Serial.print("hello");
  // IMU_sensor compasCalc;
  // compasCalc.iecompass(-imu.my, -imu.mx, imu.mz, imu.ax, imu.ay, imu.az);
}