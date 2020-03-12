/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/particle_template/src/particle_template.ino"
/*
 * Project Take Heed _ Iteration 2
 * Description: template for uploading code to the particles connected to local network
 * Author: Isabelle Charette & Nina Parenteau
 * Date: Winter 2020
 */

#include <Particle.h>

void setup();
void loop();
#line 10 "/Users/ninjacat/Documents/Particle/TakeHeed/particle_template/src/particle_template.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;


void setup() {
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
  }

/////---------------------------------------------------------------- SETUP



//-----------------------//-----------------------//-----------------------//-----------------------LOOPING
void loop() {

}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING