/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/send/src/send.ino"
/*
 * Project Take Heed _ Iteration 2
 * Description: template for sending with Particle communication
 * Author: Isabelle Charette & Nina Parenteau
 * Date: Winter 2020
 */

#include <Particle.h>
#include <simple-OSC.h>
#include "SparkCorePolledTimer.h"

void setup();
void loop();
void send();
#line 12 "/Users/ninjacat/Documents/Particle/TakeHeed/send/src/send.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

SparkCorePolledTimer updateTimer(500);  //Create a timer object and set it's timeout in milliseconds
void OnTimer(void);   //Prototype for timer callback method

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

   updateTimer.SetCallback(OnTimer);
  }

/////---------------------------------------------------------------- SETUP



//-----------------------//-----------------------//-----------------------//-----------------------LOOPING
void loop() {
 updateTimer.Update();
}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING

void send(){
  //my computer IP address: 132.205.229.249
  IPAddress ipAddress(192,168,0,101);
  unsigned int localPort = 8888;
  unsigned int outPort = 7000;

  String message = "test";
  OSCMessage outMessage(message);
  outMessage.send(udp, ipAddress, localPort);

  udp.stop(); //necessary?
  
  Serial.println("in send method");
}

void OnTimer(void) {  
  send();
}