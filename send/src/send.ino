/*
 * Project Take Heed _ Iteration 2
 * Description: template for sending with Particle communication
 * Author: Isabelle Charette & Nina Parenteau
 * Date: Winter 2020
 */

//-----------------------//-----------------------//-----------------------//-----------------------#INCLUDES
#include <Particle.h>
#include <simple-OSC.h>
#include "SparkCorePolledTimer.h"

//-----------------------//-----------------------//-----------------------//-----------------------PARTICLE
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

//-----------------------//-----------------------//-----------------------//-----------------------WIFI
unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

//-----------------------//-----------------------//-----------------------//-----------------------HEADERS
SparkCorePolledTimer updateTimer(500);  //Create a timer object and set it's timeout in milliseconds
void OnTimer(void);   //Prototype for timer callback method


//-----------------------//-----------------------//-----------------------//-----------------------SETUP
void setup() {
  //waiting for serial to correctly initialze and allocate memory
  //serial object
  while(!Serial);
  WiFi.connect();

  //wifi function
  while(!WiFi.ready());
  Serial.println("Setup");
  udp.begin(localPort);
  WiFi.setHostname("HQRouter_SEND");
  Serial.println(WiFi.hostname());
  Serial.println(WiFi.localIP()); 
  Serial.begin(9600);

  updateTimer.SetCallback(OnTimer);
  }

//-----------------------//-----------------------//-----------------------//-----------------------SETUP



//-----------------------//-----------------------//-----------------------//-----------------------LOOPING
void loop() {
 updateTimer.Update();
}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING

//-----------------------//-----------------------//-----------------------//-----------------------SEND
void send(){
  IPAddress ipAddress(192,168,0,101);
  unsigned int localPort = 8888; //necessary?

  String message = "test";
  OSCMessage outMessage(message);
  outMessage.send(udp, ipAddress, localPort);

  udp.stop(); //necessary?
  
  Serial.println("in send method");
}
//-----------------------//-----------------------//-----------------------//-----------------------SEND

//-----------------------//-----------------------//-----------------------//-----------------------ONTIMER
void OnTimer(void) {  
  send();
}
//-----------------------//-----------------------//-----------------------//-----------------------ONTIMER