/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/IR_Receiver/src/IR_Receiver.ino"
/*
 * Project: IR_Receiver
 * Description: program to test IR_Receiver
 * Author: Isabelle Charette
 * Date: Winter 2020
 */

//-----------------------//-----------------------//-----------------------//-----------------------#INCLUDES
#include <Particle.h>
#include "IRremote.h"
#include "IRTransmitter.h"
//-----------------------//-----------------------//-----------------------//-----------------------PARTICLE
void setup();
void loop();
#line 13 "/Users/ninjacat/Documents/Particle/TakeHeed/IR_Receiver/src/IR_Receiver.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC); //avoid automatic connection to the cloud

//-----------------------//-----------------------//-----------------------//-----------------------WIFI
unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

//-----------------------//-----------------------//-----------------------//-----------------------IR
// receiver variables
int RECV_PIN = 6; //necessary to be pin 6 ?
IRrecv irrecv(RECV_PIN);
decode_results results;

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
  WiFi.setHostname("HQRouter_Receiver");
  Serial.println(WiFi.hostname());
  Serial.println(WiFi.localIP()); 
  Serial.begin(9600);

  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
}
//-----------------------//-----------------------//-----------------------//-----------------------SETUP

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING
void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX); 
    irrecv.resume(); // Receive the next value
  }
}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING