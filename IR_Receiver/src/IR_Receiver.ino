/*
 * Project: IR_Receiver
 * Description: program to test IR_Receiver
 * Author: Isabelle Charette
 * Date: Winter 2020
 */

#include <Particle.h>
#include "IRremote.h"
#include "IRTransmitter.h"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

// receiver variables
int RECV_PIN = 6;

IRrecv irrecv(RECV_PIN);

decode_results results;

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

  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  }

/////---------------------------------------------------------------- SETUP



//-----------------------//-----------------------//-----------------------//-----------------------LOOPING
void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
  }
}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING