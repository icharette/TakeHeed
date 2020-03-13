/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/IR_Transmitter/src/IR_Transmitter.ino"
/*
 * Project: IR_Transmitter
 * Description: program to test IR_Transmitter
 * Author: Isabelle Charette 
 * Date: Winter 2020
 */

#include <Particle.h>
#include <IRTransmitter/IRTransmitter.h>

void setup();
void loop();
#line 11 "/Users/ninjacat/Documents/Particle/TakeHeed/IR_Transmitter/src/IR_Transmitter.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

//IR transmitter variables
#define IR_PIN D6
#define LED_PIN D7
// Raw data can be sniffed using an IR-receiver and e.g. https://github.com/z3t0/Arduino-IRremote/blob/master/examples/IRrecvDumpV2/IRrecvDumpV2.ino
unsigned int data[67] = {9000, 4450, 550, 550, 600, 500, 600, 550, 550, 1650, 600, 550, 550, 550, 600, 500, 600, 550,
                            600, 1600, 600, 1650, 600, 1650, 600, 500, 600, 1650, 600, 1600, 600, 1650, 600, 1650, 600,
                            500, 600, 1650, 600, 1650, 550, 550, 600, 1650, 550, 550, 600, 500, 600, 550, 550, 1650,
                            600, 550, 550, 550, 600, 1650, 550, 550, 600, 1650, 550, 1650, 650, 1600,
                            600};  // NEC 10EF6897

IRTransmitter transmitter(IR_PIN, LED_PIN);

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
 transmitter.Transmit(data, sizeof(data) / sizeof(data[0]));
}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING