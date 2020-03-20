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

//option: short powerful pulse to LED : micro second with 1 AM (in the datasheet look for pulse per second, ormaximum surge per second)
//pour contourner les limitations de distance

//array of data: connected to commands coming from télécommande
//what you need to do: choose unique number: KEY: that will be sent to receiver and the receiver will only interpret if it receives that number
// this way it ignores any other ambiant infrared signals

//-----------------------//-----------------------//-----------------------//-----------------------#INCLUDES
#include <Particle.h>
#include <IRTransmitter/IRTransmitter.h>

//-----------------------//-----------------------//-----------------------//-----------------------PARTICLE
void setup();
void loop();
#line 20 "/Users/ninjacat/Documents/Particle/TakeHeed/IR_Transmitter/src/IR_Transmitter.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC); //avoid automatic connection to the cloud

//-----------------------//-----------------------//-----------------------//-----------------------WIFI
unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

//-----------------------//-----------------------//-----------------------//-----------------------IR
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
  WiFi.setHostname("HQRouter_Transmitter");
  Serial.println(WiFi.hostname());
  Serial.println(WiFi.localIP()); 
  Serial.begin(9600);
}
//-----------------------//-----------------------//-----------------------//-----------------------SETUP



//-----------------------//-----------------------//-----------------------//-----------------------LOOPING
void loop() {
 transmitter.Transmit(data, sizeof(data) / sizeof(data[0]));
}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING