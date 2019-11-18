/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/TakeHeed_ParticleSetup/src/TakeHeed_ParticleSetup.ino"
#include "application.h"
#include "Particle.h"

#line 4 "/Users/ninjacat/Documents/Particle/TakeHeed/TakeHeed_ParticleSetup/src/TakeHeed_ParticleSetup.ino"
void setup();
void loop();

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);


unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

int Index;

int buttonIn = D0;
int buttonOut = D1;
int testPin =D7;

int enable = A5;
int step = A4;
int direction = A3;
void setup() {
  Serial.begin(9600);
   bool pressed = false;
  
    pinMode(buttonIn, INPUT);    
    pinMode(buttonOut, INPUT);                                            
  
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

     pinMode(enable, OUTPUT); //Enable
  pinMode(step, OUTPUT); //Step
  pinMode(direction, OUTPUT); //Direction

  digitalWrite(enable,LOW);
    // Set the initial state of subscriber
    // to off just in case.
    //  Particle.publish("elaEvent", "high", 60, PUBLIC); //PUBLIC OR PRIVATE OR DEVICE ID
    //  Particle.publish("ledToggle", "off", 60, PUBLIC);
}

void loop() 
{
  digitalWrite(direction,HIGH);
int time = 500;

  for(Index = 0; Index < 2000; Index++)
  {
    digitalWrite(testPin, HIGH);
    digitalWrite(step,HIGH);
    delayMicroseconds(time);
    digitalWrite(step,LOW);
    digitalWrite(testPin, LOW);
    delayMicroseconds(time);

    
  }
  delay(1000);

  digitalWrite(direction,LOW);

  for(Index = 0; Index < 2000; Index++)
  {
    digitalWrite(testPin, HIGH);
    digitalWrite(step,HIGH);
    delayMicroseconds(time);
    digitalWrite(step,LOW);
    digitalWrite(testPin, LOW);
    delayMicroseconds(time);
  }
  delay(1000);
}