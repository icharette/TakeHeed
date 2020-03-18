/*
 * Project Take Heed _ Iteration 2
 * Description: template for receiving with Particle communication
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
  WiFi.setHostname("HQRouter_RECEIVE");
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

//-----------------------//-----------------------//-----------------------//-----------------------ONTIMER
void OnTimer(void) {  //Handler for the timer, will be called automatically
    int size = 0;
     OSCMessage inMessage;
      
      Serial.println("LISTENING---------------");
       
  // Check if data has been received
      if ((size = udp.parsePacket()) > 0) {
        Serial.println("receiving message");

        char c;
        //printing message to console
        while(size--){
          Serial.println("---in while---");
          c=udp.read();
          Serial.println(c);
          inMessage.fill(c);
          
        }
  //if there is somtheing to parse
        if(inMessage.parse()){

        Serial.println("PARSING");
          //trigger method according to message received
          inMessage.route("test", TEST);
          inMessage.route("still", STILL);
          inMessage.route("/move", MOVE);
        }
        Serial.println();
      }else{
        Serial.println("No message");
      }
}
//-----------------------//-----------------------//-----------------------//-----------------------SETUP

//-----------------------//-----------------------//-----------------------//-----------------------PARSED-METHODS
void TEST(OSCMessage &inMessag){
  Serial.println("Receiving message with object -test- and triggering TEST method.");

}
void STILL(OSCMessage &inMessag){
  Serial.println("STILL");

}

void MOVE(OSCMessage &inMessag){
  Serial.println("MOVE");

}
//-----------------------//-----------------------//-----------------------//-----------------------PARSED-METHODS
