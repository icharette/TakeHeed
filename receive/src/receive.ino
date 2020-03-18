/*
 * Project Take Heed _ Iteration 2
 * Description: template for receiving with Particle communication
 * Author: Isabelle Charette & Nina Parenteau
 * Date: Winter 2020
 */

#include <Particle.h>
#include <simple-OSC.h>
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

void OnTimer(void) {  //Handler for the timer, will be called automatically
    int size = 0;
     OSCMessage inMessage;
      
      Serial.println("LISTENING---------------");
       
  // Check if data has been received
      if ((size = udp.parsePacket()) > 0) {
        Serial.println("receiving message");

        char c;
        while(size--){
          Serial.println("---in while---");
          c=udp.read();
          Serial.println(c);
          inMessage.fill(c);
          
        }

        if(inMessage.parse()){

        Serial.println("PARSING");

          inMessage.route("still", STILL);
          inMessage.route("/move", MOVE);
        }
        Serial.println();
      }else{
        if(!checkSpeed()){
          trouble();
        }
      }
}

void STILL(OSCMessage &inMessag){
  Serial.println("STILL");
checkMatch(true);
}

void MOVE(OSCMessage &inMessag){
  Serial.println("MOVE");
checkMatch(false);
}

boolean match = true;

void checkMatch(bool alge){
  if(alge && checkSpeed()){
    match =  true;
  }else{
    match = false;
  }
    Serial.println(match);
}