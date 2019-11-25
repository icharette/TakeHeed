/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/button/src/button.ino"
/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "application.h"
// #include "OSCMessage.h"

#include "Particle.h"
#include "/Users/ninjacat/Documents/Particle/TakeHeed/button/src/simple-OSC.h"
void send();
void light(OSCMessage &inMessage);
#line 10 "/Users/ninjacat/Documents/Particle/TakeHeed/button/src/button.ino"
#line 1 "/Users/ninjacat/Documents/Particle/button/src/button.ino"
/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.

  The circuit:
  - LED attached from pin 13 to ground
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Button
*/

bool pressed = false;
bool toggle = true;
int period = 100;


void setup();
void loop();

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);


unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int LEDpin = D2;
int buttonPin = D0;

int emitter = D1;
int sensorIR = A0;

int testPin =D7;
void setup() {
  Serial.begin(9600);
   bool pressed = false;
  
   pinMode(emitter, OUTPUT);
   pinMode(sensorIR, INPUT);
    // initialize the LED pin as an output:
  pinMode(LEDpin, OUTPUT);     
    pinMode(testPin, OUTPUT);                                            
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  

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

    
    // Set the initial state of subscriber
    // to off just in case.
    //  Particle.publish("elaEvent", "high", 60, PUBLIC); //PUBLIC OR PRIVATE OR DEVICE ID
    //  Particle.publish("ledToggle", "off", 60, PUBLIC);
}
void loop(){
  digitalWrite(D1, HIGH);
  float value = analogRead(sensorIR);
  Serial.print("VALUE: ");
  value-=1000;
  Serial.println(value);
}
// void loop() {
// digitalWrite(testPin, HIGH);
// buttonState = digitalRead(buttonPin);
//   //RECEIVE
//      int size = 0;
//      OSCMessage inMessage;
        
       
//     // Check if data has been received
//       if ((size = udp.parsePacket()) > 0) {
//         Serial.println("receiving message");
//         Serial.println("size: " + size);
//           toggle=!toggle;
     
//         char c;
//         while(size--){
//           Serial.println("IN WHILE");
//           c=udp.read();
//           Serial.print(c);
//           inMessage.fill(c);
          
//         }
//   //  àlight(inMessage);

//       Serial.println(inMessage.parse());
//         if(inMessage.parse()){
//           Serial.println("if parse is TRUE");
//           inMessage.route("/particle1", light);
//         }
//         //this prints 0.0
//          Serial.println(inMessage.getFloat(0));
//         Serial.println();
//       }


    
//   // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
//   if (buttonState == HIGH) {

//       if(pressed == true)
//             return;
        
//         // The flag has not been set so set it now.
        
//         pressed = true;
        
//         // Publish the off event.
//         Serial.println("Pressing button. Publishing from button script.");
//         send();
//         Serial.println("message sent");
//         // Particle.publish("ledToggle", "on", 60, PUBLIC);


//     // // turn LED on:
//     // digitalWrite(LEDpin, HIGH);

//     //     digitalWrite(testPin, HIGH);
//     // Serial.println("HIGH");
//   } else {
//     // turn LED off:
     
//          if(pressed == false)
//             return;
        
//         // The flag has not been set so set it now.
        
//         pressed = false;
        
//         // Publish the on event.
//         // Serial.println("NOT Pressing button. Publishing from button script.");
//         // Particle.publish("ledToggle", "off", 60, PUBLIC);

//     // digitalWrite(LEDpin, LOW);

//     //     digitalWrite(testPin, LOW);
//     // Serial.println("LOW");
//   }

//   delay(period);
// }
 
void send(){
  IPAddress ipAddress(192,168,0,100);
  unsigned int localPort = 8888;

///from 
  OSCMessage outMessage("/izzyParticle");
  outMessage.addString("test");
  outMessage.send(udp, ipAddress, localPort);
  Serial.println("in send method");
}


//why need OSCMessage &inMessage arguments?
void light(OSCMessage &inMessage){
  Serial.println("in light method");

    
      if(toggle){
        digitalWrite(testPin, HIGH);
      }else{
        digitalWrite(testPin, LOW);
      }
}