
#include "application.h"
// #include "OSCMessage.h"

#include "Particle.h"
#include "/Users/ninjacat/Documents/Particle/button/src/simple-OSC.h"

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

int testPin =D7;
void setup() {
  Serial.begin(9600);
   bool pressed = false;
  
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

void loop() {
buttonState = digitalRead(buttonPin);
  //RECEIVE
     int size = 0;
     OSCMessage inMessage;
        
       
    // Check if data has been received
      if (udp.parsePacket() > 0) {
        Serial.println("receiving message");
       toggle=!toggle;
        // Read first char of data received
        char c;

        // Ignore other chars
        while(udp.available()){
          
          c=udp.read();
          Serial.print(c);
        }
      }
    
    if(toggle){
 digitalWrite(testPin, HIGH);
    }else{
 digitalWrite(testPin, LOW);
    }

    
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {

      if(pressed == true)
            return;
        
        // The flag has not been set so set it now.
        
        pressed = true;
        
        // Publish the off event.
        Serial.println("Pressing button. Publishing from button script.");
        send();
        Serial.println("message sent");
        // Particle.publish("ledToggle", "on", 60, PUBLIC);


    // // turn LED on:
    // digitalWrite(LEDpin, HIGH);

    //     digitalWrite(testPin, HIGH);
    // Serial.println("HIGH");
  } else {
    // turn LED off:
     
         if(pressed == false)
            return;
        
        // The flag has not been set so set it now.
        
        pressed = false;
        
        // Publish the on event.
        Serial.println("NOT Pressing button. Publishing from button script.");
        // Particle.publish("ledToggle", "off", 60, PUBLIC);

    // digitalWrite(LEDpin, LOW);

    //     digitalWrite(testPin, LOW);
    // Serial.println("LOW");
  }

  delay(period);
}

void send(){
  IPAddress ipAddress(192,168,0,100);
  unsigned int localPort = 8888;

  OSCMessage outMessage("from sender Izzy");
  /* OSC DATA */ 
    // outMessage.addString("a");
    outMessage.addString("a");
  /* BANG TO MAX */
  outMessage.send(udp, ipAddress, localPort);
  Serial.println("in send method");
}