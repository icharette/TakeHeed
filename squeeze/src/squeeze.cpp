/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/squeeze/src/squeeze.ino"
/*
 * Project Take Heed _ Iteration 2
 * Description: testing LED activity
 * Author: Isabelle Charette & Nina Parenteau
 * Date: Winter 2020
 */

//-----------------------//-----------------------//-----------------------//-----------------------#INCLUDES
#include <Particle.h>
//NEOPIXELS
#include "neopixel.h"
#include "math.h"
//-----------------------//-----------------------//-----------------------//-----------------------PARTICLE
void setup();
void loop();
uint32_t Wheel(byte WheelPos);
uint8_t red(uint32_t c);
uint8_t green(uint32_t c);
uint8_t blue(uint32_t c);
void  healthyWave(uint8_t wait, int rainbowLoops, int whiteLoops);
void sectionSqueezeWave(int numLimit, int down, int up, uint32_t colorUP, uint32_t colorDOWN);
void sectionSqueeze(int numLimit, int down, int up);
void colorFace(uint32_t color);
void theaterChase(uint32_t color, int wait);
#line 14 "/Users/ninjacat/Documents/Particle/TakeHeed/squeeze/src/squeeze.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC); //avoid automatic connection to the cloud

//-----------------------//-----------------------//-----------------------//-----------------------WIFI
unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXELS
#define PIXEL_PIN D2
#define PIXEL_COUNT 24
#define PIXEL_TYPE SK6812RGBW
#define BRIGHTNESS 50 // 0 - 255
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN);
uint32_t  colorArrSaved[PIXEL_COUNT];
bool pixels[PIXEL_COUNT];
int pixelPointer;
//-----------------------//-----------------------//-----------------------//-----------------------HEADERS
//NEOPIXELS
void colorWipe(uint8_t wait);

//-----------------------//-----------------------//-----------------------//-----------------------SETUP
void setup() {
  //-----------------------//-----------------------//-----------------------//SETUP-WIFI
  //waiting for serial to correctly initialze and allocate memory
  //serial object
  while(!Serial);
  WiFi.connect();

  //wifi function
  while(!WiFi.ready());
  Serial.println("Setup");
  udp.begin(localPort);
  WiFi.setHostname("HQRouter_NEOPIXELS");
  Serial.println(WiFi.hostname());
  Serial.println(WiFi.localIP()); 
  Serial.begin(9600);
  //-----------------------//-----------------------//-----------------------//SETUP-LED_STRIP
   for(int i = 0; i < PIXEL_COUNT; i++){
    pixels[i] = true;
  }
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show();
}

//-----------------------//-----------------------//-----------------------//-----------------------SETUP



//-----------------------//-----------------------//-----------------------//-----------------------LOOPING
void loop() {
//--->LED testing with activity/personality
healthyWave(500,3,3);
// theaterChase(strip.Color(255,0,0),1000); // testing LED pulsing pattern, I haven't tried it on the full LED strip yet
// theaterChase(strip.Color(255,0,255),100);
}

//-----------------------//-----------------------//-----------------------//-----------------------LOOPING

//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXELS

//FROM EXAMPLES IN NEOPIXEL LIBRARY
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3,0);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3,0);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0,0);
}

uint8_t red(uint32_t c) {
  return (c >> 8);
}
uint8_t green(uint32_t c) {
  return (c >> 16);
}
uint8_t blue(uint32_t c) {
  return (c);
}

//FROM EXAMPLES IN NEOPIXEL LIBRARY + MODIFICATIONS
void  healthyWave(uint8_t wait, int rainbowLoops, int whiteLoops) {
  bool toggle = true;
  float fadeMax = 100.0;
  int fadeVal = 0;
  uint32_t wheelVal;
  int redVal, greenVal, blueVal;

  for(int k = 0 ; k < rainbowLoops ; k ++) {
    // if(toggle){
    //   stepperLeft.step(STEPS);
    //   toggle = false;
    // }else{
    //   stepperLeft.step(-STEPS);
    //   toggle = true;
    // }
    
  //   // getMouvement();
  // updateTimer.Update();
  //    if(!match){
       
  //      for(int i = 0; i < strip.numPixels() ; i++){
  //        colorArrSaved[i] = strip.getPixelColor(i);
  //      }
  //      trouble();
  //    }
    for(int j=0; j<256; j++) { // 5 cycles of all colors on wheel
    // getMouvement();
          int val =  strip.numPixels()/2;
          
      for(int i= strip.numPixels()/2; i>7; i--) {
        //for (; down > limit; down--)
 
         wheelVal = Wheel(((i * 256 / strip.numPixels()) + j) & 255);

        redVal = red(wheelVal) * float(fadeVal/fadeMax);
        greenVal = green(wheelVal) * float(fadeVal/fadeMax);
        blueVal = blue(wheelVal) * float(fadeVal/fadeMax);
        strip.clear();
        sectionSqueezeWave(7,i,val,strip.Color( 0, greenVal, blueVal ),strip.Color( redVal, 0, blueVal ));

        // strip.setPixelColor( i, strip.Color( 0, greenVal, blueVal ) );
        // strip.setPixelColor( val, strip.Color( 0, greenVal, blueVal ) );
        val++;
         strip.show();
      delay(wait);
      }

      // First loop, fade in!
      if(k == 0 && fadeVal < fadeMax-1) {
        fadeVal++;
      }
      // Last loop, fade out!
      else if(k == rainbowLoops - 1 && j > 255 - fadeMax ) {
        fadeVal--;
      }

     
    }
  }

  // delay(500);
}

void sectionSqueezeWave(int numLimit, int down, int up, uint32_t colorUP, uint32_t colorDOWN) {

  int valUP;
  int valDOWN;
  int numHalfPixels=strip.numPixels()/2;
  Serial.print("NUM_LIMIT--:");
  Serial.println(numLimit);
  for(int step = 0; step < numLimit ; step++){
    valUP=up+step;
    valDOWN=down-step;

    uint32_t colorOFF = strip.Color(0,0,0);
    Serial.print("UP--:");
    Serial.println(valUP);
    
    Serial.print("DOWN--:");
    Serial.println(valDOWN);
    // if(valUP==numHalfPixels){
    //   strip.setPixelColor(valUP, colorOFF); 
    // }else if(valDOWN==numHalfPixels){
    //   strip.setPixelColor(valDOWN, colorOFF); 
    // }else{

    strip.setPixelColor(valDOWN, colorDOWN); 
    strip.setPixelColor(valUP, colorUP);
    // }
  }
}

void sectionSqueeze(int numLimit, int down, int up) {

  int valUP;
  int valDOWN;
  int numHalfPixels=strip.numPixels()/2;
  Serial.print("NUM_LIMIT--:");
  Serial.println(numLimit);
  Serial.print("UP--:");
  Serial.println(up);
  
  Serial.print("DOWN--:");
  Serial.println(down);
  uint32_t colorBlue = strip.Color(0,0,255);
  uint32_t colorPink = strip.Color(255,0,255);
  uint32_t colorOFF = strip.Color(0,0,0);

  uint32_t colorStep;
  int colorStepRed;
  int colorStepGreen;
  int colorStepBlue;
  int jump = 100;

  for(int step = 0; step < numLimit; step++){
    valUP=up+step;
    valDOWN=down-step;
    float pulseCounter = 0;
    int SCALE = 50;
    float pulseSpeed = 0.1;

    if(valUP<numHalfPixels){
      strip.setPixelColor(valUP, colorOFF); 
    }else if(valDOWN>numHalfPixels){
      strip.setPixelColor(valDOWN, colorOFF); 
    }else{
      // for(int i = 0; i < 255; i++){
        // colorBlue = strip.Color(0,0,i);
        // colorPink = strip.Color(i,0,i);

          if(step>numLimit/2){
            colorStep=colorBlue;
            colorStepRed=0;
            colorStepGreen=0;
            colorStepBlue=255;
          }else if(step>numLimit/4){
            // colorStep=colorPink;
            colorStepRed=0;
            colorStepGreen=255;
            colorStepBlue=255;
          }else{
            colorStep=colorPink;
            colorStepRed=255;
            colorStepGreen=0;
            colorStepBlue=255;
          }
        // delay(5);
       // strip.setPixelColor(valDOWN, colorStep); 
     //   strip.setPixelColor(valUP, colorStep);
     if(jump<255){
        jump+=10;
     }else{
      jump-=10;
     }
      strip.setColorDimmed(valUP, colorStepRed, colorStepGreen, colorStepBlue,jump );
      strip.setColorDimmed(valDOWN, colorStepRed, colorStepGreen, colorStepBlue, jump);
      // }

      //  delay(tan(pulseCounter)*SCALE);
      //       pulseCounter += pulseSpeed;
    }
   
  }
}

void colorFace(uint32_t color){
  for(int i = 0; i < 255; i++){
   
  }
}
void theaterChase(uint32_t color, int wait)
{
  
  int limit = 20;

  int numHalfPixels=strip.numPixels()/2;
  int numLimit = numHalfPixels-limit;
  int stretch = numHalfPixels;
  int up = numHalfPixels-limit;
  int down = numHalfPixels+limit;

  float pulseCounter = 0;
  int SCALE = 1000;
  float pulseSpeed = 0.05;

    for (; down > stretch; down--)
    {     
      strip.clear(); //   Set all pixels in RAM to 0 (off)
      sectionSqueeze(limit,down,up);
      /*
      strip.setPixelColor(down-1, strip.Color(0,0,255)); // Set pixel 'c' to value 'color'
      strip.setPixelColor(down-2, strip.Color(0,0,255));
      strip.setPixelColor(down-3, color);
      strip.setPixelColor(down, strip.Color(0,0,255));

      strip.setPixelColor(up, strip.Color(0,0,255));
      strip.setPixelColor(up+1, strip.Color(0,0,255));
      strip.setPixelColor(up+2, strip.Color(0,0,255));
      strip.setPixelColor(up+3, color);
      */


      up++;
      strip.show(); // Update strip with new contents
      delay(sin(pulseCounter)*SCALE);
      Serial.print("SIN-----");
      Serial.println(tan(pulseCounter)*SCALE);
      pulseCounter += pulseSpeed;
      // delay(wait*=0.8);  // Pause for a moment
  // }à
    }
pulseCounter = 0;
    //wait = 1000;
      Serial.println("END FIRST LOOP");
      Serial.print("LIMIT--");
      Serial.println(limit);
        Serial.print("DOWN--");
      Serial.println(down);
    for (; down<numHalfPixels+limit; down++)
    { 
      Serial.println("SECOND LOOP");
      strip.clear(); //   Set all pixels in RAM to 0 (off)
      sectionSqueeze(limit,down,up);
      /*
      strip.setPixelColor(a-1, strip.Color(0,0,255)); // Set pixel 'c' to value 'color'
      strip.setPixelColor(a-2, color);
      strip.setPixelColor(a-3, color);
      strip.setPixelColor(a, strip.Color(0,0,255));

      strip.setPixelColor(j, strip.Color(0,0,255));
      strip.setPixelColor(j+1, strip.Color(0,0,255));
      strip.setPixelColor(j+2, color);
      strip.setPixelColor(j+3, color);
      */
      up--;
      strip.show(); // Update strip with new contents
      delay(sin(pulseCounter)*SCALE);
      pulseCounter += pulseSpeed;
      //delay(wait*=0.8); // Pause for a moment
    }
}

void colorWipe(uint8_t wait) {
  uint32_t c;
  uint8_t _wait = wait;
  _wait-=10;
  uint16_t j = strip.numPixels()/2;
  int increment=0;
  for(int round = 0; round < 3 ; round ++){
  j = strip.numPixels()/2;
  
    for(uint16_t i=strip.numPixels()/2; i>0; i--) {
      if(round==0){
        c =strip.Color(255, 0, 0);
      }else if(round==1){
        c =strip.Color(0, 255, 0);
      }else if(round==2){
        c = strip.Color(255, 0, 255);
      }
      strip.setPixelColor(i, c);
      strip.show();
      strip.setPixelColor(j, c);
      strip.show();
      j++;
      delay(_wait);
    }
  }
}
//-----------------------//-----------------------//-----------------------//-----------------------NEOPIXELS