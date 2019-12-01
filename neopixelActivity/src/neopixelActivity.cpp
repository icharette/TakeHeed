/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/ninjacat/Documents/Particle/TakeHeed/neopixelActivity/src/neopixelActivity.ino"
/**
 * This is a RGB+W NeoPixel example, see extra-examples.cpp for a version
 * with more explantory documentation, example routines, how to
 * hook up your pixels and all of the pixel types that are supported.
 *
 * On Photon, Electron, P1, Core and Duo, any pin can be used for Neopixel.
 *
 * On the Argon, Boron and Xenon, only these pins can be used for Neopixel:
 * - D2, D3, A4, A5
 * - D4, D6, D7, D8
 * - A0, A1, A2, A3
 *
 * In addition on the Argon/Boron/Xenon, only one pin per group can be used at a time.
 * So it's OK to have one Adafruit_NeoPixel instance on pin D2 and another one on pin
 * A2, but it's not possible to have one on pin A0 and another one on pin A1.
 */

/* ======================= includes ================================= */
#include <Particle.h>
// #include <Arduino.h>
// #include <math.h>
#include "neopixel.h"

void setup();
void colorAll(uint32_t c, uint8_t wait);
static void chase(uint32_t c);
void loop();
void symbiosis(uint8_t wait);
void slowOrangeWave(uint8_t wait, int rainbowLoops, int whiteLoops);
void randomPixels(uint8_t wait, int rainbowLoops, int whiteLoops);
void whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength );
#line 24 "/Users/ninjacat/Documents/Particle/TakeHeed/neopixelActivity/src/neopixelActivity.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);


unsigned int localPort = 8888;
IPAddress ipAddress;
int port;
UDP udp;

/* ======================= prototypes =============================== */

uint32_t Wheel(byte WheelPos);
uint8_t red(uint32_t c);
uint8_t green(uint32_t c);
uint8_t blue(uint32_t c);
void colorWipe(uint32_t c, uint8_t wait);
void pulseWhite(uint8_t wait);
void rainbowFade2White(uint8_t wait, int rainbowLoops, int whiteLoops);
void whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength);
void fullWhite();
void rainbowCycle(uint8_t wait);
void rainbow(uint8_t wait);

/* ======================= rgbw-strandtest.cpp ====================== */

// SYSTEM_MODE(AUTOMATIC);

// IMPORTANT: Set pixel COUNT, PIN and TYPE
#define PIXEL_PIN D2
#define PIXEL_COUNT 80
#define PIXEL_TYPE SK6812RGBW
#define BRIGHTNESS 50 // 0 - 255

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN);

int gamma[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

void setup() {

    while(!Serial);
    WiFi.connect();

    //wifi function
    while(!WiFi.ready());
    Serial.println("Setup");
    udp.begin(localPort);
    WiFi.setHostname("HQRouter_PUBLISH");
    Serial.println(WiFi.hostname());
    Serial.println(WiFi.localIP());


  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

// Set all pixels in the strip to a solid color, then wait (ms)
void colorAll(uint32_t c, uint8_t wait) {
  uint16_t i;

  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

// void loop() {
//   chase(strip.Color(255, 0, 0)); // Red
//   // chase(strip.Color(0, 255, 0)); // Green
//   chase(strip.Color(0, 0, 255)); // Blue
//   //  colorAll(strip.Color(255, 0, 0),50);
// }
 
static void chase(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels()+4; i++) {
      strip.setPixelColor(i  , c); // Draw new pixel
      strip.setPixelColor(i-4, 0); // Erase pixel a few steps back
      strip.show();
      delay(250);
  }
}

int cycles = 500;
void loop() {

  // colorAll(strip.Color(255, 128, 0),10);
  // Some example procedures showing ow to display to the pixels:
  // Do not run more than 15 seconds of these, or the b/g tasks
  // will be blocked.
  // --------------------------------------------------------------
 
 
  // colorWipe(strip.Color(255, 0, 0), 3000); // Red

  
  // colorWipe(strip.Color(0, 255, 0), 50); // Green
  // colorWipe(strip.Color(0, 0, 255), 50); // Blue
  // colorWipe(strip.Color(0, 0, 0, 255), 50); // White

  // whiteOverRainbow(20,75,5);

  // pulseWhite(5);

  // fullWhite();
  // delay(2000);

  //NICE TRANSITION: slow wave, then blink out randomnly, then blink back randomly  and into fadded color
  //would nice a cycle value bigger for function randomPixels
  rainbowFade2White(10,10,1);
  slowOrangeWave(1,1,1);
  randomPixels(3,1,1);
  
  symbiosis(100);
  // pulseWhite(10);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
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

void rainbowFade2White(uint8_t wait, int rainbowLoops, int whiteLoops) {
  float fadeMax = 100.0;
  int fadeVal = 0;
  uint32_t wheelVal;
  int redVal, greenVal, blueVal;

  for(int k = 0 ; k < rainbowLoops ; k ++) {
    for(int j=0; j<256; j++) { // 5 cycles of all colors on wheel
      for(int i=0; i< strip.numPixels(); i++) {
        wheelVal = Wheel(((i * 256 / strip.numPixels()) + j) & 255);

        redVal = red(wheelVal) * float(fadeVal/fadeMax);
        greenVal = green(wheelVal) * float(fadeVal/fadeMax);
        blueVal = blue(wheelVal) * float(fadeVal/fadeMax);

        strip.setPixelColor( i, strip.Color( redVal, 128, 0 ) );
      }

      // First loop, fade in!
      if(k == 0 && fadeVal < fadeMax-1) {
        fadeVal++;
      }
      // Last loop, fade out!
      else if(k == rainbowLoops - 1 && j > 255 - fadeMax ) {
        fadeVal--;
      }

      strip.show();
      delay(wait);
    }
  }

  delay(500);

  for(int k = 0 ; k < whiteLoops ; k ++) {
    for(int j = 0; j < 256 ; j++) {
      for(uint16_t i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0,0,0, gamma[j] ) );
      }
      strip.show();
    }

    delay(2000);
    for(int j = 255; j >= 0 ; j--) {
      for(uint16_t i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0,0,0, gamma[j] ) );
      }
      strip.show();
    }
  }

  delay(500);
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {

  Serial.println(strip.numPixels());
int randomNumList[strip.numPixels()];
bool checkNum = false;
bool full = false;
int val=0;
  Serial.println("lignthing");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
  
     while(checkNum){
        checkNum = false;
        //int nRandonNumber = rand()%((nMax+1)-nMin) + nMin;
        val = (rand() % ((strip.numPixels())));
        for(int k = 0; k < strip.numPixels(); k++){
          if(randomNumList[k] == val){
            checkNum = true;
          }
        }
        

      }
        checkNum = true;
       Serial.print(i);
       Serial.print(" ::  ");
       Serial.println(val);
    
    for(int j = 0; j <= 255 ; j++){
 strip.setPixelColor(val, strip.Color(j, 0, 0));
        randomNumList[i] = val;
 delay(10);

strip.show();
     
    }
     
    delay(wait);
  }

  delay(2000);
  int randomNumList2[strip.numPixels()];
Serial.println("turning off");
   for(uint16_t i=0; i<strip.numPixels(); i++) {

     while(checkNum){
        checkNum = false;
        // val = (rand() % (strip.numPixels() - 0 + 1)+1);
        val = (rand() % ((strip.numPixels())));
            Serial.print("checknum");
        for(int k = 0; k < strip.numPixels(); k++){
          if(randomNumList2[k] == val){
            checkNum = true;
          }
        }
        

      }
        checkNum = true;
        Serial.print(i);
       Serial.print(" ::  ");
       Serial.println(val);
    
    
    for(int j = 255; j >=0 ; j--){
 strip.setPixelColor(val, strip.Color(j, 0, 0));
        randomNumList2[i] = val;
 
    strip.show();
     
    }
 
    delay(wait);
  }

    delay(2000);

}

void pulseWhite(uint8_t wait) {
  Serial.println("IN PULSE WHITE");
  for(int j = 0; j < 256 ; j++) {
    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(255,255,255, gamma[j] ) );
    }
    delay(wait);
    strip.show();
  }

  for(int j = 255; j >= 0 ; j--){
    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0,0,0, gamma[j] ) );
    }
    delay(wait);
    strip.show();
  }
}

void symbiosis(uint8_t wait){
  int brightness = 0;
  int red = 0 ;//255
  int blue = 0;//128
  int green = 0; //0
  // while(brightness<256){
  for(int i=0; i< strip.numPixels(); i++) {
    // while(red<255){
        strip.setPixelColor((rand() % (strip.numPixels() - 0 + 1)), 255, 128,0);
        // red++;
        // blue++;
        // delay(10);
    // }
    
    strip.setBrightness(brightness);
   strip.show();
  delay(500);
    brightness+=5;
  }

  // }
}
void slowOrangeWave(uint8_t wait, int rainbowLoops, int whiteLoops) {
  Serial.println("SLOW ORANGE WAVE");
  float fadeMax = 100.0;
  int fadeVal = 0;
  uint32_t wheelVal;
  int redVal, greenVal, blueVal;
float count = 0;
  for(int k = 0 ; k < rainbowLoops ; k ++) {
    for(int j=0; j<200; j++) { // 5 cycles of all colors on wheel
      for(int i=0; i< strip.numPixels(); i++) {


        wheelVal = Wheel(((i * 256 / strip.numPixels()) + j) & 255);
        redVal = red(wheelVal) * float(fadeVal/fadeMax);
        // Serial.print("wheelVal : " );
        // Serial.println(wheelVal);

        greenVal = green(wheelVal) * float(fadeVal/fadeMax);
        blueVal = blue(wheelVal) * float(fadeVal/fadeMax);

              strip.setPixelColor(i, redVal, 128,0, wheelVal); //very slow orange wave on green !!!! smooth and nice
           Serial.println(j);
           strip.show();
      // delay(wait);
   
      }

      // delay(wait);

      // First loop, fade in!
      if(k == 0 && fadeVal < fadeMax-1) {
        fadeVal++;
      }
      // // Last loop, fade out!
      else if(k == rainbowLoops - 1 && j > 255 - fadeMax ) {
        fadeVal--;
      }
    }
  }
}
void randomPixels(uint8_t wait, int rainbowLoops, int whiteLoops) {
  Serial.println("IN RANDOM PIXELS");
  float fadeMax = 100.0;
  int fadeVal = 0;
  uint32_t wheelVal;
  int redVal, greenVal, blueVal;
float count = 0;
  for(int k = 0 ; k < rainbowLoops ; k ++) {
    for(int j=0; j<cycles; j++) { // 5 cycles of all colors on wheel
      for(int i=0; i< strip.numPixels(); i++) {


        wheelVal = Wheel(((i * 256 / strip.numPixels()) + j) & 255);
        redVal = red(wheelVal) * float(fadeVal/fadeMax);
        // Serial.print("wheelVal : " );
        // Serial.println(wheelVal);

        greenVal = green(wheelVal) * float(fadeVal/fadeMax);
        blueVal = blue(wheelVal) * float(fadeVal/fadeMax);
           strip.show();
      delay(wait);
   
      }
 
      float val = (rand() % (strip.numPixels() - 0 + 1));
          strip.setColorDimmed(val, 255, 128,0, j);

      // First loop, fade in!
      if(k == 0 && fadeVal < fadeMax-1) {
        fadeVal++;
      }   
    }
  }
}


void whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength ) {
Serial.println("IN white over rainbow");
  if(whiteLength >= strip.numPixels()) whiteLength = strip.numPixels() - 1;

  int head = whiteLength - 1;
  int tail = 0;
  int loops = 3;
  int loopNum = 0;
  static unsigned long lastTime = 0;

  while(true) {
    for(int j=0; j<256; j++) {
      for(uint16_t i=0; i<strip.numPixels(); i++) {
        if( (i >= tail && i <= head)
          || (tail > head && i >= tail)
          || (tail > head && i <= head) ) {
          strip.setPixelColor(i, strip.Color(0,0,0, 255 ) );
        } else {
          strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }
      }

      if(millis() - lastTime > whiteSpeed) {
        head++;
        tail++;
        if(head == strip.numPixels()) {
          loopNum++;
        }
        lastTime = millis();
      }

      if(loopNum == loops) return;

      head %= strip.numPixels();
      tail %= strip.numPixels();
      strip.show();
      delay(wait);
    }
  }

}

void fullWhite() {
  Serial.println("IN full white");
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0,0,0, 255 ) );
  }
  strip.show();
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
Serial.println("IN rainbow cycle");
  for(j=0; j<256 * 5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// void rainbow(uint8_t wait) {
//   Serial.println("IN rainbow");
//   uint16_t i, j;

//   for(j=0; j<256; j++) {
//     for(i=0; i<strip.numPixels(); i++) {
//       strip.setPixelColor(i, Wheel((i+j) & 255));
//     }
//     strip.show();
//     delay(wait);
//   }
// }