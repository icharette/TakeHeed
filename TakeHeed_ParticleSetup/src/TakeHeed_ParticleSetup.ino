/*
SOURCES:

*/


#include "application.h"
#include "Particle.h"
#include "neopixel.h"
#include "math.h"
#include "SparkFunLSM9DS1.h"
#include "simple-OSC.h"
#ifdef __AVR__
#include <avr/power.h>
#endif
LSM9DS1 imu;
// #define LED_PIN 8
#define LED_PIN D0
#define NUM_LED 20
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LED, 8, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LED, D2, WS2812B);
bool pixels[NUM_LED];
int pixelPointer;


#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//sensor variables
float refX, refY, refZ;
float dX, dY, dZ;
float avMvmt;
int state;
float gainThreshold, lossThreshold;

//stepper variables
int stepperIndex;


//Function definitions 
void printAccel();    
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
void calibrateSensor();

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

    Serial.begin(115200);
  setupImu();
  calibrateSensor();
  setupMotor();
  setupNeopixel();
  state = 0;
  gainThreshold = 0.5;
  lossThreshold = 2;
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

void setupImu(){
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  lossThreshold = 5;
  gainThreshold = 1;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void setupMotor(){
  pinMode(6, OUTPUT); //Enable
  pinMode(5, OUTPUT); //Step
  pinMode(4, OUTPUT); //Direction

  digitalWrite(6,LOW);
}

void setupNeopixel(){
    // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  strip.begin();
  strip.show();
  for(int i = 0; i < NUM_LED; i++){
    pixels[i] = 1;
  }
  pixelPointer = 20;
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}


// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? M_PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * M_PI / 180;
  
  if (heading > M_PI) heading -= (2 * M_PI);
  else if (heading < -M_PI) heading += (2 * M_PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  roll  *= 180.0 / M_PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}

void calibrateSensor(){
  Serial.print("calibrating sensor...");
  for(int i = 0; i < 10; i++){
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
    refX += imu.calcAccel(imu.ax);
    refY += imu.calcAccel(imu.ay);
    refZ += imu.calcAccel(imu.az);
  }
  refX = refX / 10;
  refY = refY / 10;
  refZ = refZ / 10; 
  Serial.println("done");
//  Serial.print("ref X: ");
//  Serial.print(refX);
//  Serial.print(" refY: ");
//  Serial.print(refY);
//  Serial.print(" refZ: ");
//  Serial.print(refZ);
//  Serial.println(" ");
}

void getMouvement(){
//    reset values
    dX = 0;
    dY = 0;
    dZ = 0;
    avMvmt = 0;
    for (int i = 0; i < 10; i++){
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
    dX += abs(imu.calcAccel(imu.ax) - refX);
    dY += abs(imu.calcAccel(imu.ay) - refY);
    dZ += abs(imu.calcAccel(imu.az) - refZ);
    avMvmt = (dX + dY + dZ) / 3;
    delay(100);
    }
    if (avMvmt < gainThreshold && pixelPointer <= NUM_LED){
      pixels[pixelPointer] = 1;
      pixelPointer++;
    }
    if (avMvmt > lossThreshold && pixelPointer >= 0){
      pixels[pixelPointer] = 0;
      pixelPointer--;
    }
    printMvmt();
}

void getState(){
  float average = 0;
  for(int i = 0; i < NUM_LED; i++){
    average += pixels[i];
  }
  average = average / NUM_LED;
  if (average > 0.8) state = 0;
  if (average <= 0.8 && average > 0.5) state = 1;
  if (average <=0.5 && average > 0.2) state = 2;
  if (average <= 0.2) state = 3;
}

void prosperity(){
  Serial.println("prosperity");
  setPixels(strip.Color(0,255,0));
  spinStepper(500);
}

void maintaining(){
  Serial.println("prosperity");
  setPixels(strip.Color(0,0,255));
  spinStepper(1000);
}

void endangered(){
  Serial.println("endangered");
  setPixels(strip.Color(255,255,0));
  spinStepper(2000);
}

void bleach() {
  Serial.println("bleach");
  setPixels(strip.Color(255, 0, 0));
}

// Fill the dots one after the other with a color
void setPixels(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    if (pixels[i]) strip.setPixelColor(i, c);
    else strip.setPixelColor(i, strip.Color(255, 255, 255));
    strip.show();
  }
}

void spinStepper(int pace){
  digitalWrite(4,HIGH);

  for(stepperIndex = 0; stepperIndex < 2000; stepperIndex++)
  {
    digitalWrite(5,HIGH);
    delayMicroseconds(pace);
    digitalWrite(5,LOW);
    delayMicroseconds(pace);
  }
  delay(1000);

  digitalWrite(4,LOW);

  for(stepperIndex = 0; stepperIndex < 2000; stepperIndex++)
  {
    digitalWrite(5,HIGH);
    delayMicroseconds(pace);
    digitalWrite(5,LOW);
    delayMicroseconds(pace);
  }
  delay(1000);
}


void printMvmt(){
    Serial.print("x: ");
    Serial.print(dX);
    Serial.print( " Y:");
    Serial.print(dY);
    Serial.print(" Z:");
    Serial.print(dZ);
    Serial.print(" av: ");
    Serial.print(avMvmt);
    Serial.println(" ");
}