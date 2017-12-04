
#include <neopixel.h>
#include <math.h>
#define PI 3.14159265
#define NEO_GRB  ((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_KHZ800 0x0000 // 800 KHz datastream

const int servoPin = D2;
const int NEOPIXEL_STRIP = D4;
const int numAvgs = 5000;
const int NUM_CALIBRATION_READS = 5000;
const int PUBLISH_INTERVAL = 2000; //milliseconds. Make sure this is long enough to capture at least one cycle of waves.
const int smoothing = 12;

long lastPublish = 0;
double totalRoll = 0.0;


int amplitude = 0;
int oldPosition = 0;
int newPosition = 0;
float smoothedPosition = 0.0;

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, NEOPIXEL_STRIP, NEO_GRB + NEO_KHZ800);
Servo servoOne;   //create Servo object. LIbrary is included in particle firmware by default.

void setup()
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  
  pixel.begin();            //neopixel object start
  pixel.setBrightness(25);  // Lower brightness and save eyeballs!
  pixel.show();             // Initialize all pixels to 'off'

  servoOne.attach(D2);   
  Particle.subscribe("totalRoll", setShoreRollAmplitude); 
}

void loop()
{

  //take accelerometer readings and capture min/max, publish on PUBLISH_INTERVAL
  while (millis() - lastPublish < PUBLISH_INTERVAL) {
    newPosition =  1500 + (int)(amplitude * sin(PI * millis()/PUBLISH_INTERVAL));  // 1500 microseconds is 90degrees. PI radians is one half revolution. Do a half cycle of a sine wave every PUBLISH_INTERVAL
    smoothedPosition += (newPosition-smoothedPosition)/smoothing;                   //this makes a rolling average to smooth sudden moves.
    
    //if there is a sudden step in the roll angle, smooth the servo output. Otherwise, don't smooth the servo output.
    if (abs(newPosition-oldPosition) > 3) {
      servoOne.writeMicroseconds(smoothedPosition);
      Serial.print("  smoothed: ");
      Serial.print(smoothedPosition);
      oldPosition = smoothedPosition;
    } else {
      servoOne.writeMicroseconds(newPosition);
      Serial.print("  new position: ");
      Serial.print(newPosition);
      oldPosition = newPosition;
    }
    
     Serial.print("  total Roll: ");
     Serial.print(totalRoll);
     Serial.print("  amplitude: ");
     Serial.println(amplitude);
    Particle.process();
  } //end while
  lastPublish = millis();
  if (totalRoll >= 500) {
     nPixel(pixel.Color(255, 0, 0));
  } else {
      nPixel(pixel.Color(0, 255, 0));
  }
}


void setShoreRollAmplitude(const char *tRoll, const char *data)
{
  int roll = 0;
  String data1 = "";

//convert const char* to Int
  std::string s = data;  
  int strLength = s.length();
  for(int i = 0; i < strLength; i++) {
    data1 = data1 + data[i];    
  }
  roll = data1.toInt();
  
  //convert boat roll to shore side roll ("amplitude")
  amplitude = round(pow(roll, 1./3.)*20) + 2;  //cubic scaling amplifies small roll angles and limits large ones

}

void nPixel(uint32_t c) {
    pixel.setPixelColor(0, pixel.Color(0,150,0)); // Moderately bright green color.
    pixel.show();
   
}