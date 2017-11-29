{\rtf1\ansi\ansicpg1252\cocoartf1561\cocoasubrtf100
{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\partightenfactor0

\f0\fs24 \cf0 \
#include <Adafruit_LSM9DS0.h>\
#include <Adafruit_Sensor.h>  // not used in this demo but required!\
\
// i2c\
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();\
\
// You can also use software SPI\
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(13, 12, 11, 10, 9);\
// Or hardware SPI! In this case, only CS pins are passed in\
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(10, 9);\
const int numAvgs = 5000;\
const int NUM_CALIBRATION_READS = 50000;\
//const int arraySize = 1000;\
\
int accelSumX = 0;\
int accelSumY = 0;\
int accelSumZ = 0;\
int accelAvgX = 0;\
int accelAvgY = 0;\
int accelAvgZ = 0;\
\
\
int magSumX = 0;\
int magSumY = 0;\
int magSumZ = 0;\
int magAvgX = 0;\
int magAvgY = 0;\
int magAvgZ = 0;\
\
int initialMagX = 0;\
int initialMagY = 0;\
int initialMagZ = 0;\
\
int pitchAngle = 0;\
int readTime = 0;\
\
// long j= 0;\
// int accelArrayX[arraySize];\
\
void setupSensor()\
\{\
  // 1.) Set the accelerometer range\
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);\
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);\
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);\
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);\
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);\
  \
  // 2.) Set the magnetometer sensitivity\
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);\
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);\
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);\
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);\
\
  // 3.) Setup the gyroscope\
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);\
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);\
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);\
\}\
\
\
void setup() \
\{\
#ifndef ESP8266\
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens\
#endif\
\
  Serial.begin(9600);\
  Serial.println("LSM raw read demo");\
  \
  // Try to initialise and warn if we couldn't detect the chip\
  if (!lsm.begin())\
  \{\
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");\
    while (1);\
  \}\
  Serial.println("Found LSM9DS0 9DOF");\
  Serial.println("");\
  Serial.println("");\
  delay(500); //let lsm readings stabilize;\
  \
  //get the initial state of the lsm sensors\
  lsm.read();\
  for (int i = 0; i < NUM_CALIBRATION_READS; i++) \{\
      magSumX += (int)lsm.magData.x;\
      magSumY += (int)lsm.magData.y; \
      magSumZ += (int)lsm.magData.z;\
  \}\
  \
  initialMagX = magSumX / NUM_CALIBRATION_READS;\
  initialMagY = magSumY / NUM_CALIBRATION_READS;\
  initialMagZ = magSumZ / NUM_CALIBRATION_READS;\
\
//   Particle.variable("pitch", pitchAngle);   //use Particle.variable() if we want the website to request the data each time. Use .publish() if we want to broadcast\
    \
\}\
\
void loop() \
\{\
  accelSumX=0;\
  accelSumY=0;\
  accelSumZ=0;\
  \
  magSumX = 0;\
  magSumY = 0;\
  magSumZ = 0;\
  \
  lsm.read();\
  for (int i = 0; i < numAvgs; i++) \{\
      accelSumX += (int)lsm.accelData.x;\
      accelSumY += (int)lsm.accelData.y;\
      accelSumZ += (int)lsm.accelData.z;\
      magSumX += (int)lsm.magData.x;\
      magSumY += (int)lsm.magData.y; \
      magSumZ += (int)lsm.magData.z;\
      //delay(1);\
  \}\
  \
  \
  //   avg -= avg/N;\
  //   avg += input/N;\
  accelAvgX = accelSumX / (100*numAvgs);  //100 here reduces the noise further by chopping off the last two digits of the number\
  accelAvgY = accelSumY / (100*numAvgs);\
  accelAvgZ = accelSumZ / (100*numAvgs);\
  \
  magAvgX = magSumX / numAvgs - initialMagX;\
  magAvgY = magSumY / numAvgs - initialMagY;\
  magAvgZ = magSumZ / numAvgs - initialMagZ;\
  \
  pitchAngle = magAvgX;\
  readTime = millis();\
  Particle.publish("Pitch", "test", 1);\
  readTime = millis() - readTime;\
  delay(5000);\
  \
  Serial.print("ReadTime: "); \
  Serial.print(readTime);\
  Serial.print("  Initial Mag X: "); \
  Serial.print(initialMagX);\
  Serial.print("  magAvgX: "); \
  Serial.print(magAvgX);\
  Serial.print("  Pitch Angle: "); \
  Serial.print(pitchAngle);\
  Serial.print("  Accel X: "); \
  Serial.print(accelAvgX);\
  Serial.print("    Accel Y: "); \
  Serial.print(accelAvgY);\
  Serial.print("    Accel Z: "); \
  Serial.print(accelAvgZ);\
  Serial.print("    Mag X: "); \
  Serial.print(magAvgX / 100);  //100 here reduces the apparent noise further by chopping off the last two digits of the number\
  Serial.print("    Mag Y: "); \
  Serial.print(magAvgY / 100);\
  Serial.print("    Mag Z: "); \
  Serial.print(magAvgZ / 100);\
  Serial.print("   Temp: ");\
  Serial.println(lsm.temperature);\
  \
 \
  \
//   accelArrayX[j++ % (arraySize -1)] = accelAvgX;\
  \
//   if (j++ % 1000 == 0) \{\
//      Particle.publish("AccelX", (String)*std::max_element(std::begin(accelArrayX), std::end(accelArrayX));\
//      Serial.println(*std::max_element(std::begin(accelArrayX), std::end(accelArrayX));\
//   \}\
\
//   delay(1000);\
\
/*\
  Serial.print("Accel X: "); Serial.print((int)lsm.accelData.x); Serial.print(" ");\
  Serial.print("Y: "); Serial.print((int)lsm.accelData.y);       Serial.print(" ");\
  Serial.print("Z: "); Serial.println((int)lsm.accelData.z);     Serial.print(" ");\
  Serial.print("Mag X: "); Serial.print((int)lsm.magData.x);     Serial.print(" ");\
  Serial.print("Y: "); Serial.print((int)lsm.magData.y);         Serial.print(" ");\
  Serial.print("Z: "); Serial.println((int)lsm.magData.z);       Serial.print(" ");\
  Serial.print("Gyro X: "); Serial.print((int)lsm.gyroData.x);   Serial.print(" ");\
  Serial.print("Y: "); Serial.print((int)lsm.gyroData.y);        Serial.print(" ");\
  Serial.print("Z: "); Serial.println((int)lsm.gyroData.z);      Serial.println(" ");\
  Serial.print("Temp: "); Serial.print((int)lsm.temperature);    Serial.println(" ");\
  delay(200);\
*/\
\}}