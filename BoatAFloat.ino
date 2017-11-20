{\rtf1\ansi\ansicpg1252\cocoartf1561\cocoasubrtf100
{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\partightenfactor0

\f0\fs24 \cf0 //#include <Wire.h>\
//#include <SPI.h>\
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
\
long sumX = 0;\
long sumY = 0;\
long sumZ = 0;\
int avgX = 0;\
int avgY = 0;\
int avgZ = 0;\
int numAvgs = 5000;\
\
\
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
\}\
\
void loop() \
\{\
  lsm.read();\
  for (int i = 0; i < numAvgs; i++) \{\
      sumX += (int)lsm.accelData.x;\
      sumY += (int)lsm.accelData.y;\
      sumZ += (int)lsm.accelData.z;\
      //delay(1);\
  \}\
  avgX = sumX / (10*numAvgs);\
  avgY = sumY / (10*numAvgs);\
  avgZ = sumZ / (10*numAvgs);\
  \
  sumX=0;\
  sumY=0;\
  sumZ=0;\
      \
  Serial.print("Accel X: "); \
  Serial.print(avgX);\
  Serial.print("    Accel Y: "); \
  Serial.print(avgY);\
  Serial.print("    Accel Z: "); \
  Serial.println(avgZ);\
  delay(1000);\
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