// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_LSM9DS0.h>


#include <Adafruit_LSM9DS0.h>
#include <math.h>
#define PI 3.14159265

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

const int numAvgs = 5000;
const int NUM_CALIBRATION_READS = 50000;
const int PUBLISH_INTERVAL = 2000; //milliseconds
//const int arraySize = 1000;

int accelSumX = 0;
int accelSumY = 0;
int accelSumZ = 0;

double accelAvgX = 0;
double accelAvgY = 0;
double accelAvgZ = 0;

int initialAccelX = 0;
int initialAccelY = 0;
int initialAccelZ = 0;

int magSumX = 0;
int magSumY = 0;
int magSumZ = 0;

int magAvgX = 0;
int magAvgY = 0;
int magAvgZ = 0;

int initialMagX = 0;
int initialMagY = 0;
int initialMagZ = 0;

double rollAngle = 0.0;
double pitchAngle = 0.0;
// float yawAngle = 0.0;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


void setup() 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check connections.");
    while (1);  //do nothing forever
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
  delay(500); //let lsm readings stabilize;
  
  //get the initial state of the lsm sensors
  lsm.read();
  for (int i = 0; i < NUM_CALIBRATION_READS; i++) {
      magSumX += (int)lsm.magData.x;
      magSumY += (int)lsm.magData.y; 
      magSumZ += (int)lsm.magData.z;
  }
  
  initialMagX = magSumX / NUM_CALIBRATION_READS;
  initialMagY = magSumY / NUM_CALIBRATION_READS;
  initialMagZ = magSumZ / NUM_CALIBRATION_READS;
  
  initialAccelX = accelSumX / NUM_CALIBRATION_READS;
  initialAccelY = accelSumY / NUM_CALIBRATION_READS;
  initialAccelZ = accelSumZ / NUM_CALIBRATION_READS;

//   Particle.variable("pitch", pitchAngle);   //use Particle.variable() if we want the website to request the data each time. Use .publish() if we want to broadcast
    
}

void loop() 
{
  accelSumX=0;
  accelSumY=0;
  accelSumZ=0;
  
  magSumX = 0;
  magSumY = 0;
  magSumZ = 0;
  
  lsm.read();
  for (int i = 0; i < numAvgs; i++) {
      accelSumX += (int)lsm.accelData.x;
      accelSumY += (int)lsm.accelData.y;
      accelSumZ += (int)lsm.accelData.z;
      magSumX += (int)lsm.magData.x;
      magSumY += (int)lsm.magData.y; 
      magSumZ += (int)lsm.magData.z;
      //delay(1);
  }
  
  //here's a simple rolling average formula. 
  //   avg -= avg/N;
  //   avg += input/N;
  
  accelAvgX = accelSumX / numAvgs;
  accelAvgY = accelSumY / numAvgs;
  accelAvgZ = accelSumZ / numAvgs;
  
  magAvgX = magSumX / numAvgs - initialMagX;
  magAvgY = magSumY / numAvgs - initialMagY;
  magAvgZ = magSumZ / numAvgs - initialMagZ;
  
  rollAngle = 180/PI * atan(accelAvgY/accelAvgZ);    //prevent accelAvgZ from being 0
  pitchAngle = 180/PI * atan(accelAvgX/accelAvgZ);

//   if (rollAngle < 0) {
//       rollAngle += 360;
//   }
//   Particle.publish("Pitch", String(roll), 1);

//  delay(PUBLISH_INTERVAL);
  
//   Serial.print("  Initial Mag X: "); 
//   Serial.print(initialMagX);
  Serial.print("  roll: "); 
  Serial.print(int(rollAngle));
  Serial.print("  Pitch Angle: "); 
  Serial.print(pitchAngle);
  Serial.print("  Accel X: "); 
  Serial.print(accelAvgX);
  Serial.print("    Accel Y: "); 
  Serial.print(accelAvgY);
  Serial.print("    Accel Z: "); 
  Serial.print(accelAvgZ);
  Serial.print("    Mag X: "); 
  Serial.print(magAvgX / 100);  //100 here reduces the apparent noise further by chopping off the last two digits of the number
  Serial.print("    Mag Y: "); 
  Serial.print(magAvgY / 100);
  Serial.print("    Mag Z: "); 
  Serial.print(magAvgZ / 100);
  Serial.print("   Temp: ");
  Serial.println(lsm.temperature);
  
}