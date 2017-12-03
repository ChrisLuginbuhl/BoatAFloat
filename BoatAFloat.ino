

#include <Adafruit_LSM9DS0.h>
#include <math.h>

const int servoPin = D2;
const int numAvgs = 5000;
const int NUM_CALIBRATION_READS = 5000;
const int PUBLISH_INTERVAL = 2000; //milliseconds. Make sure this is long enough to capture at least one cycle of waves.
const int smoothing = 12;

long lastPublish = 0;

int accelSumX = 0;
int accelSumY = 0;
int accelSumZ = 0;

double accelAvgX = 0;
double accelAvgY = 0;
double accelAvgZ = 0;

int initialAccelX = 0;
int initialAccelY = 0;
int initialAccelZ = 0;

double rollAngle = 0.0;
double pitchAngle = 0.0;
double totalRoll = 0.0;

double minRoll = 0;
double maxRoll = 0;
double minPitch = 0;
double maxPitch = 0;

int amplitude = 0;
int oldPosition = 0;
int newPosition = 0;
float smoothedPosition = 0.0;
bool alarmSent = FALSE;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(); //create sensor object
Servo servoOne;   //create Servo object. LIbrary is included in particle firmware by default.

void setupSensor()
{
  //  Set the accelerometer range, magnetometer sensitivity and gyro scale.
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
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

  servoOne.attach(D2);

  Particle.variable("totalRoll", int(totalRoll));   //We are publishing this for IFTTT and web
  Particle.subscribe("totalRoll", setShoreRollAmplitude);
}

void loop()
{
  getRollAndPitch();
  minRoll = rollAngle;
  maxRoll = rollAngle;
  minPitch = pitchAngle;
  maxPitch = pitchAngle;

  //take accelerometer readings and capture min/max, publish on PUBLISH_INTERVAL
  while (millis() - lastPublish < PUBLISH_INTERVAL) {
    accelSumX = 0;
    accelSumY = 0;
    accelSumZ = 0;
    getRollAndPitch();
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

    //check for maxima and minima. These are what we will combine and publish
    if (rollAngle > maxRoll) {
      maxRoll = rollAngle;
    } else if (rollAngle < minRoll) {
      minRoll = rollAngle;
    }

    if (pitchAngle > maxPitch) {
      maxPitch = pitchAngle;
    } else if (pitchAngle < minPitch) {
      minPitch = pitchAngle;
    }
  } //end while
  lastPublish = millis();

  totalRoll = sqrt(pow((maxRoll - minRoll), 2) + pow((maxPitch - minPitch), 2)) * 10; //pythagoras's theorem. Used for finding the max angle from vertical using pitch and roll angles. Multiplying by 10 so we can send it as an int and still have precision for small roll angles.
  Particle.publish("totalRoll ", String(int(totalRoll)), 1);  //publish roll in tenths of a degree to avoid having to publish & later parse a float
  if (totalRoll > 500 && alarmSent == FALSE) {
    Particle.publish("alarm_status", "boat motion alarm");
    alarmSent = TRUE;
  }
}

void getRollAndPitch() {
  lsm.read();
  // take many samples and average them to smooth out noise (e.g. sensor noise, electrical noise, and high frequency vibration from machinery etc)
  for (int i = 0; i < numAvgs; i++) {
    accelSumX += (int)lsm.accelData.x;
    accelSumY += (int)lsm.accelData.y;
    accelSumZ += (int)lsm.accelData.z;
  }

  accelAvgX = accelSumX / numAvgs;      //no need for rolling averages because this operation is performed in 10ms
  accelAvgY = accelSumY / numAvgs;
  accelAvgZ = accelSumZ / numAvgs;

  rollAngle = 180 / PI * atan(accelAvgY / accelAvgZ);  //don't need to prevent accelAvgZ from being 0 because it's a double and will never be exactly zero.
  pitchAngle = 180 / PI * atan(accelAvgX / accelAvgZ); //accelAvgZ is mostly gravity unless the waves are giant & steep. Use the inverse tangent to determine how far we are from vertical.
  //sudden knocks will appear as high pitch/roll angles.
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
