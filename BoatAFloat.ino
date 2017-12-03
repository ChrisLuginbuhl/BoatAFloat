#include <Adafruit_LSM9DS0.h>
#include <math.h>

#define PI 3.14159265

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

const int servoPin = D2;
const int numAvgs = 5000;
const int NUM_CALIBRATION_READS = 5000;
const int PUBLISH_INTERVAL = 2000; //milliseconds. Make sure this is long enough to capture at least one cycle of waves.
//const int arraySize = 1000;
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
double totalRoll = 0.0;

double minRoll = 0;
double maxRoll = 0;
double minPitch = 0;
double maxPitch = 0;

Servo servoOne;   //create Servo object. LIbrary is included in particle firmware by default.

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

  // 3.) Setup the gyroscope
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
  servoOne.write(90);  // We try to leave the servo and start the servo in the same position to prevent a "lurch" - a full speed move to zero from wherever it was left.

  /*
    //get the initial state of the lsm sensors
    lsm.read();
    //necessary to calibrate magnetometer data to be able to separate static from changing dynamic fields.
    for (int i = 0; i < NUM_CALIBRATION_READS; i++) {
      magSumX += (int)lsm.magData.x;
      magSumY += (int)lsm.magData.y;
      magSumZ += (int)lsm.magData.z;
    }

    initialMagX = magSumX / NUM_CALIBRATION_READS;
    initialMagY = magSumY / NUM_CALIBRATION_READS;
    initialMagZ = magSumZ / NUM_CALIBRATION_READS;

    //   initialAccelX = accelSumX / NUM_CALIBRATION_READS;
    //   initialAccelY = accelSumY / NUM_CALIBRATION_READS;
    //   initialAccelZ = accelSumZ / NUM_CALIBRATION_READS;

    //   Particle.variable("pitch", pitchAngle);   //use Particle.variable() if we want the website to request the data each time. Use .publish() if we want to broadcast
  */

  Particle.subscribe("totalRoll", moveBoat);
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

    //   magSumX = 0;
    //   magSumY = 0;
    //   magSumZ = 0;
    getRollAndPitch();

    //check for maxima and minima.
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
  }
  lastPublish = millis();

  totalRoll = sqrt(pow((maxRoll - minRoll), 2) + pow((maxPitch - minPitch), 2)); //pythagoras's theorem. Used for finding the max angle from vertical using pitch and roll angles.
  Particle.publish("totalRoll ", String(int(totalRoll)), 1);

  delay(500); //from the Particle docs: "The built-in delay function safely interleaves required background activity, so arbitrarily long delays can safely be done if you need them."

  Serial.print("Total roll: ");
  Serial.print(totalRoll);
  Serial.print("  Max roll: ");
  Serial.print(maxRoll);
  Serial.print("  Min roll: ");
  Serial.print(minRoll);
  Serial.print("  Max pitch: ");
  Serial.print(maxPitch);
  Serial.print("  Min pitch: ");
  Serial.print(minPitch);

  Serial.print("  Roll Angle: ");
  Serial.print(int(rollAngle));
  Serial.print("  Pitch Angle: ");
  Serial.print(int(pitchAngle));
  Serial.print("  Accel X: ");
  Serial.print(accelAvgX);
  Serial.print("    Accel Y: ");
  Serial.print(accelAvgY);
  Serial.print("    Accel Z: ");
  Serial.println(accelAvgZ);
  //   Serial.print("    Mag X: ");
  //   Serial.print(magAvgX / 100);  //100 here reduces the apparent noise further by chopping off the last two digits of the number
  //   Serial.print("    Mag Y: ");
  //   Serial.print(magAvgY / 100);
  //   Serial.print("    Mag Z: ");
  //   Serial.print(magAvgZ / 100);
  //   Serial.print("   Temp: ");
  //   Serial.println(lsm.temperature);

}

void getRollAndPitch() {
  lsm.read();
  for (int i = 0; i < numAvgs; i++) {
    accelSumX += (int)lsm.accelData.x;
    accelSumY += (int)lsm.accelData.y;
    accelSumZ += (int)lsm.accelData.z;
    //       magSumX += (int)lsm.magData.x;
    //       magSumY += (int)lsm.magData.y;
    //       magSumZ += (int)lsm.magData.z;
  }

  accelAvgX = accelSumX / numAvgs;      //no need for rolling averages because this operation is performed in 10ms
  accelAvgY = accelSumY / numAvgs;
  accelAvgZ = accelSumZ / numAvgs;

  //   magAvgX = magSumX / numAvgs - initialMagX;
  //   magAvgY = magSumY / numAvgs - initialMagY;
  //   magAvgZ = magSumZ / numAvgs - initialMagZ;

  rollAngle = 180 / PI * atan(accelAvgY / accelAvgZ); //don't need to prevent accelAvgZ from being 0 because it's a double.
  pitchAngle = 180 / PI * atan(accelAvgX / accelAvgZ); //accelAvgZ is mostly gravity unless the waves are giant & steep. Use the inverse tangent to determine how far we are from vertical.
  //sudden knocks will appear as high pitch/roll angles.
}


void moveBoat(const char *tRoll, const char *data)
{
  int angle = 90;
  int amplitude = 0;
  int roll = 0;
  String data1 = "";
  std::string s = data;

  Serial.print("Total Roll received: ");
  if (data) {
    Serial.println(data);
  }
  else
    Serial.println("NULL");

  int commaIndex = s.length();
  for (int i = 0; i < commaIndex; i++) {
    data1 = data1 + data[i];
  }

  roll = data1.toInt();


  if (roll < 2) {
    amplitude = 25;
    Serial.println("Sinwave 4");
  } else if (roll < 10) {
    amplitude = 50;
    Serial.println("Sinewave 8");
  } else if (roll < 30) {
    amplitude = 75;
  } else if (roll >= 30) {
    amplitude = 120;
  }

  for (int i = 0; i < 360 * 10; i++) {
    servoOne.writeMicroseconds(1500 + (int)(amplitude * sin(i * PI / 180))); //1500 microseconds corresponds to 90degress. The full motion range for 9g servos is usually 1000-2000us.
    delay(PUBLISH_INTERVAL / 180);
  }

  servoOne.write(90);
}


