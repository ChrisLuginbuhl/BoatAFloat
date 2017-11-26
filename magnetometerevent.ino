// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_Sensor.h>
// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_HMC5883.h>

//#include <Wire.h>


/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//this is the code for device1 which publishes events as 'Photon1' and subscribes to events from 'Photon2'
int ledPin = D0;
int buttonPin = D1;
bool on = false;

int refreshRate = 10000;
long lastRefresh = 0;

int headingDegrees = 0;
//long headingDegreesLONG = 0;

int lastHeading = 0;

bool alarmState = false;


void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  // This is the event this device is subscribed to
  Particle.subscribe("Photon2", eventData);

  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  //vvvvvvvvvMagnetomerCode

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
  //^^^^^^MagnetomerCode

magnetometerRead();

    int sum = 0;
    for(int i = 0; i < 10000; i++)  {
       sum += headingDegrees;
    }
	lastHeading = sum / 10000;

}

void loop() {
  PubNubTimer();








  //delay(500);
  //magentomerCode^^^^^^^^
}

void magnetometerRead()
 { //delay(100);
  //magnetomerCodeVVVVVVVVVV
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
 headingDegrees = heading * 180/M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  }

// this is the function that receives incoming data from the esubscribed event
void eventData(const char *name, const char *data) {
  String buttonState = String(data);
  if(buttonState == "on"){
    digitalWrite(ledPin, HIGH);
  } else if (buttonState == "off") {
    digitalWrite(ledPin, LOW);
  }
}

// void CompassEvent() {
// //char headingString = headingDegrees;
//     //headingDegreesLONG = lround(headingDegrees);
//     Particle.publish("$$$$$$Bearing$$$$$$", (String)headingDegrees);
// }

void headingChangeAlarm()
    {
    if (abs(lastHeading - headingDegrees) > 10 && alarmState == false)
        {
            Particle.publish("$$$BearingAlarm$$$", "Bearing has changed!");
            alarmState = true;
        }
    if (abs(lastHeading - headingDegrees) <= 10 && alarmState == true)
        {
            Particle.publish("$$$BearingAlarm$$$", "Bearing has reverted");
            alarmState = false;
        }
    }

void PubNubTimer()
  {
    if(millis()-lastRefresh>=refreshRate)
    {
    lastRefresh=millis();
    magnetometerRead();
    //CompassEvent();
    headingChangeAlarm();



    }


  }

/***************************************************************************
  This is a library example for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746

  *** You will also need to install the Adafruit_Sensor library! ***

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/



void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
