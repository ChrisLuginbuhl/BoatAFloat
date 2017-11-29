#include <math.h>

int refreshRate = 42;
long lastRefresh = 0;

void setup() {
Serial.begin(9600);
delay(10000);
Serial.print("currentTime"); Serial.print(", "); Serial.print("potentiometer");Serial.print(", "); Serial.print("pot2");Serial.print(", "); Serial.print("pot2/2");Serial.print(", "); Serial.print("pot2/2/2");Serial.print(", "); Serial.print("pot2/2/2/2");Serial.print(", "); Serial.print("pot2/2/2/2/2");Serial.print(", "); Serial.println("pot2/2/2/2/2/2");


}

void loop() {
    int potentiometer = analogRead(A0);
    long pot2 = pow(potentiometer, 2);

    long currentTime = millis();
    

    
    
    if (millis() - lastRefresh >= refreshRate)
  {
      Serial.print(currentTime); Serial.print(", "); Serial.print(potentiometer);Serial.print(", "); Serial.print(pot2);Serial.print(", "); Serial.print(pot2/2);Serial.print(", "); Serial.print(pot2/2/2);Serial.print(", "); Serial.print(pot2/2/2/2);Serial.print(", "); Serial.print(pot2/2/2/2/2);Serial.print(", "); Serial.println(pot2/2/2/2/2/2);
    lastRefresh = millis();
  }
    

    
    
}
