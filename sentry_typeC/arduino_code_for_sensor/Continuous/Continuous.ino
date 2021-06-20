/*
  This example shows how to take simple range measurements with the VL53L1X. The
  range readings are in units of mm.
*/
#include <SharpIR.h>
#include <Wire.h>
//#include <VL53L1X.h>

#define leftPin A0
#define rightPin A1
#define model 1080
#define DETECT_DISTANCE 50

int distance_left;
int distance_right;
SharpIR leftSensor = SharpIR(leftPin, model);
SharpIR rightSensor = SharpIR(rightPin, model);


//VL53L1X sensor;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, 0);
  pinMode(12, OUTPUT);

}

void loop()
{
    distance_left = leftSensor.distance();
    distance_right = rightSensor.distance();
//    Serial.print("left: ");
//      Serial.println(distance_left);
//      Serial.print("right");
//      Serial.println(distance_right);
//      delay(50);
  if (distance_left < DETECT_DISTANCE) {
    Serial.print("left: ");
    Serial.println(distance_left);
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }

  if (distance_right < DETECT_DISTANCE) {
    Serial.print("right");
    Serial.println(distance_right);
    digitalWrite(12, HIGH);
  } else {
    digitalWrite(12, LOW);
  }
//  delay(50);
}
