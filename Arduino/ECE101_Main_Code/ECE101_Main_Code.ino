#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

Servo myservo;  // create servo object to control a servo, later attatched to D9
float distance = 0.0;
float distance_previous_error, distance_error;
long period = 0;

float levang = 95; // sometimes its not really levelled when its inserted

//Things to tune
///////////////////PID constants///////////////////////
float kp = 0.06; //Mine was 8
float ki = 0; //Mine was 0.2
float kd = 34; //Mine was 3100
float distance_setpoint = 140;           //Should be the distance from sensor to the middle of the bar in mm

float d = distance_setpoint;
float PID_p, PID_i, PID_d, PID_total;
unsigned long currtime; 
unsigned long prevtime;
long duration;
float distance_filt;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  distance_filt = 0;
  Serial.begin(115200);
  Serial.println("Raw:,Period:,Distance:,DistError:,D:,Control:");
 
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object
  myservo.write(levang); //Put the servco at angle 125, so the balance is in the middle
  unsigned long currtime = millis();

  //  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  //  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  while (! Serial) {
    delay(1);
  }
  //Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  lox.startRangeContinuous();
}

void loop() {
  d = get_distVL();
  distance = d;
  Serial.print(d);
  Serial.print(",");
  currtime=millis();
  period = currtime-prevtime;
  delay(10);
  Serial.print(period);
  Serial.print(",");
  prevtime = currtime;
//  distance = 0.0013 * d * d + 0.6 * d + 1.18;  //NEEDS CALIBRATION
  //Serial.println(distance);
  distance_filt = 1 * distance + 0 * distance_filt;
  distance=distance_filt;
  distance = constrain(distance, 10, 300);
  Serial.print(distance);
  Serial.print(",");

  //delay(10);
  distance_error = distance;
  //if(abs(distance_error)<15){distance_error=0; distance_previous_error=0;}
  
  Serial.print(distance_error);
  Serial.print(",");
  PID_p = kp * distance_error;
  float dist_difference = distance_error - distance_previous_error;
  PID_d = kd * ((distance_error - distance_previous_error) / period);
  Serial.print(PID_d);
  Serial.print(",");

  //    if(-3 < distance_error && distance_error < 3)
  //    {

  //    }
  //    else
  //    {
  //      PID_i = 0;
  //    }
  PID_i = PID_i + (ki * distance_error);

  PID_total = PID_p + PID_i + PID_d;
  Serial.println(PID_total);
  delay(1);
  //PID_total = map(PID_total, -200, 200, 45, 135);
  PID_total = levang-PID_total;
  

//  if (PID_total < (levang - 20)) {
//    PID_total = levang - 20;
//  }
//  if (PID_total > (levang + 20)) {
//    PID_total = levang + 20;
//  }

//  angle=PID_total;
  myservo.write(PID_total);
  //Serial.println(PID_total);
  
  distance_previous_error = distance_error;

}




float get_distVL()
{
  float n = 0;
  if (lox.isRangeComplete()) {
    n = lox.readRange();
    //Serial.println(n);
    return n;
  }
}
