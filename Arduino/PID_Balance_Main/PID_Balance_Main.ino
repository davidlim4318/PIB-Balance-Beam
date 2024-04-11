#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

// General variables/objects
Servo myServo;   // create servo object to control a servo, later attatched to D9
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();
float distance = 0.0;
float distance_previous_error;
float distance_error;
long period = 0;
long totaltime=0;
long angle=0;
long pos = 0;

// Tunable parameters
float kp = 0.04;   // mine was 8
float ki = 0;   // mine was 0.2
float kd = 0.5;   // mine was 3100
float distance_setpoint = 112.0;   // should be the distance from sensor to the middle of the bar in mm
float levang = 1445;

float d = distance_setpoint; 
float PID_p, PID_i, PID_d, PID_total;   // necessary for calculations
unsigned long time_current; 
unsigned long time_prev;
long duration;
float distance_filt;

void setup() {
  time_current = millis();

  Serial.begin(9600);
  Serial.println("Raw:,Period:,Distance:,DistError:,P:,D:,I:,Control:");

  myServo.attach(6);   // attaches the servo on pin 6 to the servo object
  myServo.writeMicroseconds(levang);   // put the servco at an angle, so the balance is in the middle

  while (! Serial) {
    delay(1);
  }

  if (!mySensor.begin()) {
    Serial.println(F("Failed to boot VL53 L0X"));
    while (1);
  }
  mySensor.startRangeContinuous();
}

void loop() {
  time_current = millis();

  d = get_distVL();
  distance = d;

  period = time_current - time_prev;

  time_prev = time_current;
// distance = 0.0013 * d * d + 0.6 * d + 1.18;  //NEEDS CALIBRATION
// Serial.println(distance);
// distance_filt = 0.5 * distance + 0.5 * distance_filt;
// distance=distance_filt;
  distance = constrain(distance, 0, 300);
  Serial.print(distance);
  Serial.print(",");
  distance_error = distance_setpoint - distance;

  
  Serial.print(distance_error);
  Serial.print(",");
  
  
  //////// PID Calculations /////////////
  PID_p = kp * distance_error;
  Serial.print(PID_p);
  Serial.print(",");
  float dist_difference = distance_error - distance_previous_error;

  PID_i = PID_i + (ki * distance_error);
  Serial.print(PID_i);
  Serial.print(",");

  PID_d = kd * ((distance_error - distance_previous_error) / period);
  distance_previous_error = distance_error;
  Serial.print(PID_d);
  Serial.print(",");

  PID_total = PID_p + PID_i + PID_d;
  PID_total=constrain(PID_total,-30,30);
  angle=PID_total;
  PID_total=PID_total*10.3;
  Serial.println(PID_total);
//  Serial.print(",");
  delay(1);
  PID_total = PID_total + levang;


//  if (PID_total < (levang - 20)) {
//    PID_total = levang - 20;
//  }
//  if (PID_total > (levang + 20)) {
//    PID_total = levang + 20;
//  }

  PID_total = constrain(PID_total, 1000, 2000);


//  if(abs(distance_error)<5){PID_total=levang;}

  myServo.writeMicroseconds(PID_total);
//  Serial.println(PID_total);
  angle=angle+87;
  pos=myServo.read();
  
  while (abs(pos-angle) >2) {
    pos=myServo.read();
    Serial.print(pos);
    Serial.print(PID_total);
    Serial.println(angle);    
    myServo.writeMicroseconds(PID_total);
  }
}

float get_distVL()
{
  if (mySensor.isRangeComplete()) {
    float n = mySensor.readRange();
    return n;
  }
}

void print_data () {
  Serial.print(d); //print the distance received by the sensor
  Serial.print(", ");
  Serial.print(period); //print the period it takes for a cycle
  Serial.print(", ");
}
