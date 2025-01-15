#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor4 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor5 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor6 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor7 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor8 = Adafruit_VL53L0X();

int sensorOne;
int sensorTwo;
int sensorThree;
int sensorFour;
int sensorFive;
int sensorSix;
int sensorSeven;
int sensorEight;
float r;
float theta;
int x;
int y;
int  sumx=0;
int  sumy=0;

int  n=0;
unsigned long time;       // current time (mm)
unsigned long time_prev;  // previous time, used to calculate time step
unsigned long time_step;  // period of one loop cycle (ms)
long period = 0;
long duration;

void setup() {
  time = millis();                      // get current time

  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);

  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
  
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  
  pinMode(30, OUTPUT);
  digitalWrite(30, LOW);
    
  Wire.begin();
  Serial.begin(115200);

  Serial.println("Setting Addresses.");

  digitalWrite(6, HIGH);
  sensor1.begin(0x30);
  // sensor1.setAddress((uint8_t)00);
  Serial.println("Sensor 1 Address Set.");

  digitalWrite(7, HIGH);
  sensor2.begin(0x31);
  // sensor2.setAddress((uint8_t)42);
  Serial.println("Sensor 2 Address Set.");

  digitalWrite(8, HIGH);
  sensor3.begin(0x32);
  // sensor3.setAddress((uint8_t)81);
  Serial.println("Sensor 3 Address Set.");

  digitalWrite(9, HIGH);
  sensor4.begin(0x33);
  //ensor4.setAddress((uint8_t)03);
  Serial.println("Sensor 4 Address Set.");

  digitalWrite(10, HIGH);
  sensor5.begin(0x34);
  // sensor5.setAddress((uint8_t)04);
  Serial.println("Sensor 5 Address Set.");

  digitalWrite(11, HIGH);
  sensor6.begin(0x35);
  // sensor6.setAddress((uint8_t)05);
  Serial.println("Sensor 6 Address Set.");

  digitalWrite(12, HIGH);
  sensor7.begin(0x36);
  // sensor7.setAddress((uint8_t)06);
  Serial.println("Sensor 7 Address Set.");

  digitalWrite(30, HIGH);
  sensor8.begin(0x3f);
  //sensor8.setAddress((uint8_t)07);
  Serial.println("Sensor 8 Address Set.");

  Serial.println("Addresses Set.");

  sensor1.startRangeContinuous();
  sensor2.startRangeContinuous();
  sensor3.startRangeContinuous();
  sensor4.startRangeContinuous();
  sensor5.startRangeContinuous();
  sensor6.startRangeContinuous();
  sensor7.startRangeContinuous();
  sensor8.startRangeContinuous();

  // put your setup code here, to run once:
  Serial.println("Setup Complete.");
}

void loop() {
  time_prev = time;                                // get previous time
  time = millis();                                 // get current time
  period = time - time_prev;                       // compute time step
  // Serial.print(period);
  // Serial.println("period");
  // put your main code here, to run repeatedly:
  sensorOne = sensor1.readRange();
  sensorTwo = sensor2.readRange();
  sensorThree = sensor3.readRange();
  sensorFour = sensor4.readRange();
  sensorFive = sensor5.readRange();
  sensorSix = sensor6.readRange();
  sensorSeven = sensor7.readRange();
  sensorEight = sensor8.readRange();
  
  Serial.print("One: ");
  Serial.print(sensorOne);
  Serial.print(" Two: ");
  Serial.print(sensorTwo);
  Serial.print(" Three: ");
  Serial.print(sensorThree);
  Serial.print(" Four: ");
  Serial.print(sensorFour);
  Serial.print(" Five: ");
  Serial.print(sensorFive);
  Serial.print(" Six: ");
  Serial.print(sensorSix);
  Serial.print(" Seven: ");
  Serial.print(sensorSeven);
  Serial.print(" Eight: ");
  Serial.print(sensorEight);
  Serial.println("");

  n=0;
  // math();
  
  if(n!=0){

  }

}

void math() {

  if (sensorOne < 400) {
    theta = 0;
    r = 250 - sensorOne;
    sumx=sumx+r*cos(theta);
    sumy=sumy+r*sin(theta);
    n=n+1;
  }
  else{

  }
  
  if (sensorTwo < 400) {
    theta = PI / 4;
    r = 250 - sensorTwo;
    sumx=sumx+r*cos(theta);
    sumy=sumy+r*sin(theta);
    n=n+1;

  } 
  else{

  }
  // for (int i = 0; i < 8; i++) {
  //   x = x + xVal[i];
  //   y = y + yVal[i];
  // }
  if (n==0){
    x=0;
    y=0;
    return;
  }
  
  Serial.print(n);
  Serial.println(" n");
  x = sumx / n;
  y = sumy / n;
  sumx=0;
  sumy=0;

     
  // Serial.print(x);
  //   Serial.println(" x");
  // Serial.print(y);

  // Serial.println(" y");
  // Serial.print(r);
  //  Serial.println(" r");
  // Serial.print(theta);
  
  // Serial.println(" theta");


}
