#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>


Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor4 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor5 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor6 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor7 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor8 = Adafruit_VL53L0X();

// Objects
Servo myServo1;
Servo myServo2;

// Tuned parameters
int angle_level_1 = -17;
int angle_level_2 = 4;


int levangx = 90;
int levangy = 93;

// General variables
float freq = .4;
float amplitude = 5;
float phase = PI / 2;

float angle_1;
float angle_2;

double kpx = 0.05;
double kix = 0.0000;
double kdx = 1;

double kpy = 0.05;
double kiy = 0.00000;
double kdy = 1;

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
int sumx = 0;
int sumy = 0;

int n = 0;
unsigned long time;       // current time (mm)
unsigned long time_prev;  // previous time, used to calculate time step
unsigned long time_step;  // period of one loop cycle (ms)
long period = 0;
long duration;


int error_limit = 30;  // acceptable distance error (mm)
int cycle_limit = 60;  // minimum number of consecutive loop cycles within error limit to be considered "stable"
int cycle_counter;     // number of consecutive loop cycles within error limit


const int numReadings = 1;

int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

float distance_raw;  // sensor output (mm)

int readingsy[numReadings];  // the readings from the analog input
int readIndexy = 0;          // the index of the current reading
int totaly = 0;              // the running total
int averagey = 0;            // the average

float distance_rawy;  // sensor output (mm)

float distance_previous_error, distance_error;
float distance_previous_error_x, distance_error_x;
float distance_previous_error_y, distance_error_y;

float PID_px;      //proportional control action
float PID_ix;      //integral control action
float PID_dx;      //derivative control action
float PID_totalx;  //total control action

float PID_py;      //proportional control action
float PID_iy;      //integral control action
float PID_dy;      //derivative control action
float PID_totaly;  //total control action

int movex, movey;

void setup() {
  myServo1.attach(3);
  myServo2.attach(5);
  myServo1.writeMicroseconds(deg2micro(angle_level_1));
  myServo2.writeMicroseconds(deg2micro(angle_level_2));

  delay(100);

  time = millis();

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
  Serial.print(" ");
  Serial.print(" Two: ");
  Serial.print(sensorTwo);
  Serial.print(" ");
  Serial.print(" Three: ");
  Serial.print(sensorThree);
  Serial.print(" ");
  Serial.print(" Four: ");
  Serial.print(sensorFour);
  Serial.print(" ");
  Serial.print(" Five: ");
  Serial.print(sensorFive);
  Serial.print(" ");
  Serial.print(" Six: ");
  Serial.print(sensorSix);
  Serial.print(" ");
  Serial.print(" Seven: ");
  Serial.print(sensorSeven);
  Serial.print(" ");
  Serial.print(" Eight: ");
  Serial.print(sensorEight);
  Serial.print(" ");
  //Serial.println("");

  n = 0;
  math();
    ////// running average to smoothen value ///////
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = x;
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average = total / numReadings;
    // send it to the computer as ASCII digits
    distance_raw = average;

    ////// running average to smoothen value ///////
    totaly = totaly - readingsy[readIndexy];
    // read from the sensor:
    readingsy[readIndexy] = y;
    // add the reading to the total:
    totaly = totaly + readingsy[readIndexy];
    // advance to the next position in the array:
    readIndexy = readIndexy + 1;

    // if we're at the end of the array...
    if (readIndexy >= numReadings) {
      // ...wrap around to the beginning:
      readIndexy = 0;
    }

    // calculate the average:
    averagey = totaly / numReadings;
    // send it to the computer as ASCII digits
    distance_rawy = averagey;

  if (n != 0) {
    time_prev = time;           // get previous time
    time = millis();            // get current time
    period = time - time_prev;  // compute time step
    Serial.print(period);
    Serial.print("period");
    Serial.print(" ");
    //distance_raw = mySensor.readRange();             // get distance measurement

    // distance_raw = constrain(distance_raw, 10, 300);    // create approximate lower and upper ends for distance
    // distance_error = distance_setpoint - distance_raw;  // obtaining error for PID calculations





    distance_error_x = distance_raw;
    distance_error_y = -distance_rawy;
    /*Implementing PID feedback on the error*/
    PID_px = kpx * distance_error_x;
    PID_dx = kdx * ((distance_error_x - distance_previous_error_x) / period);
    PID_ix = PID_ix + (kix * distance_error_x);

    PID_py = kpy * distance_error_y;
    PID_dy = kdy * ((distance_error_y - distance_previous_error_y) / period);
    PID_iy = PID_iy + (kiy * distance_error_y);


    ////////// Permanent Stability Condition /////////////


    PID_totalx = PID_px + PID_ix + PID_dx ;
    PID_totaly = PID_py + PID_iy + PID_dy;

    Serial.print(PID_totalx);
    Serial.print(" ");
    movex = levangx + PID_totalx;                          // get the new servo-angle
    Serial.print(movex);
    Serial.print(" ");
    movex = constrain(movex, levangx - 20, levangx + 20);  // ensure the new angle does not cause drastic shifts
    myServo1.write(movex);
    Serial.print(movex);
    Serial.print(" ");

    Serial.print(PID_totaly*1000);
    Serial.print(" ");
    movey = levangy + PID_totaly;                          // get the new servo-angle
    Serial.print(movey);
    Serial.print(" ");
    movey = constrain(movey, levangy - 20, levangy + 20);  // ensure the new angle does not cause drastic shifts
    myServo2.write(movey);
    Serial.print(movey);
    Serial.print(" ");

    distance_previous_error_x = distance_error_x;  // store as previous error for ID calculations
    distance_previous_error_y = distance_error_y;  
  }
  else{
    // angle_1 = 2*amplitude * sin(2 * PI * freq * time / 1000) + angle_level_1;
    // angle_2 = 2*amplitude * sin(2 * PI * freq * time / 1000 + phase) + angle_level_2;

    myServo1.write(movex);
    myServo2.write(movey);
    // myServo1.writeMicroseconds(deg2micro(angle_1));
    // myServo2.writeMicroseconds(deg2micro(angle_2));
  }

}

void math() {

  if (sensorOne < 370) {
    theta = 0;
    r = 243 - sensorOne;
    sumx = sumx + r * cos(theta);
    sumy = sumy + r * sin(theta);
    n = n + 1;
    Serial.print(1);
  }

  if (sensorTwo < 340) {
    theta = PI / 4;
    r = 200 - sensorTwo;
    sumx = sumx + r * cos(theta);
    sumy = sumy + r * sin(theta);
    n = n + 1;
    Serial.print(2);
  }
  if (sensorThree < 350) {
    theta = 2 * PI / 4;
    r = 210 - sensorThree;
    sumx = sumx + r * cos(theta);
    sumy = sumy + r * sin(theta);
    n = n + 1;
    Serial.print(3);
  }
  if (sensorFour < 320) {
    theta = 3 * PI / 4;
    r = 205 - sensorFour;
    sumx = sumx + r * cos(theta);
    sumy = sumy + r * sin(theta);
    n = n + 1;
    Serial.print(4);
  }
  if (sensorFive < 360) {
    theta = 4 * PI / 4;
    r = 210 - sensorFive;
    sumx = sumx + r * cos(theta);
    sumy = sumy + r * sin(theta);
    n = n + 1;
    Serial.print(5);
  }
  if (sensorSix < 360) {
    theta = 5 * PI / 4;
    r = 230 - sensorSix;
    sumx = sumx + r * cos(theta);
    sumy = sumy + r * sin(theta);
    n = n + 1;
    Serial.print(6);
  }
  if (sensorSeven < 360) {
    theta = 6 * PI / 4;
    r = 230 - sensorSeven;
    sumx = sumx + r * cos(theta);
    sumy = sumy + r * sin(theta);
    n = n + 1;
    Serial.print(7);
  }
  if (sensorEight < 330) {
    theta = 7 * PI / 4;
    r = 230 - sensorEight;
    sumx = sumx + r * cos(theta);
    sumy = sumy + r * sin(theta);
    n = n + 1;
    Serial.print(8);
  }

  //Serial.println("");

  if (n == 0) {
    x = 0;
    y = 0;
    Serial.print(" ");
    Serial.print(n);
    Serial.print(" n");
    Serial.print(" ");
    return;
  }

  Serial.print(" ");
  Serial.print(n);
  Serial.print(" n");
  Serial.print(" ");
  x = sumx / n;
  y = sumy / n;
  sumx = 0;
  sumy = 0;

  Serial.print("x: ");
  Serial.print(x);

  Serial.print(" y: ");
  Serial.print(y);

  //Serial.print(" r: ");
  //Serial.print(r);

  //Serial.print(" theta: ");
  //Serial.print(theta);

  Serial.println(" ");
}

int deg2micro(float deg) {                      // convert degrees to microseconds for servo function
  int result = (deg + 90) * 1000 / 180 + 1000;  // convert range of -90 to 90 degrees to 1000 to 2000 microseconds
  return constrain(result, 1000, 2000);         // keep result within safe limits
}
