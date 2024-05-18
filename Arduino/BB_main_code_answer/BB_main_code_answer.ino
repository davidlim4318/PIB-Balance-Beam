//Libraries
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

//Objects
Servo myServo;  // declare servo object to control a servo, later attatched to D6
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X(); // declare a sensor object 

///////////////////Constants to tune///////////////////////
/* PID Controls Coefficients
kp: represents the P which is the proportional gain
ki: represents the I which is the integral gain
kd: represnts the D which is the differential gain */

float kp =  0.02;//FILL IN WITH YOUR VALUES
float ki = 0.0001; //FILL IN WITH YOUR VALUES
float kd = 40; //FILL IN WITH YOUR VALUES

/* Other key constants
distance_setpoint: the value obtained from the calibration (distance from sensor to the middle of the bar in mm)
levang: the servo angle motor in which the beam is fully horizantal */

float distance_setpoint = 120; //FILL IN WITH YOUR VALUES   
float levang = 93; //FILL IN WITH YOUR VALUES

int error_limit = 30;   // acceptable distance error (mm)
int cycle_limit = 120;   // minimum number of consecutive loop cycles within error limit to be considered "stable"
int cycle_counter;   // number of consecutive loop cycles within error limit

///////////////////Initializing internal variables///////////////////////

const int numReadings = 10;

int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

float distance_raw; // sensor output (mm)
float distance_previous_error, distance_error;
long period = 0;

float PID_p; //proportional control action
float PID_i; //integral control action
float PID_d; //derivative control action
float PID_total; //total control action

float move;
unsigned long time;   // current time (mm)
unsigned long time_prev;   // previous time, used to calculate time step
unsigned long time_step;   // period of one loop cycle (ms)

long duration;

void setup() {
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }


    // Serial setup
  Serial.begin(115200);
  while (! Serial) {
    delay(1);   // wait for serial communication to intialize
  }

  // Servo setup
  myServo.attach(6);   // assign servo signal output to pin 6
  myServo.write(levang);   // rotate the servo to the level angle

  // Sensor setup
  if (!mySensor.begin()) {   // check for sensor intialization
    Serial.println(F("Failed to boot VL53 L0X. Please reset."));
    while (1);
  }
  mySensor.setMeasurementTimingBudgetMicroSeconds(20000);   // set sensor timing budget to 20 ms
  mySensor.startRangeContinuous();   // intialize "continuous" sensor output
  
  // Control algorithm setup
  while (! mySensor.isRangeComplete()) {
    delay(1);   // wait for first distance measurement
  }
  time = millis();   // get current time
  distance_raw = mySensor.readRange();   // get distance measurement
}

void loop() {
  if (mySensor.isRangeComplete()) { // run calculations when distance measurement is updated
    
    
    ////// running average to smoothen value ///////
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = mySensor.readRange();
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

    delay(1);  // delay in between reads for stability

    time_prev = time;   // get previous time
    delay(constrain(25 - (millis() - time), 0, 5));   // delay loop cycle to at least 25 ms
    time = millis();   // get current time
    period = time - time_prev;   // compute time step

    //distance_raw = mySensor.readRange();             // get distance measurement
    distance_raw = constrain(distance_raw, 10, 300); // create approximate lower and upper ends for distance
    distance_error = distance_setpoint-distance_raw; // obtaining error for PID calculations
    
    /*Implementing PID feedback on the error*/
    PID_p = kp * distance_error;
    PID_d = kd * ((distance_error - distance_previous_error) / period);
    PID_i = PID_i + (ki * distance_error);
    


    ////////// Permanent Stability Condition /////////////

    if (cycle_counter >= cycle_limit) {   // run if distance has not stabilized within error limit and time limit
      PID_p = 0;   // set proportional control action to zero
      PID_d = 0;   // set derivatve control action to zero
      PID_i = PID_i - (ki * distance_error);  // also keep integral control action constant
      cycle_counter = cycle_limit;
    }
    
    if ( abs( distance_error ) <= error_limit ) {   // run if distance error is within error limit
      cycle_counter++;   // increase loop cycle counter if below loop cycle limit
    }
    else {
      cycle_counter = 0;   // if distance error outside error limit, reset loop cycle counter
    }
    
    ///////////////////////////////////////////////////////

    PID_total = PID_p + PID_i + PID_d;

    move = levang + PID_total;                      // get the new servo-angle 
    move = constrain(move, levang-40, levang+40); // ensure the new angle does not cause drastic shifts
    myServo.write(move);                          

    distance_previous_error = distance_error;     // store as previous error for ID calculations
    print_data();
  }
}

void print_data() {
  Serial.print("Step:");
  Serial.print(period);
  Serial.print(" ");

  Serial.print("Raw:");
  Serial.print(distance_raw);
  Serial.print(" ");

  Serial.print("Error:");
  Serial.print(distance_error);
  Serial.print(" ");

  Serial.print("P:");
  Serial.print(PID_p);
  Serial.print(" ");

  Serial.print("I:");
  Serial.print(PID_i);
  Serial.print(" ");

  Serial.print("D:");
  Serial.print(PID_d);
  Serial.print(" ");

  Serial.print("Control:");
  Serial.print(move);
  Serial.print(" ");

  Serial.print("Cycles:");
  Serial.print(cycle_counter);
  Serial.print(" ");

  Serial.println();
}

