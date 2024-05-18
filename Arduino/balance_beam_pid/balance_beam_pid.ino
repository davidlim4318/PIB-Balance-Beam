// Balance Beam Proportional-Integral-Derivative (PID) Controller Script

// Libraries
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();

// Tuned parameters
int angle_level = -23;   // level angle in degrees, should be close to 0
int distance_setpoint = 107;   // (mm)

int error_limit = 30;   // acceptable distance error (mm)
int cycle_limit = 60;   // minimum number of consecutive loop cycles within error limit to be considered "stable"

float K_P = 0.10;   // proportional coefficient
float K_I = 0.01;   // integral coefficient
float K_D = 0.06;   // derivative coefficient

float filter_const = 25;   // (ms)

// General variables
unsigned long time;   // current time (mm)
unsigned long time_prev;   // previous time, used to calculate time step
unsigned long time_step;   // period of one loop cycle (ms)

int distance_raw;   // sensor output (mm)
float distance_error;   // current distance error (mm)
float distance_error_next;   // next calculated distance error (mm), output of low-pass filter
float distance_error_prev;   // previous calculated distance error (mm)

float control_P;   // proportional control action
float control_I;   // integral control action
float control_D;   // derivative control action
float control_total;   // total control action

int cycle_counter;   // number of consecutive loop cycles within error limit

void setup() {
  // Serial setup
  Serial.begin(115200);
  while (! Serial) {
    delay(1);   // wait for serial communication to intialize
  }

  // Servo setup
  myServo.attach(5);   // assign servo signal output to pin 6
  myServo.writeMicroseconds(deg2micro(angle_level));   // rotate the servo to the level angle

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
  distance_error = distance_setpoint - distance_raw;   // initialize current distance error
  distance_error_next = distance_setpoint - distance_raw;   // intialize next distance error
}

void loop() {
  if (mySensor.isRangeComplete()) {   // run calculations when distance measurement is updated  
    time_prev = time;   // get previous time
    delay(constrain(25 - (millis() - time), 0, 5));   // delay loop cycle to at least 25 ms
    time = millis();   // get current time
    time_step = time - time_prev;   // compute time step

    distance_raw = mySensor.readRange();   // get distance measurement

    distance_error_prev = distance_error;   // get previous distance error
    distance_error = distance_error_next;   // get current distance error
    distance_error_next = (1 - time_step / filter_const) * distance_error + time_step / filter_const * (distance_setpoint - distance_raw);
      // low-pass filter the error to get the next distance error

    if (cycle_counter <= cycle_limit) {   // run if distance has not stabilized within error limit and time limit
      control_P = K_P * distance_error;   // compute proportional control action
      control_I = control_I + K_I * distance_error * time_step / 1000;   // compute integral control action
      control_D = K_D * (distance_error_next - distance_error_prev) * 1000 / time_step / 2;   // compute derivative control action with central difference approximation
    }
    else {
      control_P = 0;   // set proportional control action to zero
      control_D = 0;   // set derivatve control action to zero
        // also keep integral control action constant
    }
    control_total = control_P + control_I + control_D;   // compute total control action
    myServo.writeMicroseconds(deg2micro(control_total + angle_level));   // set servo angle

    print_data();   // print data

    if ( abs( distance_error ) <= error_limit ) {   // run if distance error is within error limit
      if (cycle_counter <= cycle_limit) {
        cycle_counter++;   // increase loop cycle counter if below loop cycle limit
      }
    }
    else {
      cycle_counter = 0;   // if distance error outside error limit, reset loop cycle counter
    }
  }
}

int deg2micro(float deg) {   // convert degrees to microseconds for servo function
  int result = (deg + 90) * 1000 / 180 + 1000;   // convert range of -90 to 90 degrees to 1000 to 2000 microseconds
  return constrain(result, 1000, 2000);   // keep result within safe limits
}

void print_data() {
  Serial.print("Step:");
  Serial.print(time_step);
  Serial.print(" ");

  Serial.print("Raw:");
  Serial.print(distance_raw);
  Serial.print(" ");

  Serial.print("Error:");
  Serial.print(distance_error);
  Serial.print(" ");

  Serial.print("P:");
  Serial.print(control_P);
  Serial.print(" ");

  Serial.print("I:");
  Serial.print(control_I);
  Serial.print(" ");

  Serial.print("D:");
  Serial.print(control_D);
  Serial.print(" ");

  Serial.print("Control:");
  Serial.print(control_total);
  Serial.print(" ");

  Serial.print("Cycles:");
  Serial.print(cycle_counter);
  Serial.print(" ");

  Serial.println();
}