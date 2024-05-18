#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>

Servo myservo;
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();

double kp = 0;
double ki = 0;
double kd = 0;

float distance_setpoint = 116;
float levang = 90;

int error_limit = 30;  // acceptable distance error (mm)
int cycle_limit = 60;  // minimum number of consecutive loop cycles within error limit to be considered "stable"
int cycle_counter;     // number of consecutive loop cycles within error limit


const int numReadings = 10;

int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

float distance_raw;  // sensor output (mm)
float distance_previous_error, distance_error;
long period = 0;

float PID_p;      //proportional control action
float PID_i;      //integral control action
float PID_d;      //derivative control action
float PID_total;  //total control action

float move;
unsigned long time;       // current time (mm)
unsigned long time_prev;  // previous time, used to calculate time step
unsigned long time_step;  // period of one loop cycle (ms)

long duration;



//NEW VARIABLES USED

//RGB manipulation pins
int redPin = 9;
int greenPin = 10;
int bluePin = 11;

//"index" of the calibration modes
int index = 0;

//Array to see which state the circuit is in while running
String states[5] = { "Inactive", "Servo", "Sensor", "PID", "Test" };

void setup() {

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // Serial setup
  Serial.begin(115200);

  // Servo setup
  myservo.attach(6);
  myservo.write(levang);

  //Sensor setup
  if (!mySensor.begin()) {  // check for sensor intialization
    Serial.println(F("Failed to boot VL53 L0X. Please reset."));
    while (1)
      ;
  }

  mySensor.setMeasurementTimingBudgetMicroSeconds(20000);  // set sensor timing budget to 20 ms
  mySensor.startRangeContinuous();                         // intialize "continuous" sensor output

  // Control algorithm setup
  while (!mySensor.isRangeComplete()) {
    delay(1);  // wait for first distance measurement
  }
  time = millis();                      // get current time
  distance_raw = mySensor.readRange();  // get distance measurement

  //Button setup
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  //RGB setup
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  changeState();
  if (index == 0) {
    Serial.println(states[index]);
    setColor(0, 0, 0);
  }

  //servo calibration
  //adjust value with buttons
  //set angle from adjusting to "levang"

  if (index == 1) {

    //remove in final code
    Serial.print(states[index]);
    Serial.print(": ");
    Serial.print(digitalRead(3));
    Serial.print(" ");
    Serial.print(digitalRead(4));
    Serial.println(" ");
    Serial.print("Angle: ");
    //up to here

    calibrateServo();
    setColor(255, 0, 0);
  }

  //sensor calibration
  //get value using distance sensor
  //then set it to "distance_setpoint"

  if (index == 2) {
    Serial.println(states[index]);
    setColor(0, 255, 0);
    distance_setpoint = mySensor.readRange();
    Serial.println(distance_setpoint);
  }

  //PID calibration
  //read potentiometer values
  //maps them to smaller values

  if (index == 3) {
    Serial.println(states[index]);
    setColor(0, 0, 255);

    //mapping potentiometer readings
    //from 0-100
    kp = map(analogRead(A2), 10, 1023, 0, 100);
    ki = map(analogRead(A1), 10, 1023, 0, 100);
    kd = map(analogRead(A0), 10, 1023, 0, 100);
    kp = kp / 1000;
    ki = ki / 100000;

    //place main code here
    // run calculations when distance measurement is updated


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

    time_prev = time;                                // get previous time
    delay(constrain(25 - (millis() - time), 0, 5));  // delay loop cycle to at least 25 ms
    time = millis();                                 // get current time
    period = time - time_prev;                       // compute time step

    //distance_raw = mySensor.readRange();             // get distance measurement
    distance_raw = constrain(distance_raw, 10, 300);    // create approximate lower and upper ends for distance
    distance_error = distance_setpoint - distance_raw;  // obtaining error for PID calculations

    /*Implementing PID feedback on the error*/
    PID_p = kp * distance_error;
    PID_d = kd * ((distance_error - distance_previous_error) / period);
    PID_i = PID_i + (ki * distance_error);



    ////////// Permanent Stability Condition /////////////

    if (cycle_counter >= cycle_limit) {       // run if distance has not stabilized within error limit and time limit
      PID_p = 0;                              // set proportional control action to zero
      PID_d = 0;                              // set derivatve control action to zero
      PID_i = PID_i - (ki * distance_error);  // also keep integral control action constant
      cycle_counter = cycle_limit;
    }

    if (abs(distance_error) <= error_limit) {  // run if distance error is within error limit
      cycle_counter++;                         // increase loop cycle counter if below loop cycle limit
    } else {
      cycle_counter = 0;  // if distance error outside error limit, reset loop cycle counter
    }

    ///////////////////////////////////////////////////////

    PID_total = PID_p + PID_i + PID_d;

    move = levang + PID_total;                         // get the new servo-angle
    move = constrain(move, levang - 40, levang + 40);  // ensure the new angle does not cause drastic shifts
    myservo.write(move);

    distance_previous_error = distance_error;  // store as previous error for ID calculations


    //printing out raw potentiometer values
    //printing out mapped potentiometer values
    Serial.print(analogRead(A2));
    Serial.print(" ");
    Serial.print(analogRead(A1));
    Serial.print(" ");
    Serial.print(analogRead(A0));
    Serial.println(" ");
    Serial.print(kp, 4);
    Serial.print(" ");
    Serial.print(ki, 5);
    Serial.print(" ");
    Serial.print(kd);
    Serial.println(" ");
  }
  if (index == 4) {
    myservo.write(levang);
    setColor(255, 255, 255);
    Serial.print("Servo Angle: ");
    Serial.println(levang);
    Serial.print("Distance: ");
    Serial.println(distance_setpoint);
    Serial.print("PID: ");
    Serial.print("kp: ");
    Serial.print(kp);
    Serial.print(" ");
    Serial.print("ki: ");
    Serial.print(ki);
    Serial.print(" ");
    Serial.print("kd: ");
    Serial.print(kd);
    Serial.println(" ");
  }
}

//increases "index" each time button is pressed
//loops back to "0" state after state 3
void changeState() {

  //check if button is pressed
  //and if any other button is being pressed at the same time
  if (digitalRead(2) == 0 && (digitalRead(2) != digitalRead(3) || digitalRead(2) != digitalRead(4))) {
    index++;
    if (index > 4) {
      index = 3;
    }
    delay(500);
  }
}

//function to set rgb colors
//so we don't need to analogWrite
//each one manually
void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

//function to calibrate servo
void calibrateServo() {

  //check if both buttons are pressed or not pressed
  //does not allow both buttons to be pressed
  if (digitalRead(3) != digitalRead(4)) {

    //check which button is being pressed
    //limits the manual levang calibration
    //to a certain range(+ or - 35)
    if (digitalRead(3) == 0 && levang < 90 + 45) {
      levang++;
      myservo.write(levang);
      delay(100);
    }
    if (digitalRead(4) == 0 && levang > 90 - 45) {
      levang--;
      myservo.write(levang);
      delay(100);
    }
  }
  Serial.println(levang);
}
