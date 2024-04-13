#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;   // create servo object to control a servo, later attatched to D9
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();

// Tunable parameters
int angle_level = -11;   // deg, should be close to 0
int distance_setpoint = 112;   // mm

// General variables
unsigned long time;
unsigned long time_prev;
unsigned long time_step;

int distance_raw;
float distance_error0;
float distance_error1;
float distance_error;

float control0;
float control1;
float control;

float K =   1.999608; // 0.6541;
float a0 =  0.346203; // 0.6778;
float a1 = -1.346156; // -1.678;
float b0 =  0.988007; // 0.9981;
float b1 = -1.987960; // -1.998;
float c0 = 0.2865;

void setup() {
  Serial.begin(31250);

  while (! Serial) {
    delay(1);
  }

  myServo.attach(6);   // attaches the servo on pin 6 to the servo object
  myServo.writeMicroseconds(deg2micro(angle_level));   // put the servco at an angle, so the balance is in the middle

  if (!mySensor.begin()) {
    Serial.println(F("Failed to boot VL53 L0X"));
    while (1);
  }
  mySensor.setMeasurementTimingBudgetMicroSeconds(20000);
  mySensor.startRangeContinuous();
  
  /*
  while (! mySensor.isRangeComplete()) {
    delay(1);
  }
  time = millis();
  distance_raw = mySensor.readRange();
  distance_error = distance_setpoint - distance_raw;
  distance_error_next = distance_setpoint - distance_raw;
  */
}

void loop() {
  if (mySensor.isRangeComplete()) {
    time_prev = time;
    delay(constrain(25 - (millis() - time), 0, 5));
    time = millis();
    time_step = time - time_prev;

    distance_raw = mySensor.readRange();

    myServo.writeMicroseconds(deg2micro(control + angle_level));

    print_data();

    distance_error0 = distance_error1;
    distance_error1 = distance_error;
    distance_error = (1 - c0) * (distance_setpoint - distance_raw) + c0 * distance_error1;

    control0 = control1;
    control1 = control;
    control = K * ( b0 * distance_error0 + b1 * distance_error1 + distance_error ) - a0 * control0 - a1 * control1;

  }
}

int deg2micro(float deg) {
  int result = (deg + 90) * 1000 / 180 + 1000;
  return constrain(result, 1000, 2000);
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

  Serial.print("Control:");
  Serial.print(control);
  Serial.print(" ");

  Serial.println();
}