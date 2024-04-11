#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;   // create servo object to control a servo, later attatched to D9
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();

// Tunable parameters
int angle_level = -11;   // deg, should be close to 0
int distance_setpoint = 112;   // mm

float K_P = 0.1;
float K_I = 0.002;
float K_D = 0.055;

float filter_const = 50;   // ms

// General variables
unsigned long time;
unsigned long time_prev;
unsigned long time_step;

int distance_raw;
float distance_error;
float distance_error_next;
float distance_error_prev;

float control_P;
float control_I;
float control_D;
float control_total;

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
  
  while (! mySensor.isRangeComplete()) {
    delay(1);
  }
  time = millis();
  distance_raw = mySensor.readRange();
  distance_error = distance_setpoint - distance_raw;
  distance_error_next = distance_setpoint - distance_raw;
}

void loop() {
  if (mySensor.isRangeComplete()) {
    time_prev = time;
    delay(constrain(25 - (millis() - time), 0, 5));
    time = millis();
    time_step = time - time_prev;

    distance_raw = mySensor.readRange();

    distance_error_prev = distance_error;
    distance_error = distance_error_next;
    distance_error_next = (1 - time_step / filter_const) * distance_error + time_step / filter_const * (distance_setpoint - distance_raw);

    control_P = K_P * distance_error;
    control_I = control_I + K_I * distance_error * time_step / 1000;
    control_D = K_D * (distance_error_next - distance_error_prev) * 1000 / time_step / 2;

    control_total = control_P + control_I + control_D;

    myServo.writeMicroseconds(deg2micro(control_total + angle_level));

    print_data();

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

  Serial.println();
}