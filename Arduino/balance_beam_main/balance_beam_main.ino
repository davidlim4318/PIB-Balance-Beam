#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;   // create servo object to control a servo, later attatched to D9
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();

// Tunable parameters
int angle_level = 1445;   // between 1000 (about -90 deg) to 2000 (about 90 deg)
int distance_setpoint = 120;   // mm

float K_P = 0.04;
float K_I = 0;
float K_D = 0;

float filter_const = 100;   // ms

void setup() {
  Serial.begin(19200);

  while (! Serial) {
    delay(1);
  }

  myServo.attach(6);   // attaches the servo on pin 6 to the servo object
  myServo.write(90);   // put the servco at an angle, so the balance is in the middle

  if (!mySensor.begin()) {
    Serial.println(F("Failed to boot VL53 L0X"));
    while (1);
  }
  mySensor.setMeasurementTimingBudgetMicroSeconds(20000);
  mySensor.startRangeContinuous();

}

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

void loop() {
  if (mySensor.isRangeComplete()) {
    distance_raw = mySensor.readRange();

    time_prev = time;
    time = millis();
    time_step = time - time_prev;

    distance_error_prev = distance_error;
    distance_error = distance_error_next;
    distance_error_next = (1 - time_step / filter_const) * distance_error + time_step / filter_const * (distance_setpoint - distance_raw);

    control_P = K_P * distance_error;
    control_I = control_I + K_I * distance_error * time_step;
    control_D = K_D * (distance_error_next - distance_error_prev) / time_step / 2.0;

    control_total = control_P + control_I + control_D;

    print_data();

  }
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
/*
  Serial.print("P:");
  Serial.print(control_P);
  Serial.print(" ");

  Serial.print("I:");
  Serial.print(control_I);
  Serial.print(" ");

  Serial.print("D:");
  Serial.print(control_D);
  Serial.print(" ");
  */

  Serial.print("Control:");
  Serial.print(control_total);
  Serial.print(" ");

  Serial.println();
}