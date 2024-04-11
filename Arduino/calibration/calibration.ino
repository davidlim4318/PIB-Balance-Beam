#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;   // create servo object to control a servo, later attatched to D9
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();

// Tunable parameters
int angle_level = -10;   // deg, should be close to 0
int distance_setpoint = 115;   // mm

float filter_const = 100;   // ms

void setup() {
  Serial.begin(19200);

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

}

// General variables
unsigned long time;
unsigned long time_prev;
unsigned long time_step;

int distance_raw;
float distance_error;
float distance_error_next;
float distance_error_prev;

void loop() {
  if (mySensor.isRangeComplete()) {
    distance_raw = mySensor.readRange();

    time_prev = time;
    time = millis();
    time_step = time - time_prev;

    distance_error_prev = distance_error;
    distance_error = distance_error_next;
    distance_error_next = (1 - time_step / filter_const) * distance_error + time_step / filter_const * (distance_setpoint - distance_raw);

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

  Serial.println();
}