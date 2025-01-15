#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;  // create servo object
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();  // pins: Vin to 3.3V, SCL to A5, SDA to A4

// Tunable parameters
int const micro_level = 1466;  // us (should be around 1500)
int distance_setpoint = 137;  // mm

float filter_const = 1;  // mu s

void setup() {
  Serial.begin(115200);

  while (! Serial) {
    delay(1);
  }

  myServo.attach(6);  // servo to pin D6
  myServo.writeMicroseconds(deg2micro(0));  // level the servo

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
    time_prev = time;
    delayMicroseconds(constrain(25000 + time - micros(), 0, 5000));
    time = micros();
    time_step = time - time_prev;

    distance_raw = mySensor.readRange() - distance_setpoint;

    distance_error_prev = distance_error;
    distance_error = distance_error_next;
    distance_error_next = (1 - time_step / 1000000. / filter_const) * distance_error - time_step / 1000000. / filter_const * distance_raw;

    print_data();

  }
}

int deg2micro(float deg) {
  int result = deg * 1000 / 90 + micro_level;
  return constrain(result, 1000, 2000);
}

void print_data() {
  Serial.print("Ts:");
  Serial.print(time_step);
  Serial.print(" ");

  Serial.print("y:");
  Serial.print(distance_raw);
  Serial.print(" ");

  Serial.print("e:");
  Serial.print(distance_error);
  Serial.print(" ");

  Serial.println();
}