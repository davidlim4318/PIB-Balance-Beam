#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;  // create servo object
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();  // pins: Vin to 3.3V, SCL to A5, SDA to A4

// Tunable parameters
int angle_level = -2;  // deg (should be close to 0)
int distance_setpoint = 135;  // mm

// float filter_const = 200;  // ms
unsigned long N = 400;
unsigned long c;

int u;

void setup() {
  Serial.begin(115200);

  while (! Serial) {
    delay(1);
  }

  myServo.attach(6);  // servo to pin D6
  myServo.writeMicroseconds(deg2micro(angle_level));  // level the servo

  if (!mySensor.begin()) {
    Serial.println(F("Failed to boot VL53 L0X"));
    while (1);
  }
  mySensor.setMeasurementTimingBudgetMicroSeconds(20000);
  mySensor.startRangeContinuous();

  delay(1000);

}

// General variables
unsigned long time;
unsigned long time_prev;
unsigned long time_step;

int distance_raw;
// float distance_error;
// float distance_error_next;
// float distance_error_prev;

void loop() {
  if (mySensor.isRangeComplete()) {
    time_prev = time;
    delay(constrain(25 + time - millis(), 0, 5));
    time = millis();
    time_step = time - time_prev;

    distance_raw = mySensor.readRange();

    // distance_error_prev = distance_error;
    // distance_error = distance_error_next;
    // distance_error_next = (1 - time_step / filter_const) * distance_error + time_step / filter_const * (distance_setpoint - distance_raw);

    u = random(-15,15);
    myServo.writeMicroseconds(deg2micro(u + angle_level));  // level the servo

    print_data();
    c = c + 1;
  }
  if (c >= N) {
    myServo.writeMicroseconds(deg2micro(angle_level));  // level the servo
    Serial.println("Done!");
    while (1) {
      delay(1000);
    }
  }
}

int deg2micro(float deg) {
  int result = (deg + 90) * 1000 / 180 + 1000;
  return constrain(result, 1000, 2000);
}

void print_data() {
  // Serial.print("T:");
  Serial.print(time);
  Serial.print(" ");

  // Serial.print("y:");
  Serial.print(distance_raw);
  Serial.print(" ");

  Serial.print(u);
  Serial.print(" ");

  Serial.println();
}