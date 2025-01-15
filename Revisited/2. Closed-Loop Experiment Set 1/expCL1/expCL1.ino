#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;  // create servo object
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();  // pins: Vin to 3.3V, SCL to A5, SDA to A4

// Tunable parameters
float angle_level = -6;  // deg (should be close to 0)
int distance_setpoint = 135;  // mm

int amplitude = 10;
int N = 2400;

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
  delay(10000);
}

// General variables
unsigned long time;
unsigned long time_prev;
unsigned long time_step;

int r = distance_setpoint;
int y;
int e;
int e1;
int e2;
float u;
float u1;
float u2;

int count;

void loop() {
  if (mySensor.isRangeComplete()) {
    time_prev = time;
    delayMicroseconds(constrain(25000 + time - micros(), 0, 5000));
    time = micros();
    time_step = time - time_prev;

    r = distance_setpoint + amplitude*(2*random(0,2)-1);
    y = mySensor.readRange();
    e2 = e1;
    e1 = e;
    e = r - y;
    u2 = u1;
    u1 = u;
    u = e - 1.9*e1 + 0.9*e2 + 1.099*u1 - 0.0999*u2;

    myServo.writeMicroseconds(deg2micro(u + angle_level));

    print_data();

    count = count + 1;
  }
  while (count >= N) {
    myServo.writeMicroseconds(deg2micro(angle_level));  // level the servo
    delay(1000);
  }
}

int deg2micro(float deg) {
  int result = (deg + 90) * 1000 / 180 + 1000;
  return constrain(result, 1000, 2000);
}

void print_data() {
  Serial.print(time);
  Serial.print(" ");

  Serial.print(r);
  Serial.print(" ");

  Serial.print(y);
  Serial.print(" ");

  Serial.print(u);
  Serial.print(" ");

  Serial.println();
}