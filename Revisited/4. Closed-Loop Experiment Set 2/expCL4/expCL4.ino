#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;  // create servo object
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();  // pins: Vin to 3.3V, SCL to A5, SDA to A4

// Tunable parameters
int const micro_level = 1466;  // us (should be around 1500)
int distance_setpoint = 137;  // mm

int amplitude = 30;
int N = 1200;
int period = 600;

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
  delay(10000);
}

// General variables
unsigned long time;
unsigned long time_prev;
unsigned long time_step;

int r;
int y;
int e;
int e1;
float u;
float u1;

int count;

void loop() {
  if (mySensor.isRangeComplete()) {
    time_prev = time;
    delayMicroseconds(constrain(25000 + time - micros(), 0, 5000));
    time = micros();
    time_step = time - time_prev;

    if (count % period == count % (period/2)) {
      r = amplitude;
    }
    else {
      r = -amplitude;
    }
    y = mySensor.readRange() - distance_setpoint;
    e1 = e;
    e = r - y;
    u1 = u;
    u = e - 0.97*e1 + 0.5*u1;

    myServo.writeMicroseconds(deg2micro(u));

    print_data();

    count = count + 1;
  }
  while (count >= N) {
    myServo.writeMicroseconds(deg2micro(0));  // level the servo
    delay(1000);
  }
}

int deg2micro(float deg) {
  int result = deg * 1000 / 90.0 + micro_level;
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