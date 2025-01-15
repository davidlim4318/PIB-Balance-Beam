#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;  // create servo object
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();  // pins: Vin to 3.3V, SCL to A5, SDA to A4

// Tunable parameters
int const micro_level = 1466;  // us (should be around 1500)
int const backlash = 15;  // us
int distance_setpoint = 133;  // mm

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

    if (count % period == count % (period/2)) {
      r = distance_setpoint + amplitude;
    }
    else {
      r = distance_setpoint - amplitude;
    }
    y = mySensor.readRange();
    e2 = e1;
    e1 = e;
    e = r - y;
    u2 = u1;
    u1 = u;
    u = e - 0.95*e1 + 0.78*u1;

    myServo.writeMicroseconds(deg2micro(u));

    print_data();

    count = count + 1;
  }
  while (count >= N) {
    myServo.writeMicroseconds(deg2micro(0));  // level the servo
    delay(1000);
  }
}

int result;
int result_prev;
int result_comp;

int deg2micro(float deg) {
  result_prev = result;
  result = deg * 1000 / 90 + micro_level;
  if (result > result_prev) {
    result_comp = result + backlash/2;
    
  }
  else if (result < result_prev) {
    result_comp = result - backlash/2;
  }
  return constrain(result_comp, 1000, 2000);
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