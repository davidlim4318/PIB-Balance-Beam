#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;  // create servo object

// Tunable parameters
int micro_level = 1462;  // us (should be around 1500)

void setup() {
  Serial.begin(115200);

  while (! Serial) {
    delay(1);
  }

  myServo.attach(6);  // servo to pin D6
  myServo.writeMicroseconds(micro2micro(0));  // level the servo

  delay(10000);
  Serial.println("Begin.");
}

// General variables
int u;
int inc = 1;
int const pause = 250;
int const limit = 60;
int const backlash = 15;

void loop() {
  u = u + inc;
  if (u >= limit) {
    inc = -1;
  }
  else if (u <= -limit) {
    inc = 1;
  }
  step();
}

int result;
int result_prev;
int result_comp;

int micro2micro(int input) {
  result_prev = result;
  result = input + micro_level;
  if (result > result_prev) {
    result_comp = result + backlash / 2;
  }
  else if (result < result_prev) {
    result_comp = result - backlash / 2;
  }
  return constrain(result_comp, 1000, 2000);
}

void step() {
  delay(pause);
  myServo.writeMicroseconds(micro2micro(u));
  Serial.print(u);
  Serial.print(" ");
  Serial.print(result_comp-micro_level);
  Serial.println(" ");
}