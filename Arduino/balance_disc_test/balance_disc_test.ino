// Balance Disc Test Script

// Libraries
#include <Wire.h>
#include <Servo.h>

// Objects
Servo myServo1;
Servo myServo2;

// Tuned parameters
int angle_level_1 = 0;
int angle_level_2 = 0;

// General variables
unsigned long time;

float freq = 1;
float amplitude = 60;
float phase = PI/2;

float angle_1;
float angle_2;

void setup() {
  // Serial setup
  Serial.begin(115200);

  // Servo setup
  myServo1.attach(3);
  myServo2.attach(6);
  myServo1.writeMicroseconds(deg2micro(angle_level_1));
  myServo2.writeMicroseconds(deg2micro(angle_level_2));

  delay(3000);

  time = millis();
}

void loop() {
  time = millis();

  //angle_1 = amplitude*sin(2*PI*freq*time/1000) + angle_level_1;
  //angle_2 = amplitude*sin(2*PI*freq*time/1000 + phase) + angle_level_2;

  myServo1.writeMicroseconds(deg2micro(angle_1));
  myServo2.writeMicroseconds(deg2micro(angle_2));

  print_data();
}

int deg2micro(float deg) {   // convert degrees to microseconds for servo function
  int result = (deg + 90) * 1000 / 180 + 1000;   // convert range of -90 to 90 degrees to 1000 to 2000 microseconds
  return constrain(result, 1000, 2000);   // keep result within safe limits
}

void print_data() {
  Serial.print("Time:");
  Serial.print(time);
  Serial.print(" ");

  Serial.print("Upper_Bound:");
  Serial.print(amplitude);
  Serial.print(" ");

  Serial.print("Lower_Bound:");
  Serial.print(-amplitude);
  Serial.print(" ");

  Serial.print("Angle_1:");
  Serial.print(angle_1);
  Serial.print(" ");

  Serial.print("Angle_2:");
  Serial.print(angle_2);
  Serial.print(" ");

  Serial.println("");
}