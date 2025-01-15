#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();

int distance_raw;

void setup() {
  Serial.begin(115200);
  while (! Serial) {
    delay(1);   // wait for serial communication to intialize
  }
  if (!mySensor.begin()) {   // check for sensor intialization
    Serial.println(F("Failed to boot VL53 L0X. Please reset."));
    while (1);
  }
  mySensor.setMeasurementTimingBudgetMicroSeconds(20000);   // set sensor timing budget to 20 ms
  mySensor.startRangeContinuous();   // intialize "continuous" sensor output
  while (! mySensor.isRangeComplete()) {
    delay(1);   // wait for first distance measurement
  }
}

void loop() {
  if (mySensor.isRangeComplete()) {
    distance_raw = mySensor.readRange();   // get distance measurement
  }
  print_data();   // print data
}

void print_data() {
  Serial.print("Distance:");
  Serial.print(distance_raw);
  Serial.print(" ");

  Serial.print("Upper_Bound:");
  Serial.print(1000);
  Serial.print(" ");

  Serial.print("Lower_Bound:");
  Serial.print(0);
  Serial.print(" ");

  Serial.println(" ");
}