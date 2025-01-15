#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Objects
Servo myServo;  // create servo object
Adafruit_VL53L0X mySensor = Adafruit_VL53L0X();  // pins: Vin to 3.3V, SCL to A5, SDA to A4

// Pre-calibrated parameters
int const micro_level = 1466;  // us (should be around 1500)
int const distance_setpoint = 137;  // mm

// Reference signal parameters
int amplitude = 30;
int N = 1200;
int period = 600;

void setup() {
  Serial.begin(115200);

  while (! Serial) delay(1);

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
float u;
int y;
float ypred;
int e;

const int n = 3;  // maximum filter order
float U[n + 1];
int Y[n + 1];

const int nP = 3;  // order of filter 1
float Ypred[nP + 1];

const int nF = 1;  // order of controller filter
int E[nF + 1];

int count;

void loop() {
  if (mySensor.isRangeComplete()) {
    time_prev = time;
    delayMicroseconds(constrain(25000 + time - micros(), 0, 5000));
    time = micros();
    time_step = time - time_prev;

    if (count % period == count % (period/2)) r = amplitude;
    else r = -amplitude;

    shiftBuffer();

    y = mySensor.readRange() - distance_setpoint;
    Y[0] = y;

    ypred = predictionFilter();
    Ypred[0] = ypred;

    e = r - ypred;
    E[0] = e;

    u = controllerFilter();
    U[0] = u;

    myServo.writeMicroseconds(deg2micro(u));

    print_data();

    count = count + 1;
  }
  while (count >= N) {
    myServo.writeMicroseconds(deg2micro(0));  // level the servo
    delay(1000);
  }
}

void shiftBuffer() {
  for (int i = n; i > 0; i--) {
    Y[i] = Y[i - 1];
    U[i] = U[i - 1];
  }
  for (int i = nP; i > 0; i--) Ypred[i] = Ypred[i - 1];
  for (int i = nF; i > 0; i--) E[i] = E[i - 1];
}

float bP1[nP + 1] = {0, 0, 0.033931599523769, -0.016965799761884};
float bP2[nP + 1] = {0, 0.620356131558136, -0.778419064623254, 0.173746325929788};
float aP[nP + 1] = {1, -1.859817866121445, 1.191841931896118, -0.316340672910003};
float predictionFilter() {
  float output = 0;
  for (int i = 0; i <= nP; i++) output += bP1[i] * U[i] + bP2[i] * Y[i];
  for (int i = 1; i <= nP; i++) output -= aP[i] * Ypred[i];
  return output;
}

float bF[nF + 1] = {1, -0.97};
float aF[nF + 1] = {1, -0.5};
float controllerFilter() {
  float output = 0;
  for (int i = 0; i <= nF; i++) output += bF[i] * E[i];
  for (int i = 1; i <= nF; i++) output -= aF[i] * U[i];
  return output;
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

  Serial.print(ypred);
  Serial.print(" ");

  Serial.println();
}