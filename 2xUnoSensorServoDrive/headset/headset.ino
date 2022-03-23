#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_180MS, TCS34725_GAIN_60X);

//  TCS34725_GAIN_1X = 0x00,  /**<  No gain  */
//  TCS34725_GAIN_4X = 0x01,  /**<  4x gain  */
//  TCS34725_GAIN_16X = 0x02, /**<  16x gain */
//  TCS34725_GAIN_60X = 0x03  /**<  60x gain */

const int zeroSetJumperPin = 3; // global on purpose
float rZero, gZero, bZero;
float gbNeutralDifference;

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(zeroSetJumperPin, INPUT);
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  tcs.setInterrupt(true); // turn off LED - set to false to turn on LED

  delay(1000);

  float rRaw, gRaw, bRaw;
  tcs.getRGB(&rRaw, &gRaw, &bRaw); // read RGB values from the color sensor
  resetCenter(rRaw, gRaw, bRaw);
}

void loop() {
  const float gbDifferenceMax = 50;
  const float gbDifferenceMin = 50;
  const float forwardRMaxRange = 20;
  const float reverseRMaxRange = 20;

  static float rRaw, gRaw, bRaw;
  tcs.getRGB(&rRaw, &gRaw, &bRaw); // read RGB values from the color sensor

  bool resetIsPressed = 0;
  resetIsPressed = digitalRead(zeroSetJumperPin);
  if (resetIsPressed) {
    resetCenter(rRaw, gRaw, bRaw);
  }

  static float r = rRaw - rZero;
  static float g = gRaw - gZero;
  static float b = bRaw - bZero;
  static float gbDifference = gRaw - bRaw;

  float forwardSpeed;

  if (r > 0) {
    forwardSpeed = (-r) / (forwardRMaxRange);
  }
  else {
    forwardSpeed = (-r) / (reverseRMaxRange);
  }

  float turningRate;

  if ((gbDifference - gbNeutralDifference) > 0) {
    turningRate = (gbDifference - gbNeutralDifference) / (gbDifferenceMax);
  }
  else {
    turningRate = (gbDifference - gbNeutralDifference) / (gbDifferenceMin);
  }

  //Serial.print("R: "); Serial.print(rRaw, DEC); Serial.print(" ");
  //Serial.print("G: "); Serial.print(gRaw, DEC); Serial.print(" ");
  //Serial.print("B: "); Serial.print(bRaw, DEC); Serial.print(" ");
  Serial.print("Forward: "); Serial.print(forwardSpeed, DEC); Serial.print(" ");
  Serial.print("Turning: "); Serial.print(turningRate, DEC); Serial.print(" ");
  Serial.println(" ");

  /* Clip forwardSpeed and turningRate to vary between -1 and 1. To minimize this
  effect, forwardRMaxRange, reverseRMaxRange, gbDifferenceMax, and gbDifferenceMin
  should be adjusted */
  if (forwardSpeed > 1) {forwardSpeed = 1;}
  else if (forwardSpeed < -1) {forwardSpeed = -1;}
  if (turningRate > 1) {turningRate = 1;}
  else if (turningRate < -1) {turningRate = -1;}

  const float turningSensitivity = 0.25;

  static float rightMotorPower = forwardSpeed - (turningRate * turningSensitivity);
  static float leftMotorPower = forwardSpeed + (turningRate * turningSensitivity);

  const int servoTopSpeed = 100; // I believe true max speed is 200
  transmitMotorSpeeds(rightMotorPower*servoTopSpeed, leftMotorPower*servoTopSpeed);
}

void transmitMotorSpeeds(int right, int left) {
  Wire.beginTransmission(4); // transmit to device #4

  static String rightString;
  static String leftString;
  static String outputString;
  static char outputArray[9];

  //dtostrf(right, 4, 0, rightString);
  //dtostrf(left, 4, 0, leftString);
  rightString = String(right);
  leftString = String(left);
  outputString = rightString + ',' + leftString;
  outputString.toCharArray(outputArray, 9);

  Wire.write(outputArray);
  Wire.endTransmission();    // stop transmitting

  //delay(10);
}

void resetCenter(float rCenter, float gCenter, float bCenter) {
  rZero = rCenter;
  gZero = gCenter;
  bZero = bCenter;
  gbNeutralDifference = gZero - bZero;
}
