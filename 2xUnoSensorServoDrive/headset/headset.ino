#include <Wire.h>
#include "Adafruit_TCS34725.h"

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


  //transmitDrivingInstructions(forwardSpeed, turningRate);
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

void transmitDrivingInstructions(float forward, float turning) {
  // This function assumes forwardSpeed and turningRate vary between -1 and 1
  const float turningSensitivity = 0.25;
  
  int rightMotorPowerSignalPin = 5;
  int leftMotorPowerSignalPin = 6;

  static float rightMotorPower = forward - (turning * turningSensitivity);
  static float leftMotorPower = forward + (turning * turningSensitivity);
  //analogWrite(128*(1+rightMotorPowerSignalPin), rightMotorPower);
  //analogWrite(128*(1+leftMotorPowerSignalPin), leftMotorPower);
}
