#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_180MS, TCS34725_GAIN_60X);

//  TCS34725_GAIN_1X = 0x00,  /**<  No gain  */
//  TCS34725_GAIN_4X = 0x01,  /**<  4x gain  */
//  TCS34725_GAIN_16X = 0x02, /**<  16x gain */
//  TCS34725_GAIN_60X = 0x03  /**<  60x gain */

// Variable and constant definitions
int zeroSetJumperPin = 3;
int resetIsPressed = 0;
int rightMotorPowerSignalPin = 5;
int leftMotorPowerSignalPin = 6;
float rRaw, gRaw, bRaw;
float rZero, gZero, bZero;
float r, g, b;
float gbDifference;
float gbNeutralDifference;
float gbDifferenceMax = 50;
float gbDifferenceMin = 50;
float forwardRMaxRange = 20;
float reverseRMaxRange = 20;
float forwardSpeed;
float turningRate;
float rightMotorPower;
float leftMotorPower;
float turningSensitivity = 0.25;

void setup(void) {
  pinMode(zeroSetJumperPin, INPUT);
  //pinMode(rightMotorPowerSignalPin, OUTPUT);
  //pinMode(leftMotorPowerSignalPin, OUTPUT);

  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  //tcs.setInterrupt(false);  // turn on LED
  tcs.setInterrupt(true); // turn off LED

  delay(1000);
  tcs.getRGB(&rRaw, &gRaw, &bRaw); // read RGB values from the color sensor
  resetCenter(rRaw, gRaw, bRaw);
}

void loop(void) {
  tcs.getRGB(&rRaw, &gRaw, &bRaw); // read RGB values from the color sensor

  resetIsPressed = digitalRead(zeroSetJumperPin);
  if (resetIsPressed) {resetCenter(rRaw, gRaw, bRaw);}

  r = rRaw - rZero;
  g = gRaw - gZero;
  b = bRaw - bZero;
  gbDifference = gRaw - bRaw;
  
  if (r > 0) {
    forwardSpeed = (-r)/(forwardRMaxRange);
  }
  else {
    forwardSpeed = (-r)/(reverseRMaxRange);
  }

  if ((gbDifference - gbNeutralDifference) > 0) {
    turningRate = (gbDifference - gbNeutralDifference)/(gbDifferenceMax);
  }
  else {
    turningRate = (gbDifference - gbNeutralDifference)/(gbDifferenceMin);
  }

  //Serial.print("R: "); Serial.print(rRaw, DEC); Serial.print(" ");
  //Serial.print("G: "); Serial.print(gRaw, DEC); Serial.print(" ");
  //Serial.print("B: "); Serial.print(bRaw, DEC); Serial.print(" ");
  Serial.print("Forward: "); Serial.print(forwardSpeed, DEC); Serial.print(" ");
  Serial.print("Turning: "); Serial.print(turningRate, DEC); Serial.print(" ");
  Serial.println(" ");
  

  //transmitDrivingInstructions(forwardSpeed, turningRate);
}

void resetCenter(float rCenter, float gCenter, float bCenter) {
  rZero = rCenter;
  gZero = gCenter;
  bZero = bCenter;
  gbNeutralDifference = gZero - bZero;
}

void transmitDrivingInstructions(float forward, float turning) {
  // This function assumes forwardSpeed and turningRate vary between -1 and 1
  rightMotorPower = forwardSpeed - (turningRate * turningSensitivity);
  leftMotorPower = forwardSpeed + (turningRate * turningSensitivity);
  //analogWrite(128*(1+rightMotorPowerSignalPin), rightMotorPower);
  //analogWrite(128*(1+leftMotorPowerSignalPin), leftMotorPower);
}
