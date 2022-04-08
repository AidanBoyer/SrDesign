#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_180MS, TCS34725_GAIN_60X); // Initialize color sensor, set integration time and gain value

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
  resetCenter(rRaw, gRaw, bRaw); // set the initial RGB readings as the zero values
}

void loop() {
  const int gbDifferenceMax = 50; // (green-blue) value range in the positive direction that corresponds to full turning right
  const int gbDifferenceMin = 30; // (green-blue) value range in the negative direction that corresponds to full turning left
  const int forwardRMaxRange = 30; // maximum positive red value that corresponds to full forward
  const int reverseRMaxRange = 30; // maximum negative red value that corresponds to full reverse
  const int luminanceCutoff = 30; // luminance value below which the motors shut down
  const int rDeadZone = 4; // width of the dead zone for the red value
  const int gbDeadZone = 6; // width of the dead zone for the (green-blue) value

  static float rRaw, gRaw, bRaw;
  tcs.getRGB(&rRaw, &gRaw, &bRaw); // read RGB values from the color sensor

  // If the reset pin is powered, set the RGB zero values to the last read values
  bool resetIsPressed = 0;
  resetIsPressed = digitalRead(zeroSetJumperPin);
  if (resetIsPressed) {
    resetCenter(rRaw, gRaw, bRaw); // If the jumper pin is connected to 5V power, reset the zero point
  }

  // r, g, and b are the difference between currently read value and the zero values
  static float r, g, b, lux;
  r = rRaw - rZero;
  g = gRaw - gZero;
  b = bRaw - bZero;
  float gbDifference = gRaw - bRaw;
  lux = tcs.calculateLux(rRaw, gRaw, bRaw); // calculates luminance

  // If the luminance is below the threshold, reset zero position
  /*if (lux < luminanceCutoff) {
    resetCenter(rRaw, gRaw, bRaw);
  }*/
  
  float forwardSpeed = 0;

  if (r > rDeadZone) { // if the r value is outside of the dead zone in the positive direction
    forwardSpeed = ((-1)*(r-rDeadZone)) / (forwardRMaxRange);
  }
  else if (r < (-1*rDeadZone)) { // if the r value is outside of the dead zone in the negative direction
    forwardSpeed = ((-1)*(r+rDeadZone)) / (reverseRMaxRange);
  }
  // note that negative r corresponds to forward movement

  float turningRate = 0;

  if ((gbDifference - gbNeutralDifference) > gbDeadZone) {
    // if the difference between the current (green-blue) value and the zero (green-blue) value is outside of the (green-blue) dead zone in the positive direction
    turningRate = ((gbDifference - gbNeutralDifference)-gbDeadZone) / (gbDifferenceMax);
  }
  else if ((gbDifference - gbNeutralDifference) < (-1*gbDeadZone)){
    // if the difference between the current (green-blue) value and the zero (green-blue) value is outside of the (green-blue) dead zone in the negative direction
    turningRate = ((gbDifference - gbNeutralDifference)+gbDeadZone) / (gbDifferenceMin);
  }

  // print the current r,g,b,lux, calculated forward speed and calculated turning rate to the USB serial monitor
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" ");
  Serial.print("Forward: "); Serial.print(forwardSpeed, DEC); Serial.print(" ");
  Serial.print("Turning: "); Serial.print(turningRate, DEC); Serial.print(" ");
  Serial.println(" ");

  // Clip forwardSpeed and turningRate to vary between -1 and 1. To minimize this
  // effect, forwardRMaxRange, reverseRMaxRange, gbDifferenceMax, and gbDifferenceMin should be adjusted
  if (forwardSpeed > 1) {forwardSpeed = 1;}
  else if (forwardSpeed < -1) {forwardSpeed = -1;}
  if (turningRate > 1) {turningRate = 1;}
  else if (turningRate < -1) {turningRate = -1;}

  const float turningSensitivity = 0.50; // adjust this value to set how turning aggressiveness

  static float rightMotorPower;
  static float leftMotorPower;
  // translates forwardSpeed and turningRate into right and left motor power values, assuming a skid-steer or tank drive system
  rightMotorPower = forwardSpeed - (turningRate * turningSensitivity);
  leftMotorPower = forwardSpeed + (turningRate * turningSensitivity);

  const int servoTopSpeed = 100; // Sets the servo speed value that corresponds to 1 or maximum (full power is 200)
  transmitMotorSpeeds(rightMotorPower*servoTopSpeed, leftMotorPower*servoTopSpeed); // passes the scaled right and left motor power values to the rover arduino
}

void transmitMotorSpeeds(int right, int left) {
  Wire.beginTransmission(4); // transmit to device address 4 (rover arduino I2C is initialized as address 4)

  static String rightString;
  static String leftString;
  static String outputString;
  static char outputArray[9]; // 9 characters to allow for signed 3-digit motor power values and a delineator character

  //dtostrf(right, 4, 0, rightString);
  //dtostrf(left, 4, 0, leftString);
  rightString = String(right); // convert the right and left motor power values to a string datatype
  leftString = String(left);
  outputString = rightString + ',' + leftString; // concatenates the two motor power values with a comma as a delineator
  outputString.toCharArray(outputArray, 9); // converts this string to a character array with 9 elements

  Wire.write(outputArray); // transmit the output character array over the I2C bus
  Wire.endTransmission(); // close the I2C bus

  //delay(10);
}

void resetCenter(float rCenter, float gCenter, float bCenter) {
  rZero = rCenter;
  gZero = gCenter;
  bZero = bCenter;
  gbNeutralDifference = gZero - bZero;
}
