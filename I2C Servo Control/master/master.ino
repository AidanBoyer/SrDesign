#include <Wire.h>

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
}

void loop() {
  static int i = 0;
  static int j = 0;

  transmitMotorSpeeds(i, j);
  i = i + 1;
  j = j - 1;
  delay(10);
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
  outputString.toCharArray(outputArray,9);
  
  Wire.write(outputArray);
  Wire.endTransmission();    // stop transmitting

  //delay(10);
}
