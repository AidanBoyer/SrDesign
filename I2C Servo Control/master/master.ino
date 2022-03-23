#include <Wire.h>

int i = 0;
int j = 0;

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)

  Serial.begin(9600);
}

void loop() {
  
  transmitMotorSpeeds(i, j);
  i = i + 1;
  j = j - 1;
  delay(1000);
}

void transmitMotorSpeeds(int right, int left) {
  Wire.beginTransmission(4); // transmit to device #4

  String rightString;
  String leftString;
  String outputString;
  char outputArray[9];

  //dtostrf(right, 4, 0, rightString);
  //dtostrf(left, 4, 0, leftString);
  rightString = String(right);
  leftString = String(left);
  outputString = rightString + ',' + leftString;
  outputString.toCharArray(outputArray,9);
  
  Serial.println(outputArray);
  
  Wire.write(outputArray);
  Wire.endTransmission();    // stop transmitting

  delay(10);
}
