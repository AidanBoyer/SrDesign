#include <Wire.h>
#include <Servo.h>

Servo servoRight;
Servo servoLeft;

int rightServoValue = 1500;
int leftServoValue = 1500;

void setup() {
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event

  Serial.begin(9600);           // start serial for output

  servoRight.attach(12);
  servoRight.writeMicroseconds(1500);
  servoLeft.attach(13);
  servoLeft.writeMicroseconds(1500);
}

void loop() {
  //delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  String firstReceivedString = "";
  String secondReceivedString = "";
  bool delineatorFound = false;
  static int firstOutputInt;
  static int secondOutputInt;

  while (Wire.available()) {
    char c = Wire.read();
    if (c == ',') {
      delineatorFound = true;
    }
    else {
      if (delineatorFound) {
        secondReceivedString = secondReceivedString + c;
      }
      else {
        firstReceivedString = firstReceivedString + c;
      }
    }
  }

  firstOutputInt = firstReceivedString.toInt();
  secondOutputInt = secondReceivedString.toInt();

  rightServoValue = 1500 - firstOutputInt;
  leftServoValue = 1500 + secondOutputInt;

  servoRight.writeMicroseconds(rightServoValue);
  servoLeft.writeMicroseconds(leftServoValue);
  
  Serial.print(rightServoValue, DEC);
  Serial.print(",");
  Serial.print(leftServoValue, DEC);
  Serial.println(" ");

  //delay(1000);
}
