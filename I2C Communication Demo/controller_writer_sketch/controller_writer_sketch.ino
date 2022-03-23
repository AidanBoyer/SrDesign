// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI Peripheral device
// Refer to the "Wire Peripheral Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
char outputString[3];
void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
}

float x = 0;

void loop()
{
  Wire.beginTransmission(4); // transmit to device #4
  String(outputString);
  Wire.write(outputString);              // sends one byte  
  Wire.endTransmission();    // stop transmitting

  x=x+1;
  delay(500);
}
