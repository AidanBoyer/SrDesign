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

zeroSetJumperPin = 3;

void setup(void)
{
  pinMode(zeroSetJumperPin, INPUT);

  Serial.begin(9600);

  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    //Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  tcs.setInterrupt(false);  // turn on LED
  //tcs.setInterrupt(true); // turn off LED

}

void loop(void)
{
  analogWrite(3, 255);
  float r, g, b;

  tcs.getRGB(&r, &g, &b);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  //colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  //lux = tcs.calculateLux(r, g, b);

  //Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  //Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
}

void resetCenter()
{

}
