#include < Wire.h > // <-- remove spaces

int LED = 13;

int x = 0;

void setup()

{

  // Define the LED pin as Output

  pinMode (LED, OUTPUT);

  // Start the I2C Bus as Slave on address 9

  Wire.begin(9);

  // Attach a function to trigger when something is received.

  Wire.onReceive(receiveEvent);

}

void receiveEvent( int bytes )

{

  x = Wire.read(); // read one character from the I2C

  Wire.endTransmission(); // stop transmitting

}

void loop()

{

  if (x == 0) {

    digitalWrite(LED, HIGH);

    delay(200);

  }

  if (x == 0) {

    digitalWrite(LED, LOW);

    delay(200);

  }

}
