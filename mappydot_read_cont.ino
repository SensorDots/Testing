/**
   MappyDot Read Multiple Demo

   Copyright (C) 2017  Blair Wyatt

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <i2c_t3.h> //Teensy 3 I2C library.

uint8_t address_max;
uint8_t address_min;
const int ledPin = 13;

void simple_read(uint8_t address) {
  Wire.requestFrom(address, 2);
  uint16_t distance = Wire.read() << 8; distance |= Wire.read(); 
  Serial.print(address, DEC);
  Serial.print(",");
  Serial.println(distance, DEC);
}

void setup() {

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
  Serial.begin(115200);

  //Stop bus locking up when I2C glitches occur.
  Wire.setDefaultTimeout(10000);

  // initialize the LED pin as an output.
  pinMode(ledPin, OUTPUT);

  address_min = 8;
  address_max = 15;
}

void loop() {
  digitalWrite(ledPin, LOW); // set the LED off
  // put your main code here, to run repeatedly:
  for (int i = address_min; i <= address_max; i++) {
    simple_read(i);
  }
  digitalWrite(ledPin, HIGH); // set the LED off
  delay(100);
}
