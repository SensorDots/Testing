/**
   SensorDots New Master Address - new_master_addr.c

   Copyright (C) 2019 SensorDots.org

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

   Call with new_master_addr_init(new_master_address, pin); at the beginning of your setup code.
   (dont forget to #include "new_master_addr.h")

   Note in current firmware (1.2) sync enable won't work with this because it doesn't
   know that it's the master.

*/

#include "new_master_addr.h"
#include "Arduino.h"

uint8_t pin = 0;

/**
   \brief New Master Address Init

   \param new_address - Address to send to next device
   \param pin_number - Pin connected to ADDR_IN of first device

   \return uint8_t - Return 1 if response, 0 if no device connected/error.
*/
uint8_t new_master_addr_init(uint16_t new_address, uint8_t pin_number)
{
  if (pin_number > 0) pin = pin_number;
  else return 0;

  /* Initialise address procedure with address */
  init_pins();

  /* Wait for bootloader to finish */
  delay(500);

  /* Perform detect presence procedure */
  if (addr_detect_presence())
  {
    /* Send address */
    addr_write_byte(new_address);

    if (addr_read_byte() == new_address)
    {
      /* Send ACK and return success  */
      addr_write_byte(0x01);
      return 1;
    }

    else /* Error with address parsing/device */
    {
      /* Send NACK and return error */
      addr_write_byte(0xfe);
      return 0;
    }
  }
  /* Return error */
  return 0;
}

/**
   \brief Init function. Sets/resets the initial pin states.

   \param

   \return void
*/
void init_pins(void)
{
  /* Set pin direction to input (no pullup) */
  pinMode(pin, INPUT);
}


/**
   \brief Reset function (Detect Presence)

   \param

   \return uint8_t - Return 1 if response, 0 if no device connected.
   Waits up to 1s for response due to bus
   address prorogation (112 devices @ ~1ms each). If there's no response then there's no
   more devices in chain.
*/
uint8_t addr_detect_presence(void)
{
  uint8_t response;
  uint32_t retries = 500000;
  /* Disable interrupts */
  noInterrupts();
  /* Wait for line to go high */
  pinMode(pin, INPUT);

  do
  {
    if (retries == 0) return 0;

    retries--;
    delayMicroseconds(2);
  }
  while ( !digitalRead(pin));

  /* Drive bus low */
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  /* Delay H */
  delayMicroseconds(480);
  /* Release bus */
  pinMode(pin, INPUT_PULLUP);
  /* Delay I */
  delayMicroseconds(70);
  /* Read bus state */
  response = !digitalRead(pin);
  /* Delay J */
  delayMicroseconds(410);
  /* Restore interrupts */
  interrupts();
  return response;
}


/**
   \brief Write 1Wire protocol bit

   \param bit - value of state

   \return void
*/
void addr_write_bit(uint8_t bit)
{
  /* Disable interrupts */
  noInterrupts();
  /* Drive bus low */
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  if (bit & 1)
  {
    /* Delay A */
    delayMicroseconds(6);
    /* Release bus */
    pinMode(pin, INPUT);
    /* Delay B */
    delayMicroseconds(64);
  }

  else
  {
    /* Delay C */
    delayMicroseconds(60);
    /* Release bus */
    pinMode(pin, INPUT_PULLUP);
    /* Delay D */
    delayMicroseconds(10);
  }

  /* Restore interrupts */
  interrupts();
}


/**
   \brief Read 1Wire Protocol bit

   \param

   \return uint8_t
*/
uint8_t addr_read_bit(void)
{
  uint8_t response;
  /* Disable interrupts */
  noInterrupts();
  /* Drive bus low */
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  /* Delay A */
  delayMicroseconds(6);
  /* Release bus */
  pinMode(pin, INPUT_PULLUP);
  /* Delay E */
  delayMicroseconds(9);
  /* Read bus state */
  response = digitalRead(pin);
  /* Delay F */
  delayMicroseconds(55);
  /* Restore interrupts */
  interrupts();
  return response;
}

/**
   \brief Write byte

   \param byte_to_write

   \return void
*/
void addr_write_byte(uint8_t byte_to_write)
{
  uint8_t bitMask;

  for (bitMask = 0x01; bitMask; bitMask <<= 1)
  {
    addr_write_bit( (bitMask & byte_to_write) ? 1 : 0);
  }
}

/**
   \brief Read byte


   \return uint8_t
*/
uint8_t addr_read_byte()
{
  uint8_t bitMask;
  uint8_t response = 0;

  for (bitMask = 0x01; bitMask; bitMask <<= 1)
  {
    if ( addr_read_bit()) response |= bitMask;
  }

  return response;
}
