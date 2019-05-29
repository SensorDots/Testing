/**
   SensorDots New Master Address - new_master_addr.h

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

   Call with new_master_addr_init(address, pin); at the beginning of your setup code.
   (dont forget to #include "new_master_addr.h")
   
   Note in current firmware (1.2) sync enable won't work with this because it doesn't
   know that it's the master.


*/


#ifndef NEW_MASTER_ADDR_H_
#define NEW_MASTER_ADDR_H_

#include <stdint.h>

/* New Master Address Init */
uint8_t new_master_addr_init(uint16_t new_address, uint8_t pin_number);

/* Init function. Sets/resets the initial pin states. */
void init_pins(void);

/* Reset function (Detect Presence) */
uint8_t addr_detect_presence(void);

/* Write bit */
void addr_write_bit(uint8_t bit);

/* Read bit */
uint8_t addr_read_bit(void);

/* Write byte */
void addr_write_byte(uint8_t byte_to_write);

/* Read byte */
uint8_t addr_read_byte();

#endif /* NEW_MASTER_ADDR_H_ */
