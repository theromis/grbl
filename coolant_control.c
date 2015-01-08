/*
  coolant_control.c - coolant control methods
  Part of Grbl v0.9

  Copyright (c) 2012-2014 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/  

#include "system.h"
#include "coolant_control.h"
#include "protocol.h"
#include "gcode.h"
#include "twi.h"


void coolant_init()
{
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
  #ifdef ENABLE_M7
    COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
  #endif
  coolant_stop();
}


void coolant_stop()
{
  COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
  #ifdef ENABLE_M7
    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
  #endif
}


void coolant_run(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }

  protocol_auto_cycle_start();   //temp fix for M8 lockup
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.
  if (mode == COOLANT_FLOOD_ENABLE) {
  twi_write(DAC_ADDR, DAC_X, 65535);
  twi_write(DAC_ADDR, DAC_Y, 65535);
  twi_write(DAC_ADDR, DAC_I, 65535);
    COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);

  #ifdef ENABLE_M7  
    } else if (mode == COOLANT_MIST_ENABLE) {
      COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
  #endif

  } else {
      twi_write(DAC_ADDR, DAC_X, 0);
      twi_write(DAC_ADDR, DAC_Y, 0);
//      twi_write(DAC_ADDR, DAC_I, 0);
      coolant_stop();
  }
}
