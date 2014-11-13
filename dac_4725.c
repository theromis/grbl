/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl v0.9

  Copyright (c) 2014 Roman Vasilyev

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
/* 
  This file is based on work from Grbl v0.9, distributed under the 
  terms of the MIT-license. See COPYING for more details.  
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011 Sungeun K. Jeon
*/ 

#include <compat/twi.h>
#include "system.h"

#define TWI_FREQ 100000L
void
twi_init(void)
{
    TWSR &= (~(1<<TWPS1))|(~(1<<TWPS0)); // Prescaler as 1
    TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
}

#include "print.h"

int
twi_write(uint8_t addr, uint16_t value)
{
    uint8_t ret=0xFF, retry=0;

i2c_retry:
    if (retry++>32)
        goto i2c_quit;
    //First send start condition over bus
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA);

    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    ret = TWSR & 0xF8;

    if (ret == TW_MT_ARB_LOST)
        goto i2c_retry;
    if ((ret != TW_START) && (ret != TW_REP_START))
        goto i2c_quit;


    //Start transmitted now transmit SLA+W
    TWDR = addr | TW_WRITE; // 1100 000 0 = 0xc0 for MCP4725

    //Clear TWINT flag to start transmission
    TWCR = (1<<TWINT) | (1<<TWEN);

    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    ret = TWSR & 0xF8;
    if ((ret == TW_MT_SLA_NACK) || (ret == TW_MT_ARB_LOST))
        goto i2c_retry;
    if (ret != TW_MT_SLA_ACK)
        goto i2c_quit;

#if 0
    //MCP4726_CMD_WRITEDACEEPROM
    TWDR = 0x60;
#else
    //MCP4726_CMD_WRITEDAC
    TWDR = 0x40;
#endif

    //Clear TWINT flag to start transmission
    TWCR = (1<<TWINT) | (1<<TWEN);

    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    ret = TWSR & 0xF8;
    if (ret != TW_MT_DATA_ACK)
        goto i2c_quit;

    //load high data byte into TWDR
    TWDR =(value >> 4) & 0xFF; // (D11.D10.D9.D8.D7.D6.D5.D4)

    //Clear TWINT flag to start transmission
    TWCR = (1<<TWINT) | (1<<TWEN);

    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    ret = TWSR & 0xF8;
    if (ret != TW_MT_DATA_ACK)
        goto i2c_quit;

    //load low data byte into TWDR
    TWDR = (value&0x0F)<<4;

    //Clear TWINT flag to start transmission
    TWCR = (1<<TWINT) | (1<<TWEN);

    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    ret = TWSR & 0xF8;
    if (ret != TW_MT_DATA_ACK)
        goto i2c_quit;

    ret = 0;
i2c_quit:
    //Now transmit STOP condition
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

    return ret;
}

int
twi_read(uint8_t addr, uint8_t *status, uint16_t *value)
{
    uint8_t data;
    //First send start condition over bus
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    data = TWSR;

    //Check if Start was transmitted or not
    if((data & 0xF8) != 0x08)
        return -1;


    //Start transmitted now transmit SLA+W
    TWDR = addr|TW_READ; // 1100 000 0 = 0xc0 for MCP4725

    //Clear TWINT flag to start transmission
    TWCR = (1<<TWINT) | (1<<TWEN);

    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    // Read start
    TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    *status = TWDR;

    TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    *value = TWDR<<4;

    TWCR = (1<<TWINT) | (1<<TWEN);
    //Wait for it to get done
    while(!(TWCR & (1<<TWINT)));

    *value |= TWDR>>4;

    //Now transmit STOP condition

    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

    return 0;
}
