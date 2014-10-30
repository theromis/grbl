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

#include "system.h"

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define SLA_W 0xC0 //write address

static void
i2c_bitrate_set(unsigned short bitrateKHz)
{
    unsigned char bitrate_div;
    // set i2c bitrate
    // SCL freq = F_CPU/(16+2*TWBR))
    //#ifdef TWPS0
    // for processors with additional bitrate division (mega128)
    // SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
    // set TWPS to zero
    cbi(TWSR, TWPS0);
    cbi(TWSR, TWPS1);
    //#endif
    // calculate bitrate division   
    bitrate_div = ((F_CPU/1000l)/bitrateKHz);
    if(bitrate_div >= 16)
        bitrate_div = (bitrate_div-16)/2;
    TWBR = bitrate_div;
}

void
i2cSendStart(void)
{
    // send start condition
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
}

void i2cSendStop(void)
{
    // transmit stop condition
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void i2cWaitForComplete(void)
{
    // wait for i2c interface to complete operation
    while (!(TWCR & (1<<TWINT)));
}

void i2cSendByte(unsigned char data)
{
    // save data to the TWDR
    TWDR = data;
    // begin send
    TWCR = (1<<TWINT)|(1<<TWEN);
}

#define Vout 3000//binary value going into register, 0-4095

// Initialize and setup the stepper motor subsystem
void
stepper_init()
{
    // set pull-up resistors on I2C bus pins
    //sbi(PORTC, 0);    // i2c SCL on ATmega163,323,16,32,etc
    //sbi(PORTC, 1);    // i2c SDA on ATmega163,323,16,32,etc
    //sbi(PORTD, 0);    // i2c SCL on ATmega128,64
    //sbi(PORTD, 1);    // i2c SDA on ATmega128,64

    // set i2c bit rate to 40KHz
    i2c_bitrate_set(100);
    // enable TWI (two-wire interface)
    sbi(TWCR, TWEN);

    //initilize I2C hardware
    TWCR = 0x00;
    TWBR = 8;
    cbi(TWCR, TWEA);    
    sbi(TWCR, TWEN);

    //Send start condition 
    i2cSendStart(); 
#if 0
    i2cWaitForComplete();

    // send slave device address with write
    i2cSendByte(SLA_W); 
    i2cWaitForComplete();   

    //set control bytes
    char lVout = Vout & 0xFF;
    char hVout = (Vout>>8) & 0x0F;

    // send first byte to MCP
    TWDR = hVout;
    // begin send
    TWCR = (1<<TWINT)|(1<<TWEN);    
    i2cWaitForComplete();

    // send second byte to MCP
    TWDR = lVout;
    // begin send
    TWCR = (1<<TWINT)|(1<<TWEN);    
    i2cWaitForComplete();

#endif
    //send stop condition
    i2cSendStop();

    TWCR = 0x00;//stop I2C
}

// Enable steppers, but cycle does not start unless called by motion control or runtime command.
void
st_wake_up()
{
}

// Immediately disables steppers
void
st_go_idle()
{
}

// Generate the step and direction port invert masks.
void
st_generate_step_dir_invert_masks()
{
}

// Reset the stepper subsystem variables       
void
st_reset()
{
}
             
// Reloads step segment buffer. Called continuously by runtime execution system.
void
st_prep_buffer()
{
}

// Called by planner_recalculate() when the executing block is updated by the new plan.
void
st_update_plan_block_parameters()
{
}

// Called by runtime status reporting if realtime rate reporting is enabled in config.h.
#ifdef REPORT_REALTIME_RATE
float
st_get_realtime_rate()
{
    return .0f;
}
#endif
