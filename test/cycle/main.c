// this code sets up a timer0 for 1ms @ 16Mhz clock cycle
// in order to function as a time delay at the begining of the main loop
// using no interrupts


#include <avr/io.h>
#include "twi.h"


int main(void)
{
#if 0
    while (1)
    {

        // Set the Timer Mode to CTC
        TCCR0A |= (1 << WGM01);

        // Set the value that you want to count to
        OCR0A = 0xF9;

        // start the timer
        TCCR0B |= (1 << CS01) | (1 << CS00);
        // set prescaler to 64 and start the timer

        while ( (TIFR0 & (1 << TOV0) ) > 0)        // wait for the overflow event
        {
        }
        
        TIFR0 &= ~(1 << TOV0);
        // reset the overflow flag

    }
#else
    uint16_t pos=0;
    twi_init();
    while (1)
        twi_write(DAC_ADDR , DAC_X, pos++);
#endif
    return 0;
}
