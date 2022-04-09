/*
 * Author: Augustin.B
 * Date  : 02.04.2022
 */

#include "board.h"

#include "avr/delay.h"

int main(void)
{
    // Disable 1/6 clock prescaler
    CCP = CCP_IOREG_gc;
    CLKCTRL_MCLKCTRLB = 0;

    // Set LED pin as output
    LEDBUILTIN_PORT.DIRSET = (1 << LEDBUILTIN_PIN);

    for(;;)
    {
        // Blink LED
        LEDBUILTIN_PORT.OUTTGL = (1 << LEDBUILTIN_PIN);
        _delay_ms(1000);
    }

    return 0;
}