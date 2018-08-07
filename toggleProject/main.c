/*
 *    Frequency counter for ATTiny84A
 *    Copyright (C) 2014  Bjorn Gustavsson
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Reciprocal Frequency Counter.
 *
 * Count and show the frequency of the square wave on input pins
 * INT0/PB2 and T1/PA4. We use INT0 for low frequencies and T1
 * for high frequencies.
 *
 * We use the reciprocal counter method. Instead of counting
 * the number of events (falling edges) during a fixed time
 * period, we measure the time period for a fixed number of
 * events and calculate the frequency as the number of events
 * divided by the length of the time period. That gives us
 * good resolution even for very low frequencies.
 */

#include <avr/io.h>

#define F_CPU 16000000UL
#include "util/delay.h"

#include "freq_cnt.h"
#include "HD44780.h"

// Blinking LED for tests.
#define LCD_LED_CONFIG	DDRB |=  (1 << PB4)
#define LCD_LED_SET		PORTB |= (1 << PB4)
#define LCD_LED_RESET	PORTB &= ~(1 << PB4)


static uint8_t buff_txt[10];

int main(void)
{

/*
	LCD_LED_CONFIG;
    while (1)   //loop forever
    {
	    LCD_LED_SET;
	    _delay_ms(500);
		LCD_LED_RESET;
		_delay_ms(500);
	    //wait 1 second
    }
*/

	// HD44780 lcd test
/*
	char *msg = "1234567890ABCDEF";

	HD44780_init();
	HD44780_on();
	HD44780_clear();
	HD44780_puts(msg);

	LCD_LED_CONFIG;
	while (1)
	{
		LCD_LED_SET;
		_delay_ms(500);
		LCD_LED_RESET;
		_delay_ms(500);
	}
*/


	// Frequency counter
	_delay_ms(100);		/* Wait for stable power */
		
	FREQCNT_Init();
	lcd_init();
		
	for (;;) {
		_delay_ms(100);
		FREQCNT_Cyclic100ms();
		FREQCNT_GetFrequencyTxt(buff_txt);
		lcd_display_msg(buff_txt);
	}



}

