/*
 * toggleProject.c
 *
 * Created: 2018-07-22 18:45:12
 * Author : arek
 */ 

#include <avr/io.h>

#define F_CPU 16000000UL
#include "util/delay.h"

#include "HD44780.h"

// Blinking LED for tests.
#define LCD_LED_CONFIG	DDRD |=  (1 << PD7)
#define LCD_LED_SET		PORTD |= (1 << PD7)
#define LCD_LED_RESET	PORTD &= ~(1 << PD7)


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

}

