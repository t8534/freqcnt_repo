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
#define LCD_LED_CONFIG	DDRB |=  (1 << PB4)
#define LCD_LED_SET		PORTB |= (1 << PB4)
#define LCD_LED_RESET	PORTB &= ~(1 << PB4)


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


	lcd_init();
	lcd_on();
	lcd_clear();
	lcd_puts(msg);

	LCD_LED_CONFIG;
	while (1)
	{
		LCD_LED_SET;
		_delay_ms(500);
		LCD_LED_RESET;
		_delay_ms(500);
	}

}

