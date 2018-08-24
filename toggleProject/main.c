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
#include "freqcnt_window.h"


// Blinking LED for tests.
#define LCD_LED_CONFIG	DDRD |=  (1 << PD7)
#define LCD_LED_SET		PORTD |= (1 << PD7)
#define LCD_LED_RESET	PORTD &= ~(1 << PD7)


static uint8_t buff_txt[15] = {'a', 'r', 'e', 'k', ' ', '1', '2', '3', '4', '5', 'A', 'B', 'C', 'D', 'E'};

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

	// HD44780 lcd test - working
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
		_delay_ms(250);
		LCD_LED_RESET;
		_delay_ms(250);
	}
	*/

	//////////////////////////////////////////////////////////////////////////////
	// Frequency counter
	//////////////////////////////////////////////////////////////////////////////
	
	// final proposal
	/*
	_delay_ms(100);		// Wait for stable power 
		
	FREQCNT_Init();
	lcd_init();
		
	for (;;) {
		_delay_ms(100);
		FREQCNT_Cyclic100ms();
		FREQCNT_GetFrequencyTxt(buff_txt);
		lcd_display_msg(buff_txt);
	}
	*/



	// tests
	
	uint8_t buff_empty_txt[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	char *msg = "1234567890ABCDEF";
	
	//todo: not working when uncoment it.
    //_delay_ms(100);		// Wait for stable power 
    //init_time_keeping();
    //init_event_counting();
    //sei();

	// Debug pin init
	DEBUG0_PIN_CONFIG;
	DEBUG1_PIN_CONFIG;
	DEBUG2_PIN_CONFIG;

	HD44780_init();
	HD44780_on();
	HD44780_clear();
	//HD44780_puts(buff_txt);
	//HD44780_puts(msg);
	
	InitWindowTimer();
	InitSampleCounter();
	sei();
	
	
	for (;;) {
//		_delay_ms(1000);

		LCD_LED_SET;
		_delay_ms(100);
		LCD_LED_RESET;
		_delay_ms(100);
		
		//getTicksT0(buff_txt);
		getT1Counts(buff_txt);

		HD44780_clear();
		HD44780_puts(buff_txt);
	}

}

