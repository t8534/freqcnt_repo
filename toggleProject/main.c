/*
 * toggleProject.c
 *
 * Created: 2018-07-22 18:45:12
 * Author : arek
 *
 */ 

// Make a tests with PWM.
// Compare frequence of PWM output read by:
// Freqmeter
// Oscilloscope USSR
// Saelogic
//

#include <avr/io.h>

//#define F_CPU 16000000UL  // set i the project properties
#include "util/delay.h"

#include "HD44780.h"
#include "freqcnt_window.h"


// Blinking LED for tests.
#define LCD_LED_CONFIG	DDRD |=  (1 << PD7)
#define LCD_LED_SET		PORTD |= (1 << PD7)
#define LCD_LED_RESET	PORTD &= ~(1 << PD7)


static uint8_t buff_txt[15];

int main(void)
{
	_delay_ms(100);		// Wait for stable power 

	HD44780_init();
	HD44780_on();
	HD44780_clear();

	FREQCNT_Init();
	
	for (;;) {
		_delay_ms(200);  // For any case, because measurement window is 1s
		FREQCNT_GetFrequencyTxt(buff_txt);
		HD44780_clear();
		HD44780_puts(buff_txt);
	}



	// tests
/*	
	// Debug pin init
	DEBUG0_PIN_CONFIG;
	DEBUG1_PIN_CONFIG;
	DEBUG2_PIN_CONFIG;

	HD44780_init();
	HD44780_on();
	HD44780_clear();
	
	InitWindowTimer();
	InitSampleCounter();
	sei();
	
	for (;;) {
		LCD_LED_SET;
		_delay_ms(100);
		LCD_LED_RESET;
		_delay_ms(100);
		
		//getTicksT0(buff_txt);
		getT1Counts(buff_txt);

		HD44780_clear();
		HD44780_puts(buff_txt);
	}
*/

}

