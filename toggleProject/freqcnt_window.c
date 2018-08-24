 


// Notes:
//
// 1.
//
// 20 MHz = 50 ns
// 14 MHz = 71.428572... ns
// 12 MHz = 83.33333333 ns
// 16 MHz = 62.5 ns
//  8 MHz = 125 ns
//  4 MHz = 250 ns
//  1 MHz = 1000 ns
//
// 64 ns = 15.625 MHz
//
// 2.
// Tests results:
// External PWM signal, reported by Saelogic:
// Frequency: 31.3725 kHz
// Width: 15.8750 us / 16.0000 us
// Period: 31.8750 us
// DutyCycle: 49.804 %
//
// Result displayed by LCD of freqcnt_window: 37.371 kHz
// 
// todo:
//
// 1.
// Change windows size to be dynamic in depend on frequency ?
//
 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   // ultoa()
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <limits.h>

#include "freqcnt_window.h"



// Set measurement window to 1s, this is more convenient we get frequency just in Hz.
// With 8:1 prescaler, the tick is 125ns, so overflow is at 32 us. 
// To get 1 s counted, the T0 overflow ISR should be called 1000000 us / 32 us = 31250 times.
#define FREQCNT_MEASUREMENT_WINDOW_SIZE_US	31250  // 1s


#define START_TIMER1	TCCR1B |= (1<<CS12)|(1<<CS11)
#define STOP_TIMER1		TCCR1B &= 0B11111000
#define CLEAR_TIMER1	TCNT1 = 0

typedef uint32_t tick_t;

static volatile tick_t t0_WindowOverflowCnt = 0;  // max = 4 294 967 295 
static volatile tick_t t1_SampleOverflowCnt = 0;
static volatile tick_t countedSamplesInWindow = 0;  // todo: probably not necessary, because window is constant

static void InitWindowTimer(void);
static void InitSampleCounter(void);


//todo: tests, remove after.
//static volatile tick_t test_t0_WindowOverflowCnt = 0;

void FREQCNT_Init(void)
{
	cli();  // Disable interrupts until initialize items.
	InitWindowTimer();
	InitSampleCounter();
    sei();	// Enable interrupts for all.	
}




// Timer T0 - Counting measurement window size.
// todo: check the window size for max and min frequency
static void InitWindowTimer(void)
{
	//todo: test, remove after
	//test_t0_WindowOverflowCnt = 0x00;


	// Normal mode (mode 0), TOP = 0xFF, TOV flag set on 0xFF
	// WGM00 = 0x00, WGM01 = 0x00, WGM02 = 0x00.
	// WGM02 is located in TCCR0B.
	TCCR0A = 0x00;
	
	// Set prescaler.
	// 8MHz/1 = 125 ns. Overflow 32 us - here we could count exactly 1s, so it is better - change code to use it !
	// 8MHz/8 = 1us. Overflow 256 us - seems to be the best, easy to count 4x256 = 1ms
	// 8MHz/64 = 8us. Overflow 2048 us
	// Selected is prescaler 8:1, most easy to count 1 s measurement window.
	TCCR0B = 0x00;
	// Prescaler :8 -> CS02 = 0, CS01 = 0, CS00 = 1
	TCCR0B = _BV(CS00);
	
	// Reset timer data register
	TCNT0 = 0x00;
	
	// Enable interrupts.
	// This is also necessary the I-bit in the Status Register is set to Enable this timer.
	TIMSK0 = 0x00;
	TIMSK0 = _BV(TOIE0);
	
}


// Overflow
ISR(TIMER0_OVF_vect)
{
	// todo remove
	//DEBUG0_PIN_SET;
	
	// With 8 MHz clock, and :8 prescaler, the ISR is called every 1us
    t0_WindowOverflowCnt++;
	if (t0_WindowOverflowCnt >= FREQCNT_MEASUREMENT_WINDOW_SIZE_US)
	{
		
		// todo remove
		//DEBUG1_PIN_SET;
		
		
		// We have just count 1 s, so get counted samples.

		t0_WindowOverflowCnt = 0x00;
		
		// Read sample counter T1, reset counter T1 and sampleCnt variable.
		cli(); //todo: Is it necessary to switch off general interrups inside of this ISR ?
		STOP_TIMER1;
		tick_t t = TCNT1;	//todo: replace with tict_t
		countedSamplesInWindow = (t1_SampleOverflowCnt << 16) | t;
		t1_SampleOverflowCnt = 0x0000;
		TCNT1 = 0x00;
		
		// test
		//test_t0_WindowOverflowCnt++;
		
		START_TIMER1;
		sei();
		
		// todo remove
		//DEBUG1_PIN_RESET;

		
	}
	
	
	// todo remove
	//DEBUG0_PIN_RESET;
	
}


// This is T1 based.
// Counting external signal falling edges in the measurement window defined by T0.
static void InitSampleCounter(void)
{

	// The simplest mode of operation is the Normal mode (TCCR1A.WGM1[3:0]=0x0). In this mode the
	// counting direction is always up (incrementing), and no counter clear is performed.
	// The counter simply overruns when it passes its maximum 16-bit value (MAX=0xFFFF) and then
	// restarts from BOTTOM=0x0000.
	//
	// Mode 0, TOP = 0xFFFF, TOV1 flag set on 0xFFFF
	// WGM10 = 0, WGM11 = 0, WGM12 = 0, WGM13 = 0, WGM12 and WGM13 is set in TCCR1B.
	TCCR1A = 0x00;

	// Set prescaler:
	// External clock source on T1 pin. Clock on falling edge.
	// CS12 = 1, CS11 = 1, CS10 = 0
	TCCR1B = 0x00;  // CS10 = 0
	TCCR1B = _BV(CS12) | _BV(CS11);
	
	// Reset timer data register
	TCNT1 = 0x00;
	
	// Enable interrupts.
	// This is also necessary the I-bit in the Status Register is set to Enable this timer.
	TIMSK1 = 0x00;
	TIMSK1 = _BV(TOIE1);
	
}


// Overflow
ISR(TIMER1_OVF_vect)
{
	//todo: remove
	//DEBUG2_PIN_SET;
	
    t1_SampleOverflowCnt++;
	if (t1_SampleOverflowCnt >= ULONG_MAX)
	{
		t1_SampleOverflowCnt = 0x00;
		
		//todo: Display error of overflow sample counter, perhaps it could lead to decrease measurement window size.
		// Perhaps it should never happens with dynamic measurement window size.
	}

	//todo: remove
	//DEBUG2_PIN_RESET;
}



/*
// If we would like to measure period (for low frequency samples)
ISR(INT0_vect)  // atmega328p
{
}
*/


// return measured frequency in Hz.
uint32_t FREQCNT_GetFrequencyHz()
{
	tick_t samplesNbr = 0x00;
	uint32_t measuredFreqHz = 0x00;
	
	
	cli();
	samplesNbr = countedSamplesInWindow;  // because window is 1 s, we get frequency in Hz.
	sei();

	// todo: the rest of implementation.
	// It is ok only for window size 100ms, so x10 give the value in Hz.
	// todo: replace it with version returning Hz independed of measurement windows size.
	//measuredFreqHz = samplesNbr * ((FREQCNT_MEASUREMENT_WINDOW_SIZE_US/1000) * 10);
	
	measuredFreqHz = samplesNbr;
	
	return measuredFreqHz;
}


// todo: should be displayed as scalable, with Hz, kHz, MHz.
void FREQCNT_GetFrequencyTxt(unsigned char *buff)
{
	unsigned long val = 0;

	cli();
	val = countedSamplesInWindow;
	sei();

	ultoa(val, buff, 10);

}


///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
/*
void getTicksT0(char *buff)
{
	static tick_t t = 0;


	cli();
	t = test_t0_WindowOverflowCnt;
	sei();

	//t = t + 1;

	//char bufor[ROZMIAR];
	//int liczba;
	//sprintf(bufor, "wynik: %d", liczba);

	//char buffer[7];//przykadowo 7
	//int  num;
	//itoa( num, buffer, 10);   // convert interger into string (decimal format)

	// Function ultoa()
	// Convert an unsigned long integer to a string.
	//
	// char * ultoa(unsigned long val, char * s, int radix)
	ultoa(t, buff, 10);

	
}


void getT1Counts(char *buff)
{
	unsigned long val = 0;

	cli();
	val = countedSamplesInWindow;
	sei();

	ultoa(val, buff, 10);

}
*/
