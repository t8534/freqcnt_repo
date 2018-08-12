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

 // TODO:
 //
 // 1.
 // Check all algo, the project is designed for 20 Mhz - 100 ticks givs 320 uS.
 // It should be based on the available uP frequence, if possible.
 //
 //
 
// atmega32u4
//
// atmega84: T1 is 16-bit at PA4
// atmega32u4: T1 is 16-bit at PD6
//
// atmega84: INT0 is at PB2
// atmega32u4: INT0 is at PD0, unfortunately it touch lcd display, could we move it to INT6 (PE6)


 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   // ultoa()
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "freq_cnt.h"

#define DEBUG 0

#define MAX_PERIOD 0xffffffffUL
typedef unsigned long tick_t;


/*
 * Other functions.
 */
#if DEBUG
static void inline debug_show_state(uint8_t n);
#endif
static inline unsigned long cli_ticks(void)  // Used only by ISR from T0 overload and cli_ticks()
    __attribute__ ((always_inline));
void init_time_keeping(void);  //todo: after tests move back to static
void init_event_counting(void);  //todo: after tests move back to static
static void slow_mode(void);
static void inline set_timer_cmp_reg(uint8_t log2ne)
    __attribute__ ((always_inline));
static unsigned long display_measurement(uint8_t n, tick_t p);
static void show_line(char* s);
static void display_freq(unsigned long freq, unsigned char *buff);

struct counter {
    /*
     * The current frequency can be calculated as:
     *
     *     2^log2num_events / (64 * F_CPU * period)
     *
     * A tick is 64 cpu cycles.
     */
    tick_t period;	/* Length of last measured period (in ticks) */
    uint8_t log2num_events;	/* log2 of number of events that occurred. */

    /*
     * Internal data for the interrupt routines to keep track of the
     * current measuring period.
     */
    uint8_t first_time;
    uint8_t current_log2num_events; /* For the current counting period.   */
    tick_t prev_ticks;	     /* Number of ticks at start of period. */
};

static volatile struct counter slow_cnt = { MAX_PERIOD, 0, 1, 0, 0 };
static volatile struct counter fast_cnt = { MAX_PERIOD, 0, 1, 1, 0 };
static struct counter volatile *current;

#define WD_TOP 4
static volatile signed char fast_wd = WD_TOP;


void FREQCNT_Init(void)
{
    _delay_ms(100);		/* Wait for stable power */
    init_time_keeping();
    init_event_counting();
    sei();
    //lcd_init();
}


// Should be called as quick as possible. todo: think again this ide, to not force to call Cyclic as quick as possible
// but only get measure results and display it outside of freq counter.
void FREQCNT_Cyclic100ms(void)
{
	//_delay_ms(100);  //arek: probably this is to improve, if we make that way measurement window.
					// Seems to be this is only the period of refresh lcd display.
					// But no, because we call here slow_mode()
				
#if DEBUG
	debug_show_state(n);
#endif

	//todo: make it independent from cyclic call
	/*
	* See if we should switch to slow mode.
	*/
	if (fast_wd-- < 0 && current == &fast_cnt) {
		/*
		* We have not got any fast mode interrupts for 400 ms.
		* Switch to slow mode.
		*/
		fast_wd = WD_TOP;
		slow_mode();
	}
	
}


//todo: it should be in the form ready to convert to ASCII
uint32_t FREQCNT_GetFrequencyHz(void)
{
	uint32_t freq = 0;
	uint8_t n;
	unsigned long p;

	
		/*
	* Read out information about the latest completed
	* measurement period.
	*/
	cli();
	n = current->log2num_events;
	p = current->period;
	sei();
	
	freq = (uint32_t)display_measurement(n, p);  //todo: remove cast in future
	
	return freq;
}


void FREQCNT_GetFrequencyTxt(uint8_t *buff)
{
	uint32_t freq = 0;

	
	freq = FREQCNT_GetFrequencyHz();
	display_freq(freq, buff);
	
}



/*
 * API functions for the display. A different kind
 * of display can be used if those functions are
 * reimplemnted.
 */
/*
static void lcd_init(void);
static void lcd_home(void);
static void lcd_putc(char c);
*/


/*
#if DEBUG
void debug_show_state(uint8_t n)
{
    int nc;

    DDRA |= _BV(PA5) | _BV(PA6);

    //
    // Show the 2 logarithm for the number of events as a number of
    // square wave cycles on output port PA5. (Connect an oscilloscope
    // probe to PA5 to see it. Set the triggering mode to Normal or
    // increase the trigger Holdoff time so that the square wave stays
    // on the screen.)
    //

    nc = 2 * n;
    while (nc-- > 0) {
	PINA |= _BV(PA5);
	_delay_us(20);
	if (nc % 10 == 0) {
	    _delay_us(50);
	}
    }

    //
    // Set PA6 high if we are in slow mode and low otherwise.
    // (Connect a LED or logic probe to PA6 to see the mode.)
    //

    if (current == &slow_cnt) {
	PORTA |= _BV(PA6);
    } else {
	PORTA &= ~_BV(PA6);
    }
}
#endif
*/


/*
 * Time keeping. We count "ticks" (1 tick = 64 cpu cycles) and only
 * convert to time when we'll need to show the frequency.
 */
// The Timer0 is initialized here, which is 8 bit.
// Once initiated this is running continuously until whole application is running.
// The cli_ticks() is called by ISR for slow and fast mode, to get the current tics values.
//
// Atmega328p T0 prescaler settings:
//
// MCU clock 8 MHz - 125ns
//
//   8:1  -> tick = 125ns, overflow = 256 x 125ns =   32us
//   8:8  -> tick = 1us,   overflow = 256 x   1us =  256us
//   8:64 -> tick = 8us,   overflow = 256 x   8us = 2048us
//
// MCU clock 16 MHz - 62.5ns
//
//  16:1   -> tick = 62.5ns, overflow = 256 x 62.5ns =     16us
//  16:8   -> tick = 500ns,  overflow = 256 x 500ns  =    128us
//  16:64  -> tick = 4us,    overflow = 256 x 4us    =   1024us
//  16:256 -> tick = 16us,   overflow = 256 x 16us   =   4096us
//
void init_time_keeping(void)  // todo: after tests move back to static
{
	
	///////////////////////////////////////////////////////////////////////////
	// Atiny 84
    ///////////////////////////////////////////////////////////////////////////
	/*
    //
    // Set prescaler to 64. Using a 20Mhz clock, that will give
    // a timer tick time of appr. 3.2 us, and timer overflow appr.
    // 819 us. The Timer 0 is 8 bit. 819/3.2 = 255.9375
    //
	 // We have 16 MHz clock
	 // The prescaled clock has a frequency: of either fCLK_I/O/8, fCLK_I/O/64, fCLK_I/O/256, or fCLK_I/O/1024.
	 // 16/64 = 2.5 us -> 640us while timer overflow
	 // 16/8  = 20 uS -> 5129 us while timer overflow
	 
	 
    TCCR0B = _BV(CS01) | _BV(CS00); // Set prescaler to clk/64

    // Enable the overflow interrupt for time 0 

	// When the TOIE0 bit is written to one, and the I-bit in the Status Register is set, the
	// Timer/Counter0 Overflow interrupt is enabled. The corresponding interrupt is executed if an
	// overflow in Timer/Counter0 occurs, i.e., when the TOV0 bit is set in the Timer/Counter 0 Interrupt
	// Flag Register – TIFR0.
	
    TIMSK0 = _BV(TOIE0);
	*/
	
	////////////////////////////////////////////////////////////////////////////////
	// Code for T0 atmega32u4
	////////////////////////////////////////////////////////////////////////////////

	/*
	// Set prescaler to clk/64 -> this is 2.5us per tic, with 16MHz clock.
	// it is 256 x 2.5 = 640 us.
	TCCR0B = _BV(CS01) | _BV(CS00); 
	
    // Enable the overflow interrupt for time 0 
	// When the TOIE0 bit is written to one, and the I-bit in the Status Register is set, the
    // Timer/Counter0 Overflow interrupt is enabled. The corresponding interrupt is executed if an
    // overflow in Timer/Counter0 occurs, i.e., when the TOV0 bit is set in the Timer/Counter 0 Interrupt
    // Flag Register – TIFR0.
	
    TIMSK0 = _BV(TOIE0); 
	*/


	///////////////////////////////////////////////////////////////////////////
	// Code for Atmega328p
	
	TCCR0A = 0x00;  // Normal mode, TOP=0xFF, TOV flag set on MAX = 0xFF.
	
	TCCR0B = 0x00;  // Normal mode, 
	
	// MCU clock 8 MHz
	// Prescaler 8:64
	// Timer clock tick = 8us
	// Timer overflow = 256 x 8us = 2048us
	//
	// CS00 = 1 
	// CS01 = 1
	// CS02 = 0
	TCCR0B = _BV(CS01) | _BV(CS00);  
	
	// Enable interrupts
	TIMSK0 = 0x00;
	TIMSK0 = _BV(TOIE0); 
	
}

static volatile unsigned long timer0_overflow_count = 0;   // move it up

// ISR from T0 overflow
ISR(TIM0_OVF_vect)
{
    timer0_overflow_count++;
}

/*
 * Return the number of timer ticks elapsed. Interrupts MUST
 * be disabled when calling this function.
 */
 // It return T0 tics continuously from the beginning of application start.
 // This is called by ISR for slow and fast mode.
static tick_t cli_ticks(void)
{
    uint8_t t;
    tick_t m;

    m = timer0_overflow_count;
    t = TCNT0;  // Timer data
    if (TIFR0 & _BV(TOV0) && t < 255) { // Timer Interrupt Flag Register (TIFR0), The Timer/Counter Overflow Flag (TOV0)
		m++;
    }
    return (m << 8) | t;
}

/* =====================================================================
 *
 * Count rising edges of the input signal and note their time (in ticks).
 * Output will be the number of rising edges and the time in ticks between
 * them. By dividing the number of edges by the time we get the frequency.
 *
 * We decide beforehand how many events (edges) we want to count. To
 * simplify the calculations, the number of events are the powers of
 * two: 1, 2, 3, 4, ..., 1048576. We adjust the number of events,
 * aiming for a period of at least 10000 ticks. If the period goes below
 * 10000 ticks, we'll adjust the number of events to count upwards.
 * If the number of ticks goes above 30000, we will adjust fewer
 * events next time.
 *
 * We can count events using timer 1 in CTC mode. However, it seems
 * that the minimum number of events that can be counted is 2.
 * Therefore, to react faster while counting low frequencies (less
 * than 20Hz), we use the external interrupt to count individual
 * falling edges.
 *
 * When we switch to slow mode (using the external interrupt) we
 * don't turn off timer 1. That way, we can quickly switch back to
 * fast mode.
 *
 * ====================================================================
 */

// todo:
// Check is it really true for atmega32U4.
// And if not simplify code.
//
// * We can count events using timer 1 in CTC mode. However, it seems
// * that the minimum number of events that can be counted is 2.
// * Therefore, to react faster while counting low frequencies (less
// * than 20Hz), we use the external interrupt to count individual
// * falling edges.


void init_event_counting(void)  //todo after tests move back to static
{
    /*
     * Our hardware outside the microcontroller has converted the
     * incoming signal to a square wave and inverted it, and is
     * feeding to both input pins PB2 (INT0) and PA4 (T1). We
     * want to count the rising of the original signal, so since
     * our input signal is inverted, we will need react to the
     * FALLING edges.
     */

	//////////////////////////////////////////////////////////////////////////////
	// atmega32u4
	//////////////////////////////////////////////////////////////////////////////
	
	/*
	//
	// atmega84: T1 is 16-bit at PA4
	// atmega32u4: T1 is 16-bit at PD6
	//
	// atmega84: INT0 is at PB2
	// atmega32u4: INT0 is at PD0, unfortunately it touch lcd display, could we move it to INT6 (PE6)

    //
    // Generate interrupts on the falling edge of the signal on PB2.
    //
	// This is for slow mode, measure by external pin.

	// Bits 1:0 – ISC01, ISC00: Interrupt Sense Control 0 Bit 1 and Bit 0
	// The External Interrupt 0 is activated by the external pin INT0 if the SREG I-flag and the corresponding
	// interrupt mask are set. The level and edges on the external INT0 pin that activate the
	// interrupt are defined in Table 9-2. The value on the INT0 pin is sampled before detecting edges.
	// If edge or toggle interrupt is selected, pulses that last longer than one clock period will generate
	// an interrupt. Shorter pulses are not guaranteed to generate an interrupt. If low level interrupt is
	// selected, the low level must be held until the completion of the currently executing instruction to
	// generate an interrupt.
    MCUCR = _BV(ISC01);

	// Bit 6 – INT0: External Interrupt Request 0 Enable
	// When the INT0 bit is set (one) and the I-bit in the Status Register (SREG) is set (one), the external
	// pin interrupt is enabled. The Interrupt Sense Control bits (ISC01 and ISC00) in the External
	// Interrupt Control Register A (EICRA) define whether the external interrupt is activated on rising
	// and/or falling edge of the INT0 pin or level sensed. Activity on the pin will cause an interrupt
	// request even if INT0 is configured as an output. The corresponding interrupt of External Interrupt
	// Request 0 is executed from the INT0 Interrupt Vector.
	
    GIMSK = _BV(INT0);  // General Interrupt Mask Register

    //
    // Set up timer 1 in CTC mode, using the input signal on PA4 as
    // the clock for the timer. PA4 this is also T1 label
    //

    // Timer/Counter1 Control Register A,
    // Compare Output disconnected
	// WGM12 = 1, and WGM 10, 11, 13 = 0 Wafeorm Generation Mode4 - CTC (Clear Timer on Compare)
	//   TOP - OCR1A - Update of OCR1x immediate, TOV1 flag set on MAX
	TCCR1A = 0;  
    
	// Bit 7 – ICNC1: Input Capture Noise Canceler - not activated
	// Bit 6 – ICES1: Input Capture Edge Select
	//   This bit selects which edge on the Input Capture pin (ICP1) that is used to trigger a capture
	//   event. When the ICES1 bit is written to zero, a falling (negative) edge is used as trigger, and
	//   when the ICES1 bit is written to one, a rising (positive) edge will trigger the capture.
	//   When a capture is triggered according to the ICES1 setting, the counter value is copied into the
	//   Input Capture Register (ICR1). The event will also set the Input Capture Flag (ICF1), and this
	//   can be used to cause an Input Capture Interrupt, if this interrupt is enabled.
	//   When the ICR1 is used as TOP value (see description of the WGM13:0 bits located in the
	//   TCCR1A and the TCCR1B Register), the ICP1 is disconnected and consequently the Input Capture
	//   function is disabled.
	//
	// WGM12 = 1, and WGM 10, 11, 13 = 0 Wafeorm Generation Mode4 - CTC (Clear Timer on Compare)
	//   TOP - OCR1A - Update of OCR1x immediate, TOV1 flag set on MAX
    // Clock select:
	// CS10=0, CS11=1, CS12=1 - External clock source on T1 pin. Clock on falling edge.
	//   If external pin modes are used for the Timer/Counter1, transitions on the T1 pin will clock the
	//   counter even if the pin is configured as an output. This feature allows software control of the
	//   counting.
	TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS11);  //CS12, CS11, CS10: 110: External clock source on T1 pin. Clock on falling edge.
    set_timer_cmp_reg(fast_cnt.current_log2num_events);  //WGM 10/11/12/13: CTC (Clear Timer on Compare), TOP in OCR1A
    TCNT1 = 0;    // 16 bit counter 
	
	// Timer/Counter Interrupt Mask Register 1
	// Bit 1 – OCIE1A: Timer/Counter1, Output Compare A Match Interrupt Enable
	//   When this bit is written to one, and the I-flag in the Status Register is set (interrupts globally
	//   enabled), the Timer/Counter1 Output Compare A Match interrupt is enabled. The corresponding
	//   Interrupt Vector (see “Interrupts” on page 48) is executed when the OCF1A flag, located in
	//   TIFR1, is set.
    TIMSK1 = _BV(OCIE1A);  // If interrupts are globally enabled (I-flag in the Status Register is set), enable interrupt if Counter achive OCR1A value.
    
	// Timer/Counter Interrupt Flag Register 1
	// Bit 1 – OCF1A: Timer/Counter1, Output Compare A Match Flag
	//   This flag is set in the timer clock cycle after the counter (TCNT1) value matches the Output
	//   Compare Register A (OCR1A).
	//   Note that a Forced Output Compare (1A) strobe will not set the OCF1A flag.
	//   OCF1A is automatically cleared when the Output Compare Match A Interrupt Vector is executed.
	//   Alternatively, OCF1A can be cleared by writing a logic one to its bit location.
	TIFR1 |= _BV(OCF1A);

    //
    // Start up in slow mode. The interrupt handler for slow
    // mode will shift to fast mode if the frequency is too high.
    //

    current = &slow_cnt;
	*/
	
	////////////////////////////////////////////////////////////////////////
	// Code related to atmega32u4
	////////////////////////////////////////////////////////////////////////
	/*
	//todo for pin ISR
	// Configure for INT0, but it could be INT6, INT3:0
	EICRA = 0;
	EICRA = _BV(ISC01);  // The falling edge of INT0 generates asynchronously an interrupt request.
	
	// Bits 7..0 – INT6, INT3 – INT0: External Interrupt Request 6, 3 - 0 Enable
	// When an INT[6;3:0] bit is written to one and the I-bit in the Status Register (SREG) is set (one), the
	// corresponding external pin interrupt is enabled. The Interrupt Sense Control bits in the External Interrupt Control
	// Registers – EICRA and EICRB – defines whether the external interrupt is activated on rising or falling edge or
	// level sensed. Activity on any of these pins will trigger an interrupt request even if the pin is enabled as an output.
	// This provides a way of generating a software interrupt.
	EIMSK = 0;
	EIMSK = _BV(INT0); 
	
	// todo: what about this ?
	// Bits 7..0 – INTF6, INTF3 - INTF0: External Interrupt Flags 6, 3 - 0
	// When an edge or logic change on the INT[6;3:0] pin triggers an interrupt request, INTF7:0 becomes set (one). If
	// the I-bit in SREG and the corresponding interrupt enable bit, INT[6;3:0] in EIMSK, are set (one), the MCU will
	// jump to the interrupt vector. The flag is cleared when the interrupt routine is executed. Alternatively, the flag can
	// be cleared by writing a logical one to it. These flags are always cleared when INT[6;3:0] are configured as level
	// interrupt. Note that when entering sleep mode with the INT3:0 interrupts disabled, the input buffers on these
	// pins will be disabled. This may cause a logic change in internal signals which will set the INTF3:0 flags. See
	//“Digital Input Enable and Sleep Modes” on page 71 for more information.
	
	// EIFR -> but we do not use GIFR in atmega84
	
    //
    // Set up timer 1 in CTC mode, using the input signal on PA4 as
    // the clock for the timer. PA4 this is also T1 label
    //
	// Timer/Counter mode of operation: CTC - Mode 4
	// Top: OCRnA
	// Update of OCRnX at: Immediate
	// TOVn flag set on: MAX
	// WGMn0 = 0
	// WGMn1 = 0
	// WGMn2 = 1
	// WGMn3 = 0
	TCCR1A = 0;  // WGM10 = 0, WGM11 = 0
	// WGM12 = 1, WGM13 = 0, CS12, CS11, CS10: 110: External clock source on T1 pin. Clock on falling edge.
	TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS11);
	set_timer_cmp_reg(fast_cnt.current_log2num_events);  //todo: check is something to change inside
	TCNT1 = 0;  // Clear data counter.

	// If interrupts are globally enabled (I-flag in the Status Register is set),
    // enable interrupt if Counter achive OCR1A value.
    TIMSK1 = _BV(OCIE1A);  

	// This flag is set in the timer clock cycle after the counter (TCNT1) value matches the Output
	// Compare Register A (OCR1A).
	// OCF1A is automatically cleared when the Output Compare Match A Interrupt Vector is executed.
	// Alternatively, OCF1A can be cleared by writing a logic one to its bit location.
    TIFR1 |= _BV(OCF1A);  // Clear the flag related on interrupt when OCR1A match TCNT1

    //
    // Start up in slow mode. The interrupt handler for slow
    // mode will shift to fast mode if the frequency is too high.
    //

    current = &slow_cnt;
	*/
	
	
	////////////////////////////////////////////////////////////////////////
	// Code related to atmega328p
	////////////////////////////////////////////////////////////////////////
	
	//todo for pin ISR (slow freq algo)
	
	// Configure for INT0, but it could be INT6, INT3:0
	EICRA = 0;
	EICRA = _BV(ISC01);  // The falling edge of INT0 generates asynchronously an interrupt request.

	// External Interrupt Request 0 Enable
	// Bit 0 – INT0: External Interrupt Request 0 Enable
	// When the INT0 bit is set and the I-bit in the Status Register (SREG) is set, the external pin interrupt is
	// enabled. The Interrupt Sense Control0 bits 1/0 (ISC01 and ISC00) in the External Interrupt Control
	// Register A (EICRA) define whether the external interrupt is activated on rising and/or falling edge of the
	// INT0 pin or level sensed. Activity on the pin will cause an interrupt request even if INT0 is configured as
	// an output. The corresponding interrupt of External Interrupt Request 0 is executed from the INT0 Interrupt
	// Vector.	
	EIMSK = 0;
	EIMSK = _BV(INT0);


	// Timer T1 (fast freq algo)
	// Timer/Counter mode of operation: CTC - Mode 4
	// Top: OCRnA
	// Update of OCRnX at: Immediate
	// TOVn flag set on: MAX
	// WGMn0 = 0
	// WGMn1 = 0
	// WGMn2 = 1
	// WGMn3 = 0
	TCCR1A = 0;  // WGM10 = 0, WGM11 = 0
	
	// WGM12 = 1, WGM13 = 0 -> part of CTC mode 4 settings
	// External clock source on T1 pin. Clock on falling edge:
	// CS12 = 1
	// CS11 = 1
	// CS10 = 0
	TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS11);
	set_timer_cmp_reg(fast_cnt.current_log2num_events);  //todo: check is something to change inside
	TCNT1 = 0;  // Clear data counter.

	// If interrupts are globally enabled (I-flag in the Status Register is set),
	// enable interrupt if Counter achive OCR1A value.
	TIMSK1 = _BV(OCIE1A);
	
	// This flag is set in the timer clock cycle after the counter (TCNT1) value matches the Output
	// Compare Register A (OCR1A).
	// OCF1A is automatically cleared when the Output Compare Match A Interrupt Vector is executed.
	// Alternatively, OCF1A can be cleared by writing a logic one to its bit location.
	TIFR1 |= _BV(OCF1A);  // Clear the flag related on interrupt when OCR1A match TCNT1

    /*
     * Start up in slow mode. The interrupt handler for slow
     * mode will shift to fast mode if the frequency is too high.
     */

    current = &slow_cnt;

	
	
	
}

/*
 * Interrupt service routine for the slow counting mode.
 */
 // This is ISR from external pin irq.
ISR(EXT_INT0_vect)
{
    tick_t cs = cli_ticks();

    if (slow_cnt.first_time) {
	/* We can't calculate a period because it's the first time. */
	slow_cnt.first_time = 0;
	slow_cnt.prev_ticks = cs;
    } else {
	/*
	 * Calculate the length of period that just ended. We don't need
	 * to update log2num_events since it is always 0 (= one event).
	 */
	tick_t period = cs - slow_cnt.prev_ticks;
	slow_cnt.period = period;
	slow_cnt.prev_ticks = cs;

	/*
	 * If the period is less than 100 ticks (320 us at 20Mhz), do
	 * an emergency switch to fast counter mode. This is normally
	 * handled by the interrupt routine for the fast mode, but if
	 * the frequency rises quickly it might not react sufficiently
	 * quickly. Note that the external pin interrupt has a higher
	 * priority then any of the timer interrupts, so if the
	 * incoming frequency is too high no other interrupt routine
	 * than the external interrupt will ever be called.
	 */
	if (period < 100UL) {
	    
		//GIMSK = 0;
		EIMSK = 0;
		
	    fast_cnt.period = MAX_PERIOD;
	    fast_cnt.first_time = 1;
	    fast_cnt.current_log2num_events = 1;
	    fast_cnt.prev_ticks = cs;
		
		// The Output Compare Registers contain a 16-bit value that is continuously compared with the
		// counter value (TCNT1). A match can be used to generate an Output Compare interrupt, or to
		// generate a waveform output on the OC1x pin.
		// The Output Compare Registers are 16-bit in size. To ensure that both the high and low bytes are
		// written simultaneously when the CPU writes to these registers, the access is performed using an
		// 8-bit temporary high byte register (TEMP). This temporary register is shared by all the other 16-
		// bit registers. See “Accessing 16-bit Registers” on page 105.
	    OCR1A = (1 << fast_cnt.current_log2num_events) - 1;

	    TCNT1 = 0;
	    current = &fast_cnt;
	}
    }
}

/*
 * Force switch to slow mode. Reinitialize the fast mode
 * counter to count two events.
 */
static void slow_mode(void)
{
    cli();
    
	//GIMSK = _BV(INT0);
	EIMSK = _BV(INT0);
	
    slow_cnt.first_time = 1;
    slow_cnt.period = MAX_PERIOD;
    current = &slow_cnt;
    fast_cnt.first_time = 1;
    fast_cnt.current_log2num_events = 1;
    set_timer_cmp_reg(fast_cnt.current_log2num_events);
    TCNT1 = 0;
    sei();
}

static volatile uint8_t counter_high;
static volatile uint8_t cmp_high;

static void inline set_timer_cmp_reg(uint8_t log2ne)
{
    if (log2ne <= 16) {
	/*
	 * Set up the counter from 2 up to 2^16 events.
	 */
	OCR1A = (1UL << log2ne) - 1;
	cmp_high = 0;
    } else {
	/*
	 * Set up our extended counter to count more than
	 * 2^16 events.
	 */
	OCR1A = 0xffff;
	cmp_high = (1 << (log2ne-16)) - 1;
	counter_high = 0;
    }
}

/*
 * Desired minimum number of ticks.
 */
#define MIN_PERIOD 10000UL

/*
 * Interrupt service routine for the fast counting mode.
 */
ISR(TIM1_COMPA_vect)
{
    tick_t ticks = cli_ticks();

    fast_wd = WD_TOP;

    if (counter_high++ != cmp_high) {
	/*
	 * Counter set up to count more than 2^16 events
	 * (for high frequencies).
	 */
	return;
    }
    counter_high = 0;

    if (fast_cnt.first_time) {
	/*
	 * The very first time. We can't calculate a period.
	 */
	fast_cnt.first_time = 0;
	fast_cnt.prev_ticks = ticks;
	return;
    }

    /*
     * Calculate the result for the period that was just finished.
     */
    uint8_t log2ne = fast_cnt.current_log2num_events;
    fast_cnt.log2num_events = log2ne;
    tick_t period = fast_cnt.period = ticks - fast_cnt.prev_ticks;
    fast_cnt.prev_ticks = ticks;

    /*
     * Now see if we should adjust the number of events we are counting
     * for each period.
     */

    if (period < MIN_PERIOD && log2ne < 20) {
	/*
	 * Too short period. Count more events next time.
	 */
	do {
	    log2ne++;
	    period *= 2;
	} while (period < MIN_PERIOD && log2ne < 20);
	set_timer_cmp_reg(log2ne);
	
	//GIMSK = 0;
	EIMSK = 0;
	
	current = &fast_cnt;
	fast_cnt.current_log2num_events = log2ne;
    } else if (period > MIN_PERIOD*3 && log2ne > 1) {
	/*
	 * Too long period. Count fewer events next time.
	 */
	do {
	    log2ne--;
	    period /= 2;
	} while (period > MIN_PERIOD*3 && log2ne > 1);
	set_timer_cmp_reg(log2ne);
	fast_cnt.current_log2num_events = log2ne;
    }

    /*
     * Check if we should change mode.
     */

    if (current == &fast_cnt) {
	if (period > MIN_PERIOD*3 && log2ne == 1) {
	    /*
	     * Too long period. Switch to slow mode.
	     */
		//GIMSK = _BV(INT0);
		EIMSK = _BV(INT0);
		
	    slow_cnt.period = period / 2;
	    slow_cnt.prev_ticks = ticks;
	    slow_cnt.first_time = 1;
	    current = &slow_cnt;
	}
    } else if (slow_cnt.period < MIN_PERIOD) {
	/*
	 * Running too fast for slow mode. Switch to fast mode.
	 */
	//GIMSK = 0;
	EIMSK = 0;
	
	current = &fast_cnt;
    }
}

/* ================================================================
 *
 * Display the frequency from the last measurement.
 *
 * ================================================================
 */

// Orig
#if 0
static void display_measurement(uint8_t n, tick_t ticks)
{
    unsigned long f = 0;

    if (ticks == 0) {
		show_line("---");
    } else {
		/*
		* Here we want to calculate the frequency in dHz
		* (tenths of Hz). We use dHz instead of Hz so that
		* we don't have to use any floating point arithmetics.
		*
		* The frequency expressed in ticks is (1 << n) / ticks.
		* To get the frequency in Hz we must multiply by the
		* tick frequency, which is F_CPU / 64. To get dHz we
		* must multiply by 10. That is, the frequency in dHz
		* is:
		*
		*    10 * F_CPU / 64 * (1 << n) / ticks
		*
		* or
		*
		*    (10 * F_CPU / 64) << n / ticks
		*
		* To round up to the nearest dHz, we first add ticks / 2
		* before the division. Thus the final formula is:
		*
		*    (((10 * F_CPU / 64) << n) + ticks / 2) / ticks
		*
		*/
		if (n < 11) {
			/*
			* If F_CPU is 20Mhz, the largest n that will not
			* overflow a 32 bit unsigned integer is 10:
			*
			*  10*20000000/64 << 10 = 0xBEBC2000
			*
			* Using 32-bit arithmetic for this formula is
			* more than 3 times faster than 64-bit arithmetic.
			*/
			f = (((10UL*F_CPU/64) << n) + ticks/2) / ticks;
		} else {		/* n >= 11 */
			/*
			* Do the arithmethic in 64 bits to avoid overflow.
			*/
			unsigned long long events = ((10ULL * F_CPU/64) << n);
			f = (events + ticks/2) / ticks;
		}
		
		display_freq(f);
    }
}
#endif


//new
static unsigned long display_measurement(uint8_t n, tick_t ticks)
{
    unsigned long f = 0;

    if (ticks == 0) {
		show_line("---");
    } else {
		/*
		* Here we want to calculate the frequency in dHz
		* (tenths of Hz). We use dHz instead of Hz so that
		* we don't have to use any floating point arithmetics.
		*
		* The frequency expressed in ticks is (1 << n) / ticks.
		* To get the frequency in Hz we must multiply by the
		* tick frequency, which is F_CPU / 64. To get dHz we
		* must multiply by 10. That is, the frequency in dHz
		* is:
		*
		*    10 * F_CPU / 64 * (1 << n) / ticks
		*
		* or
		*
		*    (10 * F_CPU / 64) << n / ticks
		*
		* To round up to the nearest dHz, we first add ticks / 2
		* before the division. Thus the final formula is:
		*
		*    (((10 * F_CPU / 64) << n) + ticks / 2) / ticks
		*
		*/
		if (n < 11) {
			/*
			* If F_CPU is 20Mhz, the largest n that will not
			* overflow a 32 bit unsigned integer is 10:
			*
			*  10*20000000/64 << 10 = 0xBEBC2000
			*
			* Using 32-bit arithmetic for this formula is
			* more than 3 times faster than 64-bit arithmetic.
			*/
			f = (((10UL*F_CPU/64) << n) + ticks/2) / ticks;
		} else {		/* n >= 11 */
			/*
			* Do the arithmethic in 64 bits to avoid overflow.
			*/
			unsigned long long events = ((10ULL * F_CPU/64) << n);
			f = (events + ticks/2) / ticks;
		}
		
		
		// display_freq(f);
		return f;
		
    }
}



// todo: finally it will be not necessary.
static void show_line(char* s)
{
    int i;
    static char prev_line[9];

    if (strcmp(s, prev_line) == 0) {
	return;			/* No change */
    }

    lcd_home();
    for (i = 0; i < 8 && s[i]; i++) {
	prev_line[i] = s[i];
	lcd_putc(s[i]);
    }
    prev_line[i] = '\0';
    while (i < 8) {
	lcd_putc(' ');
	i++;
    }
}

/*
 * Frequency ranges.
 */
static struct {
    unsigned long min;		/* Lowest value for this range */
    unsigned long max;		/* Highest value for this range */
    signed char point;		/* Position of decimal point */
    signed char lsd;		/* Position of least significant digit */
    unsigned int divisor;	/* Divisor */
    uint8_t prefix;		/* Hz prefix character */
} range[] = {
    /*  01234567         min         max   . lsd   div  prefix */
    {/*  999.9Hz */       0UL,     9999UL, 4, 5,     1, ' '},
    {/* 9.999kHz */    9900UL,    99999UL, 1, 4,    10, 'k'},
    {/* 99.99kHz */   99000UL,   999999UL, 2, 4,   100, 'k'},
    {/* 999.9kHz */  990000UL,  9999999UL, 3, 4,  1000, 'k'},
    {/* 9.999MHz */ 9900000UL, 99999999UL, 1, 4, 10000, 'M'},
};
static unsigned int curr_range = 0;

// old
#if 0
/* Frequency in dHz (10dHz = 1Hz). */
static void display_freq(unsigned long freq)
{
    char line[9];
    signed char pos;
    signed char point;
    static unsigned long prev_freq = 0xffffffffUL;

    if (freq == prev_freq) {
	return;
    }
    prev_freq = freq;

    if (freq == 0) {
	show_line("---");
	return;
    }

    /*
     * See if we'll need to change range.
     */

    while (freq > range[curr_range].max) {
	curr_range++;
    }
    while (freq < range[curr_range].min) {
	curr_range--;
    }

    /*
     * Now format the frequency within the range.
     *
     * First set up the end of the line (prefix + "Hz").
     */

    line[5] = range[curr_range].prefix;
    line[6] = 'H';
    line[7] = 'z';
    line[8] = '\0';

    pos = range[curr_range].lsd;
    point = range[curr_range].point;
    freq /= range[curr_range].divisor;

    /* Format digits to the right of the decimal point */
    while (pos > point) {
	line[pos--] = freq % 10 + '0';
	freq /= 10;
    }

    /*
     * Fill in the decimal point and one digit to the left
     * of the decimal point.
     */
    line[pos--] = '.';
    line[pos--] = freq % 10 + '0';
    freq /= 10;

    /* Fill in non-zero digits to the left of the decimal point. */
    while (freq) {
	line[pos--] = freq % 10 + '0';
	freq /= 10;
    }

    /* Out of significant digits. Fill in spaces. */
    while (pos >= 0) {
	line[pos--] = ' ';
    }

    show_line(line);
}
#endif


// new
// todo: new implementation
/* Frequency in dHz (10dHz = 1Hz). */
static void display_freq(unsigned long freq, unsigned char *buff)  //todo: there is no check for buffer len
{
    char line[9];
    signed char pos;
    signed char point;
    static unsigned long prev_freq = 0xffffffffUL;

    if (freq == prev_freq) {
		return;
    }
    prev_freq = freq;

    if (freq == 0) {
		show_line("---");
		return;
    }

    /*
     * See if we'll need to change range.
     */

    while (freq > range[curr_range].max) {
		curr_range++;
    }
    while (freq < range[curr_range].min) {
		curr_range--;
    }

    /*
     * Now format the frequency within the range.
     *
     * First set up the end of the line (prefix + "Hz").
     */

    line[5] = range[curr_range].prefix;
    line[6] = 'H';
    line[7] = 'z';
    line[8] = '\0';

    pos = range[curr_range].lsd;
    point = range[curr_range].point;
    freq /= range[curr_range].divisor;

    /* Format digits to the right of the decimal point */
    while (pos > point) {
		line[pos--] = freq % 10 + '0';
		freq /= 10;
    }

    /*
     * Fill in the decimal point and one digit to the left
     * of the decimal point.
     */
    line[pos--] = '.';
    line[pos--] = freq % 10 + '0';
    freq /= 10;

    /* Fill in non-zero digits to the left of the decimal point. */
    while (freq) {
		line[pos--] = freq % 10 + '0';
		freq /= 10;
    }

    /* Out of significant digits. Fill in spaces. */
    while (pos >= 0) {
		line[pos--] = ' ';
    }

	
    //show_line(line);
	strcpy(buff, line);
	
}




///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////



void getTicksT0(char *buff)
{
	tick_t t = 0;


//	cli();
//	t = cli_ticks();
//	sei();

	t = 11223344;

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


/* ================================================================
 * HD44780 display support.
 * ================================================================
 */
/*
static void lcd_init(void)
{
	HD44780_Init();
}


static void lcd_home(void)
{
    HD44780_Home();
}


static void lcd_putc(char c)
{
    HD44780_Putc(c);
}
*/



// ================================================================
// DOG display support.
// ================================================================
//

/*

//
// Define the different display models.
//
// M081 1 line by 8 chars.
// M162 2 lines by 16 chars.
// M163 3 lines by 16 chars.
//

#define DOG_LCD_M081 81
#define DOG_LCD_M162 82
#define DOG_LCD_M163 83
#define DOG_MODEL DOG_LCD_M081

#define DOG_LCD_CONTRAST 0x28


// All pins for the DOG display must be connected to the same port.

#define DOG_DDR DDRA
#define DOG_PIN PINA
#define DOG_PORT PORTA

#define DOG_SI_BIT  PA0
#define DOG_CLK_BIT PA1
#define DOG_CSB_BIT PA2
#define DOG_RS_BIT  PA3

#define DOG_ALL_BITS (_BV(DOG_SI_BIT) | _BV(DOG_CLK_BIT) | _BV(DOG_CSB_BIT) | _BV(DOG_RS_BIT))

static void spi_transfer(uint8_t value);
static void set_instruction_set(uint8_t is);
static inline void write_command(uint8_t value, unsigned execution_time)
    __attribute__ ((always_inline));
static void set_instruction_set(uint8_t is);
static inline void execute(uint8_t value, unsigned execution_time)
    __attribute__ ((always_inline));

static void lcd_init(void)
{
    DOG_DDR |= DOG_ALL_BITS;
    DOG_PORT |= DOG_ALL_BITS;

    // The commands that follow are in instruction set 1. 
    set_instruction_set(1);

    // Bias 1/4. 
    write_command(0x1D, 30);

    // Set up contrast. (For 5V.) 
    write_command(0x50 | (DOG_LCD_CONTRAST>>4), 30);
    write_command(0x70 | (DOG_LCD_CONTRAST & 0x0F), 30);

    // Set amplification ratio for the follower control. 
    write_command(0x69, 30);

    // Get back to the default instruction set. 
    set_instruction_set(0);

    // Clear the display 
    write_command(0x01, 1100);

    // Move cursor left to right; no autoscroll. 
    write_command(0x04 | 0x02, 30);

    // Display on; no cursor; no blink. 
    write_command(0x08 | 0x04, 30);
}

static void lcd_home(void)
{
    write_command(0x80, 30);
}

static void lcd_putc(char c)
{
    DOG_PORT |= _BV(DOG_RS_BIT);
    execute((uint8_t) c, 30);
}

//
// Set instruction set. 'is' is in the range 1..3.
//
static void set_instruction_set(uint8_t is)
{
#if DOG_MODEL == DOG_LCD_M081
    const uint8_t template = 0x30;
#else
    const uint8_t template = 0x38;
#endif
    write_command(template | is, 30);
}

static void write_command(uint8_t value, unsigned execution_time)
{
    DOG_PORT &= ~_BV(DOG_RS_BIT);
    execute(value, execution_time);
}

static void inline execute(uint8_t value, unsigned execution_time)
{
    spi_transfer(value);
    _delay_us(execution_time);
}

static void spi_transfer(uint8_t value)
{
    int i;

    DOG_PORT |= _BV(DOG_CLK_BIT);
    DOG_PORT &= ~_BV(DOG_CSB_BIT);
    for (i = 7; i >= 0; i--) {
	if (value & _BV(i)) {
	    DOG_PORT |= _BV(DOG_SI_BIT);
	} else {
	    DOG_PORT &= ~_BV(DOG_SI_BIT);
	}
	DOG_PIN |= _BV(DOG_CLK_BIT);
	_delay_us(1);
	DOG_PIN |= _BV(DOG_CLK_BIT);
    }
    DOG_PORT |= _BV(DOG_CSB_BIT);
}

*/