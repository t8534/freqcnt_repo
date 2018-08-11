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

 
 // TODO: There is no blocker for many times include this file.
 
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void FREQCNT_Init(void);
void FREQCNT_Cyclic100ms(void);  //todo: period of call should be not related to frequency measurement window.
uint32_t FREQCNT_GetFrequencyHz(void)
void FREQCNT_GetFrequencyTxt(uint8_t *buff);





