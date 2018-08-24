 
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


// Debug pins

// DEBUG PIN 0
#define DEBUG0_PIN_CONFIG	DDRD |=  (1 << PD0)
#define DEBUG0_PIN_SET		PORTD |= (1 << PD0)
#define DEBUG0_PIN_RESET	PORTD &= ~(1 << PD0)

// DEBUG PIN 1
#define DEBUG1_PIN_CONFIG	DDRD |=  (1 << PD1)
#define DEBUG1_PIN_SET		PORTD |= (1 << PD1)
#define DEBUG1_PIN_RESET	PORTD &= ~(1 << PD1)

// DEBUG PIN 2
#define DEBUG2_PIN_CONFIG	DDRD |=  (1 << PD2)
#define DEBUG2_PIN_SET		PORTD |= (1 << PD2)
#define DEBUG2_PIN_RESET	PORTD &= ~(1 << PD2)


void FREQCNT_Init(void);
uint32_t FREQCNT_GetFrequencyHz();
void FREQCNT_GetFrequencyTxt(unsigned char *buff);

// tests
void InitWindowTimer(void);
void InitSampleCounter(void);
void getTicksT0(char *buff);
void getT1Counts(char *buff);

