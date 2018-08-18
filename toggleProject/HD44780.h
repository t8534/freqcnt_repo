

//todo:
// 1.
// Add descriptions to the interfaces
//
// 2.
// Move into .c communication wih uP, or into stay in HD44780.h, but
// move interface functions into HD44780_if.h


#pragma once

#include <avr/io.h>


///////////////////////////////////////////////////////////////////////////////
// Communication with uP
///////////////////////////////////////////////////////////////////////////////
		
#define HD44780_DATA_NIBBLE_CONFIG	DDRC |= 0b00001111;  // 1 - output for PC0 to PC3
#define HD44780_DATA_NIBBLE_WRITE(nibble)        do {                                    \
                                                 PORTC = PORTC & 0xf0;               \
                                                 PORTC = PORTC | (nibble & 0x0F);    \
                                             } while(0)

#define HD44780_RS_CONFIG	DDRC |=  (1 << PC5)
#define HD44780_RS_SET		PORTC |= (1 << PC5) 
#define HD44780_RS_RESET	PORTC &= ~(1 << PC5) 

#define HD44780_RW_CONFIG	DDRB |=  (1 << PB0);
#define HD44780_RW_SET		PORTB |= (1 << PB0)
#define HD44780_RW_RESET	PORTB &= ~(1 << PB0)

#define HD44780_EN_CONFIG	DDRC |=  (1 << PC4);
#define HD44780_EN_SET		PORTC |= (1 << PC4)
#define HD44780_EN_RESET	PORTC &= ~(1 << PC4)

///////////////////////////////////////////////////////////////////////////////

#define HD44780_COL_COUNT 16
#define HD44780_ROW_COUNT 2

// The rest should be left alone
#define HD44780_CLEARDISPLAY   0x01
#define HD44780_RETURNHOME     0x02
#define HD44780_ENTRYMODESET   0x04
#define HD44780_DISPLAYCONTROL 0x08
#define HD44780_CURSORSHIFT    0x10
#define HD44780_FUNCTIONSET    0x20
#define HD44780_SETCGRAMADDR   0x40
#define HD44780_SETDDRAMADDR   0x80

#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

#define HD44780_DISPLAYON  0x04
#define HD44780_DISPLAYOFF 0x00
#define HD44780_CURSORON   0x02
#define HD44780_CURSOROFF  0x00
#define HD44780_BLINKON    0x01
#define HD44780_BLINKOFF   0x00

#define HD44780_DISPLAYMOVE 0x08
#define HD44780_CURSORMOVE  0x00
#define HD44780_MOVERIGHT   0x04
#define HD44780_MOVELEFT    0x00

#define HD44780_8BITMODE 0x10
#define HD44780_4BITMODE 0x00
#define HD44780_2LINE    0x08
#define HD44780_1LINE    0x00
#define HD44780_5x10DOTS 0x04
#define HD44780_5x8DOTS  0x00

void HD44780_init(void);

void HD44780_command(uint8_t command);
void HD44780_write(uint8_t value);

void HD44780_on(void);
void HD44780_off(void);

void HD44780_clear(void);
void HD44780_return_home(void);

void HD44780_enable_blinking(void);
void HD44780_disable_blinking(void);

void HD44780_enable_cursor(void);
void HD44780_disable_cursor(void);

void HD44780_scroll_left(void);
void HD44780_scroll_right(void);

void HD44780_set_left_to_right(void);
void HD44780_set_right_to_left(void);

void HD44780_enable_autoscroll(void);
void HD44780_disable_autoscroll(void);

void HD44780_create_char(uint8_t location, uint8_t *charmap);

void HD44780_set_cursor(uint8_t col, uint8_t row);

void HD44780_puts(char *string);
void HD44780_printf(char *format, ...);
