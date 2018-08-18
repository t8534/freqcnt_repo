
// todo:
// 1.
// Add static to functions which are external
//



#include "HD44780.h"

#include <stdarg.h>
#include <stdio.h>
#include <util/delay.h>

void HD44780_send(uint8_t value, uint8_t mode);
void HD44780_write_nibble(uint8_t nibble);

static uint8_t HD44780_displayparams;
static char HD44780_buffer[HD44780_COL_COUNT + 1];

void HD44780_command(uint8_t command) {
  HD44780_send(command, 0);
}

void HD44780_write(uint8_t value) {
  HD44780_send(value, 1);
}

void HD44780_send(uint8_t value, uint8_t mode) {
  _delay_ms(1);
  if (mode) {
	HD44780_RS_SET;
  } else {
	HD44780_RS_RESET;
  }

  _delay_ms(1);
  HD44780_RW_RESET;
  _delay_ms(1);

  HD44780_write_nibble(value >> 4);
  _delay_ms(5);
  HD44780_write_nibble(value);
}

void HD44780_write_nibble(uint8_t nibble) {
  
  _delay_ms(5);
  HD44780_DATA_NIBBLE_WRITE(nibble);
  _delay_ms(1);
  HD44780_EN_RESET;
  _delay_ms(1);
  HD44780_EN_SET;
  _delay_ms(1);
  HD44780_EN_RESET;
  _delay_ms(1);  // If delay less than 0.3 ms this value, the data is not correctly displayed  

}

void HD44780_init(void) {
 
  HD44780_RS_CONFIG;
  HD44780_RW_CONFIG;
  HD44780_EN_CONFIG;
  HD44780_DATA_NIBBLE_CONFIG;
  
  // Wait for LCD to become ready (docs say 15ms+)
  _delay_ms(15);

  HD44780_EN_RESET;
  HD44780_RS_RESET;
  HD44780_RW_RESET;

  _delay_ms(4.1);

  HD44780_write_nibble(0x03); // Switch to 4 bit mode
  _delay_ms(5);
  HD44780_write_nibble(0x03); // 2nd time
  _delay_ms(5);
  HD44780_write_nibble(0x03); // 3rd time
  _delay_ms(5);
  HD44780_write_nibble(0x02); // Set 8-bit mode (?)

  HD44780_command(HD44780_FUNCTIONSET | HD44780_4BITMODE | HD44780_2LINE | HD44780_5x8DOTS);
  HD44780_displayparams = HD44780_CURSOROFF | HD44780_BLINKOFF;
  HD44780_command(HD44780_DISPLAYCONTROL | HD44780_displayparams);

}

void HD44780_on(void) {
  HD44780_displayparams |= HD44780_DISPLAYON;
  HD44780_command(HD44780_DISPLAYCONTROL | HD44780_displayparams);
}

void HD44780_off(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams &= ~HD44780_DISPLAYON;
  HD44780_command(HD44780_DISPLAYCONTROL | HD44780_displayparams);
}

void HD44780_clear(void) {
  HD44780_command(HD44780_CLEARDISPLAY);
  // This is important delay, if too short or not, display is displaying wrong characters
  // or not.
  _delay_ms(50);   
}

void HD44780_return_home(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_command(HD44780_RETURNHOME);
  _delay_ms(2);
}

void HD44780_enable_blinking(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams |= HD44780_BLINKON;
  HD44780_command(HD44780_DISPLAYCONTROL | HD44780_displayparams);
}

void HD44780_disable_blinking(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams &= ~HD44780_BLINKON;
  HD44780_command(HD44780_DISPLAYCONTROL | HD44780_displayparams);
}

void HD44780_enable_cursor(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams |= HD44780_CURSORON;
  HD44780_command(HD44780_DISPLAYCONTROL | HD44780_displayparams);
}

void HD44780_disable_cursor(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams &= ~HD44780_CURSORON;
  HD44780_command(HD44780_DISPLAYCONTROL | HD44780_displayparams);
}

void HD44780_scroll_left(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_command(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}

void HD44780_scroll_right(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_command(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}

void HD44780_set_left_to_right(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams |= HD44780_ENTRYLEFT;
  HD44780_command(HD44780_ENTRYMODESET | HD44780_displayparams);
}

void HD44780_set_right_to_left(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams &= ~HD44780_ENTRYLEFT;
  HD44780_command(HD44780_ENTRYMODESET | HD44780_displayparams);
}

void HD44780_enable_autoscroll(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams |= HD44780_ENTRYSHIFTINCREMENT;
  HD44780_command(HD44780_ENTRYMODESET | HD44780_displayparams);
}

void HD44780_disable_autoscroll(void) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_displayparams &= ~HD44780_ENTRYSHIFTINCREMENT;
  HD44780_command(HD44780_ENTRYMODESET | HD44780_displayparams);
}

void HD44780_create_char(uint8_t location, uint8_t *charmap) {
  _delay_ms(5);  //todo: test is it necessary
  HD44780_command(HD44780_SETCGRAMADDR | ((location & 0x7) << 3));
  for (int i = 0; i < 8; i++) {
    HD44780_write(charmap[i]);
  }
}

void HD44780_set_cursor(uint8_t col, uint8_t row) {
  static uint8_t offsets[] = { 0x00, 0x40, 0x14, 0x54 };

  if (row > 1) {
    row = 1;
  }

  _delay_ms(5);  //todo: test is it necessary
  HD44780_command(HD44780_SETDDRAMADDR | (col + offsets[row]));
}

void HD44780_puts(char *string) {
  for (char *it = string; *it; it++) {
	//_delay_ms(2);   // seems to be not necessary, but in case something is wrong add it
    HD44780_write(*it);
  }
}

void HD44780_printf(char *format, ...) {
  va_list args;

  va_start(args, format);
  vsnprintf(HD44780_buffer, HD44780_COL_COUNT + 1, format, args);
  va_end(args);

  HD44780_puts(HD44780_buffer);
}
