#ifndef __I2C_LCD_H__
#define __I2C_LCD_H__

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define LCD_ADDR (0x27 << 1)   // typical PCF8574 backpack address, adjust if needed

void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_put_cur(uint8_t row, uint8_t col);
void lcd_clear(void);

#endif

