#include "i2c-lcd.h"

extern I2C_HandleTypeDef hi2c1;  // use I2C1

static void lcd_send(uint8_t data, uint8_t rs)
{
    uint8_t data_t[4];
    uint8_t highnib = data & 0xF0;
    uint8_t lownib  = (data << 4) & 0xF0;

    data_t[0] = highnib | rs | 0x08; // en=1, rs as passed, backlight=1
    data_t[1] = highnib | rs | 0x0C; // en=0
    data_t[2] = lownib  | rs | 0x08;
    data_t[3] = lownib  | rs | 0x0C;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100);
}

void lcd_send_cmd(char cmd)
{
    lcd_send(cmd, 0x00);
}

void lcd_send_data(char data)
{
    lcd_send(data, 0x01);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_put_cur(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_cmd(addr);
}

void lcd_send_string(char *str)
{
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_init(void)
{
    HAL_Delay(50); // wait for >40ms after power on
    lcd_send_cmd(0x30);
    HAL_Delay(5);
    lcd_send_cmd(0x30);
    HAL_Delay(1);
    lcd_send_cmd(0x30);
    lcd_send_cmd(0x20); // 4-bit mode

    // function set, display on/off, clear, entry mode
    lcd_send_cmd(0x28); // 2 line, 5x8 matrix
    lcd_send_cmd(0x08); // display off
    lcd_send_cmd(0x01); // clear
    HAL_Delay(2);
    lcd_send_cmd(0x06); // entry mode set
    lcd_send_cmd(0x0C); // display on, cursor off
}
