#include "ds3231.h"
#include "main.h" // for I2C handle

extern I2C_HandleTypeDef hi2c1; // defined in i2c.c

#define DS3231_ADDR (0x68 << 1) // 7-bit addr shifted for HAL

// Helpers for BCD <-> binary
static uint8_t bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

bool DS3231_ReadTime(rtc_time_t *t) {
  uint8_t reg = 0x00;
  uint8_t buf[7];

  if (HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDR, &reg, 1, 100) != HAL_OK)
    return false;
  if (HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDR, buf, 7, 100) != HAL_OK)
    return false;

  t->seconds = bcd2bin(buf[0] & 0x7F);
  t->minutes = bcd2bin(buf[1] & 0x7F);
  t->hours = bcd2bin(buf[2] & 0x3F); // 24h mode
  t->day = bcd2bin(buf[3] & 0x07);
  t->date = bcd2bin(buf[4] & 0x3F);
  t->month = bcd2bin(buf[5] & 0x1F);
  t->year = 2000 + bcd2bin(buf[6]);

  return true;
}

bool DS3231_SetTime(const rtc_time_t *t) {
    // Buffer for writing time (register 0x00 .. 0x06)
    uint8_t timebuf[8];
    timebuf[0] = 0x00; // start register
    timebuf[1] = bin2bcd(t->seconds);
    timebuf[2] = bin2bcd(t->minutes);
    timebuf[3] = bin2bcd(t->hours);
    timebuf[4] = bin2bcd(t->day);
    timebuf[5] = bin2bcd(t->date);
    timebuf[6] = bin2bcd(t->month);
    timebuf[7] = bin2bcd(t->year % 100);

    if (HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDR,
                                timebuf, sizeof(timebuf), 100) != HAL_OK) {
        return false;
    }

    // --- Clear Oscillator Stop Flag (OSF) ---
    uint8_t reg = 0x0F;   // Status register
    uint8_t status;

    if (HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDR, &reg, 1, 100) != HAL_OK)
        return false;
    if (HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDR, &status, 1, 100) != HAL_OK)
        return false;

    status &= ~(1 << 7);  // clear OSF bit

    uint8_t statbuf[2] = {0x0F, status};
    if (HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDR, statbuf, 2, 100) != HAL_OK)
        return false;

    return true;
}
