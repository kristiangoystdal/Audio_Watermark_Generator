#include "ds3231.h"
#include "main.h"
#include "stm32g4xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

#define DS3231_ADDR (0x68 << 1)

/* ---------------------- BCD HELPERS ---------------------- */

static uint8_t decToBcd(int val) {
  return (uint8_t)((val / 10 * 16) + (val % 10));
}

static int bcdToDec(uint8_t val) { return (int)((val / 16 * 10) + (val % 16)); }

/* ---------------------- TIME FUNCTIONS ---------------------- */

void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom,
              uint8_t month, uint16_t year) {
  uint8_t buf[7];

  buf[0] = decToBcd(sec);
  buf[1] = decToBcd(min);
  buf[2] = decToBcd(hour) & 0x3F; // 24h mode
  buf[3] = decToBcd(dow);
  buf[4] = decToBcd(dom);
  buf[5] = decToBcd(month);
  buf[6] = decToBcd(year % 100);

  HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 7,
                    HAL_MAX_DELAY);
}

void Get_Time(rtc_time_t *t) {
  uint8_t buf[7];

  if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 7,
                       HAL_MAX_DELAY) != HAL_OK) {
    t->seconds = t->minutes = t->hours = 0;
    t->day = t->date = t->month = 0;
    t->year = 2000;
    return;
  }

  t->seconds = bcdToDec(buf[0] & 0x7F);
  t->minutes = bcdToDec(buf[1] & 0x7F);
  t->hours = bcdToDec(buf[2] & 0x3F);

  t->day = bcdToDec(buf[3] & 0x07);
  t->date = bcdToDec(buf[4] & 0x3F);
  t->month = bcdToDec(buf[5] & 0x1F);

  t->year = 2000 + bcdToDec(buf[6]);
}

/* ---------------------- TEMP READ ---------------------- */

void Read_Temperature(int8_t *temp) {
  uint8_t msb;

  if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDR, 0x11, I2C_MEMADD_SIZE_8BIT, &msb, 1,
                       HAL_MAX_DELAY) != HAL_OK) {
    *temp = 0;
    return;
  }

  *temp = (int8_t)msb;
}

/* ---------------------- INITIALIZE ---------------------- */

void DS3231_Init(void) {
  uint8_t sec;

  // Read seconds register
  if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &sec, 1,
                       HAL_MAX_DELAY) != HAL_OK) {
    return;
  }

  // Clear CH bit (bit 7)
  sec &= 0x7F;

  // Write back
  HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &sec, 1,
                    HAL_MAX_DELAY);
}

/* ---------------------- ALARM1 CONFIG ---------------------- */
/*
   Trigger EVERY MINUTE at seconds == 00
   A1M1 = 0 (match seconds)
   A1M2 = 1 (ignore minutes)
   A1M3 = 1 (ignore hour)
   A1M4 = 1 (ignore date/day)
*/

void Set_Alarm1_Every_Minute(void) {
  uint8_t alarm[4];

  // Trigger when seconds == 00
  alarm[0] = decToBcd(0); // seconds = 00, A1M1 = 0
  alarm[1] = 0x80;        // A1M2 = 1
  alarm[2] = 0x80;        // A1M3 = 1
  alarm[3] = 0x80;        // A1M4 = 1

  HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, alarm, 4,
                    HAL_MAX_DELAY);
}

void Enable_Alarm1(void) {
  uint8_t control;

  HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDR, 0x0E, I2C_MEMADD_SIZE_8BIT, &control, 1,
                   HAL_MAX_DELAY);

  control |= 0x01;  // A1IE  = 1 (enable Alarm1 interrupt)
  control |= 0x04;  // INTCN = 1 (use INT pin instead of SQW)
  control &= ~0x40; // Ensure BBSQW = 0

  HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDR, 0x0E, I2C_MEMADD_SIZE_8BIT, &control,
                    1, HAL_MAX_DELAY);
}

void Clear_Alarm1_Flag(void) {
  uint8_t status;

  HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT, &status, 1,
                   HAL_MAX_DELAY);

  status &= ~0x01; // clear A1F

  HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT, &status, 1,
                    HAL_MAX_DELAY);
}
