#include "ds3231.h"
#include "log.h"
#include "main.h"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_gpio.h>

extern I2C_HandleTypeDef hi2c2; // defined in i2c.c

#define DS3231_REG_A1 0x07
#define DS3231_REG_CTRL 0x0E
#define DS3231_REG_STAT 0x0F

#define DS3231_ADDR (0x68 << 1) // 7-bit address shifted for HAL

// Initialize DS3231
void DS3231_Init(void) {
  uint8_t sec;

  // Read current seconds register
  if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &sec, 1,
                       1000) != HAL_OK) {
    return;
  }

  // Clear CH (bit 7)
  sec &= 0x7F;

  // Write it back
  if (HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &sec,
                        1, 1000) != HAL_OK) {
    return;
  }
  return;
}

// Power on the DS3231 by setting the control pin high
void DS3231_PowerOn(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  LOGF("DS3231 Power ON\r\n");

  uint32_t start = HAL_GetTick();
  while (HAL_GetTick() - start < 100) {
    if (HAL_I2C_IsDeviceReady(&hi2c2, DS3231_ADDR, 1, 10) == HAL_OK) {
      LOGF("DS3231 ready after %lu ms\r\n", HAL_GetTick() - start);
      return;
    }
  }
  LOGF("DS3231 not responding after 100ms\r\n");
}

// Power off the DS3231 by setting the control pin low
void DS3231_PowerOff(void) {
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  LOGF("DS3231 Power OFF\r\n");
}

// Convert normal decimal numbers to binary coded decimal
static uint8_t DS3231_DecToBcd(int val) {
  return (uint8_t)((val / 10 * 16) + (val % 10));
}

// Convert binary coded decimal to normal decimal numbers
static int DS3231_BcdToDec(uint8_t val) {
  return (int)((val / 16 * 10) + (val % 16));
}

// Set time/date into DS3231
void DS3231_SetTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow,
                    uint8_t dom, uint8_t month, uint16_t year) {
  uint8_t set_time[7];
  set_time[0] = DS3231_DecToBcd(sec);         // Seconds
  set_time[1] = DS3231_DecToBcd(min);         // Minutes
  set_time[2] = DS3231_DecToBcd(hour) & 0x3F; // Hours (24h mode)
  set_time[3] = DS3231_DecToBcd(dow);         // Day of week (1–7)
  set_time[4] = DS3231_DecToBcd(dom);         // Date (1–31)
  set_time[5] = DS3231_DecToBcd(month);       // Month (1–12)
  set_time[6] = DS3231_DecToBcd(year % 100);  // Year (00–99)

  HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, set_time,
                    7, 1000);
  LOGF("RTC Time Set: %02d:%02d:%02d %02d/%02d/%04d\r\n", hour, min, sec, dom,
       month, year);
}

// Read time/date from DS3231
void DS3231_GetTime(rtc_time_t *time) {
  LOGF("Reading time from DS3231...\r\n");
  uint8_t get_time[7];
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
      &hi2c2, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, get_time, 7, 1000);
  if (ret != HAL_OK) {
    LOGF("Failed to read time from DS3231, HAL error=%d, I2C error=0x%lX\r\n",
         ret, HAL_I2C_GetError(&hi2c2));

    time->seconds = 0;
    time->minutes = 0;
    time->hours = 0;
    time->day = 0;
    time->date = 0;
    time->month = 0;
    time->year = 2000;

    return;
  }

  time->seconds = DS3231_BcdToDec(get_time[0] & 0x7F);
  time->minutes = DS3231_BcdToDec(get_time[1] & 0x7F);
  time->hours = DS3231_BcdToDec(get_time[2] & 0x3F); // 24h
  time->day = DS3231_BcdToDec(get_time[3] & 0x07);   // Day of week (1–7)
  time->date = DS3231_BcdToDec(get_time[4] & 0x3F);
  time->month = DS3231_BcdToDec(get_time[5] & 0x1F);
  time->year = 2000 + DS3231_BcdToDec(get_time[6]);

  LOGF("Current Time: %02d:%02d:%02d %02d/%02d/%04d\r\n", time->hours,
       time->minutes, time->seconds, time->date, time->month, time->year);

  return;
}

// Read temperature from DS3231 (integer °C only)
void DS3231_ReadTemperature(int8_t *temperature) {
  uint8_t msb;
  if (HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDR, 0x11, I2C_MEMADD_SIZE_8BIT, &msb, 1,
                       1000) != HAL_OK) {
    *temperature = 0;
    LOGF("Failed to read temperature from DS3231\r\n");
    return;
  }

  *temperature = (int8_t)msb; // signed integer °C

  return;
}

void DS3231_FirstBootSetup(void) {}