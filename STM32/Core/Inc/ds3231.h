#ifndef DS3231_H
#define DS3231_H

#include <stdbool.h>
#include <stdint.h>

// Simple struct for time/date
typedef struct {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours; // 24h format
  uint8_t day;   // 1–7 (Mon=1 … Sun=7)
  uint8_t date;  // 1–31
  uint8_t month; // 1–12
  uint16_t year; // e.g. 2025
} rtc_time_t;

// Public functions
void Get_Time(rtc_time_t *time);
void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom,
              uint8_t month, uint16_t year);
void DS3231_Init(void);
void Read_Temperature(int8_t *temperature);

void Set_Alarm(rtc_time_t *alarm);
void DS3231_ClearAllAlarms(void);

#endif // DS3231_H
