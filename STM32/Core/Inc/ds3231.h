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
bool DS3231_ReadTime(rtc_time_t *t);
bool DS3231_SetTime(const rtc_time_t *t);

#endif // DS3231_H
