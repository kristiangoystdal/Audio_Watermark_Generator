#ifndef WAKEUP_H
#define WAKEUP_H

#include "ds3231.h"
#include "main.h"
#include "user_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#define UNINITIALIZED 0xFF

void WakeUp_Init(rtc_time_t now);
void WakeUp_SetCurrentMinute(uint8_t minute);
void WakeUp_CalculateNextValidMinute(void);
bool WakeUp_CheckCurrentMinute(rtc_time_t now);

#endif // WAKEUP_H