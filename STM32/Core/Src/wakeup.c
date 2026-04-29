#include "wakeup.h"
#include "ds3231.h"
#include "log.h"
#include "main.h"
#include "user_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

uint8_t current_minute = 0;

uint8_t next_valid_minute = UNINITIALIZED;
uint8_t active_until_minute = UNINITIALIZED;

void WakeUp_Init(rtc_time_t now) {
  WakeUp_SetCurrentMinute(now.minutes);
  LOGF("Initializing WakeUp module with current minute: %02d\r\n",
       current_minute);
  WakeUp_CalculateNextValidMinute();
  LOGF("Initial next valid minute: %02d\r\n", next_valid_minute);
}

void WakeUp_SetCurrentMinute(uint8_t minute) { current_minute = minute; }

void WakeUp_CalculateNextValidMinute(void) {
  if (next_valid_minute == UNINITIALIZED) {
    if (ENABLE_DELAYED_START && current_minute != STARTING_MINUTE) {
      LOGF("Setting next valid minute to starting minute %02d\r\n",
           STARTING_MINUTE);
      next_valid_minute = STARTING_MINUTE;
      return;

    } else {

      LOGF("Calculating next valid minute based on current minute %02d and "
           "configured interval of %u minutes\r\n",
           current_minute, (unsigned int)INTERVAL_BETWEEN_REPEATS_MINUTES);
      next_valid_minute =
          (current_minute + INTERVAL_BETWEEN_REPEATS_MINUTES) % 60;

      return;
    }
  } else {
    LOGF("Calculating next valid minute based on last valid minute %02d and "
         "configured interval of %u minutes\r\n",
         next_valid_minute, (unsigned int)INTERVAL_BETWEEN_REPEATS_MINUTES);
    next_valid_minute =
        (next_valid_minute + INTERVAL_BETWEEN_REPEATS_MINUTES) % 60;
  }
}

bool WakeUp_CheckCurrentMinute(rtc_time_t now) {
  WakeUp_SetCurrentMinute(now.minutes);
  LOGF("Current minute: %02d\r\n", current_minute);

  bool is_valid_minute = (current_minute == next_valid_minute);

  if (is_valid_minute) {
    LOGF("Valid minute hit!\r\n");
    if (INTERVAL_BETWEEN_REPEATS_MINUTES > 5) {
      active_until_minute = (current_minute + 4) % 60;
    }
    WakeUp_CalculateNextValidMinute();
    LOGF("Next valid minute after this window: %02d\r\n", next_valid_minute);
  }

  if (active_until_minute != UNINITIALIZED) {
    if (current_minute > active_until_minute) {
      LOGF("Active window ended.\r\n");
      active_until_minute = UNINITIALIZED;
      return false;
    }
    LOGF("Currently in active window.\r\n");
    return true;
  }

  return is_valid_minute; // Return true if valid minute hit
}

// 1 second off at 12:19:58 (really 12:19:59)