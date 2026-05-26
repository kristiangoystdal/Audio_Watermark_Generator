#include "wakeup.h"
#include "ds3231.h"
#include "log.h"
#include "main.h"
#include "user_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

uint16_t current_hour = 0;
uint16_t current_minute = 0;

uint16_t next_valid_hour = UNINITIALIZED;
uint16_t next_valid_minute = UNINITIALIZED;

uint16_t schedule_start_hour = STARTING_HOUR;
uint16_t schedule_start_minute = STARTING_MINUTE;
uint16_t schedule_end_hour = END_HOUR;
uint16_t schedule_end_minute = END_MINUTE;

uint16_t schedule_start_total_minutes = 0;
uint16_t schedule_end_total_minutes = 0;

uint16_t schedule_interval_minutes = RUN_MINUTES + SLEEP_MINUTES;

void WakeUp_TimeToMinutes(uint16_t hour, uint16_t minute,
                          uint16_t *out_minutes) {
  *out_minutes = hour * 60 + minute;
}

void WakeUp_Init(rtc_time_t now) {
  // Convert schedule start and end times to total minutes for easier comparison
  WakeUp_TimeToMinutes(schedule_start_hour, schedule_start_minute,
                       &schedule_start_total_minutes);
  WakeUp_TimeToMinutes(schedule_end_hour, schedule_end_minute,
                       &schedule_end_total_minutes);

  LOGF("Schedule start time: %02d:%02d (total minutes: %u)\r\n",
       schedule_start_hour, schedule_start_minute,
       schedule_start_total_minutes);
  LOGF("Schedule end time: %02d:%02d (total minutes: %u)\r\n",
       schedule_end_hour, schedule_end_minute, schedule_end_total_minutes);
  LOGF("Schedule interval: %u minutes\r\n", schedule_interval_minutes);
  LOGF("Current time: %02d:%02d\r\n", now.hours, now.minutes);
  WakeUp_SetCurrentTime(now.hours, now.minutes);
  WakeUp_CalculateNextValidTime();
  LOGF("Next valid time for operation: %02d:%02d\r\n", next_valid_hour,
       next_valid_minute);
}

bool WakeUp_CheckIfWithinSchedule(rtc_time_t now) {
  uint16_t current_time_total_minutes = 0;

  WakeUp_TimeToMinutes(now.hours, now.minutes, &current_time_total_minutes);

  if (schedule_start_total_minutes <= schedule_end_total_minutes) {
    return schedule_start_total_minutes <= current_time_total_minutes &&
           current_time_total_minutes <= schedule_end_total_minutes;
  } else {
    return schedule_start_total_minutes <= current_time_total_minutes ||
           current_time_total_minutes <= schedule_end_total_minutes;
  }
}

void WakeUp_SetCurrentTime(uint8_t hour, uint8_t minute) {
  current_hour = hour;
  current_minute = minute;
}

void WakeUp_CalculateNextValidTime(void) {
  uint16_t current_time_total_minutes = 0;
  uint16_t offset_minutes = 0;
  WakeUp_TimeToMinutes(current_hour, current_minute,
                       &current_time_total_minutes);

  if (!WakeUp_CheckIfWithinSchedule(
          (rtc_time_t){.hours = current_hour, .minutes = current_minute})) {
    // If current time is before the start of the schedule, set next valid
    // time to the start of the schedule
    next_valid_hour = schedule_start_hour;
    next_valid_minute = schedule_start_minute;
    return;
  }

  if (schedule_start_total_minutes <= schedule_end_total_minutes) {
    offset_minutes = current_time_total_minutes - schedule_start_total_minutes;
  } else {
    if (current_time_total_minutes >= schedule_start_total_minutes) {
      offset_minutes =
          current_time_total_minutes - schedule_start_total_minutes;
    } else {
      offset_minutes =
          (24 * 60 - schedule_start_total_minutes) + current_time_total_minutes;
    }
  }

  uint16_t last_interval_offset_minutes =
      (offset_minutes / schedule_interval_minutes) * schedule_interval_minutes;
  uint16_t last_interval =
      (schedule_start_total_minutes + last_interval_offset_minutes) % (24 * 60);

  if (offset_minutes - last_interval_offset_minutes < RUN_MINUTES) {
    next_valid_hour = current_hour;
    next_valid_minute = current_minute;
  } else {
    uint16_t next_valid_total_minutes =
        (last_interval + schedule_interval_minutes) % (24 * 60);
    next_valid_hour = next_valid_total_minutes / 60;
    next_valid_minute = next_valid_total_minutes % 60;
  }

  return;
}

bool WakeUp_CheckCurrentMinute(rtc_time_t now) {
  WakeUp_SetCurrentTime(now.hours, now.minutes);

  WakeUp_CalculateNextValidTime();

  bool is_valid_time = (current_minute == next_valid_minute) &&
                       (current_hour == next_valid_hour);

  return is_valid_time;
}
