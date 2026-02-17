#include "frequency_config.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#define FS_HZ 95952u
#define MIN_BIT_US 3000u

#define MIN_BIT_SAMPLES                                                        \
  ((size_t)((((uint64_t)FS_HZ * (uint64_t)MIN_BIT_US) + 500000ull) /           \
            1000000ull))

#define NUM_SAMPLES_MIN 280
#define NUM_SAMPLES_MAX 295

freq_pair_t find_frequency_pair(uint16_t lower_freq, uint16_t higher_freq) {

  // Initial guess based on user config, but will be adjusted if it doesn't meet
  freq_pair_t result = {0};

  uint16_t best_lower_freq = lower_freq;
  uint16_t best_higher_freq = higher_freq;
  uint16_t lower_freq_samples = 0;
  uint16_t higher_freq_samples = 0;
  uint16_t lower_freq_periods = 0;
  uint16_t higher_freq_periods = 0;
  uint16_t lower_freq_total_samples = 0;
  uint16_t higher_freq_total_samples = 0;

  printf("Finding frequency pair for lower=%u Hz, higher=%u Hz\r\n",
         (unsigned int)lower_freq, (unsigned int)higher_freq);

  // Loop until we find a valid pair of frequencies that meet the sample count
  // requirements
  while (true) {
    bool lower_in_range = true;
    bool higher_in_range = true;

    lower_freq_samples =
        (uint16_t)floor((double)FS_HZ / (double)best_lower_freq);
    higher_freq_samples =
        (uint16_t)floor((double)FS_HZ / (double)best_higher_freq);

    lower_freq_periods =
        (uint16_t)llround((double)MIN_BIT_SAMPLES / (double)lower_freq_samples);
    higher_freq_periods = (uint16_t)llround((double)MIN_BIT_SAMPLES /
                                            (double)higher_freq_samples);

    lower_freq_total_samples = lower_freq_samples * lower_freq_periods;
    higher_freq_total_samples = higher_freq_samples * higher_freq_periods;

    // Check if both frequencies are within the acceptable range of samples
    if (lower_freq_total_samples < NUM_SAMPLES_MIN ||
        lower_freq_total_samples > NUM_SAMPLES_MAX) {
      lower_in_range = false;
      best_lower_freq += 1;
    }

    if (higher_freq_total_samples < NUM_SAMPLES_MIN ||
        higher_freq_total_samples > NUM_SAMPLES_MAX) {
      higher_in_range = false;
      best_higher_freq += 1;
    }

    // If both frequencies are in range, we found our pair
    if (lower_in_range && higher_in_range) {
      printf("Found frequency pair: lower=%u Hz, higher=%u Hz\r\n",
             (unsigned int)best_lower_freq, (unsigned int)best_higher_freq);
      break;
    }
  }

  result.lower_freq = best_lower_freq;
  result.higher_freq = best_higher_freq;
  result.lower_freq_samples = lower_freq_samples;
  result.higher_freq_samples = higher_freq_samples;
  result.lower_freq_periods = lower_freq_periods;
  result.higher_freq_periods = higher_freq_periods;

  return result;
}
