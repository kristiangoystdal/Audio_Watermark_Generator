#include "frequency_config.h"
#include "log.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#define MIN_BIT_US 3000u

#define NUM_SAMPLES_MIN 1120
#define NUM_SAMPLES_MAX 1180

freq_pair_t find_frequency_pair(uint32_t fs) {

  size_t min_bit_samples =
      ((size_t)((((uint64_t)fs * (uint64_t)MIN_BIT_US) + 500000ull) /
                1000000ull));

  // Initial guess based on user config, but will be adjusted if it doesn't meet
  freq_pair_t result = {0};

  uint16_t lower_freq = FSK_LOWER_FREQUENCY;
  uint16_t higher_freq = FSK_HIGHER_FREQUENCY;
  uint16_t lower_freq_samples = 0;
  uint16_t higher_freq_samples = 0;
  uint16_t lower_freq_periods = 0;
  uint16_t higher_freq_periods = 0;
  uint16_t lower_freq_total_samples = 0;
  uint16_t higher_freq_total_samples = 0;

  LOGF("Finding frequency pair for lower=%u Hz, higher=%u Hz\r\n",
       (unsigned int)lower_freq, (unsigned int)higher_freq);

  // Loop until we find a valid pair of frequencies that meet the sample count
  // requirements
  while (true) {
    bool lower_in_range = true;
    bool higher_in_range = true;

    lower_freq_samples = (uint16_t)floor((double)fs / (double)lower_freq);
    higher_freq_samples = (uint16_t)floor((double)fs / (double)higher_freq);

    lower_freq_periods =
        (uint16_t)llround((double)min_bit_samples / (double)lower_freq_samples);
    higher_freq_periods = (uint16_t)llround((double)min_bit_samples /
                                            (double)higher_freq_samples);

    lower_freq_total_samples = lower_freq_samples * lower_freq_periods;
    higher_freq_total_samples = higher_freq_samples * higher_freq_periods;

    lower_freq = (uint16_t)floor((double)fs / (double)lower_freq_samples);
    higher_freq = (uint16_t)floor((double)fs / (double)higher_freq_samples);

    LOGF("Found frequency pair: lower=%u Hz, higher=%u Hz\r\n",
          (unsigned int)lower_freq, (unsigned int)higher_freq);
    LOGF("Lower freq samples: %u, periods: %u, total samples: %u\r\n",
          (unsigned int)lower_freq_samples, (unsigned int)lower_freq_periods,
          (unsigned int)lower_freq_total_samples);
    LOGF("Higher freq samples: %u, periods: %u, total samples: %u\r\n",
          (unsigned int)higher_freq_samples, (unsigned int)higher_freq_periods,
          (unsigned int)higher_freq_total_samples);
    break;
  }

  result.lower_freq = lower_freq;
  result.higher_freq = higher_freq;
  result.lower_freq_samples = lower_freq_samples;
  result.higher_freq_samples = higher_freq_samples;
  result.lower_freq_periods = lower_freq_periods;
  result.higher_freq_periods = higher_freq_periods;

  return result;
}
