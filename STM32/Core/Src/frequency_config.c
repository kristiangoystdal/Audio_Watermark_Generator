#include "frequency_config.h"
#include "log.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#define MIN_BIT_US 3000u

freq_pair_t find_frequency_pair(uint32_t fs) {
  freq_pair_t result = {0};

  const size_t min_bit_samples =
      (size_t)((((uint64_t)fs * (uint64_t)MIN_BIT_US) + 500000ull) /
               1000000ull);

  uint16_t lower_freq = FSK_LOWER_FREQUENCY;
  uint16_t higher_freq = FSK_HIGHER_FREQUENCY;

  LOGF("Finding frequency pair for lower=%u Hz, higher=%u Hz\r\n",
       (unsigned int)lower_freq, (unsigned int)higher_freq);

  // Samples per period at the requested frequencies
  uint16_t lower_samples_per_period =
      (uint16_t)floor((double)fs / (double)lower_freq);
  uint16_t higher_samples_per_period =
      (uint16_t)floor((double)fs / (double)higher_freq);

  // Number of periods needed to approximately satisfy minimum bit duration
  uint16_t lower_period_count = (uint16_t)llround(
      (double)min_bit_samples / (double)lower_samples_per_period);
  uint16_t higher_period_count = (uint16_t)llround(
      (double)min_bit_samples / (double)higher_samples_per_period);

  while (lower_period_count == higher_period_count) {
    LOGF("Warning: lower and higher frequencies have the same period count "
         "(%u), adjusting higher frequency\r\n",
         (unsigned int)lower_period_count);
    higher_samples_per_period -= 1;

    uint16_t new_higher_period_count = (uint16_t)llround(
        (double)min_bit_samples / (double)higher_samples_per_period);
    if (new_higher_period_count != higher_period_count) {
      LOGF("Adjusted higher frequency samples per period to %u, new period "
           "count: %u\r\n",
           (unsigned int)higher_samples_per_period,
           (unsigned int)new_higher_period_count);
    }
    higher_period_count = new_higher_period_count;
  }

  // Total samples used for one bit
  const uint16_t lower_total_samples =
      (uint16_t)(lower_samples_per_period * lower_period_count);
  const uint16_t higher_total_samples =
      (uint16_t)(higher_samples_per_period * higher_period_count);

  // Quantized frequencies after forcing integer samples per period
  lower_freq = (uint16_t)floor((double)fs / (double)lower_samples_per_period);
  higher_freq = (uint16_t)floor((double)fs / (double)higher_samples_per_period);

  LOGF("Found frequency pair: lower=%u Hz, higher=%u Hz\r\n",
       (unsigned int)lower_freq, (unsigned int)higher_freq);
  LOGF("Lower freq samples: %u, periods: %u, total samples: %u\r\n",
       (unsigned int)lower_samples_per_period, (unsigned int)lower_period_count,
       (unsigned int)lower_total_samples);
  LOGF("Higher freq samples: %u, periods: %u, total samples: %u\r\n",
       (unsigned int)higher_samples_per_period,
       (unsigned int)higher_period_count, (unsigned int)higher_total_samples);

  result.lower_freq = lower_freq;
  result.higher_freq = higher_freq;
  result.lower_freq_samples = lower_samples_per_period;
  result.higher_freq_samples = higher_samples_per_period;
  result.lower_freq_periods = lower_period_count;
  result.higher_freq_periods = higher_period_count;

  return result;
}