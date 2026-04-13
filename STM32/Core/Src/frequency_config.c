#include "frequency_config.h"
#include "log.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define MIN_BIT_US 3000u
#define MIN_FREQ_DIFF_HZ 500u
#define MAX_FSK_FREQ_HZ 24000u
#define MIN_FSK_FREQ_HZ 1000u
#define BIT_SAMPLE_TOLERANCE_PERCENT 1u

static uint16_t quantized_freq_from_samples(uint32_t fs,
                                            uint16_t samples_per_period) {
  if (samples_per_period == 0u) {
    return 0u;
  }
  return (uint16_t)floor((double)fs / (double)samples_per_period);
}

static uint16_t period_count_from_samples(size_t min_bit_samples,
                                          uint16_t samples_per_period) {
  return (uint16_t)llround((double)min_bit_samples /
                           (double)samples_per_period);
}

static uint16_t freq_diff_u16(uint16_t a, uint16_t b) {
  return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static bool total_samples_within_tolerance(uint16_t total_samples,
                                           size_t target_samples,
                                           size_t tolerance_samples) {
  size_t total = (size_t)total_samples;
  return (total >= (target_samples - tolerance_samples)) &&
         (total <= (target_samples + tolerance_samples));
}

freq_pair_t find_frequency_pair(uint32_t fs) {
  freq_pair_t result = {0};

  const size_t min_bit_samples =
      (size_t)((((uint64_t)fs * (uint64_t)MIN_BIT_US) + 500000ull) /
               1000000ull);

  const size_t tolerance_samples =
      (size_t)((min_bit_samples * BIT_SAMPLE_TOLERANCE_PERCENT + 50u) / 100u);

  uint16_t lower_freq = FSK_LOWER_FREQUENCY;
  uint16_t higher_freq = FSK_HIGHER_FREQUENCY;

  LOGF("Finding frequency pair for lower=%u Hz, higher=%u Hz\r\n",
       (unsigned int)lower_freq, (unsigned int)higher_freq);
  LOGF("Target bit samples: %u, tolerance: +/- %u samples (%u%%)\r\n",
       (unsigned int)min_bit_samples, (unsigned int)tolerance_samples,
       (unsigned int)BIT_SAMPLE_TOLERANCE_PERCENT);
  LOGF("\r\n");

  uint16_t lower_samples_per_period =
      (uint16_t)floor((double)fs / (double)lower_freq);
  uint16_t higher_samples_per_period =
      (uint16_t)floor((double)fs / (double)higher_freq);

  if (lower_samples_per_period == 0u) {
    lower_samples_per_period = 1u;
  }
  if (higher_samples_per_period == 0u) {
    higher_samples_per_period = 1u;
  }

  uint16_t lower_period_count =
      period_count_from_samples(min_bit_samples, lower_samples_per_period);
  uint16_t higher_period_count =
      period_count_from_samples(min_bit_samples, higher_samples_per_period);

  lower_freq = quantized_freq_from_samples(fs, lower_samples_per_period);
  higher_freq = quantized_freq_from_samples(fs, higher_samples_per_period);

  while (true) {
    const uint16_t lower_total_samples =
        (uint16_t)(lower_samples_per_period * lower_period_count);
    const uint16_t higher_total_samples =
        (uint16_t)(higher_samples_per_period * higher_period_count);

    const bool same_period_count = (lower_period_count == higher_period_count);
    const bool freq_too_close = (freq_diff_u16(higher_freq, lower_freq) <
                                 (300u + (400000u / lower_freq)));
    const bool lower_bad_timing = !total_samples_within_tolerance(
        lower_total_samples, min_bit_samples, tolerance_samples);
    const bool higher_bad_timing = !total_samples_within_tolerance(
        higher_total_samples, min_bit_samples, tolerance_samples);

    if (!same_period_count && !freq_too_close && !lower_bad_timing &&
        !higher_bad_timing) {
      break;
    }

    LOGF("Adjusting pair: lower=%u Hz (sp=%u, p=%u, total=%u), "
         "higher=%u Hz (sp=%u, p=%u, total=%u), diff=%u Hz\r\n",
         (unsigned int)lower_freq, (unsigned int)lower_samples_per_period,
         (unsigned int)lower_period_count, (unsigned int)lower_total_samples,
         (unsigned int)higher_freq, (unsigned int)higher_samples_per_period,
         (unsigned int)higher_period_count, (unsigned int)higher_total_samples,
         (unsigned int)freq_diff_u16(higher_freq, lower_freq));

    if (same_period_count) {
      LOGF("  Reason: same period count\r\n");
    }
    if (freq_too_close) {
      LOGF("  Reason: frequency spacing too small\r\n");
    }
    if (lower_bad_timing) {
      LOGF("  Reason: lower total samples not within tolerance of target\r\n");
    }
    if (higher_bad_timing) {
      LOGF("  Reason: higher total samples not within tolerance of target\r\n");
    }

    /* First try to move the higher tone upward */
    if (higher_freq < MAX_FSK_FREQ_HZ && higher_samples_per_period > 1u) {
      uint16_t candidate_higher_samples =
          (uint16_t)(higher_samples_per_period - 1u);
      uint16_t candidate_higher_freq =
          quantized_freq_from_samples(fs, candidate_higher_samples);

      if (candidate_higher_freq <= MAX_FSK_FREQ_HZ) {
        higher_samples_per_period = candidate_higher_samples;
        higher_freq = candidate_higher_freq;
        higher_period_count = period_count_from_samples(
            min_bit_samples, higher_samples_per_period);
        continue;
      }
    }

    /* If higher cannot go up anymore, move the lower tone downward */
    {
      uint16_t candidate_lower_samples =
          (uint16_t)(lower_samples_per_period + 1u);
      uint16_t candidate_lower_freq =
          quantized_freq_from_samples(fs, candidate_lower_samples);

      if (candidate_lower_freq >= MIN_FSK_FREQ_HZ) {
        lower_samples_per_period = candidate_lower_samples;
        lower_freq = candidate_lower_freq;
        lower_period_count = period_count_from_samples(
            min_bit_samples, lower_samples_per_period);
        continue;
      }
    }

    LOGF("Could not satisfy spacing/period/timing constraints within frequency "
         "limits\r\n");
    break;
  }

  const uint16_t lower_total_samples =
      (uint16_t)(lower_samples_per_period * lower_period_count);
  const uint16_t higher_total_samples =
      (uint16_t)(higher_samples_per_period * higher_period_count);

  LOGF("Found frequency pair: lower=%u Hz, higher=%u Hz\r\n",
       (unsigned int)lower_freq, (unsigned int)higher_freq);
  LOGF("Lower freq samples: %u, periods: %u, total samples: %u\r\n",
       (unsigned int)lower_samples_per_period, (unsigned int)lower_period_count,
       (unsigned int)lower_total_samples);
  LOGF("Higher freq samples: %u, periods: %u, total samples: %u\r\n",
       (unsigned int)higher_samples_per_period,
       (unsigned int)higher_period_count, (unsigned int)higher_total_samples);
  LOGF("\r\n");

  result.lower_freq = lower_freq;
  result.higher_freq = higher_freq;
  result.lower_freq_samples = lower_samples_per_period;
  result.higher_freq_samples = higher_samples_per_period;
  result.lower_freq_periods = lower_period_count;
  result.higher_freq_periods = higher_period_count;

  return result;
}