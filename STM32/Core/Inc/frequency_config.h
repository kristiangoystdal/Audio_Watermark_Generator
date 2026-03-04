#include <sys/types.h>
#ifdef FREQUENCY_CONFIG_H
#error "frequency_config.h should not be included more than once"
#else
#define FREQUENCY_CONFIG_H

#include "user_config.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
  uint16_t lower_freq;
  uint16_t higher_freq;
  uint16_t lower_freq_samples;
  uint16_t higher_freq_samples;
  uint16_t lower_freq_periods;
  uint16_t higher_freq_periods;
} freq_pair_t;

freq_pair_t find_frequency_pair(uint32_t fs);

#endif // DEBUG