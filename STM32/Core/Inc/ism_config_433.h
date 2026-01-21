#include "ism.h"

// Minimal: just set frequency.
// Add more registers here later (modulation, data rate, packet settings, PA
// table, etc.)
static const ism_reg_t cc1101_cfg_433_92_26mhz[] = {
    {0x0D, 0x10}, // FREQ2
    {0x0E, 0xB0}, // FREQ1
    {0x0F, 0x71}, // FREQ0
};
