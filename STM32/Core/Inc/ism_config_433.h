#include "ism.h"

// Minimal: just set frequency.
// Add more registers here later (modulation, data rate, packet settings, PA
// table, etc.)
static const ism_reg_t cc1101_cfg_rx[] = {
    {0x02, 0x07}, // IOCFG0: GDO0 asserts on sync, deassert end/FIFO overflow
    {0x04, 0x04}, // Set sync word (SYNC1)
    {0x05, 0x05}, // Set sync word (SYNC0)
    {0x09, 0xEB}, // Set ADDR = EB
    {0x07, 0x05}, // Enable address check
    {0x0D, 0x10}, // FREQ2
    {0x0E, 0xA7}, // FREQ1
    {0x0F, 0x62}, // FREQ0
};

static const ism_reg_t cc1101_cfg_tx[] = {
    {0x02, 0x07}, // IOCFG0: GDO0 asserts on sync, deassert end/FIFO overflow
    {0x04, 0x04}, // Set sync word (SYNC1)
    {0x05, 0x05}, // Set sync word (SYNC0)
    {0x0D, 0x10}, // FREQ2
    {0x0E, 0xA7}, // FREQ1
    {0x0F, 0x62}, // FREQ0
};