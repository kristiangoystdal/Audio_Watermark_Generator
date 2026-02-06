#include "ism.h"

// Minimal: just set frequency.
// Add more registers here later (modulation, data rate, packet settings, PA
// table, etc.)
static const ism_reg_t cc1101_cfg_rx[] = {
    {0x04, 0x8C}, // Set sync word (SYNC1)
    {0x05, 0x36}, // Set sync word (SYNC0)
    {0x12, 0x03}, // MDMCFG2: 30/32 sync bits detected
    {0x0D, 0x13}, // FREQ2
    {0x0E, 0x3E}, // FREQ1
    {0x0F, 0x98}, // FREQ
    {0x02, 0x06}, // IOCFG0: GDO0 asserts on sync, deassert end/FIFO overflow
    {0x08, 0x01}, // Set device address to 0x42
    {0x03, 0x37}, // Enable address check
};

static const ism_reg_t cc1101_cfg_tx[] = {
    {0x02, 0x06}, // IOCFG0: GDO0 asserts on sync, deassert end/FIFO overflow
    {0x04, 0x04}, // Set sync word (SYNC1)
    {0x05, 0x05}, // Set sync word (SYNC0)
    {0x0D, 0x10}, // FREQ2
    {0x0E, 0xAA}, // FREQ1
    {0x0F, 0x6E}, // FREQ
};

static const ism_reg_t cc1101_tx[] = {
    {0x3F, 0x69}, // Write 1 byte to TX FIFO (example data)
    {0x35, 0x00}  // Go to TX state (STX)
};
