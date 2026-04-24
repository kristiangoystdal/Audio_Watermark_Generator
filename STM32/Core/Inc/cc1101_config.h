// Struct to hold register address and value pairs for CC1101 configuration
typedef struct {
  uint8_t addr;
  uint8_t value;
} ism_reg_t;

static const ism_reg_t cc1101_cfg_rx[] = {
    {0x02, 0x07}, // IOCFG0: GDO0 asserts when packet received with CRC OK,
                  //         deasserts on first FIFO byte read
    {0x04, 0xD3}, // SYNC1: sync word high byte = 0xD3
    {0x05, 0x91}, // SYNC0: sync word low byte  = 0x91
    {0x06, 0x3D}, // PKTLEN: max packet length = 61 bytes
    {0x07, 0x05}, // PKTCTRL1: address check enabled, no status bytes appended
    {0x08, 0x05}, // PKTCTRL0: variable length, CRC enabled, no data whitening
    {0x09, 0xEB}, // ADDR: device address = 0xEB
    {0x0D, 0x10}, // FREQ2: carrier frequency high byte  \
    {0x0E, 0xA7}, // FREQ1: carrier frequency mid byte    > 433 MHz
    {0x0F, 0x62}, // FREQ0: carrier frequency low byte   /
    {0x10, 0x85}, // MDMCFG4: channel BW=203kHz, DRATE_E=5 -> 1200 bps
    {0x11, 0x83}, // MDMCFG3: DRATE_M=131 -> 1200 bps
    {0x12,
     0x02}, // MDMCFG2: 2-FSK, 16/16 sync word bits, no Manchester encoding
    {0x13, 0xF2}, // MDMCFG1: 24-byte preamble, FEC disabled
    {0x17,
     0x07}, // MCSM2: RX_TIME=7, stay in RX until packet done after WOR wake
    {0x18, 0x18}, // MCSM0: FS_AUTOCAL=01, calibrate on IDLE->RX transition
    {0x1E, 0x43}, // WOREVT1: EVENT0 high byte \  0x43CC = 17356 counts
    {0x1F, 0xCC}, // WOREVT0: EVENT0 low byte   > ~500 ms WOR wake interval
    {0x20, 0x78}, // WORCTRL: RC_PD=0, EVENT1=7 (~1.8ms CS window), RC_CAL=1,
                  // WOR_RES=0
};

static const ism_reg_t cc1101_cfg_tx[] = {
    {0x02, 0x07}, // IOCFG0: GDO0 asserts when packet received with CRC OK,
                  //         deasserts on first FIFO byte read
    {0x04, 0xD3}, // SYNC1: sync word high byte = 0xD3
    {0x05, 0x91}, // SYNC0: sync word low byte  = 0x91  must match RX
    {0x06, 0x3D}, // PKTLEN: max packet length = 61 bytes
    {0x07, 0x05}, // PKTCTRL1: address check enabled, no status bytes appended
    {0x08, 0x05}, // PKTCTRL0: variable length, CRC enabled, no data whitening
    {0x09, 0xEB}, // ADDR: device address = 0xEB
    {0x0D, 0x10}, // FREQ2: carrier frequency high byte  \
    {0x0E, 0xA7}, // FREQ1: carrier frequency mid byte    > 433 MHz
    {0x0F, 0x62}, // FREQ0: carrier frequency low byte   /
    {0x10, 0x85}, // MDMCFG4: channel BW=203kHz, DRATE_E=5 -> 1200 bps
    {0x11, 0x83}, // MDMCFG3: DRATE_M=131 -> 1200 bps
    {0x12,
     0x02}, // MDMCFG2: 2-FSK, 16/16 sync word bits, no Manchester encoding
    {0x13, 0xF2}, // MDMCFG1: 24-byte preamble (160ms at 1200 bps), FEC disabled
};