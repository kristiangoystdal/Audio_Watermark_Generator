struct {
  uint8_t addr;
  uint8_t value;
} typedef ism_reg_t;

static const ism_reg_t cc1101_cfg_rx[] = {
    {0x02, 0x07}, // IOCFG0:   asserts on sync, deasserts at end of packet
    {0x04, 0xD3}, // SYNC1
    {0x05, 0x91}, // SYNC0
    {0x06, 0x3D}, // PKTLEN:   61 byte max
    {0x07, 0x05}, // PKTCTRL1: addr check on, status appended
    {0x08, 0x05}, // PKTCTRL0: variable length, CRC on, no whitening
    {0x09, 0xEB}, // ADDR
    {0x0B,
     0x06}, // FSCTRL1:  IF = 152 kHz
    {0x0C, 0x00}, // FSCTRL0:  no freq offset
    {0x0D, 0x10}, // FREQ2:    433.0 MHz
    {0x0E, 0xA7}, // FREQ1
    {0x0F, 0x62}, // FREQ0
    {0x10, 0xC4}, // MDMCFG4:  BW=101kHz
    {0x11, 0x83}, // MDMCFG3:  DRATE_M=131 -> 599.8 baud
    {0x12, 0x03}, // MDMCFG2:  GFSK, 30/32 sync
    {0x13, 0x22}, // MDMCFG1:  4 byte preamble min, FEC off
    {0x14, 0xF8}, // MDMCFG0
    {0x15, 0x31}, // DEVIATN:  14.3 kHz
    {0x17, 0x3F}, // MCSM1:    CCA, RX->RX, TX->RX (kept)
    {0x18, 0x18}, // MCSM0:    auto-cal IDLE->RX/TX
    {0x19, 0x16}, // FOCCFG:   freq offset comp ±BW/4
    {0x1A, 0x6C}, // BSCFG:    bit sync defaults
    {0x1B, 0x84}, // AGCCTRL2: MAGN_TARGET=24dB + VGA off
    {0x1C, 0x00}, // AGCCTRL1: disable LNA
    {0x1D, 0x70}, // AGCCTRL0: keep fastest
    {0x1E, 0x10}, // WOREVT1   (kept)
    {0x1F, 0x80}, // WOREVT0   (kept)
    {0x20, 0x70}, // WORCTRL   (kept)
    {0x21, 0x56}, // FREND1:   RX front-end currents
    {0x23, 0xE9}, // FSCAL3:   seed value, overwritten by auto-cal
    {0x24, 0x2A}, // FSCAL2:   high VCO
    {0x25, 0x00}, // FSCAL1
    {0x26, 0x1F}, // FSCAL0
    {0x2C, 0x81}, // TEST2:    SmartRF analog preset
    {0x2D, 0x35}, // TEST1:    SmartRF analog preset
    {0x2E, 0x09}, // TEST0:    SmartRF analog preset
};

static const ism_reg_t cc1101_cfg_tx[] = {
    {0x02, 0x07}, // IOCFG0
    {0x04, 0xD3}, // SYNC1
    {0x05, 0x91}, // SYNC0
    {0x06, 0x3D}, // PKTLEN
    {0x07, 0x05}, // PKTCTRL1
    {0x08, 0x05}, // PKTCTRL0
    {0x09, 0xEB}, // ADDR
    {0x0B, 0x06}, // FSCTRL1
    {0x0C, 0x00}, // FSCTRL0
    {0x0D, 0x10}, // FREQ2
    {0x0E, 0xA7}, // FREQ1
    {0x0F, 0x62}, // FREQ0
    {0x10, 0xC4}, // MDMCFG4: BW=101kHz
    {0x11, 0x83}, // MDMCFG3
    {0x12, 0x13}, // MDMCFG2
    {0x13, 0x22}, // MDMCFG1
    {0x14, 0xF8}, // MDMCFG0
    {0x15, 0x31}, // DEVIATN
    {0x17, 0x00}, // MCSM1:    everything -> IDLE
    {0x18, 0x18}, // MCSM0
    {0x19, 0x16}, // FOCCFG
    {0x1A, 0x6C}, // BSCFG
    {0x1B, 0x43}, // AGCCTRL2
    {0x1C, 0x40}, // AGCCTRL1
    {0x1D, 0x50}, // AGCCTRL0
    {0x20, 0x70}, // WORCTRL
    {0x21, 0x56}, // FREND1
    {0x22, 0x10}, // FREND0:   PA_POWER=0 -> PATABLE[0]
    {0x23, 0xE9}, // FSCAL3
    {0x24, 0x2A}, // FSCAL2
    {0x25, 0x00}, // FSCAL1
    {0x26, 0x1F}, // FSCAL0
    {0x2C, 0x81}, // TEST2
    {0x2D, 0x35}, // TEST1
    {0x2E, 0x09}, // TEST0
};