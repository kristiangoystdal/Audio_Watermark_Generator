// Struct to hold register address and value pairs for CC1101 configuration
typedef struct {
  uint8_t addr;
  uint8_t value;
} ism_reg_t;

static const ism_reg_t cc1101_cfg_rx[] = {
    {0x02, 0x07}, // IOCFG0: GDO0 asserts when packet received
    {0x04, 0xD3}, // SYNC1
    {0x05, 0x91}, // SYNC0
    {0x06, 0x3D}, // PKTLEN
    {0x07, 0x05}, // PKTCTRL1
    {0x08, 0x05}, // PKTCTRL0
    {0x09, 0xEB}, // ADDR
    {0x0D, 0x10}, // FREQ2
    {0x0E, 0xA7}, // FREQ1
    {0x0F, 0x62}, // FREQ0
    {0x10, 0x85}, // MDMCFG4
    {0x11, 0x83}, // MDMCFG3
    {0x12, 0x02}, // MDMCFG2
    {0x13, 0xF2}, // MDMCFG1: FEC enabled
    {0x14, 0x01}, // MDMCFG0: bit 0 = 1 enables external XTAL
    {0x17, 0x37}, // MCSM2: RX_TIME=7, EXITMASK enabled
    {0x18, 0x18}, // MCSM0: FS_AUTOCAL=01 (auto-calibrate on state transitions)
    {0x1E, 0x10}, // WOREVT1: EVENT0 high byte (0x1080 = 264ms)
    {0x1F, 0x80}, // WOREVT0: EVENT0 low byte
    {0x20, 0x70}, // WORCTRL: WOR_RES=0, EVENT1=7, RC_PD=0 (RC osc OFF)
};

static const ism_reg_t cc1101_cfg_tx[] = {
    {0x02, 0x07}, // IOCFG0: GDO0 asserts when packet received
    {0x04, 0xD3}, // SYNC1
    {0x05, 0x91}, // SYNC0
    {0x06, 0x3D}, // PKTLEN
    {0x07, 0x05}, // PKTCTRL1
    {0x08, 0x05}, // PKTCTRL0
    {0x09, 0xEB}, // ADDR
    {0x0D, 0x10}, // FREQ2
    {0x0E, 0xA7}, // FREQ1
    {0x0F, 0x62}, // FREQ0
    {0x10, 0x85}, // MDMCFG4
    {0x11, 0x83}, // MDMCFG3
    {0x12, 0x02}, // MDMCFG2
    {0x13, 0xF2}, // MDMCFG1
    {0x14, 0x01}, // MDMCFG0: bit 0 = 1 enables external XTAL
    {0x18, 0x18}, // MCSM0: FS_AUTOCAL=01 (auto-calibrate on state transitions)
};