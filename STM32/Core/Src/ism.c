/* ism.c - CC1101 SPI driver (robust + debug)
 *
 * Drop-in replacement.
 * Assumes your ism.h defines these:
 *   - ism_handle_t, ism_status_t (ISM_OK, ISM_ERR_TIMEOUT, ISM_ERR_PARAM,
 * ISM_ERR_HAL)
 *   - ism_reg_t { uint8_t addr; uint8_t value; }
 *   - ISM_CMD_* strobes (SRES=0x30, SIDLE=0x36, STX=0x35, SFTX=0x3B, SFRX=0x3A,
 * ...)
 *   - ISM_REG_TXFIFO = 0x3F
 *   - ISM_STATUS_PARTNUM = 0x30, ISM_STATUS_VERSION = 0x31, ISM_STATUS_TXBYTES
 * = 0x3A
 *
 * If your ism.h doesn't define ISM_SPI_READ / ISM_SPI_BURST, we define them
 * here.
 */

#include "ism.h"
#include "log.h"

#include <stdio.h>
#include <stm32g4xx_hal.h>
#include <string.h>

/* =========================
 * CC1101 SPI command bits
 * ========================= */
#ifndef ISM_SPI_READ
#define ISM_SPI_READ (0x80u)
#endif
#ifndef ISM_SPI_BURST
#define ISM_SPI_BURST (0x40u)
#endif

/* =========================
 * DEBUG CONFIG
 * ========================= */
#ifndef ISM_DEBUG
#define ISM_DEBUG 1
#endif

#ifndef ISM_DEBUG_DUMP_BYTES
#define ISM_DEBUG_DUMP_BYTES 1
#endif

#ifndef ISM_DEBUG_MAX_DUMP
#define ISM_DEBUG_MAX_DUMP 24
#endif

#ifndef ISM_DEBUG_PIN_SNAPSHOT
#define ISM_DEBUG_PIN_SNAPSHOT 1
#endif

#ifndef ISM_DEBUG_RETRY_MISO_READY
#define ISM_DEBUG_RETRY_MISO_READY 1
#endif

#ifndef ISM_DEBUG_RETRY_DELAY_MS
#define ISM_DEBUG_RETRY_DELAY_MS 2
#endif

/* =========================
 * Small debug helpers
 * ========================= */
static inline int ism_pin_read(GPIO_TypeDef *port, uint16_t pin) {
  if (!port)
    return -1;
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1 : 0;
}

#if ISM_DEBUG
static void ism_dump_hex(const char *tag, const uint8_t *buf, uint16_t len) {
#if ISM_DEBUG_DUMP_BYTES
  if (!buf) {
    LOGF("%s: (null)\r\n", tag);
    return;
  }
  uint16_t n = len;
  if (n > ISM_DEBUG_MAX_DUMP)
    n = ISM_DEBUG_MAX_DUMP;
  LOGF("%s[%u]: ", tag, (unsigned)len);
  for (uint16_t i = 0; i < n; i++)
    LOGF("%02X ", buf[i]);
  if (n < len)
    LOGF("...");
  LOGF("\r\n");
#else
  (void)tag;
  (void)buf;
  (void)len;
#endif
}

static void ism_dbg_pins(const char *where, ism_handle_t *h) {
#if ISM_DEBUG_PIN_SNAPSHOT
  int cs = -1, miso = -1;
  if (h) {
    cs = ism_pin_read(h->cs_port, h->cs_pin);
    miso = ism_pin_read(h->miso_port, h->miso_pin);
  }
  LOGF("[ISM] %s t=%lu CS=%d MISO=%d\r\n", where, (unsigned long)HAL_GetTick(),
       cs, miso);
#else
  (void)where;
  (void)h;
#endif
}

#else
static void ism_dump_hex(const char *tag, const uint8_t *buf, uint16_t len) {
  (void)tag;
  (void)buf;
  (void)len;
}
static void ism_dbg_pins(const char *where, ism_handle_t *h) {
  (void)where;
  (void)h;
}
#endif

/* =========================
 * CS helpers
 * ========================= */
static inline void ism_cs_low(ism_handle_t *h) {
  HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_RESET);
  ism_dbg_pins("CS_LOW", h);
}
static inline void ism_cs_high(ism_handle_t *h) {
  HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);
  ism_dbg_pins("CS_HIGH", h);
}

/* =========================
 * MISO-ready wait (CC1101 SO=0 means CHIP_RDYn)
 * ========================= */
static ism_status_t ism_wait_miso_ready_once(ism_handle_t *h) {
  if (!h || !h->miso_port)
    return ISM_OK; // disabled

  uint32_t t0 = HAL_GetTick();
#if ISM_DEBUG
  ism_dbg_pins("MISO_WAIT_BEGIN", h);
#endif

  while (HAL_GPIO_ReadPin(h->miso_port, h->miso_pin) == GPIO_PIN_SET) {
    if ((HAL_GetTick() - t0) >= h->miso_ready_timeout_ms) {
      LOGF("[ISM] MISO ready timeout after %lums (timeout=%ums)\r\n",
           (unsigned long)(HAL_GetTick() - t0),
           (unsigned)h->miso_ready_timeout_ms);
      ism_dbg_pins("MISO_TIMEOUT", h);
      return ISM_ERR_TIMEOUT;
    }
  }

#if ISM_DEBUG
  LOGF("[ISM] MISO-ready OK (%lums)\r\n", (unsigned long)(HAL_GetTick() - t0));
  ism_dbg_pins("MISO_WAIT_OK", h);
#endif
  return ISM_OK;
}

static ism_status_t ism_wait_miso_ready(ism_handle_t *h) {
  ism_status_t s = ism_wait_miso_ready_once(h);
#if ISM_DEBUG_RETRY_MISO_READY
  if (s == ISM_ERR_TIMEOUT) {
    LOGF("[ISM] Retrying MISO-ready once...\r\n");
    ism_cs_high(h);
    HAL_Delay(ISM_DEBUG_RETRY_DELAY_MS);
    ism_cs_low(h);
    HAL_Delay(ISM_DEBUG_RETRY_DELAY_MS);
    s = ism_wait_miso_ready_once(h);
  }
#endif
  return s;
}

/* =========================
 * Low-level SPI transfer
 * ========================= */
/* SPI is now handled directly in cc1101.c via bit-banging. */
static ism_status_t ism_spi_trx(ism_handle_t *h, const uint8_t *tx, uint8_t *rx,
                                uint16_t len) {
  (void)h;
  (void)tx;
  (void)rx;
  (void)len;
  return ISM_ERR_HAL;
}

/* =========================
 * CC1101 command builders
 * ========================= */
static inline uint8_t cc1101_cmd_read_reg(uint8_t addr) {
  // Config regs: 0x00..0x2E read with READ bit
  return (uint8_t)(addr | ISM_SPI_READ);
}

static inline uint8_t cc1101_cmd_read_status(uint8_t addr) {
  // Status regs: 0x30..0x3D read using READ|BURST (CC1101 convention)
  return (uint8_t)(addr | ISM_SPI_READ | ISM_SPI_BURST);
}

static inline uint8_t cc1101_cmd_write_reg(uint8_t addr) {
  return (uint8_t)(addr & 0x3F); // keep for config regs (0x00..0x2E)
}

static inline uint8_t cc1101_cmd_write_burst(uint8_t addr) {
  return (uint8_t)((addr & 0x3F) | ISM_SPI_BURST);
}

static inline uint8_t cc1101_cmd_read_burst(uint8_t addr) {
  return (uint8_t)((addr & 0x3F) | ISM_SPI_READ | ISM_SPI_BURST);
}

static inline int cc1101_is_status_addr(uint8_t addr) {
  return (addr >= 0x30u && addr <= 0x3Du);
}

/* =========================
 * Helper: transaction with CS + ready wait
 * CC1101 returns a STATUS byte simultaneously with the first byte shifted out.
 * We'll capture that when rx is provided.
 * ========================= */
static ism_status_t cc1101_xfer(ism_handle_t *h, const uint8_t *tx, uint8_t *rx,
                                uint16_t len) {
  ism_cs_low(h);

  ism_status_t s = ism_wait_miso_ready(h);
  if (s != ISM_OK) {
    ism_cs_high(h);
    return s;
  }

  s = ism_spi_trx(h, tx, rx, len);
  ism_cs_high(h);
  return s;
}

/* =========================
 * Public API
 * ========================= */
ism_status_t ism_init(ism_handle_t *h) {
  if (!h || !h->cs_port)
    return ISM_ERR_PARAM;

  if (h->spi_timeout_ms == 0)
    h->spi_timeout_ms = 50;
  if (h->miso_ready_timeout_ms == 0)
    h->miso_ready_timeout_ms = 30; // slightly safer default

  LOGF("[ISM] Initialized (SPI timeout: %ums, MISO ready timeout: %ums)\r\n",
       (unsigned)h->spi_timeout_ms, (unsigned)h->miso_ready_timeout_ms);

  ism_cs_high(h);
  HAL_Delay(2);
  return ISM_OK;
}

ism_status_t ism_strobe(ism_handle_t *h, uint8_t cmd, uint8_t *status_out) {
  if (!h)
    return ISM_ERR_PARAM;

#if ISM_DEBUG
  LOGF("[ISM] STROBE cmd=0x%02X\r\n", cmd);
#endif

  uint8_t tx[1] = {cmd};
  uint8_t rx[1] = {0};

  ism_status_t s = cc1101_xfer(h, tx, rx, 1);
  if (s == ISM_OK) {
    LOGF("[ISM] Strobe cmd=0x%02X status=0x%02X\r\n", cmd, rx[0]);
    if (status_out)
      *status_out = rx[0];
  }
  return s;
}

ism_status_t ism_write_reg(ism_handle_t *h, uint8_t addr, uint8_t value) {
  if (!h)
    return ISM_ERR_PARAM;
  if (cc1101_is_status_addr(addr)) {
    // CC1101 status regs aren't writable
    LOGF("[ISM] WRITE_REG attempted on status addr=0x%02X (ignored)\r\n", addr);
    return ISM_ERR_PARAM;
  }

  uint8_t tx[2] = {cc1101_cmd_write_reg(addr), value};

#if ISM_DEBUG
  LOGF("[ISM] WRITE_REG addr=0x%02X value=0x%02X\r\n", addr, value);
#endif

  // capture status byte
  uint8_t rx[2] = {0};
  ism_status_t s = cc1101_xfer(h, tx, rx, 2);
  if (s == ISM_OK) {
    LOGF("[ISM] Write reg OK addr=0x%02X status=0x%02X\r\n", addr, rx[0]);
  }
  return s;
}

ism_status_t ism_read_reg(ism_handle_t *h, uint8_t addr, uint8_t *value_out) {
  if (!h || !value_out)
    return ISM_ERR_PARAM;

  // CC1101: normal regs read with READ, status regs read with READ|BURST
  uint8_t hdr = cc1101_is_status_addr(addr) ? cc1101_cmd_read_status(addr)
                                            : cc1101_cmd_read_reg(addr);

  uint8_t tx[2] = {hdr, 0xFF};
  uint8_t rx[2] = {0};

#if ISM_DEBUG
  LOGF("[ISM] READ_REG addr=0x%02X hdr=0x%02X\r\n", addr, hdr);
#endif

  ism_status_t s = cc1101_xfer(h, tx, rx, 2);
  if (s == ISM_OK) {
    // rx[0] = status byte, rx[1] = register value
    *value_out = rx[1];
    LOGF("[ISM] Read reg addr=0x%02X value=0x%02X status=0x%02X\r\n", addr,
         rx[1], rx[0]);
  }
  return s;
}

ism_status_t ism_write_burst(ism_handle_t *h, uint8_t addr, const uint8_t *data,
                             uint8_t len) {
  if (!h || (!data && len))
    return ISM_ERR_PARAM;
  // Burst writes are for config space or FIFO (0x3F). For FIFO, cmd still uses
  // 0x3F and burst bit.
  uint8_t header = cc1101_cmd_write_burst(addr);

#if ISM_DEBUG
  LOGF("[ISM] WRITE_BURST addr=0x%02X hdr=0x%02X len=%u\r\n", addr, header,
       (unsigned)len);
  ism_dump_hex("  DATA", data, len);
#endif

  ism_cs_low(h);
  ism_status_t s = ism_wait_miso_ready(h);
  if (s != ISM_OK) {
    ism_cs_high(h);
    return s;
  }

  // Capture status from header byte (read it back via TRX)
  uint8_t rxh = 0;
  s = ism_spi_trx(h, &header, &rxh, 1);
  if (s == ISM_OK && len)
    s = ism_spi_trx(h, data, NULL, len);

  ism_cs_high(h);

  if (s == ISM_OK)
    LOGF("[ISM] Write burst OK addr=0x%02X status=0x%02X\r\n", addr, rxh);
  return s;
}

ism_status_t ism_read_burst(ism_handle_t *h, uint8_t addr, uint8_t *data,
                            uint8_t len) {
  if (!h || (!data && len))
    return ISM_ERR_PARAM;

  // FIFO reads and config reads
  uint8_t header = cc1101_cmd_read_burst(addr);

#if ISM_DEBUG
  LOGF("[ISM] READ_BURST addr=0x%02X hdr=0x%02X len=%u\r\n", addr, header,
       (unsigned)len);
#endif

  ism_cs_low(h);
  ism_status_t s = ism_wait_miso_ready(h);
  if (s != ISM_OK) {
    ism_cs_high(h);
    return s;
  }

  uint8_t rxh = 0;
  s = ism_spi_trx(h, &header, &rxh, 1);
  if (s == ISM_OK && len)
    s = ism_spi_trx(h, NULL, data, len);

  ism_cs_high(h);

  if (s == ISM_OK) {
    LOGF("[ISM] Read burst OK addr=0x%02X status=0x%02X\r\n", addr, rxh);
#if ISM_DEBUG
    ism_dump_hex("  DATA", data, len);
#endif
  }
  return s;
}

ism_status_t ism_apply_config(ism_handle_t *h, const ism_reg_t *regs,
                              size_t count) {
  if (!h || (!regs && count))
    return ISM_ERR_PARAM;

  LOGF("[ISM] Applying %zu register configs\r\n", count);
  for (size_t i = 0; i < count; i++) {
    ism_status_t s = ism_write_reg(h, regs[i].addr, regs[i].value);
    if (s != ISM_OK) {
      LOGF("[ISM] Apply config failed at index %u (addr=0x%02X)\r\n",
           (unsigned)i, regs[i].addr);
      return s;
    }
  }
  return ISM_OK;
}

/* CC1101 recommended reset flow (robust)
 * 1) CSn high (deselect)
 * 2) CSn low, wait SO low (CHIP_RDYn), CSn high
 * 3) CSn low, wait SO low, send SRES strobe, CSn high
 */
ism_status_t ism_reset(ism_handle_t *h) {
  if (!h)
    return ISM_ERR_PARAM;

  LOGF("[ISM] Resetting (CC1101 sequence)...\r\n");

  ism_cs_high(h);
  HAL_Delay(2);

  LOGF("[ISM] Phase 1: sync\r\n");
  // Phase 1: sync
  ism_cs_low(h);
  ism_status_t s = ism_wait_miso_ready(h);
  ism_cs_high(h);
  if (s != ISM_OK) {
    LOGF("[ISM] Reset sync failed (no CHIP_RDYn)\r\n");
    return s;
  }
  HAL_Delay(2);

  LOGF("[ISM] Phase 2: strobe SRES\r\n");
  // Phase 2: strobe SRES
  uint8_t st = 0;
  s = ism_strobe(h, ISM_CMD_SRES, &st);
  if (s != ISM_OK) {
    LOGF("[ISM] Reset failed\r\n");
    return s;
  }

  HAL_Delay(2);
  LOGF("[ISM] Reset OK status=0x%02X\r\n", st);
  return ISM_OK;
}

ism_status_t ism_read_part_version(ism_handle_t *h, uint8_t *partnum,
                                   uint8_t *version) {
  if (!h)
    return ISM_ERR_PARAM;

  if (partnum) {
    ism_status_t s = ism_read_reg(h, ISM_STATUS_PARTNUM, partnum);
    if (s != ISM_OK)
      return s;
  }
  if (version) {
    ism_status_t s = ism_read_reg(h, ISM_STATUS_VERSION, version);
    if (s != ISM_OK)
      return s;
  }

  LOGF("[ISM] Part: 0x%02X Version: 0x%02X\r\n", partnum ? *partnum : 0,
       version ? *version : 0);
  return ISM_OK;
}

/* Read a status byte register (TXBYTES/RXBYTES/MARCSTATE/etc.)
 * CC1101 convention: read with READ|BURST.
 */
static ism_status_t ism_read_status(ism_handle_t *h, uint8_t status_addr,
                                    uint8_t *value_out) {
  if (!h || !value_out)
    return ISM_ERR_PARAM;

  // Always use status read framing
  uint8_t hdr = cc1101_cmd_read_status(status_addr);
  uint8_t tx[2] = {hdr, 0xFF};
  uint8_t rx[2] = {0};

#if ISM_DEBUG
  LOGF("[ISM] READ_STATUS addr=0x%02X hdr=0x%02X\r\n", status_addr, hdr);
#endif

  ism_status_t s = cc1101_xfer(h, tx, rx, 2);
  if (s == ISM_OK) {
    *value_out = rx[1];
#if ISM_DEBUG
    LOGF("[ISM] STATUS[0x%02X]=0x%02X statusbyte=0x%02X\r\n", status_addr,
         rx[1], rx[0]);
#endif
  }
  return s;
}

ism_status_t ism_send_packet(ism_handle_t *h, const uint8_t *payload,
                             uint8_t len, bool wait_done) {
  if (!h || (!payload && len))
    return ISM_ERR_PARAM;
  if (len > 61) {
    LOGF("[ISM] Payload too large: %d\r\n", len);
    return ISM_ERR_PARAM;
  }

  LOGF("[ISM] Sending packet len=%d wait_done=%d\r\n", len, wait_done);
#if ISM_DEBUG
  ism_dump_hex("  PAYLOAD", payload, len);
#endif

  uint8_t st = 0;
  ism_status_t s;

  // Ensure IDLE, flush TX
  s = ism_strobe(h, ISM_CMD_SIDLE, &st);
  if (s != ISM_OK)
    return s;
  s = ism_strobe(h, ISM_CMD_SFTX, &st);
  if (s != ISM_OK)
    return s;

  // CC1101: with variable length packets, first byte in TXFIFO is length
  uint8_t buf[1 + 64];
  buf[0] = len;
  if (len)
    memcpy(&buf[1], payload, len);

  s = ism_write_burst(h, ISM_REG_TXFIFO, buf, (uint8_t)(len + 1));
  if (s != ISM_OK)
    return s;

  // Start TX
  s = ism_strobe(h, ISM_CMD_STX, &st);
  if (s != ISM_OK)
    return s;

  if (!wait_done) {
    LOGF("[ISM] Packet queued (not waiting)\r\n");
    return ISM_OK;
  }

  // Poll TXBYTES until empty
  uint32_t t0 = HAL_GetTick();
  while (1) {
    uint8_t txbytes = 0;
    s = ism_read_status(h, ISM_STATUS_TXBYTES, &txbytes);
    if (s != ISM_OK)
      return s;

    // Bit7 indicates underflow/overflow
    if (txbytes & 0x80) {
      LOGF("[ISM] TX under/overflow flagged (TXBYTES=0x%02X)\r\n", txbytes);
      (void)ism_strobe(h, ISM_CMD_SFTX, &st);
      return ISM_ERR_HAL;
    }

    if ((txbytes & 0x7F) == 0)
      break;

    if ((HAL_GetTick() - t0) > 500) {
      LOGF("[ISM] TX timeout (TXBYTES=0x%02X)\r\n", txbytes);
      return ISM_ERR_TIMEOUT;
    }
    HAL_Delay(2);
  }

  LOGF("[ISM] Packet sent successfully\r\n");
  (void)ism_strobe(h, ISM_CMD_SIDLE, &st);
  return ISM_OK;
}
