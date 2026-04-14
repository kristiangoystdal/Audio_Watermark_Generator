/* ism.h
 *
 * Minimal ISM/CC1101-style SPI radio helper (AS07-M1101S modules are commonly
 * CC1101-based).
 * - Uses HAL SPI
 * - Handles CS + optional "MISO ready" wait
 * - Provides: reset, strobe, read/write regs, burst FIFO, init from reg table,
 * send packet
 *
 * Drop this into Core/Inc/ism.h and Core/Src/ism.c
 */

#pragma once
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// -------- CC1101 SPI flags --------
#define ISM_SPI_READ 0x80
#define ISM_SPI_BURST 0x40

// -------- CC1101 register / FIFO addresses --------
#define ISM_REG_TXFIFO 0x3F
#define ISM_REG_RXFIFO 0x3F

// -------- CC1101 command strobes --------
#define ISM_CMD_SRES 0x30
#define ISM_CMD_SFSTXON 0x31
#define ISM_CMD_SXOFF 0x32
#define ISM_CMD_SCAL 0x33
#define ISM_CMD_SRX 0x34
#define ISM_CMD_STX 0x35
#define ISM_CMD_SIDLE 0x36
#define ISM_CMD_SAFC 0x37
#define ISM_CMD_SWOR 0x38
#define ISM_CMD_SPWD 0x39
#define ISM_CMD_SFRX 0x3A
#define ISM_CMD_SFTX 0x3B
#define ISM_CMD_SWORRST 0x3C
#define ISM_CMD_SNOP 0x3D

// -------- Some useful status registers (read with BURST|READ) --------
#define ISM_STATUS_PARTNUM 0x30
#define ISM_STATUS_VERSION 0x31
#define ISM_STATUS_MARCSTATE 0x35
#define ISM_STATUS_TXBYTES 0x3A
#define ISM_STATUS_RXBYTES 0x3B

typedef enum {
  ISM_OK = 0,
  ISM_ERR_PARAM,
  ISM_ERR_HAL,
  ISM_ERR_TIMEOUT
} ism_status_t;

// A single register write entry (addr,value)
typedef struct {
  uint8_t addr;
  uint8_t value;
} ism_reg_t;

typedef struct {
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;

  // Optional: Many CC1101s require waiting for MISO/SO to go LOW after CS LOW.
  // If you don't want this, set miso_port=NULL.
  GPIO_TypeDef *miso_port;
  uint16_t miso_pin;

  // Timeouts (ms)
  uint32_t spi_timeout_ms;
  uint32_t miso_ready_timeout_ms;
} ism_handle_t;

// ---------- Core ----------
ism_status_t
ism_init(ism_handle_t *h); // basic sanity (no register programming)
ism_status_t ism_reset(ism_handle_t *h);

ism_status_t ism_strobe(ism_handle_t *h, uint8_t cmd, uint8_t *status_out);

ism_status_t ism_write_reg(ism_handle_t *h, uint8_t addr, uint8_t value);
ism_status_t ism_read_reg(ism_handle_t *h, uint8_t addr, uint8_t *value_out);

ism_status_t ism_write_burst(ism_handle_t *h, uint8_t addr, const uint8_t *data,
                             uint8_t len);
ism_status_t ism_read_burst(ism_handle_t *h, uint8_t addr, uint8_t *data,
                            uint8_t len);

// Program a table of register writes (typical output from SmartRF Studio)
ism_status_t ism_apply_config(ism_handle_t *h, const ism_reg_t *regs,
                              size_t count);

// Packet-mode helper:
// - Writes [len][payload...] to TXFIFO (variable length packet mode)
// - STX strobe
// - Optionally waits for TX FIFO to drain (poll TXBYTES), then returns
ism_status_t ism_send_packet(ism_handle_t *h, const uint8_t *payload,
                             uint8_t len, bool wait_done);

// Convenience: read partnum/version (nice for "is SPI wiring OK?")
ism_status_t ism_read_part_version(ism_handle_t *h, uint8_t *partnum,
                                   uint8_t *version);

/* ism.h EOF */
