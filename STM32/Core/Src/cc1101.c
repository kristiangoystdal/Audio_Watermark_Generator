#include "cc1101.h"
#include "cc1101_config.h"
#include "log.h"
#include "main.h"
#include <string.h>

static inline void cs_low(void) { GPIOB->BSRR = ((uint32_t)BB_CS << 16); }
static inline void cs_high(void) { GPIOB->BSRR = BB_CS; }

static bool is_miso_low(void) { return (GPIOB->IDR & BB_MISO) == 0; }

static bool wait_miso_low(uint32_t timeout_ms) {
  uint32_t t0 = HAL_GetTick();
  while (!is_miso_low()) {
    if ((HAL_GetTick() - t0) >= timeout_ms)
      return false;
  }
  return true;
}

// Bit-bang SPI function
static uint8_t bb_byte(uint8_t tx) {
  uint8_t rx = 0;
  for (int i = 7; i >= 0; i--) {
    // Drive MOSI before rising edge
    if (tx & (1u << i))
      MOSI_HIGH;
    else
      MOSI_LOW;

    // Rising edge — slave samples MOSI, master samples MISO
    SCK_HIGH;
    if (MISO_READ)
      rx |= (1u << i);

    // Falling edge — return SCK to idle-low
    SCK_LOW;
  }
  return rx;
}

// CC1101 public API functions

// Perform a power-up reset of the CC1101 and verify MISO goes low
bool CC1101_PowerUpReset(void) {
  // Ensure SCK idles low (Mode 0)
  GPIOB->BSRR = ((uint32_t)BB_SCK << 16);

  cs_high();
  HAL_Delay(100);

  cs_low();
  HAL_Delay(1);
  cs_high();
  HAL_Delay(10);

  cs_low();
  bb_byte(0x30); // SRES

  // Try up to 5 times with increasing delays
  for (int i = 0; i < 5; i++) {
    HAL_Delay(10 + i * 10);
    LOGF("IDR PB5 attempt %d: %lu\r\n", i, (GPIOB->IDR & BB_MISO));
    if (wait_miso_low(50)) {
      cs_high();
      HAL_Delay(5);
      return true;
    }
  }

  LOGF("CC1101: MISO stuck high after SRES\r\n");
  cs_high();
  return false;
}

HAL_StatusTypeDef CC1101_Strobe(uint8_t strobe, uint8_t *status) {
  cs_low();
  uint8_t rx = bb_byte(strobe);
  cs_high();
  if (status)
    *status = rx;
  return HAL_OK;
}

HAL_StatusTypeDef CC1101_ReadReg(uint8_t addr, uint8_t *val, uint8_t *status) {
  cs_low();
  uint8_t rx_status = bb_byte(addr | 0x80);
  uint8_t rx_val = bb_byte(0xFF);
  cs_high();
  if (status)
    *status = rx_status;
  if (val)
    *val = rx_val;
  return HAL_OK;
}

HAL_StatusTypeDef CC1101_WriteReg(uint8_t addr, uint8_t val, uint8_t *status) {
  cs_low();
  uint8_t rx_status = bb_byte(addr & 0x7F);
  bb_byte(val);
  cs_high();
  if (status)
    *status = rx_status;
  return HAL_OK;
}

HAL_StatusTypeDef CC1101_WriteBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                       uint8_t *status) {
  if (!vals || len == 0)
    return HAL_OK;
  cs_low();
  uint8_t rx_status = bb_byte(addr | 0x40);
  for (size_t i = 0; i < len; i++)
    bb_byte(vals[i]);
  cs_high();
  if (status)
    *status = rx_status;
  return HAL_OK;
}

HAL_StatusTypeDef CC1101_ReadBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                      uint8_t *status) {
  if (!vals || len == 0)
    return HAL_OK;
  cs_low();
  uint8_t rx_status = bb_byte(addr | 0xC0);
  for (size_t i = 0; i < len; i++)
    vals[i] = bb_byte(0xFF);
  cs_high();
  if (status)
    *status = rx_status;
  return HAL_OK;
}
