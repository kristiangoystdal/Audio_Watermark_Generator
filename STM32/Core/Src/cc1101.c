#include "cc1101.h"
#include "ism.h"
#include "ism_config_433.h"
#include "log.h"
#include "main.h"
#include "spi.h"
#include <string.h>

#define CC1101_MISO_PORT GPIOB
#define CC1101_MISO_PIN GPIO_PIN_4

static bool miso_is_low(void) { return (GPIOB->IDR & GPIO_PIN_4) == 0; }

static bool wait_miso_low(uint32_t timeout_ms) {
  uint32_t t0 = HAL_GetTick();
  while (!miso_is_low()) {
    if ((HAL_GetTick() - t0) >= timeout_ms)
      return false;
  }
  return true;
}

static uint8_t spi_byte(uint8_t tx) {
  uint8_t rx = 0;
  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, SPI1_TIMEOUT_MS);
  return rx;
}

bool CC1101_PowerUpReset(void) {
  SPI1_CS_High();
  HAL_Delay(100);

  SPI1_CS_Low();
  HAL_Delay(1);
  SPI1_CS_High();
  HAL_Delay(10);

  SPI1_CS_Low();
  spi_byte(0x30); // SRES

  // Try up to 5 times with increasing delays
  for (int i = 0; i < 5; i++) {
    HAL_Delay(10 + i * 10);
    LOGF("IDR PB4 attempt %d: %lu\r\n", i, (GPIOB->IDR & GPIO_PIN_4));
    if (wait_miso_low(50)) {
      SPI1_CS_High();
      HAL_Delay(5);
      return true;
    }
  }

  LOGF("CC1101: MISO stuck high after SRES\r\n");
  SPI1_CS_High();
  return false;
}

HAL_StatusTypeDef CC1101_Strobe(uint8_t strobe, uint8_t *status) {
  SPI1_CS_Low();
  uint8_t rx = spi_byte(strobe);
  SPI1_CS_High();
  if (status)
    *status = rx;
  return HAL_OK;
}

HAL_StatusTypeDef CC1101_ReadReg(uint8_t addr, uint8_t *val, uint8_t *status) {
  SPI1_CS_Low();
  uint8_t rx_status = spi_byte(addr | 0x80);
  uint8_t rx_val = spi_byte(0xFF);
  SPI1_CS_High();
  if (status)
    *status = rx_status;
  if (val)
    *val = rx_val;
  return HAL_OK;
}

HAL_StatusTypeDef CC1101_WriteReg(uint8_t addr, uint8_t val, uint8_t *status) {
  SPI1_CS_Low();
  uint8_t rx_status = spi_byte(addr & 0x7F);
  spi_byte(val);
  SPI1_CS_High();
  if (status)
    *status = rx_status;
  return HAL_OK;
}

HAL_StatusTypeDef CC1101_WriteBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                       uint8_t *status) {
  if (!vals || len == 0)
    return HAL_OK;
  SPI1_CS_Low();
  uint8_t rx_status = spi_byte(addr | 0x40);
  for (size_t i = 0; i < len; i++)
    spi_byte(vals[i]);
  SPI1_CS_High();
  if (status)
    *status = rx_status;
  return HAL_OK;
}

HAL_StatusTypeDef CC1101_ReadBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                      uint8_t *status) {
  if (!vals || len == 0)
    return HAL_OK;
  SPI1_CS_Low();
  uint8_t rx_status = spi_byte(addr | 0xC0);
  for (size_t i = 0; i < len; i++)
    vals[i] = spi_byte(0xFF);
  SPI1_CS_High();
  if (status)
    *status = rx_status;
  return HAL_OK;
}