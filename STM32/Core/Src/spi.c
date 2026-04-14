#include "spi.h"
#include "log.h"
#include "main.h"

HAL_StatusTypeDef SPI1_Write(const uint8_t *tx, uint16_t len) {
  if (!tx || len == 0)
    return HAL_OK;
  SPI1_CS_Low();
  HAL_StatusTypeDef st =
      HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, SPI1_TIMEOUT_MS);
  SPI1_CS_High();
  return st;
}

HAL_StatusTypeDef SPI1_Read(uint8_t *rx, uint16_t len) {
  if (!rx || len == 0)
    return HAL_OK;
  SPI1_CS_Low();
  HAL_StatusTypeDef st = HAL_SPI_Receive(&hspi1, rx, len, SPI1_TIMEOUT_MS);
  SPI1_CS_High();
  return st;
}

HAL_StatusTypeDef SPI1_WriteRead(const uint8_t *tx, uint8_t *rx, uint16_t len) {
  if (!rx || len == 0)
    return HAL_OK;
  SPI1_CS_Low();
  HAL_StatusTypeDef st;
  if (tx) {
    st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len,
                                 SPI1_TIMEOUT_MS);
  } else {
    uint8_t dummy = 0xFF;
    for (uint16_t i = 0; i < len; i++) {
      st = HAL_SPI_TransmitReceive(&hspi1, &dummy, &rx[i], 1, SPI1_TIMEOUT_MS);
      if (st != HAL_OK)
        break;
    }
  }
  SPI1_CS_High();
  return st;
}