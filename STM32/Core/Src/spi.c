#include "spi.h"

// If you have Error_Handler in main.c / main.h, include main.h.
// If not, remove it and handle errors your own way.
#include "main.h"

// ---------------------------
// Transactions (CS toggled)
// ---------------------------

HAL_StatusTypeDef SPI1_Write(const uint8_t *tx, uint16_t len) {
  if (!tx || len == 0) {
    printf("SPI1_Write called with NULL tx or zero length\r\n");
    return HAL_OK;
  }

  printf("SPI1_Write: Writing %u bytes\r\n", len);
  printf("Data: ");
  for (uint16_t i = 0; i < len; i++) {
    printf("%02X ", tx[i]);
  }
  printf("\r\n");

  SPI1_CS_Low();
  HAL_StatusTypeDef st =
      HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, SPI1_TIMEOUT_MS);
  if (st != HAL_OK)
    printf("SPI1_Write error: %d\r\n", st);

  SPI1_CS_High();

  printf("SPI1_Write success: %d\r\n", st);

  return st;
}

HAL_StatusTypeDef SPI1_Read(uint8_t *rx, uint16_t len) {
  if (!rx || len == 0) {
    printf("SPI1_Read called with NULL rx or zero length\r\n");
    return HAL_OK;
  }

  printf("SPI1_Read: Reading %u bytes\r\n", len);

  SPI1_CS_Low();

  HAL_StatusTypeDef st = HAL_SPI_Receive(&hspi1, rx, len, SPI1_TIMEOUT_MS);

  if (st != HAL_OK)
    printf("SPI1_Read error: %d\r\n", st);
  else {
    printf("Data: ");
    for (uint16_t i = 0; i < len; i++) {
      printf("%02X ", rx[i]);
    }
    printf("\r\n");
  }

  SPI1_CS_High();

  printf("SPI1_Read success: %d\r\n", st);
  return st;
}

HAL_StatusTypeDef SPI1_WriteRead(const uint8_t *tx, uint8_t *rx, uint16_t len) {
  if (!rx || len == 0)
    return HAL_OK;

  // If tx is NULL, transmit dummy bytes (0xFF is common, sometimes 0x00)
  uint8_t dummy = 0xFF;

  SPI1_CS_Low();

  HAL_StatusTypeDef st;
  if (tx) {
    st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len,
                                 SPI1_TIMEOUT_MS);
  } else {
    // clock out reads with dummy writes
    for (uint16_t i = 0; i < len; i++) {
      st = HAL_SPI_TransmitReceive(&hspi1, &dummy, &rx[i], 1, SPI1_TIMEOUT_MS);
      if (st != HAL_OK)
        break;
    }
  }

  SPI1_CS_High();
  return st;
}
