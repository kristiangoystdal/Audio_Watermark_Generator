#include "ism.h"
#include "ism_config_433.h"
#include "main.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>

#define CC1101_MISO_GPIO_Port GPIOB
#define CC1101_MISO_Pin GPIO_PIN_4

bool CC1101_WaitReadyMs(uint32_t timeout_ms) {
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(CC1101_MISO_GPIO_Port, CC1101_MISO_Pin) ==
         GPIO_PIN_SET) {
    if ((HAL_GetTick() - t0) >= timeout_ms) {
      return false;
    }
  }
  return true;
}

HAL_StatusTypeDef CC1101_Strobe(uint8_t strobe, uint8_t *status) {
  uint8_t tx[1] = {strobe};
  uint8_t rx[1] = {0};

  HAL_StatusTypeDef st = SPI1_WriteRead(tx, rx, 1);
  if (st == HAL_OK && status)
    *status = rx[0];
  return st;
}

HAL_StatusTypeDef CC1101_ReadReg(uint8_t addr, uint8_t *val, uint8_t *status) {
  uint8_t tx[2] = {(uint8_t)(addr | 0x80), 0xFF};
  uint8_t rx[2] = {0};

  HAL_StatusTypeDef st = SPI1_WriteRead(tx, rx, 2);
  if (st == HAL_OK) {
    if (status)
      *status = rx[0];
    if (val)
      *val = rx[1];
  }
  return st;
}

HAL_StatusTypeDef CC1101_WriteReg(uint8_t addr, uint8_t val, uint8_t *status) {
  uint8_t tx[2] = {(uint8_t)(addr & 0x7F), val};
  uint8_t rx[2] = {0};

  HAL_StatusTypeDef st = SPI1_WriteRead(tx, rx, 2);
  if (st == HAL_OK && status)
    *status = rx[1]; // status comes back while sending addr
  return st;
}

HAL_StatusTypeDef CC1101_WriteBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                       uint8_t *status) {
  if (len == 0)
    return HAL_OK;

  uint8_t tx[len + 1];
  uint8_t rx[len + 1];

  tx[0] = (uint8_t)(addr | 0x40); // burst write
  memcpy(&tx[1], vals, len);

  HAL_StatusTypeDef st = SPI1_WriteRead(tx, rx, len + 1);
  if (st == HAL_OK && status)
    *status = rx[0];
  return st;
}

HAL_StatusTypeDef CC1101_ReadBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                      uint8_t *status) {
  if (len == 0)
    return HAL_OK;

  uint8_t tx[len + 1];
  uint8_t rx[len + 1];

  tx[0] = (uint8_t)(addr | 0xC0); // burst read
  memset(&tx[1], 0xFF, len);

  HAL_StatusTypeDef st = SPI1_WriteRead(tx, rx, len + 1);
  if (st == HAL_OK) {
    if (status)
      *status = rx[0];
    if (vals)
      memcpy(vals, &rx[1], len);
  }
  return st;
}

bool CC1101_PowerUpReset(void) {
  // Make sure CS starts high
  SPI1_CS_High();
  HAL_Delay(1);

  // CS low -> high toggle (per TI recommended sequence)
  SPI1_CS_Low();
  HAL_Delay(1);
  SPI1_CS_High();
  HAL_Delay(1);

  // Wait for SO/MISO to go low (chip ready)
  if (!CC1101_WaitReadyMs(50)) {
    printf("CC1101 not ready (MISO stuck high) after CS toggle\r\n");
    return false;
  }

  // Now issue SRES with CS low
  uint8_t sres = 0x30;

  SPI1_CS_Low();
  if (!CC1101_WaitReadyMs(50)) {
    printf("CC1101 not ready before SRES\r\n");
    SPI1_CS_High();
    return false;
  }

  if (HAL_SPI_Transmit(&hspi1, &sres, 1, SPI1_TIMEOUT_MS) != HAL_OK) {
    printf("SPI transmit SRES failed\r\n");
    SPI1_CS_High();
    return false;
  }

  // Wait again for reset completion
  if (!CC1101_WaitReadyMs(50)) {
    printf("CC1101 not ready after SRES\r\n");
    SPI1_CS_High();
    return false;
  }

  SPI1_CS_High();
  HAL_Delay(1);
  return true;
}
