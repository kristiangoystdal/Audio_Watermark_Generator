#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// ---------------------------
// Public handle
// ---------------------------
extern SPI_HandleTypeDef hspi1;

// ---------------------------
// Config (edit as needed)
// ---------------------------
#define SPI1_TIMEOUT_MS 100u

// Pick your CS pin here (CHANGE THIS to match your wiring)
#define SPI1_CS_GPIO_PORT GPIOB
#define SPI1_CS_PIN GPIO_PIN_0

// ---------------------------
// API
// ---------------------------
// Low-level CS control
static inline void SPI1_CS_Low(void) {
  HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN, GPIO_PIN_RESET);
}
static inline void SPI1_CS_High(void) {
  HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT, SPI1_CS_PIN, GPIO_PIN_SET);
}

// Transaction helpers (CS toggled automatically)
HAL_StatusTypeDef SPI1_Write(const uint8_t *tx, uint16_t len);
HAL_StatusTypeDef SPI1_Read(uint8_t *rx, uint16_t len);
HAL_StatusTypeDef SPI1_WriteRead(const uint8_t *tx, uint8_t *rx, uint16_t len);

// Optional: begin/end if you want to keep CS low across multiple calls
static inline void SPI1_Begin(void) { SPI1_CS_Low(); }
static inline void SPI1_End(void) { SPI1_CS_High(); }

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
