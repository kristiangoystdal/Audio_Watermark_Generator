#ifndef CC1101_H
#define CC1101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h" // HAL_StatusTypeDef
#include <stdbool.h>       // bool
#include <stddef.h>        // size_t
#include <stdint.h>        // uint8_t

// Bit-bang SPI pins (all on GPIOB)
#define BB_SCK GPIO_PIN_3  // PB3
#define BB_MOSI GPIO_PIN_4 // PB4
#define BB_MISO GPIO_PIN_5 // PB5
#define BB_CS GPIO_PIN_6   // PB6

#define MOSI_HIGH (GPIOB->BSRR = BB_MOSI)
#define MOSI_LOW (GPIOB->BSRR = ((uint32_t)BB_MOSI << 16))
#define SCK_HIGH (GPIOB->BSRR = BB_SCK)
#define SCK_LOW (GPIOB->BSRR = ((uint32_t)BB_SCK << 16))

#define MISO_READ (GPIOB->IDR & BB_MISO)

// strobe: CC1101 command byte (e.g. 0x30=SRES, 0x35=STX); status may be NULL
HAL_StatusTypeDef CC1101_Strobe(uint8_t strobe, uint8_t *status);

// addr: 0x00–0x2E config regs or 0xFx status regs; val/status may be NULL
HAL_StatusTypeDef CC1101_ReadReg(uint8_t addr, uint8_t *val, uint8_t *status);

HAL_StatusTypeDef CC1101_WriteReg(uint8_t addr, uint8_t val, uint8_t *status);

// addr: e.g. 0x3F for TX FIFO; status may be NULL
HAL_StatusTypeDef CC1101_WriteBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                       uint8_t *status);

// addr: e.g. 0xFF for RX FIFO burst read; status may be NULL
HAL_StatusTypeDef CC1101_ReadBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                      uint8_t *status);

// More robust than a bare SRES strobe — CS toggle sequence before reset
bool CC1101_PowerUpReset(void);

#ifdef __cplusplus
}
#endif

#endif // CC1101_H
