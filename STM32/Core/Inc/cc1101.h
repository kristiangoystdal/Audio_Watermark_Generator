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

// Macros for MOSI and SCK pin states
// MOSI
#define MOSI_HIGH (GPIOB->BSRR = BB_MOSI)
#define MOSI_LOW (GPIOB->BSRR = ((uint32_t)BB_MOSI << 16))

// SCK
#define SCK_HIGH (GPIOB->BSRR = BB_SCK)
#define SCK_LOW (GPIOB->BSRR = ((uint32_t)BB_SCK << 16))

#define MISO_READ (GPIOB->IDR & BB_MISO)

/**
 * @brief Send a CC1101 command strobe and optionally return the status byte.
 *
 * @param strobe  CC1101 strobe command (e.g. 0x30 for SRES, 0x35 for STX, etc.)
 * @param status  Optional pointer to receive status byte (may be NULL)
 * @return HAL status
 */
HAL_StatusTypeDef CC1101_Strobe(uint8_t strobe, uint8_t *status);

/**
 * @brief Read a single CC1101 register.
 *
 * @param addr    Register address (0x00-0x2E, or extended via 0x2F + ext addr)
 * @param val     Optional pointer to store the read value (may be NULL)
 * @param status  Optional pointer to receive status byte (may be NULL)
 * @return HAL status
 */
HAL_StatusTypeDef CC1101_ReadReg(uint8_t addr, uint8_t *val, uint8_t *status);

/**
 * @brief Write a single CC1101 register.
 *
 * @param addr    Register address
 * @param val     Value to write
 * @param status  Optional pointer to receive status byte (may be NULL)
 * @return HAL status
 */
HAL_StatusTypeDef CC1101_WriteReg(uint8_t addr, uint8_t val, uint8_t *status);

/**
 * @brief Burst write multiple bytes starting at a CC1101 register/FIFO address.
 *
 * @param addr    Start address (e.g. 0x3F for TX FIFO)
 * @param vals    Pointer to data to write
 * @param len     Number of bytes to write
 * @param status  Optional pointer to receive status byte (may be NULL)
 * @return HAL status
 */
HAL_StatusTypeDef CC1101_WriteBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                       uint8_t *status);

/**
 * @brief Burst read multiple bytes starting at a CC1101 register/FIFO address.
 *
 * @param addr    Start address (e.g. 0x3F for RX FIFO)
 * @param vals    Pointer to buffer for received data (may be NULL if you only
 * want status)
 * @param len     Number of bytes to read
 * @param status  Optional pointer to receive status byte (may be NULL)
 * @return HAL status
 */
HAL_StatusTypeDef CC1101_ReadBurstReg(uint8_t addr, uint8_t *vals, size_t len,
                                      uint8_t *status);

/**
 * @brief Perform a power-up reset of the CC1101. This is more robust than just
 * sending the SRES strobe, especially on cold boot.
 *
 * @return true if reset was successful (PARTNUM and VERSION read correctly),
 * false otherwise
 */
bool CC1101_PowerUpReset(void);

#ifdef __cplusplus
}
#endif

#endif // CC1101_H
