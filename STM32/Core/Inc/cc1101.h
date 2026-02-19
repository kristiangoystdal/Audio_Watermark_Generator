#ifndef CC1101_H
#define CC1101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h" // HAL_StatusTypeDef
#include <stdbool.h>       // bool
#include <stddef.h>        // size_t
#include <stdint.h>        // uint8_t

/**
 * @breif Wait for CC1101 to be ready by monitoring MISO pin. Returns true if
 * ready within timeout, false if timeout occurs.
 *
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true if CC1101 is ready (MISO goes low) within timeout, false if
 * timeout occurs
 */
bool CC1101_WaitReadyMs(uint32_t timeout_ms);

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
