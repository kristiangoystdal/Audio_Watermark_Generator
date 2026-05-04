#ifndef RADIO_H
#define RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize CC1101 into RX mode (apply RX config, calibrate, enter SRX)
 */
int8_t Radio_InitRXMode(void);

/**
 * @brief Initialize CC1101 into TX mode (apply TX config, calibrate)
 */
int8_t Radio_InitTXMode(void);

/**
 * @brief Reset CC1101, print PARTNUM/VERSION, then init either RX or TX mode.
 * @param operation_mode 0 for RX, 1 for TX, 2 for standalone (no radio) mode
 * @return 0 on success, nonzero error code on failure
 */
int8_t Radio_Init(int8_t operation_mode);

/**
 * @brief Transmit payload 3 times back-to-back for WOR preamble coverage.
 */
void Radio_Transmit(void);

/**
 * @brief Read RX FIFO (RXBYTES), dump bytes to printf, then SIDLE + SFRX + SRX.
 */
int Radio_Receive(uint8_t *out, size_t out_max);

/**
 * @brief Enter WOR mode (after configuring WOR settings in init).
 */
void Radio_EnterWOR(void);

/**
 * @brief Enter sleep mode.
 */
void Radio_EnterSleep(void);

/**
 * @brief Enter idle mode.
 */
void Radio_EnterIdle(void);

#ifdef __cplusplus
}
#endif

#endif // RADIO_H
