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
 * @param RX true => RX mode, false => TX mode
 */
int8_t Radio_Init(bool RX);

/**
 * @brief Create the payload string to transmit based on user config and RTC
 * time.
 * @return Pointer to a static buffer containing the payload string.
 */
void Radio_Transmit(void);

/**
 * @brief Read RX FIFO (RXBYTES), dump bytes to printf, then SIDLE + SFRX + SRX.
 */
int Radio_Receive(uint8_t *out, size_t out_max);

/**
 * @brief Start RX loop: continuously read RX FIFO and print.
 */
void start_RX(void);

#ifdef __cplusplus
}
#endif

#endif // RADIO_H
