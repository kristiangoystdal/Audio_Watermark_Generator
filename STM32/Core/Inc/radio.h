#ifndef RADIO_H
#define RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief Initialize CC1101 into RX mode (apply RX config, calibrate, enter SRX)
 */
void init_RX(void);

/**
 * @brief Initialize CC1101 into TX mode (apply TX config, calibrate)
 */
void init_TX(void);

/**
 * @brief Reset CC1101, print PARTNUM/VERSION, then init either RX or TX mode.
 * @param RX true => RX mode, false => TX mode
 */
void init_radio(bool RX);

/**
 * @brief Build and transmit a test packet (currently "Einar suger" + 0xEB
 * header). Loads TX FIFO, strobe STX, waits until MARCSTATE == IDLE.
 */
void transmit_bytes(void);

/**
 * @brief Start TX loop (currently runs transmit_bytes with a 2-minute window,
 *        but breaks early for quick test).
 */
void start_TX(void);

/**
 * @brief Read RX FIFO (RXBYTES), dump bytes to printf, then SIDLE + SFRX + SRX.
 */
int read_RX(uint8_t *out, size_t out_max);

/**
 * @brief Start RX loop: continuously read RX FIFO and print.
 */
void start_RX(void);

#ifdef __cplusplus
}
#endif

#endif // RADIO_H
