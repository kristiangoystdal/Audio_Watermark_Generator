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

/**
 * @brief Calibrate the crystal frequency trim by estimating the error from the
 * known capacitive load mismatch and adjusting the FREQ registers accordingly.
 * This is a workaround for the fact that the CC1101 doesn't have a built-in
 * crystal trim calibration routine, and the frequency error from the load
 * mismatch can cause significant frequency offset. By applying an empirical
 * correction based on the estimated ppm error, we can improve frequency
 * accuracy without needing to measure the actual frequency offset with test
 * equipment.
 *
 * @return 0 on success, nonzero error code on failure
 */
int8_t Radio_CalibrateXtalFrequency(void);

#ifdef __cplusplus
}
#endif

#endif // RADIO_H
