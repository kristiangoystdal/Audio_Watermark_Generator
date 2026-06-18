#ifndef RADIO_H
#define RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int8_t Radio_InitRXMode(void);
int8_t Radio_InitTXMode(void);

// operation_mode: 0=RX, 1=TX, 2=standalone; retries 3x with backoff
int8_t Radio_Init(int8_t operation_mode);

void Radio_Transmit(void);

// Returns byte count read, 0 if FIFO empty, -1 on FIFO overflow or CRC fail
int Radio_Receive(uint8_t *out, size_t out_max);

void Radio_EnterWOR(void);
void Radio_EnterSleep(void);
void Radio_EnterIdle(void);

// Applies empirical ppm correction to FREQ regs to compensate for capacitive
// load mismatch on the crystal — no test equipment needed
int8_t Radio_CalibrateXtalFrequency(void);

#ifdef __cplusplus
}
#endif

#endif // RADIO_H
