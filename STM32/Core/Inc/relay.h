#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stm32g4xx_hal.h>

void turn_on_relay(void);
void turn_off_relay(void);

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_H */