#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stm32g4xx_hal.h>

void Relay_SetMixingMode(void);
void Relay_SetBypassMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_H */