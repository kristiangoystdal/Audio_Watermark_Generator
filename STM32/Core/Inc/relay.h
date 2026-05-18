#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stm32g4xx_hal.h>

#define RELAY_GPIO_Port GPIOA
#define RELAY_PIN_ON GPIO_PIN_6
#define RELAY_PIN_OFF GPIO_PIN_5
#define RELAY_TOGGLE_DELAY_MS 250

void Relay_SetMixingMode(void);
void Relay_SetBypassMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_H */