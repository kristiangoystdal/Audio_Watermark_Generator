#ifndef __SPEAKER_H
#define __SPEAKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stm32g4xx_hal.h>

#define SPEAKER_GPIO_Port GPIOA
#define SPEAKER_Pin GPIO_PIN_7
#define SPEAKER_TOGGLE_DELAY_MS 250

void Speaker_TurnOn(void);
void Speaker_TurnOff(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPEAKER_H */