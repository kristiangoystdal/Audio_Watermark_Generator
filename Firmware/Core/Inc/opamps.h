#ifndef OPAMPS_H
#define OPAMPS_H

#define OPAMP_POWER_GPIO_Port GPIOA
#define OPAMP_POWER_Pin GPIO_PIN_1
#define OPAMP_BOOT_TIME_MS 10

#include "stm32g4xx_hal.h"

void Opamps_Enable(DAC_HandleTypeDef *hdac1);
void Opamps_Disable(DAC_HandleTypeDef *hdac1);
void DAC_SetZero(DAC_HandleTypeDef *hdac1);
void DAC_SetMidlevel(DAC_HandleTypeDef *hdac1, uint16_t mid_value);

#endif // OPAMPS_H