#ifndef OPAMPS_H
#define OPAMPS_H

#define OPAMP_POWER_GPIO_Port GPIOA
#define OPAMP_POWER_Pin GPIO_PIN_1
#define OPAMP_BOOT_TIME_MS 10

#include "stm32g4xx_hal.h"

void Turn_On_Opamps(DAC_HandleTypeDef *hdac1);
void Turn_Off_Opamps(DAC_HandleTypeDef *hdac1);
void Set_DAC_Output_To_Zero(DAC_HandleTypeDef *hdac1);
void Set_DAC_Output_To_Midlevel(DAC_HandleTypeDef *hdac1, uint16_t mid_value);

#endif // OPAMPS_H