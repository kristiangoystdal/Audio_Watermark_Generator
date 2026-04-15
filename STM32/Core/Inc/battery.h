#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32g4xx_hal.h>

uint16_t Battery_ReadVoltage(ADC_HandleTypeDef *hadc1);
void Battery_IsLow(ADC_HandleTypeDef *hadc1);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */