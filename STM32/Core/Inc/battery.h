#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32g4xx_hal.h>

uint16_t Read_Battery_Voltage_mV(ADC_HandleTypeDef *hadc1);
void is_battery_low(ADC_HandleTypeDef *hadc1);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */