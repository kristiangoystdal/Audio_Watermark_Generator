#include "battery.h"
#include "error_codes.h"
#include "log.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define LOW_BATTERY_THRESHOLD_MV 3000

uint16_t Battery_ReadVoltage(ADC_HandleTypeDef *hadc1) {
  uint32_t adc_value = 0;
  uint16_t voltage_mV = 0;

  HAL_ADC_Start(hadc1);
  if (HAL_ADC_PollForConversion(hadc1, HAL_MAX_DELAY) == HAL_OK) {
    adc_value = HAL_ADC_GetValue(hadc1);
  }
  HAL_ADC_Stop(hadc1);

  // 12-bit ADC, 3.3V ref; x2 to undo the external 1M/1M voltage divider
  voltage_mV = (uint16_t)((adc_value * 3300 * 2) / 4095);

  return voltage_mV;
}

void Battery_IsLow(ADC_HandleTypeDef *hadc1) {
  uint16_t voltage_mV = Battery_ReadVoltage(hadc1);
  if (voltage_mV < LOW_BATTERY_THRESHOLD_MV) {
    LOGF("Battery voltage is low: %u mV\r\n", voltage_mV);
    Error_Handler_Code(STATUS_CODE_BATTERY_LOW);
  }
  return;
}