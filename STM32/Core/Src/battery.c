#include "battery.h"
#include "error_codes.h"
#include "log.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define LOW_BATTERY_THRESHOLD_MV 3000 // Example threshold for low battery

uint16_t Read_Battery_Voltage_mV(ADC_HandleTypeDef *hadc1) {
  uint32_t adc_value = 0;
  uint16_t voltage_mV = 0;

  // Read ADC value
  HAL_ADC_Start(hadc1);
  if (HAL_ADC_PollForConversion(hadc1, HAL_MAX_DELAY) == HAL_OK) {
    adc_value = HAL_ADC_GetValue(hadc1);
  }
  HAL_ADC_Stop(hadc1);

  // Convert ADC value to millivolts (assuming 12-bit ADC and 3.3V reference)
  // Input voltage is divided by 2 due to external voltage divider (1M and 1M),
  // so we multiply by 2 to compensate
  LOGF("Raw ADC value: %lu\r\n", (unsigned long)adc_value);
  voltage_mV = (uint16_t)((adc_value * 3300 * 2) / 4095);

  return voltage_mV;
}

void is_battery_low(ADC_HandleTypeDef *hadc1) {
  uint16_t voltage_mV = Read_Battery_Voltage_mV(hadc1);
  if (voltage_mV < LOW_BATTERY_THRESHOLD_MV) {
    LOGF("Battery voltage is low: %u mV\r\n", voltage_mV);
    // Error_Handler_Code(STATUS_CODE_BATTERY_LOW);
  }
  return;
}