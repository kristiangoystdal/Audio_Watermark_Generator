#include "opamps.h"

#include "log.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_gpio.h"

void Opamps_Enable(DAC_HandleTypeDef *hdac1) {
  HAL_GPIO_WritePin(OPAMP_POWER_GPIO_Port, OPAMP_POWER_Pin, GPIO_PIN_SET);
  DAC_SetMidlevel(hdac1, 2048);
  HAL_Delay(OPAMP_BOOT_TIME_MS);
}

void Opamps_Disable(DAC_HandleTypeDef *hdac1) {
  DAC_SetZero(hdac1);
  HAL_GPIO_WritePin(OPAMP_POWER_GPIO_Port, OPAMP_POWER_Pin, GPIO_PIN_RESET);
}

void DAC_SetZero(DAC_HandleTypeDef *hdac1) {
  CLEAR_BIT(DAC1->CR, DAC_CR_TEN1);
  HAL_DAC_Start(hdac1, DAC_CHANNEL_1);
  HAL_DAC_SetValue(hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
}

void DAC_SetMidlevel(DAC_HandleTypeDef *hdac1, uint16_t mid_value) {
  CLEAR_BIT(DAC1->CR, DAC_CR_TEN1);
  HAL_DAC_Start(hdac1, DAC_CHANNEL_1);
  HAL_DAC_SetValue(hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, mid_value);
}