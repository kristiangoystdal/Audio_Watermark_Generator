#include "opamps.h"

#include "log.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_gpio.h"

void Turn_On_Opamps(DAC_HandleTypeDef *hdac1) {
  HAL_GPIO_WritePin(OPAMP_POWER_GPIO_Port, OPAMP_POWER_Pin, GPIO_PIN_SET);
  Set_DAC_Output_To_Midlevel(hdac1, 2048);
  HAL_Delay(OPAMP_BOOT_TIME_MS);
}

void Turn_Off_Opamps(DAC_HandleTypeDef *hdac1) {
  Set_DAC_Output_To_Zero(hdac1);
  HAL_GPIO_WritePin(OPAMP_POWER_GPIO_Port, OPAMP_POWER_Pin, GPIO_PIN_RESET);
  HAL_Delay(OPAMP_BOOT_TIME_MS);
}

void Set_DAC_Output_To_Zero(DAC_HandleTypeDef *hdac1) {
  // Set DAC output to zero (silence)

  CLEAR_BIT(DAC1->CR, DAC_CR_TEN1);

  HAL_DAC_Start(hdac1, DAC_CHANNEL_1);

  HAL_DAC_SetValue(hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  LOGF("DAC output set to zero\r\n");
}

void Set_DAC_Output_To_Midlevel(DAC_HandleTypeDef *hdac1, uint16_t mid_value) {
  CLEAR_BIT(DAC1->CR, DAC_CR_TEN1);

  // Ensure DAC channel is enabled
  HAL_DAC_Start(hdac1, DAC_CHANNEL_1);

  // Set mid-scale (12-bit right aligned)
  HAL_DAC_SetValue(hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, mid_value);
  LOGF("DAC output set to mid-level\r\n");
}