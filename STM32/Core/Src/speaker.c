#include "speaker.h"
#include "log.h"

#include <stm32g4xx_hal.h>

void Speaker_TurnOn(void) {
  HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
  HAL_Delay(SPEAKER_TOGGLE_DELAY_MS);
}

void Speaker_TurnOff(void) {
  HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
}
