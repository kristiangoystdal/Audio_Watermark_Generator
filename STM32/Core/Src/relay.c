#include "relay.h"
#include "log.h"

#include <stm32g4xx_hal.h>

#define RELAY_GPIO_Port GPIOA
#define RELAY_PIN_ON GPIO_PIN_6
#define RELAY_PIN_OFF GPIO_PIN_5
#define RELAY_TOGGLE_DELAY_MS 10

void Relay_ResetBoth(void) {
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_ON, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_OFF, GPIO_PIN_RESET);
}

void Relay_SetMixingMode(void) {
  Relay_ResetBoth();

  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_ON, GPIO_PIN_SET);
  HAL_Delay(RELAY_TOGGLE_DELAY_MS);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_ON, GPIO_PIN_RESET);
}

void Relay_SetBypassMode(void) {
  Relay_ResetBoth();

  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_OFF, GPIO_PIN_SET);
  HAL_Delay(RELAY_TOGGLE_DELAY_MS);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_OFF, GPIO_PIN_RESET);
}