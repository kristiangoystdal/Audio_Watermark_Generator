#include "relay.h"
#include "log.h"

#include <stm32g4xx_hal.h>

#define RELAY_GPIO_Port GPIOA
#define RELAY_PIN_ON GPIO_PIN_5
#define RELAY_PIN_OFF GPIO_PIN_6
#define RELAY_TOGGLE_DELAY_MS 100

void turn_on_relay(void) {
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_ON, GPIO_PIN_SET);
  HAL_Delay(RELAY_TOGGLE_DELAY_MS);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_OFF, GPIO_PIN_RESET);

  LOGF("Set relay to mixing mode\r\n");
  LOGF("--------------------------\r\n");
}

void turn_off_relay(void) {
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_OFF, GPIO_PIN_SET);
  HAL_Delay(RELAY_TOGGLE_DELAY_MS);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_PIN_OFF, GPIO_PIN_RESET);

  LOGF("Set relay to bypass mode\r\n");
  LOGF("--------------------------\r\n");
}