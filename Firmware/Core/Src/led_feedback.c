#include "led_feedback.h"

// Duration for LED blinks in milliseconds
uint16_t short_duration_ms = 100;
uint16_t long_duration_ms = 300;
uint16_t duration_between_blinks_ms = 200;
static const uint8_t NUM_BITS = 4;
static const uint8_t REPS = 3;

void LED_Blink(uint32_t duration_ms) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_Delay(duration_ms);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_ON(void) { HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); }

void LED_OFF(void) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_SetShortDuration(uint16_t duration) { short_duration_ms = duration; }

void LED_SetLongDuration(uint16_t duration) { long_duration_ms = duration; }

void LED_BlinkShort(void) { LED_Blink(short_duration_ms); }

void LED_BlinkLong(void) { LED_Blink(long_duration_ms); }

void LED_BlinkStatusCode(uint8_t code) {
  for (uint8_t rep = 0; rep < REPS; rep++) {
    // MSB-first: bit 3,2,1,0
    for (int8_t b = (int8_t)NUM_BITS - 1; b >= 0; b--) {
      if (code & (1u << (uint8_t)b)) {
        LED_BlinkLong(); // 1 = long
      } else {
        LED_BlinkShort(); // 0 = short
      }

      // gap between bits, but not after the last bit
      if (b != 0) {
        HAL_Delay(duration_between_blinks_ms);
      }
    }

    // longer gap between repetitions
    HAL_Delay(duration_between_blinks_ms * 5);
  }
}

void LED_Toggle(void) { HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); }