#ifdef LED_FEEDBACK_H
#error "led_feedback.h included multiple times, check your includes!"
#else
#define LED_FEEDBACK_H

#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define LED_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_10

void LED_Blink(uint32_t duration_ms);
void LED_ON(void);
void LED_OFF(void);
void LED_SetDuration(uint16_t duration);
void LED_SetShortDuration(uint16_t duration);
void LED_SetLongDuration(uint16_t duration);
void LED_BlinkShort(void);
void LED_BlinkLong(void);
void LED_Toggle(void);

void LED_BlinkStatusCode(uint8_t code);

#endif // LED_FEEDBACK_H
