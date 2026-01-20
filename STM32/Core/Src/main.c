/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <user_config.h>

#include "cmsis_gcc.h"
#include "ds3231.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_gpio.h"
#include <frequency_pairs.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define pi 3.14159265358979323846

// Compute NUM_CHARS based on user_config.h toggles
#define LEN_IF(cond, lit) ((cond) ? (sizeof(lit) - 1u) : 0u)

#define MAX_NUM_CHARS                                                          \
  (LEN_IF(INCLUDE_USER_STRING, "/STR" USER_STRING) +                           \
   LEN_IF(INCLUDE_DEVICE_ID, "/DID0000") +                                     \
   LEN_IF(INCLUDE_LOCATION, "/LOC00.0000,00.0000") +                           \
   LEN_IF(INCLUDE_TEMPERATURE, "/TMP000") +                                    \
   LEN_IF(INCLUDE_TIME, "/TIM00000000000000") + 4u)

#define BITSTREAM_LENGTH (MAX_NUM_CHARS * 8u)

#define MID_12B 2048

#define BIT_POLARITY 0          // 0 = normal, 1 = inverted
#define DS3231_ADDR (0x68 << 1) // 7-bit addr shifted for HAL

#define DIV_FLOOR(n, d) ((uint32_t)((n) / (d)))
#define DIV_ROUND(n, d) ((uint32_t)(((n) + ((d) / 2u)) / (d))) // round half-up

#define FSK_LOWER_NUM_SAMPLES DIV_FLOOR(FS_HZ, FSK_LOWER_FREQUENCY)
#define FSK_HIGHER_NUM_SAMPLES DIV_FLOOR(FS_HZ, FSK_HIGHER_FREQUENCY)

#define FSK_LOWER_PERIODS DIV_ROUND(MIN_BIT_SAMPLES, FSK_LOWER_NUM_SAMPLES)
#define FSK_HIGHER_PERIODS DIV_ROUND(MIN_BIT_SAMPLES, FSK_HIGHER_NUM_SAMPLES)

#define OUTPUT_SEGMENT 1000
#define BUFFER_SIZE (OUTPUT_SEGMENT * 5)

// Add/keep these near your defines
#define DMA_BUF_LEN (BUFFER_SIZE)

// (Optional but recommended) make sure it's even at compile time
#if (DMA_BUF_LEN % 2u) != 0u
#error "BUFFER_SIZE must be even (so it can be split into two halves)."
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

// Circular buffer for DAC output
__ALIGN_BEGIN __IO uint16_t output_buffer[BUFFER_SIZE] __ALIGN_END;

uint16_t sine_val_low[FSK_LOWER_NUM_SAMPLES];
uint16_t sine_val_high[FSK_HIGHER_NUM_SAMPLES];
uint16_t dc_mid[FSK_LOWER_NUM_SAMPLES];

static uint16_t bitstream[BITSTREAM_LENGTH];

static volatile uint32_t current_period = 1;
static volatile uint16_t current_bit = 2;

float total_time = 0.0;
uint32_t pulse_time = 1000; // in timer ticks
double tick_hz = 1000000.0;
uint32_t arr = 0;
uint32_t ticks = 0;

static volatile bool tx_active = false;

int counter = INTERVAL_BETWEEN_REPEATS_MINUTES;
bool first_run = true;

// User string buffer
char input_string[MAX_NUM_CHARS + 1]; // global

rtc_time_t now;
int8_t temp_int = 0;

uint32_t cable_speaker_delay_ms = 100;
uint32_t fs = 0;

float f_0_bit_duration = 0.0f;
float f_1_bit_duration = 0.0f;
float f_2_bit_duration = 0.0f;

uint32_t current_bitstream_index = 0;

bool moved_to_next_bit = false;

typedef struct {
  bool moved_to_next_bit;
  size_t current_period;
  size_t current_index;
} FillResult;

FillResult result = {false, 0, 0};

size_t current_sine_period = 0;
size_t current_sine_index = 0;
uint16_t next_bit = 2;

bool rtc_woke = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void DWT_Delay_Init(void) {
  // Enable DWT and CYCCNT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (SystemCoreClock / 1000000U) * us;
  while ((DWT->CYCCNT - start) < ticks)
    ;
}

void DWT_Delay_ms(uint32_t ms) { DWT_Delay_us(ms * 1000U); }

int make_preamble(int start_idx) {
  int k = start_idx;
  uint16_t id = 0; // 0x2DD4
  for (int i = 0; i < 16 && k < BITSTREAM_LENGTH; ++i) {
    bitstream[k++] = (((id >> (15 - i)) & 1) ^ BIT_POLARITY);
  }

  return k;
}

void make_bitstream_from_string(const char *str) {
  int k = 0;

  // Preamble bit - silence
  for (int i = 0; i < 1 && k < BITSTREAM_LENGTH; ++i) {
    bitstream[k++] = 2; // silence
  }

  // Data bits from string
  for (int i = 0; str[i] != '\0' && k < BITSTREAM_LENGTH; ++i) {
    for (int b = 7; b >= 0 && k < BITSTREAM_LENGTH; b--) {
      bitstream[k++] = ((str[i] >> b) & 1) ^ BIT_POLARITY; // data
    }
  }

  // Add 8 bits of silence at the end if there's space
  for (int i = 0; i < 8 && k < BITSTREAM_LENGTH; ++i) {
    bitstream[k++] = 2; // silence
  }

  // Fill remaining bits with silence if any
  while (k < BITSTREAM_LENGTH) {
    bitstream[k++] = 2; // silence
  }
}

void DS3231_PowerOn(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void DS3231_PowerOff(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void update_time_temperature(void) {
  DS3231_PowerOn();
  Get_Time(&now);
  Read_Temperature(&temp_int);
  // DS3231_PowerOff();
}

void update_input_string(void) {
  input_string[0] = '\0';
  size_t offset = 0;
  size_t remaining = MAX_NUM_CHARS;
  int n = 0;

  // Append USER_STRING if enabled
  if (INCLUDE_USER_STRING) {
    n = snprintf(&input_string[offset], remaining, "/STR%s", USER_STRING);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }
  // Append DEVICE_ID if enabled
  if (INCLUDE_DEVICE_ID) {
    n = snprintf(&input_string[offset], remaining, "%s/DID%d",
                 (offset > 0) ? "" : "", DEVICE_ID);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }
  // Append LOCATION if enabled
  if (INCLUDE_LOCATION) {
    n = snprintf(&input_string[offset], remaining, "%s/LOC%s",
                 (offset > 0) ? "" : "", LOCATION);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }
  // Append TEMPERATURE if enabled
  if (INCLUDE_TEMPERATURE) {
    n = snprintf(&input_string[offset], remaining, "%s/TMP%d",
                 (offset > 0) ? "" : "", temp_int);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }
  // Append TIME if enabled
  if (INCLUDE_TIME) {
    n = snprintf(&input_string[offset], remaining,
                 "%s/TIM%02d%02d%02d%02d%02d%02d%04d", (offset > 0) ? "" : "",
                 now.hours, now.minutes, now.seconds, now.day, now.date,
                 now.month, now.year);

    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }
  // Add termination sign
  if (strlen(input_string) > 0) {
    n = snprintf(&input_string[offset], remaining, "/");
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }
}

// Function to generate sine wave lookup table for low frequency
void get_sineval_low(void) {
  for (int i = 0; i < FSK_LOWER_NUM_SAMPLES; i++) {
    sine_val_low[i] =
        (uint16_t)((4095.0 / 2.0) *
                   (1.0 +
                    sinf(2.0 * pi * i / FSK_LOWER_NUM_SAMPLES) * (1.5 / 1.65)));
  }
}

// Function to generate sine wave lookup table for the high frequency
void get_sineval_high(void) {
  for (int i = 0; i < FSK_HIGHER_NUM_SAMPLES; i++) {
    sine_val_high[i] =
        (uint16_t)((4095.0 / 2.0) *
                   (1.0 + sinf(2.0 * pi * i / FSK_HIGHER_NUM_SAMPLES) *
                              (1.5 / 1.65)));
  }
}

// Function to generate mid-scale lookup table
void get_dc_mid(void) {
  for (int i = 0; i < FSK_LOWER_NUM_SAMPLES; ++i)
    dc_mid[i] = MID_12B;
}

uint32_t get_dac_sample_rate_hz(void) {
  uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    tim_clk *= 2; // timer clock doubled if APB prescaler != 1

  return tim_clk / ((htim2.Init.Prescaler + 1) * (htim2.Init.Period + 1));
}

// Function to calculate the total time it takes to send the bitstream
void calculate_pulse_time(void) {
  fs = get_dac_sample_rate_hz();

  float f_0 = FSK_LOWER_FREQUENCY;
  float f_1 = FSK_HIGHER_FREQUENCY;

  f_0_bit_duration = (float)FSK_LOWER_PERIODS / f_0;
  f_1_bit_duration = (float)FSK_HIGHER_PERIODS / f_1;
  f_2_bit_duration = f_0_bit_duration; // silence uses lower frequency

  // Exact bitstream time
  total_time = 0.0f;
  for (int i = 0; i < BITSTREAM_LENGTH; ++i) {
    if (bitstream[i] == 0) {
      total_time += ((float)FSK_LOWER_PERIODS) / f_0;
    } else if (bitstream[i] == 1) {
      total_time += (float)FSK_HIGHER_PERIODS / f_1;
    } else {
      total_time += (float)FSK_LOWER_PERIODS / f_0;
    }
  }

  // Add silence time
  total_time *= 1.05f;                            // 5% margin
  total_time += cable_speaker_delay_ms / 1000.0f; // 10ms margin

  // Convert to ticks using the PSC already set for TIM8
  uint32_t tim8_clk = HAL_RCC_GetPCLK2Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
    tim8_clk *= 2U;
  tick_hz = (double)tim8_clk / (double)(htim8.Init.Prescaler + 1U);

  arr = __HAL_TIM_GET_AUTORELOAD(&htim8);
  ticks = (uint32_t)llround(total_time * tick_hz);

  pulse_time = ticks;
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse_time);
}

size_t get_target_bit_periods(uint16_t bit) {
  if (bit == 0) {
    return FSK_LOWER_PERIODS;
  } else if (bit == 1) {
    return FSK_HIGHER_PERIODS;
  } else {
    return FSK_LOWER_PERIODS; // silence uses lower frequency periods
  }
}

#include <string.h>

FillResult fill_half_buffer(uint16_t *buffer, uint16_t bit, uint16_t next_bit,
                            size_t generation_size, size_t current_sine_period,
                            size_t current_sine_index) {
  size_t out = 0;
  size_t period = current_sine_period;
  size_t idx = current_sine_index;
  uint16_t cur_bit = bit;
  bool moved = false;

  while (out < generation_size) {

    /* Select LUT */
    const uint16_t *lut;
    size_t lut_len;
    size_t target_periods;

    if (cur_bit == 0) {
      lut = sine_val_low;
      lut_len = FSK_LOWER_NUM_SAMPLES;
      target_periods = FSK_LOWER_PERIODS;
    } else if (cur_bit == 1) {
      lut = sine_val_high;
      lut_len = FSK_HIGHER_NUM_SAMPLES;
      target_periods = FSK_HIGHER_PERIODS;
    } else {
      lut = dc_mid;
      lut_len = FSK_LOWER_NUM_SAMPLES; // constant value
      target_periods = FSK_LOWER_PERIODS;
    }

    /* How many samples can we copy this round? */
    size_t left_buf = generation_size - out;
    size_t left_lut = lut_len - idx;
    size_t n = left_buf < left_lut ? left_buf : left_lut;

    memcpy(&buffer[out], &lut[idx], n * sizeof(uint16_t));

    out += n;
    idx += n;

    /* Wrapped LUT = completed one period */
    if (idx == lut_len) {
      idx = 0;
      period++;

      /* Completed all periods for this bit */
      if (period == target_periods) {
        period = 0;
        cur_bit = next_bit;
        moved = true;
      }
    }
  }

  return (FillResult){.moved_to_next_bit = moved,
                      .current_period = period,
                      .current_index = idx};
}

// DAC conversion complete callbacks for half buffer
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  if (!tx_active)
    return;

  // Fill the first half of the buffer based on the current bit
  result = fill_half_buffer((uint16_t *)&output_buffer[0], current_bit,
                            next_bit, BUFFER_SIZE / 2, current_sine_period,
                            current_sine_index);

  moved_to_next_bit = result.moved_to_next_bit;
  current_sine_period = result.current_period;
  current_sine_index = result.current_index;

  if (moved_to_next_bit) {
    moved_to_next_bit = false;
    current_bitstream_index = (current_bitstream_index + 1) % BITSTREAM_LENGTH;
    current_bit = bitstream[current_bitstream_index];
    next_bit = bitstream[(current_bitstream_index + 1) % BITSTREAM_LENGTH];
  }
}

// DAC conversion complete callback for the full buffer
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  if (!tx_active)
    return;

  // Fill the first half of the buffer based on the current bit
  result = fill_half_buffer((uint16_t *)&output_buffer[BUFFER_SIZE / 2],
                            current_bit, next_bit, BUFFER_SIZE / 2,
                            current_sine_period, current_sine_index);

  moved_to_next_bit = result.moved_to_next_bit;
  current_sine_period = result.current_period;
  current_sine_index = result.current_index;

  if (moved_to_next_bit) {
    moved_to_next_bit = false;
    current_bitstream_index = (current_bitstream_index + 1) % BITSTREAM_LENGTH;
    current_bit = bitstream[current_bitstream_index];
    next_bit = bitstream[(current_bitstream_index + 1) % BITSTREAM_LENGTH];
  }
}

void reset_dac(void) {
  // Stop DAC DMA
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  // Restart the bitstream from the beginning
  current_period = 0;
  current_bitstream_index = 0;
  current_bit = bitstream[0];
  next_bit = bitstream[1 % BITSTREAM_LENGTH];
  current_sine_period = 0;
  current_sine_index = 0;

  // Prefill both halves of the circular buffer with the first bit
  result = fill_half_buffer((uint16_t *)&output_buffer[0], current_bit,
                            next_bit, BUFFER_SIZE / 2, 0, 0);

  moved_to_next_bit = result.moved_to_next_bit;
  current_sine_period = result.current_period;
  current_sine_index = result.current_index;

  if (moved_to_next_bit) {
    moved_to_next_bit = false;
    current_bitstream_index = (current_bitstream_index + 1) % BITSTREAM_LENGTH;
    current_bit = bitstream[current_bitstream_index];
    next_bit = bitstream[(current_bitstream_index + 1) % BITSTREAM_LENGTH];
  }

  result = fill_half_buffer((uint16_t *)&output_buffer[BUFFER_SIZE / 2],
                            current_bit, next_bit, BUFFER_SIZE / 2,
                            current_sine_period, current_sine_index);

  moved_to_next_bit = result.moved_to_next_bit;
  current_sine_period = result.current_period;
  current_sine_index = result.current_index;

  if (moved_to_next_bit) {
    moved_to_next_bit = false;
    current_bitstream_index = (current_bitstream_index + 1) % BITSTREAM_LENGTH;
    current_bit = bitstream[current_bitstream_index];
    next_bit = bitstream[(current_bitstream_index + 1) % BITSTREAM_LENGTH];
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Base_Start(&htim2);

  tx_active = true;

  // Restart DAC with the circular buffer
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)output_buffer,
                    BUFFER_SIZE, DAC_ALIGN_12B_R);
}

void Run_Transmission(void) {
  update_time_temperature();
  if (first_run && now.minutes != STARTING_MINUTE && ENABLE_DELAYED_START) {
    counter = INTERVAL_BETWEEN_REPEATS_MINUTES;
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, MID_12B);
  } else {
    first_run = false;
    if (!USE_DEFAULT_INTERVAL_BETWEEN_REPEATS) {
      counter++;
      if (counter >= INTERVAL_BETWEEN_REPEATS_MINUTES) {
        counter = 0;

        if (USE_SPEAKER_TRANSMISSION) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        }
        if (USE_CABLE_TRANSMISSION) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
        if (USE_CABLE_TRANSMISSION || USE_SPEAKER_TRANSMISSION) {
          DWT_Delay_ms(cable_speaker_delay_ms);
        }
        if (USE_CABLE_TRANSMISSION) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }

        update_time_temperature();
        update_input_string();
        make_bitstream_from_string(input_string);
        calculate_pulse_time();
        reset_dac();
      }
    } else {

      if (USE_SPEAKER_TRANSMISSION) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
      }
      if (USE_CABLE_TRANSMISSION) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      }
      if (USE_CABLE_TRANSMISSION || USE_SPEAKER_TRANSMISSION) {
        DWT_Delay_ms(cable_speaker_delay_ms);
      }
      if (USE_CABLE_TRANSMISSION) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      }
      update_time_temperature();
      update_input_string();
      make_bitstream_from_string(input_string);
      calculate_pulse_time();
      // Configure_Alarm(60);
      reset_dac();
    }
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    if (USE_SPEAKER_TRANSMISSION) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    if (USE_CABLE_TRANSMISSION) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
      DWT_Delay_ms(100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }

    tx_active = false;
    HAL_TIM_Base_Stop(&htim2); // optional: stop TIM2 base to save a few µA

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, MID_12B);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_1) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
    rtc_woke = true;
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_TIM8_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  DWT_Delay_Init();

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity
   */
  BspCOMInit.BaudRate = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits = COM_STOPBITS_1;
  BspCOMInit.Parity = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //-------------------------------------------------------------------------------------------//
  // Set initial time and date in DS3231 RTC (uncomment to set)
  //-------------------------------------------------------------------------------------------//

  if (SET_INITIAL_TIME) {
    DS3231_PowerOn();
    if (HAL_I2C_IsDeviceReady(&hi2c2, DS3231_ADDR, 3, 100) == HAL_OK) {
      DS3231_Init();
      Set_Time(INITIAL_SEC, INITIAL_MIN, INITIAL_HOUR, INITIAL_DOW, INITIAL_DOM,
               INITIAL_MONTH, INITIAL_YEAR);
    }
    // DS3231_PowerOff();
  }

  DS3231_PowerOn();
  HAL_Delay(5);
  DS3231_ClearAllAlarms();
  // DS3231_PowerOff();

  //-------------------------------------------------------------------------------------------//
  // Get current time and temperature from DS3231 RTC
  //-------------------------------------------------------------------------------------------//

  update_time_temperature();

  //-------------------------------------------------------------------------------------------//
  // Load user configuration from user_config.h
  //-------------------------------------------------------------------------------------------//

  update_input_string();

  //-------------------------------------------------------------------------------------------//
  // Create a bistream from a string
  //-------------------------------------------------------------------------------------------//

  make_bitstream_from_string(input_string);

  //-------------------------------------------------------------------------------------------//
  // Calculate the total time it takes to send the bitstream
  //-------------------------------------------------------------------------------------------//

  calculate_pulse_time();

  //-------------------------------------------------------------------------------------------//
  // Generate the sine wave and mid-scale lookup tables
  //-------------------------------------------------------------------------------------------//

  get_sineval_low();
  get_sineval_high();
  get_dc_mid();

  //-------------------------------------------------------------------------------------------//
  // Filling the circular buffer for the DAC
  //-------------------------------------------------------------------------------------------//

  current_sine_period = 0;
  current_sine_index = 0;
  current_bitstream_index = 0;

  current_bit = bitstream[current_bitstream_index];
  next_bit = bitstream[(current_bitstream_index + 1) % BITSTREAM_LENGTH];

  result = fill_half_buffer((uint16_t *)&output_buffer[0], current_bit,
                            next_bit, BUFFER_SIZE / 2, current_sine_period,
                            current_sine_index);

  moved_to_next_bit = result.moved_to_next_bit;
  current_sine_period = result.current_period;
  current_sine_index = result.current_index;

  if (moved_to_next_bit) {
    moved_to_next_bit = false;
    current_bitstream_index = (current_bitstream_index + 1) % BITSTREAM_LENGTH;
    current_bit = bitstream[current_bitstream_index];
    next_bit = bitstream[(current_bitstream_index + 1) % BITSTREAM_LENGTH];
  }

  result = fill_half_buffer((uint16_t *)&output_buffer[BUFFER_SIZE / 2],
                            current_bit, next_bit, BUFFER_SIZE / 2,
                            current_sine_period, current_sine_index);

  moved_to_next_bit = result.moved_to_next_bit;
  current_sine_period = result.current_period;
  current_sine_index = result.current_index;

  if (moved_to_next_bit) {
    moved_to_next_bit = false;
    current_bitstream_index = (current_bitstream_index + 1) % BITSTREAM_LENGTH;
    current_bit = bitstream[current_bitstream_index];
    next_bit = bitstream[(current_bitstream_index + 1) % BITSTREAM_LENGTH];
  }

  //-------------------------------------------------------------------------------------------//
  // Start timers and interrupts
  //-------------------------------------------------------------------------------------------//

  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim2);

  Run_Transmission();

  //-------------------------------------------------------------------------------------------//
  // Set timer count to right before triggering the pulse
  //-------------------------------------------------------------------------------------------//

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Let transmission run; don't spin at full speed
    if (tx_active) {
      __WFI();
      continue;
    }

    // Stop stuff only when idle
    HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim2); // <-- IMPORTANT (not Stop_IT)

    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    HAL_ResumeTick();
    SystemClock_Config();

    if (rtc_woke) {
      rtc_woke = false;

      DS3231_ClearAllAlarms();
      // Configure_Alarm(60);

      // Restart PWM cleanly (avoid immediate pulse-finished)
      __HAL_TIM_SET_COUNTER(&htim8, 0);
      __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC1);
      __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
      HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);

      Run_Transmission(); // starts TIM2 base and DAC DMA via reset_dac()
    }

    /* USER CODE END 3 */
  }
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
   */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK) {
    Error_Handler();
  }

  /** DAC channel OUT1 config
   */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10707DBC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 31;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR5;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 31999;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 60129;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return
   * state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
