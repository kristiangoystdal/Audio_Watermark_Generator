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

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <user_config.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define pi 3.14159265358979323846

#define NUM_CHARS (48 + 4) // 48 chars + 4 start/end identifiers
#define BITSTREAM_LENGTH (NUM_CHARS * 8)

#define MID_12B 2048

#define BUF_LEN 720

#define MAX_SAMPLES_21k 48
#define MAX_SAMPLES_22k 45
#define MAX_SAMPLES_MID_VAL 20

#define REPEAT_HALF 4
#define PER_HALF_21K 15
#define PER_HALF_22K 16
#define PER_HALF_SIL 18
#define PERIODS_PER_BIT_21K (PER_HALF_21K * REPEAT_HALF)
#define PERIODS_PER_BIT_22K (PER_HALF_22K * REPEAT_HALF)
#define PERIODS_PER_BIT_SIL (PER_HALF_SIL * REPEAT_HALF)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

// Circular buffer for DAC output
__ALIGN_BEGIN __IO uint16_t output_buffer[2 * BUF_LEN] __ALIGN_END;

uint16_t sine_val_21k[MAX_SAMPLES_21k];
uint16_t sine_val_22k[MAX_SAMPLES_22k];
uint16_t dc_mid[MAX_SAMPLES_MID_VAL];

uint16_t bitstream[BITSTREAM_LENGTH];

static volatile uint32_t current_period = 0;
static volatile uint32_t current_bit = 2;
static volatile uint32_t current_idx = 0;

float total_time = 0.0;
uint32_t pulse_time = 1000; // in ms, will be updated later

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void make_bitstream_from_string(const char *str) {
  int k = 0;

  // Start identifier (10101010)
  for (int i = 0; i < 8 && k < BITSTREAM_LENGTH; ++i) {
    bitstream[k++] = 1;
    bitstream[k++] = 0;
  }

  // Data bits from string
  for (int i = 0; str[i] != '\0' && k < BITSTREAM_LENGTH; ++i) {
    for (int b = 7; b >= 0 && k < BITSTREAM_LENGTH; b--) {
      bitstream[k++] = (str[i] >> b) & 1;
    }
  }

  // End identifier (10101010)
  for (int i = 0; i < 8 && k < BITSTREAM_LENGTH; ++i) {
    bitstream[k++] = 1;
    bitstream[k++] = 0;
  }

  // Fill the rest with mid-scale values if needed
  while (k < BITSTREAM_LENGTH)
    bitstream[k++] = 2;
}

// Function to generate sine wave lookup table for 21kHz
void get_sineval_21k(void) {
  for (int i = 0; i < MAX_SAMPLES_21k; i++) {
    sine_val_21k[i] = (uint16_t)((4095.0 / 2.0) *
                                 (1.0 + sinf(2.0 * pi * i / MAX_SAMPLES_21k)));
  }
}

// Function to generate sine wave lookup table for 22kHz
void get_sineval_22k(void) {
  for (int i = 0; i < MAX_SAMPLES_22k; i++) {
    sine_val_22k[i] = (uint16_t)((4095.0 / 2.0) *
                                 (1.0 + sinf(2.0 * pi * i / MAX_SAMPLES_22k)));
  }
}

// Function to generate mid-scale lookup table
void get_dc_mid(void) {
  for (int i = 0; i < MAX_SAMPLES_MID_VAL; ++i)
    dc_mid[i] = MID_12B;
}

// Function to calculate the total time it takes to send the bitstream
void calculate_pulse_time(void) {
  float f21k = 21000.0;
  float f22k = 22000.0;
  float f25k = 25000.0;

  // Bitstream transmission time
  total_time = 0.0;
  for (int i = 0; i < BITSTREAM_LENGTH; ++i) {
    if (bitstream[i] == 0) {
      total_time += (float)PERIODS_PER_BIT_21K / f21k;
    } else if (bitstream[i] == 1) {
      total_time += (float)PERIODS_PER_BIT_22K / f22k;
    }
  }

  // Add time for silence after bitstream
  total_time += 2 * (float)PERIODS_PER_BIT_SIL / f25k;

  // Set the pulse time for TIM8
  pulse_time = (uint32_t)(total_time * 10000.0); // in ms
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse_time);
}

// Function to fill a buffer with repeated patterns from a lookup table
static inline void fill_lut_repeated(uint16_t *dst, size_t buffer_len,
                                     const uint16_t *lut, size_t lut_len) {
  size_t k = 0;
  while (k < buffer_len) {
    size_t to_copy = (buffer_len - k < lut_len) ? (buffer_len - k) : lut_len;
    memcpy(&dst[k], lut, to_copy * sizeof(uint16_t));
    k += to_copy;
  }
}

// Function to fill half of the output buffer based on the current bit
static inline void fill_half(uint16_t *dst, uint32_t bit) {
  if (bit == 2) {
    for (size_t i = 0; i < BUF_LEN; ++i)
      dst[i] = MID_12B;
  } else if (bit == 1) {
    fill_lut_repeated(dst, BUF_LEN, sine_val_22k, MAX_SAMPLES_22k); // 16×
  } else {
    fill_lut_repeated(dst, BUF_LEN, sine_val_21k, MAX_SAMPLES_21k); // 15×
  }
}

// DAC conversion complete callbacks for half buffer
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  // Update current period
  uint32_t per = (current_bit == 0)   ? PER_HALF_21K
                 : (current_bit == 1) ? PER_HALF_22K
                                      : PER_HALF_SIL;

  current_period += per;

  // Check if we need to move to the next bit
  while ((current_bit == 0 && current_period >= PERIODS_PER_BIT_21K) ||
         (current_bit == 1 && current_period >= PERIODS_PER_BIT_22K) ||
         (current_bit == 2 && current_period >= PERIODS_PER_BIT_SIL)) {
    current_period = 0;
    current_idx = (current_idx + 1) % BITSTREAM_LENGTH;
    current_bit = bitstream[current_idx];
  }

  // Fill the first half of the buffer based on the current bit
  fill_half((uint16_t *)&output_buffer[0], current_bit);
}

// DAC conversion complete callback for the full buffer
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  // Update current period
  uint32_t per = (current_bit == 0)   ? PER_HALF_21K
                 : (current_bit == 1) ? PER_HALF_22K
                                      : PER_HALF_SIL;

  current_period += per;

  // Check if we need to move to the next bit
  while ((current_bit == 0 && current_period >= PERIODS_PER_BIT_21K) ||
         (current_bit == 1 && current_period >= PERIODS_PER_BIT_22K) ||
         (current_bit == 2 && current_period >= PERIODS_PER_BIT_SIL)) {
    current_period = 0;
    current_idx = (current_idx + 1) % BITSTREAM_LENGTH;
    current_bit = bitstream[current_idx];
  }

  // Fill the first half of the buffer based on the current bit
  fill_half((uint16_t *)&output_buffer[BUF_LEN], current_bit);
}

// Timer interrupt callback for TIM8
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM8) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    // Stop DAC DMA
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

    // Restart the bitstream from the beginning
    current_idx = 0;
    current_period = 0;
    current_bit = bitstream[0];

    // Prefill both halves of the circular buffer with the first bit
    fill_half((uint16_t *)&output_buffer[0], current_bit);
    fill_half((uint16_t *)&output_buffer[BUF_LEN], current_bit);

    // Restart DAC with the circular buffer
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)output_buffer,
                      2 * BUF_LEN, DAC_ALIGN_12B_R);
  }
}

// Timer PWM pulse finished callback for TIM8 Channel 1
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
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
  /* USER CODE BEGIN 2 */

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
  // Load user configuration from user_config.h
  //-------------------------------------------------------------------------------------------//

  const char *input_string = USER_STRING;
  int interval_between_repeats = INTERVAL_BETWEEN_REPEATS_SECONDS;

  //-------------------------------------------------------------------------------------------//
  // Create a bistream from a string
  //-------------------------------------------------------------------------------------------//

  make_bitstream_from_string(input_string);

  //-------------------------------------------------------------------------------------------//
  // Change the interval between repeats
  //-------------------------------------------------------------------------------------------//

  // __HAL_TIM_SET_AUTORELOAD(&htim8, (interval_between_repeats * 10000) - 1);

  //-------------------------------------------------------------------------------------------//
  // Calculate the total time it takes to send the bitstream
  //-------------------------------------------------------------------------------------------//

  calculate_pulse_time();

  //-------------------------------------------------------------------------------------------//
  // Generate the sine wave and mid-scale lookup tables
  //-------------------------------------------------------------------------------------------//

  get_sineval_21k();
  get_sineval_22k();
  get_dc_mid();

  //-------------------------------------------------------------------------------------------//
  // Filling the circular buffer for the DAC
  //-------------------------------------------------------------------------------------------//

  // initialize state variables
  current_idx = 0;
  current_period = 0;
  current_bit = bitstream[current_idx];

  // prefill both halves of the circular buffer with the first bit
  fill_half((uint16_t *)&output_buffer[0], current_bit);
  fill_half((uint16_t *)&output_buffer[BUF_LEN], current_bit);

  //-------------------------------------------------------------------------------------------//
  // Start DAC with the circular buffer
  //-------------------------------------------------------------------------------------------//

  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)output_buffer,
                    2 * BUF_LEN, DAC_ALIGN_12B_R);

  //-------------------------------------------------------------------------------------------//
  // Start timers and interrupts
  //-------------------------------------------------------------------------------------------//

  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim2);

  //-------------------------------------------------------------------------------------------//

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Enter Sleep Mode, wake up is done by interrupts
    __WFI();
  }

  /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
  htim2.Init.Period = 79;
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
  htim8.Init.Prescaler = 7999;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9999;
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
  sConfigOC.Pulse = 7000;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
