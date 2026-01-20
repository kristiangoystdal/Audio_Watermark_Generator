/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

#include "cmsis_gcc.h"
#include "ds3231.h"
#include <user_config.h>

#include "cmsis_gcc.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_gpio.h"
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

#define pi 3.14159265358979323846f

#define LEN_IF(cond, lit) ((cond) ? (sizeof(lit) - 1u) : 0u)

#define MAX_NUM_CHARS                                                          \
  (LEN_IF(INCLUDE_USER_STRING, "/STR" USER_STRING) +                           \
   LEN_IF(INCLUDE_DEVICE_ID, "/DID0000") +                                     \
   LEN_IF(INCLUDE_LOCATION, "/LOC00.0000,00.0000") +                           \
   LEN_IF(INCLUDE_TEMPERATURE, "/TMP000") +                                    \
   LEN_IF(INCLUDE_TIME, "/TIM00000000000000") + 4u)

#define BITSTREAM_LENGTH (MAX_NUM_CHARS * 8u)

#define BIT_POLARITY 0

#define DS3231_ADDR (0x68 << 1) // 7-bit addr shifted for HAL
#define T4B_I2C_ADDR_7BIT 0x20
#define T4B_I2C_ADDR_HAL (T4B_I2C_ADDR_7BIT << 1) // 0x40 for HAL

#define DIV_FLOOR(n, d) ((uint32_t)((n) / (d)))
#define DIV_ROUND(n, d) ((uint32_t)(((n) + ((d) / 2u)) / (d)))

#define FS_HZ 95952

#define MIN_BIT_US 3000u
#define MIN_BIT_SAMPLES ((FS_HZ * MIN_BIT_US) / 1000000u)
#define MAX_BIT_DURATION_SAMPLES MIN_BIT_SAMPLES

#define FSK_LOWER_NUM_SAMPLES DIV_FLOOR(FS_HZ, FSK_LOWER_FREQUENCY)
#define FSK_HIGHER_NUM_SAMPLES DIV_FLOOR(FS_HZ, FSK_HIGHER_FREQUENCY)

#define FSK_LOWER_PERIODS DIV_ROUND(MIN_BIT_SAMPLES, FSK_LOWER_NUM_SAMPLES)
#define FSK_HIGHER_PERIODS DIV_ROUND(MIN_BIT_SAMPLES, FSK_HIGHER_NUM_SAMPLES)

#define OUTPUT_SEGMENT 1000
#define BUFFER_SIZE (OUTPUT_SEGMENT * 5)

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
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

char input_string[MAX_NUM_CHARS] = {0};

uint8_t bitstream[BITSTREAM_LENGTH] = {0};

volatile uint8_t active_done = 0;

rtc_time_t now = {0};

int temp_int = 25;

uint32_t sine_val_low[FSK_LOWER_NUM_SAMPLES];
uint32_t sine_val_high[FSK_HIGHER_NUM_SAMPLES];
uint32_t dc_mid[FSK_LOWER_NUM_SAMPLES]; // constant mid-level DC value

// Pulse Calculation Variables
uint32_t fs = 0;
uint32_t cable_speaker_delay_ms = 0;
float f_0_bit_duration = 0.0f;
float f_1_bit_duration = 0.0f;
float total_time = 0.0;

// FSK
typedef struct {
  size_t bit_index;
  size_t current_period;
  size_t current_index;
} FillResult;

__ALIGN_BEGIN uint32_t output_buffer[BUFFER_SIZE] __ALIGN_END;

volatile bool tx_active = true;
volatile size_t current_bitstream_index = 0;
volatile uint8_t current_bit = 0;
volatile uint8_t next_bit = 0;
volatile size_t current_sine_period = 0;
volatile size_t current_sine_index = 0;
volatile bool moved_to_next_bit = false;
FillResult result;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void EnterStopMode(void);
void calculate_active_duration_ms(void);
void StartActiveWindowMs(uint32_t ms);
void make_bitstream_from_string(const char *str);
void update_input_string(void);
int make_preamble(int start_idx);
void get_sineval_low(void);
void get_sineval_high(void);
void get_dc_mid(void);
uint32_t get_dac_sample_rate_hz(void);
void Set_DAC_Output_To_Midlevel(void);

static void TX_Start(void);
static void TX_Stop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM6_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  //-----------------------------------------------------------------------------//
  // Prepare sine wave lookup tables
  //-----------------------------------------------------------------------------//

  get_sineval_low();
  get_sineval_high();
  get_dc_mid();

  //-----------------------------------------------------------------------------//
  // Prepare input string
  //-----------------------------------------------------------------------------//

  update_input_string();

  //-----------------------------------------------------------------------------//
  // Prepare bitstream from input string
  //-----------------------------------------------------------------------------//

  make_bitstream_from_string(input_string);

  //-----------------------------------------------------------------------------//
  // Calculate active duration
  //-----------------------------------------------------------------------------//

  calculate_active_duration_ms();

  //-----------------------------------------------------------------------------//
  // Start DAC DMA
  //-----------------------------------------------------------------------------//
  current_bitstream_index = 0;
  current_bit = bitstream[current_bitstream_index];
  next_bit = bitstream[(current_bitstream_index + 1) % BITSTREAM_LENGTH];
  current_sine_period = 0;
  current_sine_index = 0;
  tx_active = true;

  //---------------------------------------------------------------------------//
  // Main loop
  //---------------------------------------------------------------------------//

  /* USER CODE END 2 */

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

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to send message over COM1 port ---- */
  printf("\033[2J\033[H");
  printf("-------------------------\r\n");
  printf("Hello from VCP!\r\n");

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  printf("-------------------------\r\n");
  printf("Input string prepared:\r\n%s\r\n", input_string);
  printf("Bitstream length: %u bits\r\n", BITSTREAM_LENGTH);
  uint32_t ms = (uint32_t)(total_time * 1000.0f);
  printf("Active duration: %lu ms\r\n", (unsigned long)ms);
  printf("Bitstream:\r\n");
  for (int i = 0; i < BITSTREAM_LENGTH; ++i) {
    printf("%u", bitstream[i]);
  }
  printf("\r\n");
  printf("-------------------------\r\n");

  TX_Stop();

  Set_DAC_Output_To_Midlevel();

  DS3231_PowerOn();
  if (HAL_I2C_IsDeviceReady(&hi2c2, DS3231_ADDR, 3, 100) == HAL_OK) {
    DS3231_Init();
    Set_Time(INITIAL_SEC, INITIAL_MIN, INITIAL_HOUR, INITIAL_DOW, INITIAL_DOM,
             INITIAL_MONTH, INITIAL_YEAR);
  }
  Get_Time(&now);
  printf("-------------------------\r\n");

  while (1) {

    // 1) Start TX
    TX_Start();

    // 2) Calculate active duration
    calculate_active_duration_ms();
    printf("Starting transmission for %.2f seconds...\r\n", total_time);

    // 3) Set an active window timer and wait for completion
    StartActiveWindowMs((uint32_t)(total_time * 1000.0f));
    while (!active_done) {
      __WFI(); // CPU sleeps while DMA+TIM2+DAC run
    }
    active_done = 0;

    // 4) Stop TX
    TX_Stop();

    printf("\r\n");
    printf("Transmission complete.\r\n");
    printf("-------------------------\r\n");

    HAL_Delay(100);

    // 5) Enter STOP mode until button press
    EnterStopMode();

    printf("Woke up!\r\n");

    // 6) Debounce button
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hi2c2.Init.Timing = 0x10B17DB5;
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
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 666;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
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
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim6, TIM_OPMODE_SINGLE) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA5 PA6 PA7
                           PA10 PA11 PA12 PA13
                           PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                        GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                        GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6
                           PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                        GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void EnterStopMode(void) {
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  HAL_SuspendTick();

  // Optional small savings:
  __HAL_RCC_GPIOF_CLK_DISABLE();
  __HAL_RCC_GPIOG_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();

  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

  // After wake
  SystemClock_Config();

  // Re-enable clocks if you disabled them
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  HAL_ResumeTick();
}

void StartActiveWindowMs(uint32_t ms) {
  active_done = 0;

  HAL_TIM_Base_Stop_IT(&htim6); // <- important, resets state
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim6, ms - 1);

  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

  HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    HAL_TIM_Base_Stop_IT(&htim6); // <- makes it one-shot reliably
    active_done = 1;
  }
}

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
        (uint32_t)((4095.0 / 2.0) *
                   (1.0 +
                    sinf(2.0 * pi * i / FSK_LOWER_NUM_SAMPLES) * (1.5 / 1.65)));
  }
}

// Function to generate sine wave lookup table for the high frequency
void get_sineval_high(void) {
  for (int i = 0; i < FSK_HIGHER_NUM_SAMPLES; i++) {
    sine_val_high[i] =
        (uint32_t)((4095.0 / 2.0) *
                   (1.0 + sinf(2.0 * pi * i / FSK_HIGHER_NUM_SAMPLES) *
                              (1.5 / 1.65)));
  }
}

void get_dc_mid(void) {
  uint32_t mid_value = (uint32_t)(4095.0 / 2.0);
  for (int i = 0; i < FSK_LOWER_NUM_SAMPLES; i++) {
    dc_mid[i] = mid_value;
  }
}

uint32_t get_dac_sample_rate_hz(void) {
  uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    tim_clk *= 2; // timer clock doubled if APB prescaler != 1

  return tim_clk / ((htim2.Init.Prescaler + 1) * (htim2.Init.Period + 1));
}

void calculate_active_duration_ms(void) {
  fs = get_dac_sample_rate_hz();

  float f_0 = (float)fs / (float)FSK_LOWER_NUM_SAMPLES;
  float f_1 = (float)fs / (float)FSK_HIGHER_NUM_SAMPLES;

  f_0_bit_duration = (float)FSK_LOWER_PERIODS / f_0;
  f_1_bit_duration = (float)FSK_HIGHER_PERIODS / f_1;

  // Exact bitstream time
  total_time = 0.0f;
  for (int i = 0; i < BITSTREAM_LENGTH; ++i) {
    if (bitstream[i] == 0) {
      total_time += ((float)FSK_LOWER_PERIODS) / f_0;
    } else if (bitstream[i] == 1) {
      total_time += (float)FSK_HIGHER_PERIODS / f_1;
    } else {
      total_time += ((float)FSK_LOWER_PERIODS) / f_0; // silence
    }
  }

  // Add silence time
  // total_time *= 1.05f;                            // 5% margin
  // total_time += cable_speaker_delay_ms / 1000.0f; // 10ms margin
}

static inline FillResult
fill_half_buffer(uint32_t *buffer, size_t generation_size,
                 const uint8_t *bitstream, size_t bitstream_len,
                 size_t bit_index, size_t current_sine_period,
                 size_t current_sine_index) {
  size_t out = 0;
  size_t period = current_sine_period;
  size_t idx = current_sine_index;

  while (out < generation_size) {
    // If finished, just output mid forever (prevents repeats)
    if (bit_index >= bitstream_len) {
      for (; out < generation_size; ++out) {
        buffer[out] = dc_mid[0];
      }
      break;
    }

    uint8_t cur_bit = bitstream[bit_index];

    /* Select LUT + target periods for THIS bit */
    const uint32_t *lut;
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
    } else { // silence
      lut = dc_mid;
      lut_len = FSK_LOWER_NUM_SAMPLES;
      target_periods = FSK_LOWER_PERIODS;
    }

    /* Copy as much as we can from current LUT position */
    size_t left_buf = generation_size - out;
    size_t left_lut = lut_len - idx;
    size_t n = (left_buf < left_lut) ? left_buf : left_lut;

    memcpy(&buffer[out], &lut[idx], n * sizeof(uint32_t));
    out += n;
    idx += n;

    /* Completed one period? */
    if (idx == lut_len) {
      idx = 0;
      period++;

      /* Completed this bit? -> advance to next bit */
      if (period == target_periods) {
        period = 0;
        if (bit_index + 1u < bitstream_len) {
          bit_index++;
        } else {
          // reached end -> hold at end
          bit_index = bitstream_len;
        }
      }
    }
  }

  return (FillResult){
      .bit_index = bit_index, .current_period = period, .current_index = idx};
}

// DMA Half Transfer Complete callback
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  if (!tx_active)
    return;

  FillResult r =
      fill_half_buffer((uint32_t *)&output_buffer[0], BUFFER_SIZE / 2,
                       bitstream, BITSTREAM_LENGTH, current_bitstream_index,
                       current_sine_period, current_sine_index);

  current_bitstream_index = r.bit_index;
  current_sine_period = r.current_period;
  current_sine_index = r.current_index;
}

// DMA Transfer Complete callback
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  if (!tx_active)
    return;

  FillResult r = fill_half_buffer((uint32_t *)&output_buffer[BUFFER_SIZE / 2],
                                  BUFFER_SIZE / 2, bitstream, BITSTREAM_LENGTH,
                                  current_bitstream_index, current_sine_period,
                                  current_sine_index);

  current_bitstream_index = r.bit_index;
  current_sine_period = r.current_period;
  current_sine_index = r.current_index;
}

static void TX_Start(void) {
  printf("\r\nTX_Start()\r\n");

  current_bitstream_index = 0;
  current_sine_period = 0;
  current_sine_index = 0;
  tx_active = true;

  FillResult r1 =
      fill_half_buffer((uint32_t *)&output_buffer[0], BUFFER_SIZE / 2,
                       bitstream, BITSTREAM_LENGTH, current_bitstream_index,
                       current_sine_period, current_sine_index);

  current_bitstream_index = r1.bit_index;
  current_sine_period = r1.current_period;
  current_sine_index = r1.current_index;

  FillResult r2 = fill_half_buffer((uint32_t *)&output_buffer[BUFFER_SIZE / 2],
                                   BUFFER_SIZE / 2, bitstream, BITSTREAM_LENGTH,
                                   current_bitstream_index, current_sine_period,
                                   current_sine_index);

  current_bitstream_index = r2.bit_index;
  current_sine_period = r2.current_period;
  current_sine_index = r2.current_index;

  printf("  Buffer filled\r\n");

  // Clean start
  HAL_TIM_Base_Stop(&htim2);
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);

  DMA1->IFCR =
      DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
  NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

  printf("  Peripherals stopped & DMA flags cleared\r\n");

  // Start DAC
  HAL_StatusTypeDef st;
  st = HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  if (st != HAL_OK) {
    printf("  HAL_DAC_Start error: %d\r\n", st);
  } else {
    printf("  HAL_DAC_Start OK\r\n");
  }

  // Start DMA
  st = HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)output_buffer,
                         BUFFER_SIZE, DAC_ALIGN_12B_R);

  if (st != HAL_OK) {
    printf("  HAL_DAC_Start_DMA error: %d\r\n", st);
  } else {
    printf("  HAL_DAC_Start_DMA OK\r\n");
  }

  // Ensure trigger enabled
  SET_BIT(DAC1->CR, DAC_CR_TEN1);

  // Start timer last
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  st = HAL_TIM_Base_Start(&htim2);

  if (st != HAL_OK) {
    printf("  HAL_TIM_Base_Start error: %d\r\n", st);
  } else {
    printf("  HAL_TIM_Base_Start OK\r\n");
  }
}

static void TX_Stop(void) {
  printf("TX_Stop()\r\n");

  tx_active = false;

  // Stop trigger + DMA first
  HAL_TIM_Base_Stop(&htim2);
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  Set_DAC_Output_To_Midlevel();

  // Optional: if you want to *keep* the mid DC during STOP, leave DAC
  // running. If you want lowest power, stop it (output may go undefined
  // depending on buffer mode): HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);

  DMA1->IFCR =
      DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
  NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

  printf("  TX stopped cleanly (DAC set to mid)\r\n");
}

void Set_DAC_Output_To_Midlevel(void) {
  CLEAR_BIT(DAC1->CR, DAC_CR_TEN1);

  // Ensure DAC channel is enabled
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

  // Set mid-scale (12-bit right aligned)
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dc_mid[0]);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
  /* User can add his own implementation to report the HAL error return state */
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
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
