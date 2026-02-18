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

#include "error_codes.h"
#include <cc1101.h>
#include <ds3231.h>
#include <frequency_config.h>
#include <ism.h>
#include <ism_config_433.h>
#include <led_feedback.h>
#include <radio.h>
#include <spi.h>
#include <user_config.h>

#include "cmsis_gcc.h"
#include "stdlib.h"
#include "stm32g431xx.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include "sys/_intsup.h"
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

char input_string[MAX_NUM_CHARS] = {0};

uint8_t bitstream[BITSTREAM_LENGTH] = {0};

volatile uint8_t active_done = 0;

rtc_time_t now = {0};

int temp_int = 25;

static uint32_t *sine_val_low = NULL;
static uint32_t *sine_val_high = NULL;
static uint32_t *dc_mid = NULL;

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

bool RX = true; // set to false for TX mode, true for RX mode

uint8_t transmission[256] = {0};
int dBm_value = 0;

freq_pair_t freq_pair = {0};

volatile status_code_t g_error_code = STATUS_CODE_OK;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

void EnterStopMode(void);
void calculate_active_duration_ms(size_t bitstream_len);
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

void app_radio_test(void);

void read_buffer(void);

void process_transmission(const uint8_t *transmission, int *dBm_value);

void create_string_from_received_data(const uint8_t *transmission,
                                      int dBm_value, char *output_str,
                                      size_t output_str_size);

static int init_luts_from_freqpair(void);

void Error_Handler_Code(status_code_t code);

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
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

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

  // Turn on LED to indicate power on
  LED_BlinkStatusCode(STATUS_CODE_OK); // code 0 = "0000" = 4 short blinks

  //-----------------------------------------------------------------------------//
  // Find and set FSK frequencies based on user config and sample count
  // requirements
  //-----------------------------------------------------------------------------//

  printf("--------------------------\r\n");

  freq_pair = find_frequency_pair();

  printf("Frequency pair: lower=%u Hz, higher=%u Hz\r\n",
         (unsigned int)freq_pair.lower_freq,
         (unsigned int)freq_pair.higher_freq);

  //-----------------------------------------------------------------------------//
  // Prepare sine wave lookup tables
  //-----------------------------------------------------------------------------//

  if (init_luts_from_freqpair() != 0) {
    printf("LUT alloc failed\r\n");
    Error_Handler_Code(STATUS_CODE_LUT_ALLOC_FAIL);
  }

  get_sineval_low();
  get_sineval_high();
  get_dc_mid();

  //-----------------------------------------------------------------------------//
  // Set DAC output to mid-level (silence) before starting
  //-----------------------------------------------------------------------------//

  Set_DAC_Output_To_Midlevel();

  //-----------------------------------------------------------------------------//
  // Initialize RTC
  //-----------------------------------------------------------------------------//

  // DS3231_PowerOn();
  // if (HAL_I2C_IsDeviceReady(&hi2c2, DS3231_ADDR, 3, 100) == HAL_OK) {
  //   DS3231_Init();
  //   Set_Time(INITIAL_SEC, INITIAL_MIN, INITIAL_HOUR, INITIAL_DOW,
  //   INITIAL_DOM,
  //            INITIAL_MONTH, INITIAL_YEAR);
  // }
  // Get_Time(&now);
  // printf("-------------------------\r\n");

  printf("-------------------------\r\n");

  int8_t init_result = init_radio(RX);
  if (init_result != 0) {
    printf("Failed to initialize radio in RX mode, error code: %d\r\n",
           init_result);
    Error_Handler_Code(init_result);
  }

  printf("--------------------------\r\n");

  printf("Entering main loop...\r\n");

  while (1) {

    // 1) Enter STOP mode and wait for wakeup from EXTI (GPIOB Pin 7)
    printf("Entering STOP mode...\r\n");
    EnterStopMode();

    // 2) Debounce button
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);

    printf("Woke up!\r\n");

    // 3) Check state and either start RX or TX
    if (RX) {
      // RX mode: read from radio and store in string
      read_RX(transmission, sizeof(transmission));
      printf("Received transmission: ");
      for (size_t i = 0; i < sizeof(transmission); i++) {
        printf("%02X ", transmission[i]);
      }
      printf("\r\n");

      // Process received transmission (e.g. parse bytes, convert RSSI to dBm,
      // etc.)
      process_transmission(transmission, &dBm_value);

      printf("Processed transmission into %s\r\n", transmission);
      printf("dBm value: %d\r\n", dBm_value);

      // Create output string based on received data and dBm value (e.g.
      // "/STR.../DID.../LOC.../TMP.../TIM...")
      char output_str[256] = {0};
      create_string_from_received_data(transmission, dBm_value, output_str,
                                       sizeof(output_str));

      printf("Final output string: %s\r\n", output_str);

      // Make bitstream from output string
      make_bitstream_from_string(output_str);
      printf("Prepared bitstream from output string.\r\n");
      printf("Output string length: %d characters\r\n",
             (int)strlen(output_str));

      // Calculate active duration based on bitstream length and bit durations
      size_t bitstream_len = strlen(output_str) * 8u;
      calculate_active_duration_ms(bitstream_len);
      uint32_t total_ms = (uint32_t)(total_time * 1000.0f);
      printf("Calculated active duration for response: %lu ms\r\n",
             (unsigned long)total_ms);

      // Send transmission over audio
      printf("Starting transmission of response over audio...\r\n");
      LED_BlinkStatusCode(STATUS_CODE_STARTING_TRANSMISSION);
      TX_Start();

      // Wait for active window to complete (enters low-power sleep while
      // waiting)
      StartActiveWindowMs((uint32_t)(total_time * 1000.0f));
      while (!active_done) {
        __WFI(); // CPU sleeps while DMA+TIM2+DAC run
      }
      active_done = 0;

      // Stop transmission and go back to sleep
      TX_Stop();

    } else {
      // Send transmission over radio
      printf("Starting transmission over radio...\r\n");
      LED_BlinkStatusCode(STATUS_CODE_STARTING_TRANSMISSION);
      start_TX();
    }

    printf("Going back to sleep...\r\n");

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
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
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
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_6, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PA0 PA1 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void EnterStopMode(void) {

  // Clear EXTI pending + NVIC pending for PB7 (wakeup source)
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

  // Clear PWR wake flags
  PWR->SCR = PWR_SCR_CWUF;

  // Optional: disable debug in STOP (important on ST-LINK boards)
  DBGMCU->CR &=
      ~(DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STANDBY);

  HAL_SuspendTick();

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  PWR->SCR = PWR_SCR_CWUF;
  __DSB();
  __ISB();

  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

  // We are awake here, but clocks are still not restored:
  SystemClock_Config();
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
  if (!sine_val_low || freq_pair.lower_freq_samples == 0) {
    return; // eller Error_Handler()
  }

  for (uint16_t i = 0; i < freq_pair.lower_freq_samples; i++) {
    sine_val_low[i] =
        (uint32_t)((4095.0 / 2.0) *
                   (1.0 + sinf(2.0 * pi * i / freq_pair.lower_freq_samples) *
                              (1.5 / 1.65)));
  }
}

// Function to generate sine wave lookup table for the high frequency
void get_sineval_high(void) {
  if (!sine_val_high || freq_pair.higher_freq_samples == 0) {
    return; // eller Error_Handler()
  }

  for (uint16_t i = 0; i < freq_pair.higher_freq_samples; i++) {
    sine_val_high[i] =
        (uint32_t)((4095.0 / 2.0) *
                   (1.0 + sinf(2.0 * pi * i / freq_pair.higher_freq_samples) *
                              (1.5 / 1.65)));
  }
}

void get_dc_mid(void) {
  if (!dc_mid || freq_pair.lower_freq_samples == 0) {
    return; // eller Error_Handler()
  }

  uint32_t mid_value = (uint32_t)(4095.0 / 2.0);
  for (uint16_t i = 0; i < freq_pair.lower_freq_samples; i++) {
    dc_mid[i] = mid_value;
  }
}

uint32_t get_dac_sample_rate_hz(void) {
  uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    tim_clk *= 2; // timer clock doubled if APB prescaler != 1

  return tim_clk / ((htim2.Init.Prescaler + 1) * (htim2.Init.Period + 1));
}

void calculate_active_duration_ms(size_t bitstream_len) {
  fs = get_dac_sample_rate_hz();

  float f_0 = (float)fs / (float)freq_pair.lower_freq_samples;
  float f_1 = (float)fs / (float)freq_pair.higher_freq_samples;

  f_0_bit_duration = (float)freq_pair.lower_freq_periods / f_0;
  f_1_bit_duration = (float)freq_pair.higher_freq_periods / f_1;

  // Exact bitstream time
  total_time = 0.0f;
  for (int i = 0; i < bitstream_len; ++i) {
    if (bitstream[i] == 0) {
      total_time += ((float)freq_pair.lower_freq_periods) / f_0;
    } else if (bitstream[i] == 1) {
      total_time += (float)freq_pair.higher_freq_periods / f_1;
    } else {
      total_time += ((float)freq_pair.lower_freq_periods) / f_0; // silence
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
      lut_len = freq_pair.lower_freq_samples;
      target_periods = freq_pair.lower_freq_periods;
    } else if (cur_bit == 1) {
      lut = sine_val_high;
      lut_len = freq_pair.higher_freq_samples;
      target_periods = freq_pair.higher_freq_periods;
    } else { // silence
      lut = dc_mid;
      lut_len = freq_pair.lower_freq_samples;
      target_periods = freq_pair.lower_freq_periods;
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
    Error_Handler_Code(STATUS_CODE_TRANSMISSION_ERROR);
  } else {
    printf("  HAL_DAC_Start OK\r\n");
  }

  // Start DMA
  st = HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)output_buffer,
                         BUFFER_SIZE, DAC_ALIGN_12B_R);

  if (st != HAL_OK) {
    printf("  HAL_DAC_Start_DMA error: %d\r\n", st);
    Error_Handler_Code(STATUS_CODE_TRANSMISSION_ERROR);
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
    Error_Handler_Code(STATUS_CODE_TRANSMISSION_ERROR);
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

void string_to_hex(const char *str, uint8_t *hex_buf, size_t hex_buf_size) {
  size_t str_len = strlen(str);
  size_t max_bytes =
      (hex_buf_size < (str_len / 2)) ? hex_buf_size : (str_len / 2);

  for (size_t i = 0; i < max_bytes; i++) {
    sscanf(&str[i * 2], "%2hhx", &hex_buf[i]);
  }
}

void process_transmission(const uint8_t *transmission, int *dBm_value) {
  size_t len = strlen((const char *)transmission);
  if (len <= 4) {
    printf("Transmission too short to process.\r\n");
    return;
  }

  for (size_t i = 0; i < len; i++) {
    printf("%02X ", transmission[i]);
  }
  printf("\r\n");

  uint8_t rssi_hex = transmission[len - 2]; // 2nd from end

  // printf("Extracted RSSI hex: %02X\r\n", rssi_hex);

  int8_t rssi_signed = (int8_t)rssi_hex; // 2's complement

  // printf("RSSI signed value: %d\r\n", rssi_signed);

  int rssi_dbm = (rssi_signed / 2) - 74; // integer dBm

  // printf("RSSI: %d dBm\r\n", rssi_dbm);

  dBm_value[0] = (int)rssi_dbm;

  // Create cleaned string: skip first 2 char, skip last 2 chars
  size_t cleaned_len = (len > 4) ? (len - 4) : 0;
  char cleaned_str[MAX_NUM_CHARS] = {0};
  if (cleaned_len > 0) {
    strncpy(cleaned_str, (const char *)transmission + 2, cleaned_len);
    cleaned_str[cleaned_len] = '\0'; // Null-terminate
  }

  // Overwrite the transmission parameter (cast needed for const)
  strncpy((char *)transmission, cleaned_str, MAX_NUM_CHARS - 1);
}

void create_string_from_received_data(const uint8_t *transmission,
                                      int dBm_value, char *output_str,
                                      size_t output_str_size) {
  if (!output_str || output_str_size == 0)
    return;

  // Clear the entire output string buffer to ensure no leftover data, and set
  // first char to null for safe concatenation
  memset(output_str, 0, output_str_size);
  output_str[0] = '\0';

  // Initialize with default values
  char user_string[64] = USER_STRING; // fallback from user_config.h
  int device_id = DEVICE_ID;          // fallback from user_config.h
  char location[64] = LOCATION;       // fallback from user_config.h
  int temperature = temp_int;         // runtime value
  rtc_time_t time_val = now;          // runtime value

  // Copy transmission to local buffer for tokenization
  char buf[256];
  size_t in_len = strnlen((const char *)transmission, sizeof(buf) - 1);
  memcpy(buf, transmission, in_len);
  buf[in_len] = '\0';

  // Parse tokens in the format /KEYvalue, where KEY is 3 chars and value is
  // variable length, separated by '/'
  for (char *tok = strtok(buf, "/"); tok != NULL; tok = strtok(NULL, "/")) {

    if (strncmp(tok, "STR", 3) == 0) {
      strncpy(user_string, tok + 3, sizeof(user_string) - 1);
      user_string[sizeof(user_string) - 1] = '\0';

    } else if (strncmp(tok, "DID", 3) == 0) {
      device_id = atoi(tok + 3);

    } else if (strncmp(tok, "LOC", 3) == 0) {
      strncpy(location, tok + 3, sizeof(location) - 1);
      location[sizeof(location) - 1] = '\0';

    } else if (strncmp(tok, "TMP", 3) == 0) {
      temperature = atoi(tok + 3);

    } else if (strncmp(tok, "TIM", 3) == 0) {
      const char *p = tok + 3;
      if (strlen(p) >= 14) {
        char t[3] = {0};

        t[0] = p[0];
        t[1] = p[1];
        time_val.hours = (uint8_t)atoi(t);
        t[0] = p[2];
        t[1] = p[3];
        time_val.minutes = (uint8_t)atoi(t);
        t[0] = p[4];
        t[1] = p[5];
        time_val.seconds = (uint8_t)atoi(t);
        t[0] = p[6];
        t[1] = p[7];
        time_val.day = (uint8_t)atoi(t);
        t[0] = p[8];
        t[1] = p[9];
        time_val.date = (uint8_t)atoi(t);
        t[0] = p[10];
        t[1] = p[11];
        time_val.month = (uint8_t)atoi(t);

        char y[5] = {p[12], p[13], p[14], p[15], 0};
        time_val.year = (uint16_t)atoi(y);
      }
    }
  }

  // Build output string based on parsed tokens and compile-time flags, with
  // careful buffer management
  size_t offset = 0;
  size_t remaining = output_str_size;
  int n = 0;

  output_str[0] = '\0';

  if (INCLUDE_USER_STRING) {
    n = snprintf(&output_str[offset], remaining, "/STR%s", user_string);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_DEVICE_ID) {
    n = snprintf(&output_str[offset], remaining, "%s/DID%d",
                 (offset > 0) ? "" : "", device_id);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_LOCATION) {
    n = snprintf(&output_str[offset], remaining, "%s/LOC%s",
                 (offset > 0) ? "" : "", location);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_TEMPERATURE) {
    n = snprintf(&output_str[offset], remaining, "%s/TMP%d",
                 (offset > 0) ? "" : "", temperature);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_TIME) {
    n = snprintf(&output_str[offset], remaining,
                 "%s/TIM%02d%02d%02d%02d%02d%02d%04d", (offset > 0) ? "" : "",
                 time_val.hours, time_val.minutes, time_val.seconds,
                 time_val.day, time_val.date, time_val.month, time_val.year);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  // Add termination slash
  if (strlen(output_str) > 0) {
    n = snprintf(&output_str[offset], remaining, "/");
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  // Optional: append RSSI info for debug/logging
  // snprintf(&output_str[offset], remaining, " RSSI=%d dBm", dBm_value);

  // Hard guarantee null-termination
  output_str[output_str_size - 1] = '\0';
}

static int init_luts_from_freqpair(void) {
  // frigjør hvis de finnes fra før
  free(sine_val_low);
  sine_val_low = NULL;
  free(sine_val_high);
  sine_val_high = NULL;
  free(dc_mid);
  dc_mid = NULL;

  sine_val_low = malloc(freq_pair.lower_freq_samples * sizeof(uint32_t));
  sine_val_high = malloc(freq_pair.higher_freq_samples * sizeof(uint32_t));
  dc_mid = malloc(freq_pair.lower_freq_samples * sizeof(uint32_t));

  if (!sine_val_low || !sine_val_high || !dc_mid) {
    free(sine_val_low);
    sine_val_low = NULL;
    free(sine_val_high);
    sine_val_high = NULL;
    free(dc_mid);
    dc_mid = NULL;
    return -1;
  }
  return 0;
}

void Error_Handler_Code(status_code_t code) {
  g_error_code = code;
  Error_Handler(); // call the CubeMX-compatible one
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
  printf("Error_Handler: code=%d\r\n", (int)g_error_code);

  // If you want it to repeat forever:
  LED_BlinkStatusCode((uint8_t)g_error_code);

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
