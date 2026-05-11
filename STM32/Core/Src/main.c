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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Include headers for custom modules
#include "battery.h"
#include "cc1101.h"
#include "cc1101_config.h"
#include "ds3231.h"
#include "error_codes.h"
#include "flash_flag.h"
#include "frequency_config.h"
#include "led_feedback.h"
#include "log.h"
#include "opamps.h"
#include "radio.h"
#include "reed_solomon.h"
#include "relay.h"
#include "user_config.h"
#include "wakeup.h"

// Standard library includes
#include <cmsis_gcc.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32g431xx.h>
#include <stm32g4xx.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_dac.h>
#include <stm32g4xx_hal_def.h>
#include <stm32g4xx_hal_gpio.h>
#include <string.h>
#include <sys/_intsup.h>
#include <sys/types.h>
#include <unistd.h>
#include <usbd_cdc_if.h>

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

#define MAX_PAYLOAD_BYTES (MAX_NUM_CHARS + RS_ERROR_CORRECTION_SYMBOLS)
#define BITSTREAM_LENGTH (1u + (MAX_PAYLOAD_BYTES * 8u) + 8u)

#define BIT_POLARITY 0

#define DS3231_ADDR (0x68 << 1) // 7-bit addr shifted for HAL
#define T4B_I2C_ADDR_7BIT 0x20
#define T4B_I2C_ADDR_HAL (T4B_I2C_ADDR_7BIT << 1) // 0x40 for HAL

#define DIV_FLOOR(n, d) ((uint32_t)((n) / (d)))
#define DIV_ROUND(n, d) ((uint32_t)(((n) + ((d) / 2u)) / (d)))

#define OUTPUT_SEGMENT 500
#define BUFFER_SIZE (OUTPUT_SEGMENT * 5)

#define SYNC_DELAY_MS 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char input_string[MAX_NUM_CHARS] = {0};

uint8_t bitstream[BITSTREAM_LENGTH] = {0};

volatile uint8_t active_done = 0;

rtc_time_t now = {.seconds = 0,
                  .minutes = 0,
                  .hours = 0,
                  .day = 4,
                  .date = 1,
                  .month = 1,
                  .year = 1970};

int8_t temp_int = 99;

static uint32_t sine_val_low[LUT_LOW_SAMPLES];
static uint32_t sine_val_high[LUT_HIGH_SAMPLES];
static uint32_t dc_mid[LUT_LOW_SAMPLES];

// Pulse Calculation Variables
uint32_t fs = 0;
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
volatile size_t current_sine_period = 0;
volatile size_t current_sine_index = 0;
FillResult result;

uint8_t transmission[256] = {0};
int dBm_value = 0;

freq_pair_t freq_pair = {0};

volatile status_code_t g_error_code = STATUS_CODE_OK;

extern volatile uint8_t cdc_rx_buf[64];
extern volatile uint8_t cdc_rx_len;

uint32_t wake_up_tick = 0;

bool first_boot = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void EnterStopMode(void);
void calculate_active_duration_ms(size_t bitstream_len);
void StartActiveWindowMs(uint32_t ms);
void make_bitstream_from_bytes(const uint8_t *data, size_t length);
void update_input_string(void);
int make_preamble(int start_idx);
void get_sineval_low(void);
void get_sineval_high(void);
void get_dc_mid(void);
uint32_t get_dac_sample_rate_hz(void);
static void RTC_SetWakeupTimer(uint32_t seconds);

static void start_audio_arm(void);
static void start_audio_trigger(void);
static void stop_audio_transmission(void);

void app_radio_test(void);

void read_buffer(void);

void process_transmission(uint8_t *transmission, int *dBm_value);

void create_payload_string(const char *user_string, int device_id,
                           const char *location, int8_t temperature,
                           rtc_time_t time_val, bool include_message_id,
                           int8_t message_id, uint8_t *output_str,
                           size_t output_str_size);

void create_string_from_received_data(const uint8_t *transmission,
                                      int dBm_value, uint8_t *output_str,
                                      size_t output_str_size);

static int init_luts_from_freqpair(void);

void Error_Handler_Code(status_code_t code);

void TryReceiveTimeSync(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
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
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  if (OPERATION_MODE == 0) {
    LOGF("Starting in RX mode\r\n");
  } else if (OPERATION_MODE == 1) {
    LOGF("Starting in TX mode\r\n");
  } else {
    LOGF("Starting in Standalone mode\r\n");
  }

  //-----------------------------------------------------------------------------//
  // Set DAC output to mid-level (silence) before starting for RX and Standalone
  //-----------------------------------------------------------------------------//

  if (OPERATION_MODE == 0 || OPERATION_MODE == 2) {
    DAC_SetMidlevel(&hdac1, 2048);
  }

  //-----------------------------------------------------------------------------//
  // Initialize UART logging
  //-----------------------------------------------------------------------------//

  LOGF("\r\n");
  LOGF("UART boot OK\r\n");
  HAL_Delay(200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //-----------------------------------------------------------------------------//
  // Initialize RTC
  //-----------------------------------------------------------------------------//

  if (!FlashFlag_TimeWasSet()) {
    TryReceiveTimeSync();

    FlashFlag_SetTimeWasSet();
  }

  DS3231_PowerOn();
  DS3231_Init();
  DS3231_GetTime(&now);
  DS3231_ReadTemperature(&temp_int);
  DS3231_PowerOff();

  LOGF("Current RTC time: %02d:%02d:%02d, %02d/%02d/%04d, DOW: %u\r\n",
       now.hours, now.minutes, now.seconds, now.date, now.month,
       now.year + 2000, now.day);
  LOGF("Temperature: %d°C\r\n", temp_int);

  LOGF("-------------------------\r\n");
  LOGF("\r\n");

  //-----------------------------------------------------------------------------//
  // Set first valid minute for wakeup based on current time and user-configured
  // interval
  //-----------------------------------------------------------------------------//

  if (OPERATION_MODE != 0) {
    WakeUp_Init(now);
  } else {
    LOGF("RX mode - skipping wakeup timer configuration\r\n");
  }

  //-----------------------------------------------------------------------------//
  // Check battery voltage and go back to sleep if low
  //-----------------------------------------------------------------------------//

  Battery_IsLow(&hadc1);

  //-----------------------------------------------------------------------------//
  // Get DAC sample rate and log it
  //-----------------------------------------------------------------------------//

  uint32_t dac_sample_rate = get_dac_sample_rate_hz();
  LOGF("DAC sample rate: %lu Hz\r\n", (unsigned long)dac_sample_rate);
  LOGF("\r\n");

  //-----------------------------------------------------------------------------//
  // Find and set FSK frequencies based on user config and sample count
  // requirements (RX and Standalone modes)
  //-----------------------------------------------------------------------------//
  if (OPERATION_MODE != 1) {
    freq_pair = FreqConfig_FindFreqPair(dac_sample_rate);

    LOGF("Frequency pair: lower=%u Hz, higher=%u Hz\r\n",
         (unsigned int)freq_pair.lower_freq,
         (unsigned int)freq_pair.higher_freq);
    LOGF("--------------------------\r\n");
    LOGF("\r\n");
  } else {
    LOGF("Skipping finding frequencies\r\n");
  }

  //-----------------------------------------------------------------------------//
  // Prepare sine wave lookup tables
  //-----------------------------------------------------------------------------//

  if (OPERATION_MODE != 1) {
    if (init_luts_from_freqpair() != 0) {
      LOGF("LUT alloc failed\r\n");
      LOGF("--------------------------\r\n");
      Error_Handler_Code(STATUS_CODE_LUT_ALLOC_FAIL);
    }

    get_sineval_low();
    get_sineval_high();
    get_dc_mid();
  } else {
    LOGF("Skipping LUT preparation \r\n");
  }

  //-----------------------------------------------------------------------------//
  // Initialize radio for RX or TX mode
  //-----------------------------------------------------------------------------//

  if (OPERATION_MODE != 2) {
    HAL_Delay(20);
    int8_t init_result = Radio_Init(OPERATION_MODE);
    if (init_result != 0) {
      LOGF("Failed to initialize radio in %s mode, error code: %d\r\n",
           OPERATION_MODE == 0 ? "RX" : "TX", init_result);
      LOGF("--------------------------\r\n");
      LOGF("\r\n");
      Error_Handler_Code(init_result);
    }
  } else {
    LOGF("No radio in standalone mode - skipping radio initialization\r\n");

    // Power down CC1101 since we don't use it in standalone
    HAL_Delay(20);
    CC1101_PowerUpReset();
    HAL_Delay(10);

    // Put it in sleep to save power
    Radio_EnterSleep();
    LOGF("CC1101 powered down for standalone mode\r\n");
  }

  //-----------------------------------------------------------------------------//
  // Indicate boot complete with LED blinks
  //-----------------------------------------------------------------------------//

  LED_BlinkStatusCode(STATUS_CODE_OK); // code 0 = "0000" = 4 short blinks

  //-----------------------------------------------------------------------------//
  // Configure first wakeup timer for TX and Standalone modes
  //-----------------------------------------------------------------------------//

  Relay_SetBypassMode();
  if (OPERATION_MODE != 0) {
    LOGF("Setting first alarm for TX and Standalone mode\r\n");
    RTC_SetWakeupTimer(1); // 1 minute
  }

  //-----------------------------------------------------------------------------//
  // Main loop
  //-----------------------------------------------------------------------------//
  LOGF("\r\n");
  LOGF("Entering main loop...\r\n");
  LOGF("\r\n");

  DS3231_PowerOn();
  DS3231_GetTime(&now);
  DS3231_ReadTemperature(&temp_int);
  DS3231_PowerOff();

  int32_t wait_ms = 0;

  while (1) {
    // 1) Enter STOP mode and wait for wakeup from EXTI (GPIOB Pin 7) or RTC
    // wakeup timer
    LOGF("Entering STOP mode...\r\n");
    EnterStopMode();
    wake_up_tick = HAL_GetTick();

    // Set new wake up alarm for next cycle immediately after waking up
    // 55 seconds ensure we wake up in every minute to check if it's the valid
    // minute for operation, even if we miss the exact wakeup timer match due to
    // timing variations
    if (OPERATION_MODE != 0) {
      uint32_t sleep_seconds = 55;
      RTC_SetWakeupTimer(sleep_seconds);
    }

    // 2) Check battery voltage and go back to sleep if low
    Battery_IsLow(&hadc1);

    // Update current minute from RTC
    DS3231_PowerOn();
    DS3231_GetTime(&now);
    DS3231_ReadTemperature(&temp_int);

    // Check if we are in the active window based on the current minute and
    // configured intervals, and if not, go back to sleep
    // Only for TX and Standalone modes
    if (OPERATION_MODE != 0) {
      if (!WakeUp_CheckCurrentMinute(now)) {
        LOGF("Took %lu ms to check wakeup time\r\n",
             (unsigned long)(HAL_GetTick() - wake_up_tick));
        LOGF("Not in active window, going back to sleep...\r\n");
        DS3231_PowerOff();
        continue;
      }
    }

    if (OPERATION_MODE == 0) {
      LOGF("Woke up for RX reception\r\n");

      // RX mode: read from radio and store in string
      int rx_len = Radio_Receive(transmission, sizeof(transmission));
      if (rx_len <= 4 || memchr(transmission, '/', rx_len) == NULL) {
        LOGF(
            "Failed to receive valid transmission, going back to sleep...\r\n");
        DS3231_PowerOff();
        Radio_EnterWOR();
        continue;
      }

      // Process received transmission
      process_transmission(transmission, &dBm_value);

      // 2) Get current time from RTC
      DS3231_GetTime(&now);
      DS3231_ReadTemperature(&temp_int);
      DS3231_PowerOff();

      // Create output string based on received data (e.g.
      // "/TIM.../STR.../DID.../LOC.../TMP...")
      uint8_t output_str[256] = {0};
      create_string_from_received_data(transmission, dBm_value, output_str,
                                       sizeof(output_str));

      size_t payload_len = strlen((char *)output_str);

      if (USE_REED_SOLOMON_ERROR_CORRECTION) {
        // Prepare buffer for Reed-Solomon codeword (message + parity)
        uint8_t codeword[payload_len + RS_ERROR_CORRECTION_SYMBOLS];
        memset(codeword, 0, sizeof(codeword));

        // Encode the output string using Reed-Solomon
        RS_EncodeMsg((const uint8_t *)output_str, (int)payload_len, codeword,
                     RS_ERROR_CORRECTION_SYMBOLS);

        // Set output_str to the codeword for transmission (truncated to fit
        // if necessary)
        size_t codeword_len = payload_len + RS_ERROR_CORRECTION_SYMBOLS;
        if (codeword_len > sizeof(output_str)) {
          codeword_len = sizeof(output_str);
        }
        memcpy(output_str, codeword, codeword_len);
        payload_len = codeword_len;
      }

      // Make bitstream from output string
      make_bitstream_from_bytes(output_str, payload_len);

      calculate_active_duration_ms(BITSTREAM_LENGTH);
      uint32_t total_ms = (uint32_t)(total_time * 1000.0f);

      if (USE_CABLE_TRANSMISSION) {
        Opamps_Enable(&hdac1);
        Relay_SetMixingMode();
      } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_Delay(100);
      }

      start_audio_arm();

      uint32_t target_tick = wake_up_tick + SYNC_DELAY_MS;
      wait_ms = (int32_t)(target_tick - HAL_GetTick());
      if (wait_ms > 1) {
        HAL_Delay((uint32_t)(wait_ms - 1));
      }
      while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
        ;
      __HAL_UART_DISABLE(&huart2);
      while (HAL_GetTick() < target_tick)
        ;
      (void)SysTick->CTRL; // clear COUNTFLAG by reading it
      // Now align to the next SysTick rollover for sub-ms precision
      // SysTick counts DOWN from LOAD to 0, then reloads - catch the reload
      // moment
      while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
        ;

      start_audio_trigger();
      __HAL_UART_ENABLE(&huart2);

      // Wait for active window to complete (enters low-power sleep while
      // waiting)
      StartActiveWindowMs((uint32_t)(total_ms));

      uint32_t last_toggle = HAL_GetTick();
      while (!active_done) {
        uint32_t now = HAL_GetTick();
        if ((now - last_toggle) >= 100) { // blink every 100 ms
          last_toggle = now;
          LED_Toggle();
        }
        __WFI(); // CPU sleeps while DMA+TIM2+DAC run
      }
      active_done = 0;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

      // Stop transmission and go back to sleep
      stop_audio_transmission();

      Radio_EnterWOR();

    } else if (OPERATION_MODE == 1) {
      LOGF("Woke up for TX transmission\r\n");
      Radio_EnterIdle();

      // Send transmission over radio
      LOGF("Starting transmission over radio...\r\n");
      // Toggle on Relay to make click sound at start of transmission for easier
      // analysis
      Relay_SetMixingMode();

      LED_ON();

      Radio_Transmit();

      Relay_SetBypassMode();
      LED_OFF();

      Radio_EnterSleep();
    } else {
      LOGF("Woke up for Standalone mode\r\n");

      // 1) Get current time and temperature from RTC
      DS3231_PowerOn();
      DS3231_GetTime(&now);
      DS3231_ReadTemperature(&temp_int);
      DS3231_PowerOff();

      // Create output string based on received data (e.g.
      // "/TIM.../STR.../DID.../LOC.../TMP...")
      uint8_t output_str[256] = {0};
      create_payload_string(INCLUDE_USER_STRING ? USER_STRING : NULL,
                            INCLUDE_DEVICE_ID ? DEVICE_ID : -1,
                            INCLUDE_LOCATION ? LOCATION : NULL,
                            INCLUDE_TEMPERATURE ? temp_int : -1, now, false, -1,
                            output_str, sizeof(output_str));

      size_t payload_len = strlen((char *)output_str);

      if (USE_REED_SOLOMON_ERROR_CORRECTION) {
        // Prepare buffer for Reed-Solomon codeword (message + parity)
        uint8_t codeword[payload_len + RS_ERROR_CORRECTION_SYMBOLS];
        memset(codeword, 0, sizeof(codeword));

        // Encode the output string using Reed-Solomon
        RS_EncodeMsg((const uint8_t *)output_str, (int)payload_len, codeword,
                     RS_ERROR_CORRECTION_SYMBOLS);

        // Set output_str to the codeword for transmission (truncated to fit
        // if necessary)
        size_t codeword_len = payload_len + RS_ERROR_CORRECTION_SYMBOLS;
        if (codeword_len > sizeof(output_str)) {
          codeword_len = sizeof(output_str);
        }
        memcpy(output_str, codeword, codeword_len);
        payload_len = codeword_len;
      }

      // Make bitstream from output string
      make_bitstream_from_bytes(output_str, payload_len);

      calculate_active_duration_ms(BITSTREAM_LENGTH);
      uint32_t total_ms = (uint32_t)(total_time * 1000.0f);

      if (USE_CABLE_TRANSMISSION) {
        Opamps_Enable(&hdac1);
        HAL_Delay(20);
        Relay_SetMixingMode();
      } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_Delay(100);
      }

      start_audio_arm();
      start_audio_trigger();

      // Wait for active window to complete (enters low-power sleep while
      // waiting)
      StartActiveWindowMs((uint32_t)(total_ms));

      uint32_t last_toggle = HAL_GetTick();
      uint32_t now = 0;
      while (!active_done) {
        now = HAL_GetTick();
        if ((now - last_toggle) >= 100) { // blink every 100 ms
          last_toggle = now;
          LED_Toggle();
        }
        __WFI(); // CPU sleeps while DMA+TIM2+DAC run
      }
      active_done = 0;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

      // Stop transmission and go back to sleep
      stop_audio_transmission();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
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
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
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
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10805D88;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 59, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  // Stop the RTC wakeup timer for now - we'll start it when we go to sleep
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 24;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA5 PA6 PA7
                           PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB3 PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void EnterStopMode(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
  PWR->SCR = PWR_SCR_CWUF;

  DBGMCU->CR &=
      ~(DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STANDBY);

  if (OPERATION_MODE == 0) {
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  } else {
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  }

  __DSB();
  __ISB();
  HAL_SuspendTick();
  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

  SystemClock_Config();
  HAL_ResumeTick();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  MX_I2C2_Init();
  MX_USART2_UART_Init();

  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  wake_up_tick = HAL_GetTick();

  // RX only: wait for GDO0 to deassert after wakeup
  if (OPERATION_MODE == 0) {
    uint32_t settle = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET &&
           HAL_GetTick() - settle < 5) {
    }
  }

  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  if (OPERATION_MODE == 0) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  } else {
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
  }
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  PWR->SCR = PWR_SCR_CWUF;
}

void StartActiveWindowMs(uint32_t ms) {
  active_done = 0;

  if (ms == 0) {
    ms = 1;
  }

  HAL_TIM_Base_Stop_IT(&htim6);
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
  uint16_t id = 0x2DD4;
  for (int i = 0; i < 16 && k < BITSTREAM_LENGTH; ++i) {
    bitstream[k++] = (((id >> (15 - i)) & 1) ^ BIT_POLARITY);
  }

  return k;
}
void make_bitstream_from_bytes(const uint8_t *data, size_t length) {
  size_t k = 0;

  if (k < BITSTREAM_LENGTH) {
    bitstream[k++] = 2; // silence
  }

  for (size_t i = 0; i < length && k < BITSTREAM_LENGTH; ++i) {
    for (int b = 7; b >= 0 && k < BITSTREAM_LENGTH; --b) {
      bitstream[k++] = (uint8_t)(((data[i] >> b) & 1u) ^ BIT_POLARITY);
    }
  }

  for (int i = 0; i < 8 && k < BITSTREAM_LENGTH; ++i) {
    bitstream[k++] = 2; // silence
  }

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
  const float dac_max = 4095.0f;
  const float dac_mid = dac_max / 2.0f;
  const float amplitude = (1.5f / 1.65f) * (SIGNAL_ATTENUATION / 100.0f);

  for (uint16_t i = 0; i < freq_pair.lower_freq_samples; i++) {
    sine_val_low[i] =
        (uint32_t)(dac_mid *
                   (1.0 + sinf(2.0 * pi * i / freq_pair.lower_freq_samples) *
                              amplitude));
  }
}

// Function to generate sine wave lookup table for the high frequency
void get_sineval_high(void) {
  const float dac_max = 4095.0f;
  const float dac_mid = dac_max / 2.0f;
  const float amplitude = (1.5f / 1.65f) * (SIGNAL_ATTENUATION / 100.0f);

  for (uint16_t i = 0; i < freq_pair.higher_freq_samples; i++) {
    sine_val_high[i] =
        (uint32_t)(dac_mid *
                   (1.0 + sinf(2.0 * pi * i / freq_pair.higher_freq_samples) *
                              amplitude));
  }
}

void get_dc_mid(void) {
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

  float f_0_bit_duration = (float)freq_pair.lower_freq_periods / f_0;
  float f_1_bit_duration = (float)freq_pair.higher_freq_periods / f_1;

  // Exact bitstream time
  total_time = 0.0f;
  for (int i = 0; i < bitstream_len; ++i) {
    if (bitstream[i] == 0) {
      total_time += f_0_bit_duration;
    } else if (bitstream[i] == 1) {
      total_time += f_1_bit_duration;
    } else {
      total_time += f_0_bit_duration; // silence
    }
  }

  // Add silence time
  // total_time *= 1.05f;                            // 5% margin
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

static void start_audio_arm(void) {
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

  // Clean start
  HAL_TIM_Base_Stop(&htim2);
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  DMA1->IFCR =
      DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
  NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);

  HAL_StatusTypeDef st =
      HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)output_buffer,
                        BUFFER_SIZE, DAC_ALIGN_12B_R);

  if (st != HAL_OK) {
    LOGF("  HAL_DAC_Start_DMA error: %d\r\n", st);
    Error_Handler_Code(STATUS_CODE_TRANSMISSION_ERROR);
  }

  // Ensure trigger enabled; TIM2 not started yet
  SET_BIT(DAC1->CR, DAC_CR_TEN1);
}

static void start_audio_trigger(void) {
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_StatusTypeDef st = HAL_TIM_Base_Start(&htim2);
  if (st != HAL_OK) {
    LOGF("  HAL_TIM_Base_Start error: %d\r\n", st);
    Error_Handler_Code(STATUS_CODE_TRANSMISSION_ERROR);
  }
}

static void stop_audio_transmission(void) {
  tx_active = false;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  Relay_SetBypassMode();

  // Stop trigger + DMA first
  HAL_TIM_Base_Stop(&htim2);
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  Opamps_Disable(&hdac1);

  DMA1->IFCR =
      DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
  NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
}

void string_to_hex(const char *str, uint8_t *hex_buf, size_t hex_buf_size) {
  size_t str_len = strlen(str);
  size_t max_bytes =
      (hex_buf_size < (str_len / 2)) ? hex_buf_size : (str_len / 2);

  for (size_t i = 0; i < max_bytes; i++) {
    sscanf(&str[i * 2], "%2hhx", &hex_buf[i]);
  }
}

void process_transmission(uint8_t *transmission, int *dBm_value) {
  size_t len = strlen((const char *)transmission);
  if (len <= 4) {
    LOGF("Transmission too short to process.\r\n");
    return;
  }

  uint8_t rssi_hex = transmission[len - 2]; // 2nd from end

  int8_t rssi_signed = (int8_t)rssi_hex; // 2's complement

  int rssi_dbm = (rssi_signed / 2) - 74; // integer dBm

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

void create_payload_string(const char *user_str, int device_id,
                           const char *location, int8_t temperature,
                           rtc_time_t time_val, bool INCLUDE_MESSAGE_ID,
                           int8_t message_id, uint8_t *output_str,
                           size_t output_str_size) {
  if (!output_str || output_str_size == 0)
    return;

  char temp_buf[256] = {0};
  size_t offset = 0;
  size_t remaining = sizeof(temp_buf);
  int n = 0;

  output_str[0] = '\0';

  // Append MID if enabled
  if (INCLUDE_MESSAGE_ID) {
    n = snprintf(&temp_buf[offset], remaining, "/MID%02d", message_id);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_TIME) {
    n = snprintf(&temp_buf[offset], remaining,
                 "%s/TIM%02d%02d%02d%02d%02d%02d%04d", (offset > 0) ? "" : "",
                 time_val.hours, time_val.minutes, time_val.seconds,
                 time_val.day, time_val.date, time_val.month, time_val.year);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_USER_STRING) {
    n = snprintf(&temp_buf[offset], remaining, "/STR%s", user_str);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_DEVICE_ID) {
    n = snprintf(&temp_buf[offset], remaining, "%s/DID%d",
                 (offset > 0) ? "" : "", device_id);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_LOCATION) {
    n = snprintf(&temp_buf[offset], remaining, "%s/LOC%s",
                 (offset > 0) ? "" : "", location);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  if (INCLUDE_TEMPERATURE) {
    n = snprintf(&temp_buf[offset], remaining, "%s/TMP%d",
                 (offset > 0) ? "" : "", temperature);
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }

  // Add termination slash
  if (strlen(temp_buf) > 0) {
    n = snprintf(&temp_buf[offset], remaining, "/");
    if (n > 0 && (size_t)n < remaining) {
      offset += (size_t)n;
      remaining -= (size_t)n;
    }
  }
  // Hard guarantee null-termination
  output_str[output_str_size - 1] = '\0';
  // Copy ASCII bytes into uint8_t output buffer
  size_t out_len = strnlen(temp_buf, sizeof(temp_buf));
  if (out_len >= output_str_size) {
    out_len = output_str_size - 1;
  }
  memcpy(output_str, temp_buf, out_len);
  output_str[out_len] = 0x00;
}

void create_string_from_received_data(const uint8_t *transmission,
                                      int dBm_value, uint8_t *output_str,
                                      size_t output_str_size) {
  if (!transmission || !output_str || output_str_size == 0)
    return;

  // Defaults (fallback if missing in RX packet)
  int mid = 0;
  char user_string[64] = USER_STRING;
  int device_id = DEVICE_ID;
  char location[64] = LOCATION;
  int8_t temperature = temp_int;
  rtc_time_t time_val = now;

  // Make local copy for strtok
  char buf[256] = {0};
  strncpy(buf, (const char *)transmission, sizeof(buf) - 1);

  // Parse tokens separated by /
  for (char *tok = strtok(buf, "/"); tok != NULL; tok = strtok(NULL, "/")) {
    // Extract type (first 3 chars) and data (remaining chars)
    if (strlen(tok) < 3)
      continue;

    char type[4] = {0};
    strncpy(type, tok, 3);
    const char *data = tok + 3;

    // Route by type
    if (strcmp(type, "MID") == 0) {
      if (strlen(data) >= 2) {
        char mid_buf[3] = {0};
        strncpy(mid_buf, data, 2);
        mid = atoi(mid_buf);
      }
    } else if (strcmp(type, "TIM") == 0) {
      if (strlen(data) >= 16) {
        int hours = 0, minutes = 0, seconds = 0;
        int day = 0, date = 0, month = 0, year = 0;
        if (sscanf(data, "%2d%2d%2d%2d%2d%2d%4d", &hours, &minutes, &seconds,
                   &day, &date, &month, &year) == 7) {
          time_val.hours = (uint8_t)hours;
          time_val.minutes = (uint8_t)minutes;
          time_val.seconds = (uint8_t)seconds;
          time_val.day = (uint8_t)day;
          time_val.date = (uint8_t)date;
          time_val.month = (uint8_t)month;
          time_val.year = (uint16_t)year;
        }
      }
    } else if (strcmp(type, "STR") == 0) {
      strncpy(user_string, data, sizeof(user_string) - 1);
    }
  }

  create_payload_string(user_string, device_id, location, temperature, time_val,
                        true, mid, output_str, output_str_size);
}

static int init_luts_from_freqpair(void) {
  if (freq_pair.lower_freq_samples > LUT_LOW_SAMPLES ||
      freq_pair.higher_freq_samples > LUT_HIGH_SAMPLES) {
    return -1;
  }
  return 0;
}

void Error_Handler_Code(status_code_t code) {
  g_error_code = code;
  // Error_Handler(); // call the CubeMX-compatible one
}

int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);
  uint32_t start = HAL_GetTick();
  while (CDC_Transmit_FS((uint8_t *)ptr, len) == USBD_BUSY) {
    if (HAL_GetTick() - start > 10)
      break;
  }
  return len;
}

static void RTC_SetWakeupTimer(uint32_t seconds) {
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

  if (seconds == 0) {
    seconds = 1;
  }

  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, seconds - 1,
                                  RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) {
    Error_Handler_Code(STATUS_CODE_UNKNOWN_ERROR);
  }
}

void TryReceiveTimeSync(void) {
  DS3231_PowerOn();
  HAL_Delay(10);

  LOGF("Waiting for time sync (10s)...\r\n");

  char buf[32] = {0};
  uint32_t start = HAL_GetTick();
  uint32_t last_ready = 0;

  while (HAL_GetTick() - start < 10000) {
    if (HAL_GetTick() - last_ready >= 500) {
      LOGF("READY_FOR_TIME_SYNC\r\n");
      last_ready = HAL_GetTick();
    }

    if (cdc_rx_len > 0) {
      uint8_t len = cdc_rx_len;
      cdc_rx_len = 0;
      memcpy(buf, (uint8_t *)cdc_rx_buf, len);
      buf[len] = '\0';

      for (int i = 0; i < len; i++) {
        if (buf[i] == '\r' || buf[i] == '\n') {
          buf[i] = '\0';
          break;
        }
      }

      if (buf[0] == 'T') {
        int h, m, s, dow, day, mon, year;
        if (sscanf(buf + 1, "%d:%d:%d %d %d/%d/%d", &h, &m, &s, &dow, &day,
                   &mon, &year) == 7) {

          DS3231_Init();
          DS3231_SetTime(s, m, h, dow, day, mon, year);

          LOGF("Time synced: %02d:%02d:%02d %02d/%02d/%04d\r\n", h, m, s, day,
               mon, year);
          LED_Toggle();
          HAL_Delay(50);
          LED_Toggle();
          return;
        }
      }
    }

    HAL_Delay(10);
  }

  LOGF("No time sync received\r\n");
  LED_Toggle();
  HAL_Delay(50);
  LED_Toggle();
  LED_Toggle();
  HAL_Delay(50);
  LED_Toggle();
  DS3231_PowerOff();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  LOGF("Error_Handler: code=%d\r\n", (int)g_error_code);

  // Turn off relay and opamps to save power (optional, depending on
  // error)
  Relay_SetBypassMode();
  Opamps_Disable(&hdac1);

  // Blink error code
  LED_BlinkStatusCode((uint8_t)g_error_code);

  __disable_irq();
  while (1) {
    // Stay here - reset or power cycle needed
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
