#include "radio.h"
#include "cc1101.h"
#include "cc1101_config.h"
#include "ds3231.h"
#include "error_codes.h"
#include "log.h"
#include "main.h"
#include "user_config.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>
#include <string.h>
#include <sys/_intsup.h>

int8_t message_id = 0;

int8_t Radio_InitRXMode(void) {
  LOGF("Initializing radio in RX mode...\r\n");
  uint8_t status = 0;

  // Write configuration for RX
  for (int i = 0; i < sizeof(cc1101_cfg_rx) / sizeof(ism_reg_t); i++) {
    CC1101_WriteReg(cc1101_cfg_rx[i].addr, cc1101_cfg_rx[i].value, &status);
    if (status != 0x0F) {
      LOGF("Error writing RX config at index %d, status: 0x%02X\r\n", i,
           status);
      return STATUS_CODE_RADIO_INIT_FAIL;
    }
  }

  CC1101_Strobe(0x33, &status); // SCAL
  HAL_Delay(2);
  CC1101_Strobe(0x38, &status); // SWOR

  HAL_Delay(10);

  LOGF("RX WOR mode initialized (EVENT0~500ms, 1200bps).\r\n");
  return 0;
}

int8_t Radio_InitTXMode(void) {
  uint8_t status = 0;

  // Write configuration for TX
  for (int i = 0; i < sizeof(cc1101_cfg_tx) / sizeof(ism_reg_t); i++) {
    CC1101_WriteReg(cc1101_cfg_tx[i].addr, cc1101_cfg_tx[i].value, &status);
    if (status != 0x0F) {
      LOGF("Error writing TX config at index %d, status: 0x%02X\r\n", i,
           status);
      return STATUS_CODE_RADIO_INIT_FAIL;
    }
  }

  message_id = 0; // reset message ID counter on each init

  CC1101_Strobe(0x33, &status); // Calibrate (SCAL) before TX

  LOGF("TX mode initialized (250kbps, no WOR).\r\n");

  return 0;
}

int8_t Radio_Init(int8_t operation_mode) {
  const uint32_t delays_ms[] = {5, 20, 50};

  LOGF("Initializing radio...\r\n");
  HAL_Delay(500); // let CC1101 fully power cycle between STM32 resets

  uint8_t status = 0;
  uint8_t part = 0, ver = 0;

  for (int attempt = 0; attempt < 3; attempt++) {
    LOGF("Radio init attempt %d...\r\n", attempt + 1);

    // Power-up reset sequence (per TI recommendation) with retries
    HAL_Delay(delays_ms[attempt]);
    if (!CC1101_PowerUpReset()) {
      LOGF("CC1101 power-up reset failed on attempt %d\r\n", attempt + 1);
      continue; // retry
      // return STATUS_CODE_RADIO_INIT_FAIL;
    }

    HAL_Delay(10); // Add this

    CC1101_ReadReg(0xF0, &part, &status); // PARTNUM → 0x00
    CC1101_ReadReg(0xF1, &ver, &status);  // VERSION  → 0x14

    LOGF("CC1101 PARTNUM=0x%02X VERSION=0x%02X\r\n", part, ver);
    if (part != 0x00 || ver != 0x14) {
      LOGF("Unexpected CC1101 part/version, check wiring and power.\r\n");
      // return STATUS_CODE_RADIO_VERSION_ERROR;
      continue; // retry
    }

    if (operation_mode == 0) {
      return Radio_InitRXMode();
    } else if (operation_mode == 1) {
      return Radio_InitTXMode();
    }
  }

  return STATUS_CODE_RADIO_INIT_FAIL;
}

char *Radio_BuildPayload(uint32_t offset_ms) {
  char temp_buf[256];
  size_t offset = 0;

  // Make sure message ID is always two digits for consistent parsing, and wrap
  // around at 99
  offset += snprintf(temp_buf + offset, sizeof(temp_buf) - offset, "/MID%02d",
                     message_id);

  message_id = (message_id + 1) % 100;

  if (INCLUDE_USER_STRING) {
    offset += snprintf(temp_buf + offset, sizeof(temp_buf) - offset, "/STR%s",
                       USER_STRING);
  }

  if (INCLUDE_TIME) {
    rtc_time_t now;
    DS3231_PowerOn();
    DS3231_GetTime(&now);
    DS3231_PowerOff();

    if (now.year != 2000) {
      offset +=
          snprintf(temp_buf + offset, sizeof(temp_buf) - offset,
                   "/TIM%02d%02d%02d%02d%02d%02d%04d", now.hours, now.minutes,
                   now.seconds, now.day, now.date, now.month, now.year);
    } else {
      LOGF("RTC time not set, skipping time field\r\n");
    }
  }

  return strdup(temp_buf);
}

void Radio_Transmit(void) {
  LOGF("Starting radio transmission...\r\n");
  uint8_t status = 0;
  uint32_t t0 = HAL_GetTick(); // Capture once before loop

  for (int tx_repeat = 0; tx_repeat < 1; tx_repeat++) {
    const char *payload_str = Radio_BuildPayload(HAL_GetTick() - t0);
    size_t payload_len = strlen(payload_str);

    LOGF("TX %d payload: %s\r\n", tx_repeat + 1, payload_str);

    uint8_t pkt[2 + payload_len];
    pkt[0] = payload_len + 1;
    pkt[1] = 0xEB;
    memcpy(&pkt[2], payload_str, payload_len);

    LOGF("TX payload_len=%d pkt_size=%d\r\n", payload_len, 2 + payload_len);
    LOGF("TX payload: %s\r\n", payload_str);

    CC1101_Strobe(0x3B, &status); // SFTX: flush TX FIFO

    CC1101_WriteBurstReg(0x3F, pkt, sizeof(pkt), &status);
    if (status != 0x0F) {
      LOGF("Error writing to TX FIFO, status: 0x%02X\r\n", status);
      Error_Handler_Code(STATUS_CODE_TRANSMISSION_ERROR);
      return;
    }

    CC1101_Strobe(0x35, &status); // STX

    // DEBUG: Check TX state
    uint8_t marcstate = 0;
    HAL_Delay(50);
    CC1101_ReadReg(0xF5, &marcstate, &status);
    marcstate &= 0x1F;
    LOGF("DEBUG: After STX strobe, MARCSTATE=0x%02X ", marcstate);
    if (marcstate == 0x13)
      LOGF("(TX)\r\n");
    else if (marcstate == 0x01)
      LOGF("(IDLE - TX failed)\r\n");
    else
      LOGF("(UNKNOWN)\r\n");

    free((void *)payload_str);
  }
}

int Radio_Receive(uint8_t *out, size_t out_max_len) {
  uint8_t status = 0;

  // GDO0 will deassert on first FIFO read — read immediately
  uint8_t rxbytes = 0;
  CC1101_ReadReg(0xFB, &rxbytes, &status);

  if (rxbytes & 0x80) {
    LOGF("RX FIFO overflow, flushing and re-entering WOR.\r\n");
    Radio_EnterWOR();
    return -1;
  }

  uint8_t n = rxbytes & 0x7F;
  if (n == 0)
    return 0;

  if (n > out_max_len)
    n = (uint8_t)out_max_len;

  memset(out, 0, out_max_len);
  out[0] = '\0';

  CC1101_ReadBurstReg(0xFF, out, n, &status);

  return (int)n;
}

void Radio_EnterWOR(void) {
  uint8_t status = 0;

  CC1101_Strobe(0x36, &status); // SIDLE
  HAL_Delay(5);
  CC1101_Strobe(0x3A, &status); // SFRX
  HAL_Delay(10);
  CC1101_Strobe(0x38, &status); // SWOR
}
