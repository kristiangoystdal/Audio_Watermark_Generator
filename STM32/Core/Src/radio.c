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

  // Calibrate with XOSC active
  CC1101_Strobe(0x33, &status); // SCAL
  HAL_Delay(2);
  CC1101_Strobe(0x38, &status); // SWOR

  HAL_Delay(10);

  LOGF("RX WOR mode initialized with external XOSC (26MHz).\r\n");
  return 0;
}

int8_t Radio_InitTXMode(void) {
  uint8_t status = 0;

  // Enter IDLE first
  CC1101_Strobe(0x36, &status); // SIDLE
  HAL_Delay(5);

  // Write configuration for TX
  for (int i = 0; i < sizeof(cc1101_cfg_tx) / sizeof(ism_reg_t); i++) {
    CC1101_WriteReg(cc1101_cfg_tx[i].addr, cc1101_cfg_tx[i].value, &status);
    if (status != 0x0F) {
      LOGF("Error writing TX config at index %d, status: 0x%02X\r\n", i,
           status);
      return STATUS_CODE_RADIO_INIT_FAIL;
    }
  }

  // Calibrate (crystal auto-enabled on power-up)
  CC1101_Strobe(0x33, &status); // SCAL
  HAL_Delay(10);

  // Don't touch TEST register — let crystal run automatically

  // Set TX power (PATABLE)
  uint8_t patable[] = {0xC6}; // 12dBm
  CC1101_WriteBurstReg(0x3E, patable, sizeof(patable), &status);
  LOGF("PATABLE set to 0x%02X (12dBm)\r\n", patable[0]);

  // Verify MCSM1 was written
  uint8_t mcsm1_verify = 0;
  CC1101_ReadReg(0x17, &mcsm1_verify, &status);
  LOGF("MCSM1 verify: expected 0x04, got 0x%02X\r\n", mcsm1_verify);

  message_id = 0;

  // Read back all modem config registers
  uint8_t mdmcfg4, mdmcfg3, mdmcfg2, mdmcfg1, mdmcfg0;
  CC1101_ReadReg(0x10, &mdmcfg4, &status);
  CC1101_ReadReg(0x11, &mdmcfg3, &status);
  CC1101_ReadReg(0x12, &mdmcfg2, &status);
  CC1101_ReadReg(0x13, &mdmcfg1, &status);
  CC1101_ReadReg(0x14, &mdmcfg0, &status);

  LOGF("MDMCFG4: 0x%02X\r\n", mdmcfg4);
  LOGF("MDMCFG3: 0x%02X\r\n", mdmcfg3);
  LOGF("MDMCFG2: 0x%02X\r\n", mdmcfg2);
  LOGF("MDMCFG1: 0x%02X\r\n", mdmcfg1);
  LOGF("MDMCFG0: 0x%02X\r\n", mdmcfg0);

  // Calculate and log baud rate
  uint32_t drate_e = mdmcfg4 & 0x0F;
  uint32_t drate_m = mdmcfg3;
  float drate = (26e6 / (1 << 20)) * ((drate_m + 256) << drate_e);
  LOGF("Data rate: %u bps (with external XOSC 26MHz)\r\n", (unsigned int)drate);

  LOGF("TX mode initialized (crystal auto-enabled).\r\n");
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

    HAL_Delay(10);

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
  uint32_t t0 = HAL_GetTick();

  for (int tx_repeat = 0; tx_repeat < 1; tx_repeat++) {
    const char *payload_str = Radio_BuildPayload(HAL_GetTick() - t0);
    size_t payload_len = strlen(payload_str);

    LOGF("TX payload: %s\r\n", payload_str);

    uint8_t pkt[2 + payload_len];
    pkt[0] = payload_len + 1;
    pkt[1] = 0xEB;
    memcpy(&pkt[2], payload_str, payload_len);

    LOGF("Packet size: %u bytes\r\n", (unsigned int)sizeof(pkt));

    // Flush FIFO
    CC1101_Strobe(0x3B, &status);
    HAL_Delay(2);

    // Check TXBYTES before write
    uint8_t txbytes_before = 0;
    CC1101_ReadReg(0xFA, &txbytes_before, &status);
    LOGF("TXBYTES before write: 0x%02X\r\n", txbytes_before);

    // Write FIFO
    CC1101_WriteBurstReg(0x3F, pkt, sizeof(pkt), &status);
    LOGF("WriteBurstReg status: 0x%02X\r\n", status);

    // Check TXBYTES after write
    uint8_t txbytes_after = 0;
    CC1101_ReadReg(0xFA, &txbytes_after, &status);
    LOGF("TXBYTES after write: 0x%02X (expect 0x%02X)\r\n", txbytes_after,
         (unsigned int)sizeof(pkt));

    if ((txbytes_after & 0x7F) == 0) {
      LOGF("ERROR: FIFO write failed!\r\n");
      return;
    }

    // Issue STX
    LOGF("Issuing STX...\r\n");
    uint8_t marcstate_before = 0;
    CC1101_ReadReg(0xF5, &marcstate_before, &status);
    marcstate_before &= 0x1F;
    LOGF("MARCSTATE before STX: 0x%02X\r\n", marcstate_before);

    uint32_t tx_start = HAL_GetTick();
    CC1101_Strobe(0x35, &status); // STX
    LOGF("STX strobe status: 0x%02X\r\n", status);

    HAL_Delay(5);
    uint8_t marcstate = 0;
    CC1101_ReadReg(0xF5, &marcstate, &status);
    marcstate &= 0x1F;
    LOGF("MARCSTATE 5ms after STX: 0x%02X\r\n", marcstate);

    // Wait for TX to complete (MCSM1 auto-transitions to IDLE)
    uint32_t tx_timeout = HAL_GetTick() + 2000;
    uint32_t last_check = HAL_GetTick();
    while (HAL_GetTick() < tx_timeout && marcstate == 0x13) {
      CC1101_ReadReg(0xF5, &marcstate, &status);
      marcstate &= 0x1F;
      if (HAL_GetTick() - last_check >= 10) { // Log every 10ms
        LOGF("  MARCSTATE: 0x%02X at %lu ms\r\n", marcstate,
             HAL_GetTick() - tx_start);
        last_check = HAL_GetTick();
      }
      HAL_Delay(1);
    }

    uint32_t tx_end = HAL_GetTick();
    LOGF("MARCSTATE final: 0x%02X (took %lu ms)\r\n", marcstate,
         tx_end - tx_start);

    free((void *)payload_str);
  }

  HAL_Delay(100);
  uint8_t post_tx_marcstate = 0;
  CC1101_ReadReg(0xF5, &post_tx_marcstate, &status);
  post_tx_marcstate &= 0x1F;
  LOGF("Post-TX MARCSTATE: 0x%02X\r\n", post_tx_marcstate);
}

int Radio_Receive(uint8_t *out, size_t out_max_len) {
  uint8_t status = 0;

  // GDO0 will deassert on first FIFO read — read immediately
  uint8_t rxbytes = 0;
  CC1101_ReadReg(0xFB, &rxbytes, &status);

  LOGF("RXBYTES: 0x%02X\r\n", rxbytes);

  if (rxbytes & 0x80) {
    LOGF("RX FIFO overflow, flushing and re-entering WOR.\r\n");
    Radio_EnterWOR();
    return -1;
  }

  uint8_t n = rxbytes & 0x7F;
  if (n == 0) {
    LOGF("No data in FIFO.\r\n");
    return 0;
  }

  if (n > out_max_len)
    n = (uint8_t)out_max_len;

  memset(out, 0, out_max_len);
  out[0] = '\0';

  CC1101_ReadBurstReg(0xFF, out, n, &status);

  // Read RSSI (signal strength)
  uint8_t rssi_raw = 0;
  CC1101_ReadReg(0xF4, &rssi_raw, &status);
  int rssi_dbm = (rssi_raw & 0x7F) / 2 - 74;

  LOGF("RX packet received: %u bytes, RSSI: %d dBm\r\n", n, rssi_dbm);
  LOGF("RX data: %s\r\n", (char *)out);

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

void Radio_EnterSleep(void) {
  uint8_t status = 0;
  CC1101_Strobe(0x36, &status); // SIDLE
  HAL_Delay(5);
  CC1101_Strobe(0x39, &status); // SPWD
}

void Radio_EnterIdle(void) {
  uint8_t status = 0;
  CC1101_Strobe(0x36, &status); // SIDLE
}

int8_t Radio_CalibrateXtalFrequency(void) {
  uint8_t status = 0;

  LOGF("Starting XTAL frequency calibration...\r\n");

  // Read current FREQ2, FREQ1, FREQ0 registers
  uint8_t freq2, freq1, freq0;
  CC1101_ReadReg(0x0D, &freq2, &status);
  CC1101_ReadReg(0x0E, &freq1, &status);
  CC1101_ReadReg(0x0F, &freq0, &status);

  // Reconstruct 24-bit frequency value
  uint32_t freq_reg = ((uint32_t)freq2 << 16) | ((uint32_t)freq1 << 8) | freq0;

  // Current frequency (26 MHz assumed nominal)
  float f_xosc_nominal = 26000000.0f;
  float f_rf_current = (freq_reg / 16777216.0f) * f_xosc_nominal;

  LOGF("Current FREQ registers: 0x%02X 0x%02X 0x%02X\r\n", freq2, freq1, freq0);
  LOGF("Current XOSC assumed: %.0f Hz\r\n", f_xosc_nominal);
  LOGF("Current RF frequency: %.2f Hz\r\n", f_rf_current);

  // Target frequency (433 MHz for EU ISM band)
  float f_rf_target = 433000000.0f;

  // Read back MDMCFG4, MDMCFG3 for logging
  uint8_t mdmcfg4, mdmcfg3;
  CC1101_ReadReg(0x10, &mdmcfg4, &status);
  CC1101_ReadReg(0x11, &mdmcfg3, &status);

  uint32_t drate_e = mdmcfg4 & 0x0F;
  uint32_t drate_m = mdmcfg3;
  float drate = (26e6 / (1 << 20)) * ((drate_m + 256) << drate_e);

  LOGF("Data rate (with nominal 26MHz): %.0f bps\r\n", drate);

  // Estimate crystal frequency error from your known capacitive mismatch
  // You have 12pF target but ~9.2pF actual (3pF low)
  // This causes roughly +200 ppm frequency high across all bands

  int16_t trim_ppm = 200; // empirical: +200 ppm high due to load mismatch

  LOGF("Estimated XOSC error: +%d ppm (from capacitive mismatch)\r\n",
       trim_ppm);

  // Apply trim by adjusting FREQ registers
  // freq_correction = trim_ppm / 1e6 * freq_reg
  int32_t freq_correction = (int32_t)((trim_ppm / 1e6f) * (float)freq_reg);

  LOGF("Frequency correction: %ld LSBs\r\n", freq_correction);

  uint32_t freq_reg_trimmed = freq_reg - freq_correction;

  // Extract trimmed bytes
  uint8_t freq2_trim = (freq_reg_trimmed >> 16) & 0xFF;
  uint8_t freq1_trim = (freq_reg_trimmed >> 8) & 0xFF;
  uint8_t freq0_trim = freq_reg_trimmed & 0xFF;

  LOGF("Trimmed FREQ registers: 0x%02X 0x%02X 0x%02X\r\n", freq2_trim,
       freq1_trim, freq0_trim);

  // Write trimmed values back
  CC1101_WriteReg(0x0D, freq2_trim, &status);
  CC1101_WriteReg(0x0E, freq1_trim, &status);
  CC1101_WriteReg(0x0F, freq0_trim, &status);

  if (status != 0x0F) {
    LOGF("ERROR: Failed to write trimmed FREQ registers\r\n");
    return STATUS_CODE_RADIO_INIT_FAIL;
  }

  LOGF("XTAL frequency calibration complete. Target: 433 MHz, Trimmed by %d "
       "ppm.\r\n",
       trim_ppm);

  return 0;
}