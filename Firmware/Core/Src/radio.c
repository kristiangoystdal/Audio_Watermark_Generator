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

  CC1101_Strobe(0x34, &status); // SRX
  HAL_Delay(10);

  uint8_t freq2, freq1, freq0;
  CC1101_ReadReg(0x0D, &freq2, &status);
  CC1101_ReadReg(0x0E, &freq1, &status);
  CC1101_ReadReg(0x0F, &freq0, &status);

  uint32_t freq_reg = ((uint32_t)freq2 << 16) | ((uint32_t)freq1 << 8) | freq0;
  float f_rf = (freq_reg / 16777216.0f) * 26000000.0f;

  LOGF("RX mode initialized at %.2f MHz (crystal auto-enabled).\r\n",
       f_rf / 1e6);

  uint8_t fscal3;
  CC1101_ReadReg(0x23, &fscal3, &status);
  LOGF("FSCAL3: 0x%02X\r\n", fscal3);

  CC1101_Strobe(0x34, &status); // SRX
  HAL_Delay(10);

  uint8_t marcstate = 0;
  CC1101_ReadReg(0xF5, &marcstate, &status);
  LOGF("MARCSTATE after SRX: 0x%02X (expect 0x0D)\r\n", marcstate & 0x1F);

  uint8_t iocfg0 = 0;
  CC1101_ReadReg(0x02, &iocfg0, &status);
  LOGF("IOCFG0: 0x%02X\r\n", iocfg0);

  LOGF("RX continuous mode initialized with external XOSC (26MHz).\r\n");
  return 0;
}

int8_t Radio_InitTXMode(void) {
  uint8_t status = 0;

  CC1101_Strobe(0x36, &status); // SIDLE
  HAL_Delay(5);

  for (int i = 0; i < sizeof(cc1101_cfg_tx) / sizeof(ism_reg_t); i++) {
    CC1101_WriteReg(cc1101_cfg_tx[i].addr, cc1101_cfg_tx[i].value, &status);
    if (status != 0x0F) {
      LOGF("Error writing TX config at index %d, status: 0x%02X\r\n", i,
           status);
      return STATUS_CODE_RADIO_INIT_FAIL;
    }
  }

  CC1101_Strobe(0x33, &status); // SCAL
  HAL_Delay(10);

  uint8_t patable[] = {0xC0}; // 12dBm
  CC1101_WriteBurstReg(0x3E, patable, sizeof(patable), &status);
  LOGF("PATABLE set to 0x%02X (12dBm)\r\n", patable[0]);

  uint8_t mcsm1_verify = 0;
  CC1101_ReadReg(0x17, &mcsm1_verify, &status);
  LOGF("MCSM1 verify: expected 0x04, got 0x%02X\r\n", mcsm1_verify);

  message_id = 0;

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

  uint8_t frend0, frend1, patable_val;
  CC1101_ReadReg(0x22, &frend0, &status);
  CC1101_ReadReg(0x21, &frend1, &status);
  CC1101_ReadReg(0x3E, &patable_val, &status);

  LOGF("FREND0: 0x%02X (expect 0x10)\r\n", frend0);
  LOGF("FREND1: 0x%02X (expect 0x56)\r\n", frend1);
  LOGF("PATABLE[0]: 0x%02X (expect 0xC0)\r\n", patable_val);

  uint8_t freq2, freq1, freq0;
  CC1101_ReadReg(0x0D, &freq2, &status);
  CC1101_ReadReg(0x0E, &freq1, &status);
  CC1101_ReadReg(0x0F, &freq0, &status);

  uint32_t freq_reg = ((uint32_t)freq2 << 16) | ((uint32_t)freq1 << 8) | freq0;
  LOGF("FREQ regs: 0x%02X 0x%02X 0x%02X, freq_reg=0x%06lX\r\n", freq2, freq1,
       freq0, freq_reg);

  uint64_t f_hz = (freq_reg * 26000000UL) / 16777216UL;
  uint64_t f_mhz = f_hz / 1000000UL;
  uint64_t f_khz = (f_hz % 1000000UL) / 1000UL;

  LOGF("TX actual frequency: %lu.%03lu MHz\r\n", f_mhz, f_khz);

  uint8_t fscal3;
  CC1101_ReadReg(0x23, &fscal3, &status);
  LOGF("FSCAL3: 0x%02X\r\n", fscal3);

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
  HAL_Delay(500);

  uint8_t status = 0;
  uint8_t part = 0, ver = 0;

  for (int attempt = 0; attempt < 3; attempt++) {
    LOGF("Radio init attempt %d...\r\n", attempt + 1);

    HAL_Delay(delays_ms[attempt]);
    if (!CC1101_PowerUpReset()) {
      LOGF("CC1101 power-up reset failed on attempt %d\r\n", attempt + 1);
      continue;
    }

    HAL_Delay(10);

    CC1101_ReadReg(0xF0, &part, &status);
    CC1101_ReadReg(0xF1, &ver, &status);

    LOGF("CC1101 PARTNUM=0x%02X VERSION=0x%02X\r\n", part, ver);
    if (part != 0x00 || ver != 0x14) {
      LOGF("Unexpected CC1101 part/version, check wiring and power.\r\n");
      continue;
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

static uint32_t last_tx_calibration = 0;
const uint32_t TX_RECAL_INTERVAL_MS = 5 * 60 * 1000; // 5 minutes

void Radio_Transmit(void) {
  uint32_t now = HAL_GetTick();
  uint8_t status = 0;

  // Periodic recalibration to compensate for temperature-induced frequency
  // drift
  if (now - last_tx_calibration > TX_RECAL_INTERVAL_MS) {
    LOGF("TX: Recalibrating for temperature drift...\r\n");
    CC1101_Strobe(0x36, &status); // SIDLE
    HAL_Delay(5);
    CC1101_Strobe(0x33, &status); // SCAL
    HAL_Delay(2);
    CC1101_Strobe(0x35, &status); // STX
    HAL_Delay(10);
    last_tx_calibration = now;
  }

  const char *payload_str = Radio_BuildPayload(0);
  size_t payload_len = strlen(payload_str);

  uint8_t pkt[61]; // Match PKTLEN=0x3D
  memset(pkt, 0, sizeof(pkt));
  pkt[0] = payload_len + 1;
  pkt[1] = 0xEB;
  memcpy(&pkt[2], payload_str, payload_len);

  LOGF("Packet size: 61 bytes (PKTLEN match)\r\n");

  CC1101_Strobe(0x3B, &status); // SFTX
  HAL_Delay(1);

  CC1101_WriteBurstReg(0x3F, pkt, sizeof(pkt), &status);

  uint8_t txbytes_after = 0;
  CC1101_ReadReg(0xFA, &txbytes_after, &status);
  if ((txbytes_after & 0x7F) == 0) {
    LOGF("ERROR: FIFO write failed!\r\n");
    free((void *)payload_str);
    return;
  }

  CC1101_Strobe(0x35, &status);

  uint8_t marcstate = 0x13;
  uint32_t tx_timeout = HAL_GetTick() + 2000;
  while (HAL_GetTick() < tx_timeout) {
    CC1101_ReadReg(0xF5, &marcstate, &status);
    marcstate &= 0x1F;
    if (marcstate == 0x01)
      break;
    HAL_Delay(10);
  }

  uint32_t tx_end = HAL_GetTick();
  LOGF("Radio transmission complete: %lu ms\r\n", tx_end - now);

  free((void *)payload_str);
}

int Radio_Receive(uint8_t *out, size_t out_max_len) {
  uint8_t status = 0;

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

  // Last 2 bytes are appended status: [RSSI, LQI|CRC_OK]
  // (PKTCTRL1.APPEND_STATUS=1)
  uint8_t lqi_crc = out[n - 1];
  if (!(lqi_crc & 0x80)) {
    LOGF("CRC FAIL — dropping packet\r\n");
    return -1;
  }

  uint8_t rssi_raw = out[n - 2];
  int rssi_dbm =
      (rssi_raw >= 128) ? (rssi_raw - 256) / 2 - 74 : rssi_raw / 2 - 74;

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
  CC1101_Strobe(0x33, &status); // SCAL
  HAL_Delay(10);
  CC1101_Strobe(0x34, &status); // SRX
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

  uint8_t freq2, freq1, freq0;
  CC1101_ReadReg(0x0D, &freq2, &status);
  CC1101_ReadReg(0x0E, &freq1, &status);
  CC1101_ReadReg(0x0F, &freq0, &status);

  uint32_t freq_reg = ((uint32_t)freq2 << 16) | ((uint32_t)freq1 << 8) | freq0;

  float f_xosc_nominal = 26000000.0f;
  float f_rf_current = (freq_reg / 16777216.0f) * f_xosc_nominal;

  LOGF("Current FREQ registers: 0x%02X 0x%02X 0x%02X\r\n", freq2, freq1, freq0);
  LOGF("Current XOSC assumed: %.0f Hz\r\n", f_xosc_nominal);
  LOGF("Current RF frequency: %.2f Hz\r\n", f_rf_current);

  float f_rf_target = 433000000.0f;

  uint8_t mdmcfg4, mdmcfg3;
  CC1101_ReadReg(0x10, &mdmcfg4, &status);
  CC1101_ReadReg(0x11, &mdmcfg3, &status);

  uint32_t drate_e = mdmcfg4 & 0x0F;
  uint32_t drate_m = mdmcfg3;
  float drate = (26e6 / (1 << 20)) * ((drate_m + 256) << drate_e);

  LOGF("Data rate (with nominal 26MHz): %.0f bps\r\n", drate);

  int16_t trim_ppm = 200;

  LOGF("Estimated XOSC error: +%d ppm (from capacitive mismatch)\r\n",
       trim_ppm);

  int32_t freq_correction = (int32_t)((trim_ppm / 1e6f) * (float)freq_reg);

  LOGF("Frequency correction: %ld LSBs\r\n", freq_correction);

  uint32_t freq_reg_trimmed = freq_reg - freq_correction;

  uint8_t freq2_trim = (freq_reg_trimmed >> 16) & 0xFF;
  uint8_t freq1_trim = (freq_reg_trimmed >> 8) & 0xFF;
  uint8_t freq0_trim = freq_reg_trimmed & 0xFF;

  LOGF("Trimmed FREQ registers: 0x%02X 0x%02X 0x%02X\r\n", freq2_trim,
       freq1_trim, freq0_trim);

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