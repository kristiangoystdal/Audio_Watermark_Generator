#include "cc1101.h"
#include "ds3231.h"
#include "ism.h"
#include "ism_config_433.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <sys/_intsup.h>

void init_RX(void) {
  printf("Initializing radio in RX mode...\r\n");
  uint8_t status = 0;

  // Write configuration for RX
  for (int i = 0; i < sizeof(cc1101_cfg_rx) / sizeof(ism_reg_t); i++) {
    CC1101_WriteReg(cc1101_cfg_rx[i].addr, cc1101_cfg_rx[i].value, &status);
  }

  CC1101_Strobe(0x33, &status); // Calibrate (SCAL) before TX
  printf("Calibrate: 0x%02X\r\n", status);

  CC1101_Strobe(0x34, &status); // Go to RX state (SRX)
}

void init_TX(void) {
  uint8_t status = 0;

  // Write configuration for TX
  for (int i = 0; i < sizeof(cc1101_cfg_tx) / sizeof(ism_reg_t); i++) {
    CC1101_WriteReg(cc1101_cfg_tx[i].addr, cc1101_cfg_tx[i].value, &status);
  }

  CC1101_Strobe(0x33, &status); // Calibrate (SCAL) before TX
}

void init_radio(bool RX) {
  printf("Initializing radio...\r\n");

  uint8_t status = 0;
  uint8_t part = 0, ver = 0;

  // Reset (strobe)
  CC1101_Strobe(0x30, &status);
  HAL_Delay(1);

  // Read PARTNUM + VERSION
  CC1101_ReadReg(0xF1, &part, &status);
  CC1101_ReadReg(0x00, &ver, &status);

  printf("CC1101 PARTNUM=0x%02X VERSION=0x%02X\r\n", part, ver);

  if (RX == true) {
    init_RX();
  } else {
    init_TX();
  }

  printf("Radio test complete.\r\n");
}

void transmit_bytes(void) {
  uint8_t status = 0;

  const char payload_str[] = "Einar suger";
  const uint8_t *payload = (const uint8_t *)payload_str;
  uint8_t payload_len = (uint8_t)strlen(payload_str); // 11

  printf("Transmitting payload: %s\r\n", payload_str);
  printf("Payload in hex: ");
  for (size_t i = 0; i < payload_len; i++) {
    printf("%02X ", payload[i]);
  }
  printf("\r\n");
  uint8_t pkt[2 + payload_len]; // length byte + payload
  pkt[0] = payload_len + 1;     // length
  pkt[1] = 0xEB;
  memcpy(&pkt[2], payload, payload_len);

  // Optional: flush TX FIFO before loading (good practice)
  CC1101_Strobe(0x3B, &status); // SFTX :contentReference[oaicite:1]{index=1}

  // Burst write into TX FIFO
  CC1101_WriteBurstReg(
      0x3F, pkt, sizeof(pkt),
      &status); // 0x3F = TXFIFO :contentReference[oaicite:2]{index=2}

  // Start TX
  CC1101_Strobe(0x35, &status); // STX :contentReference[oaicite:3]{index=3}

  // Wait for IDLE (mask state bits)
  while (1) {
    uint8_t marcstate = 0;
    CC1101_ReadReg(0xF5, &marcstate, &status); // MARCSTATE (status space)
    marcstate &= 0x1F;
    if (marcstate == 0x01) {
      break; // IDLE
    }
  }
}

void start_TX(void) {
  printf("Starting TX...\r\n");
  // Transmit bytes for 2 minutes
  while (1) {

    // transmit_bytes();
    transmit_bytes();

    // Break after 2 minutes
    static uint32_t start_time = 0;
    if (start_time == 0) {
      start_time = HAL_GetTick();
    } else if (HAL_GetTick() - start_time >= 120000) {
      break;
    }

    // Optional: add delay between transmissions if desired
    // HAL_Delay(1000);

    break; // for quick test
  }
  printf("TX started.\r\n");
}

int read_RX(uint8_t *out, size_t out_max_len) {
  uint8_t status = 0;
  uint8_t rxbytes = 0;

  // RXBYTES: bit7=overflow, bits[6:0]=num bytes
  CC1101_ReadReg(0xFB, &rxbytes, &status);

  if (rxbytes & 0x80) {
    // FIFO overflow -> flush
    CC1101_Strobe(0x36, &status); // SIDLE
    CC1101_Strobe(0x3A, &status); // SFRX
    CC1101_Strobe(0x34, &status); // SRX
    return -1;
  }

  uint8_t n = rxbytes & 0x7F;
  if (n == 0)
    return 0;

  // Don't read more than the user buffer
  if (n > out_max_len)
    n = (uint8_t)out_max_len;

  // Clear the output buffer before writing new data
  memset(out, 0, out_max_len);
  out[0] = '\0';

  CC1101_ReadBurstReg(0xFF, out, n, &status); // RX FIFO burst read

  // Print received bytes in hex
  // printf("Received %d bytes: ", n);
  // for (size_t i = 0; i < n; i++) {
  //   printf("%02X ", out[i]);
  // }
  // printf("\r\n");

  // Re-enter RX (and flush to be safe)
  CC1101_Strobe(0x36, &status); // SIDLE
  CC1101_Strobe(0x3A, &status); // SFRX
  CC1101_Strobe(0x34, &status); // SRX

  return (int)n;
}
