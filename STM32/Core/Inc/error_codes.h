#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#include <stdint.h>

typedef enum {
  STATUS_CODE_OK = 0,
  STATUS_CODE_STARTING_TRANSMISSION = 1,
  STATUS_CODE_LUT_ALLOC_FAIL = 2,
  STATUS_CODE_RADIO_INIT_FAIL = 3,
  STATUS_CODE_TRANSMISSION_ERROR = 4,
} status_code_t;

extern volatile status_code_t g_error_code;

void Error_Handler_Code(status_code_t code);

#endif // ERROR_CODES_H
