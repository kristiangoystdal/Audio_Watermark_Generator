#include <stdbool.h>

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#define USER_STRING "Hello World"
#define DEVICE_ID 42
#define LOCATION "63.4190,10.4015"
#define TEMPERATURE 20

#define INCLUDE_USER_STRING true
#define INCLUDE_DEVICE_ID true
#define INCLUDE_LOCATION true
#define INCLUDE_TEMPERATURE true
#define INCLUDE_TIME true

#define USE_CABLE_TRANSMISSION false

// Starting points — algorithm adjusts to nearest valid sample-aligned frequency
#define FSK_LOWER_FREQUENCY 10000
#define FSK_HIGHER_FREQUENCY 12000

#define SIGNAL_ATTENUATION 100 // 0=silence, 100=full amplitude

// Adds RS parity symbols; improves noise robustness at the cost of data rate
#define USE_REED_SOLOMON_ERROR_CORRECTION true
#define RS_ERROR_CORRECTION_SYMBOLS 20

#define OPERATION_MODE 0 // 0=RX, 1=TX, 2=Standalone

#define STARTING_HOUR 0
#define STARTING_MINUTE 0
#define END_HOUR 23
#define END_MINUTE 59

// Active window per interval: device runs for RUN_MINUTES then sleeps
#define RUN_MINUTES 3
#define SLEEP_MINUTES 0

#define LUT_LOW_SAMPLES (1920000u / FSK_LOWER_FREQUENCY)
#define LUT_HIGH_SAMPLES (1920000u / FSK_HIGHER_FREQUENCY)

#endif // USER_CONFIG_H