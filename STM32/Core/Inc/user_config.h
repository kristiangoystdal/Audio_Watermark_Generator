#include <stdbool.h>

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

//------------------ USER CONFIGURATION ------------------//
// Modify these values as needed
//-------------------------------------------------------//

// Configuration for first time booting up the RTC
#define SET_INITIAL_TIME false
#define INITIAL_HOUR 12
#define INITIAL_MIN 54
#define INITIAL_SEC 12
#define INITIAL_DOW 3
#define INITIAL_DOM 8
#define INITIAL_MONTH 10
#define INITIAL_YEAR 2025

// Configuration for values to include in the watermark
#define USER_STRING "Hello World"
#define DEVICE_ID 42
#define LOCATION "63.4190,10.4015"
#define TEMPERATURE 20

// Configuration for toggles of values to include in the watermark
#define INCLUDE_USER_STRING true
#define INCLUDE_DEVICE_ID true
#define INCLUDE_LOCATION true
#define INCLUDE_TEMPERATURE true
#define INCLUDE_TIME true

// Configuration for transmission method
#define USE_CABLE_TRANSMISSION true
#define USE_SPEAKER_TRANSMISSION true

// Configuration for FSK frequencies (in Hz) - these will be adjusted by the
// algorithm if they don't meet the sample count requirements, but should be set
// close to the desired frequencies to minimize adjustments
#define FSK_LOWER_FREQUENCY 18461
#define FSK_HIGHER_FREQUENCY 19200

// Configuration for attenuation of the signal in percent (0-100), where 100 is
// no attenuation and 0 is maximum attenuation (silence)
#define SIGNAL_ATTENUATION 100

// Configuration for whether to use Reed-Solomon error correction coding, which
// adds redundancy to the transmitted data to allow for error detection and
// correction at the receiver. This can improve reliability in noisy
// environments but reduces the effective data rate due to the added redundancy.
#define USE_REED_SOLOMON_ERROR_CORRECTION true
#define RS_ERROR_CORRECTION_SYMBOLS 20

// Configuration for whether to start in RX mode (listening for incoming
// signals) or TX mode (transmitting signals)
#define RX_MODE true

// Configuration for base station transmission intervals and timing
// If delayed start is enabled, the device will wait until the specified
// starting time is reached
#define ENABLE_DELAYED_START false
#define STARTING_MINUTE 0

// Configuration for the interval between repeats
#define USE_DEFAULT_INTERVAL_BETWEEN_REPEATS true // Default is 60 seconds
#define INTERVAL_BETWEEN_REPEATS_MINUTES 10

#define FSK_LOWER_FREQUENCY 20884
#define FSK_HIGHER_FREQUENCY 22222

#define FSK_LOWER_FREQUENCY 20884
#define FSK_HIGHER_FREQUENCY 22222

#endif // USER_CONFIG_H