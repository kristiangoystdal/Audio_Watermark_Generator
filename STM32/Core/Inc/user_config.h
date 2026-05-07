#include <stdbool.h>

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

//------------------ USER CONFIGURATION ------------------//
// Modify these values as needed
//-------------------------------------------------------//

// Configuration for first time booting up the RTC
#define SET_INITIAL_TIME true
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
#define FSK_LOWER_FREQUENCY 18113
#define FSK_HIGHER_FREQUENCY 20000

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
// signals), TX mode (transmitting signals) or Standalone mode (run on internal
// timer without radio)
#define OPERATION_MODE 0 // 0=RX, 1=TX, 2=Standalone

// Confirguration for active window timing
#define STARTING_HOUR 0
#define STARTING_MINUTE 0
#define END_HOUR 23
#define END_MINUTE 59

// Configuration for how long the device should stay active during each active
// minute before going back to sleep, in minutes
#define RUN_MINUTES 5
#define SLEEP_MINUTES 55

#define LUT_LOW_SAMPLES (1920000u / FSK_LOWER_FREQUENCY)
#define LUT_HIGH_SAMPLES (1920000u / FSK_HIGHER_FREQUENCY)

#endif // USER_CONFIG_H