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

// Configuration for which minute in the hour to start transmissions (0-59)
#define ENABLE_DELAYED_START false
#define STARTING_MINUTE 0

// Configuration for the interval between repeats
#define USE_DEFAULT_INTERVAL_BETWEEN_REPEATS true // Default is 60 seconds
#define INTERVAL_BETWEEN_REPEATS_MINUTES 10

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

#define FSK_LOWER_FREQUENCY 1234  // in Hz
#define FSK_HIGHER_FREQUENCY 8442 // in Hz

#define FSK_LOWER_FREQUENCY 20884
#define FSK_HIGHER_FREQUENCY 22222

#endif // USER_CONFIG_H