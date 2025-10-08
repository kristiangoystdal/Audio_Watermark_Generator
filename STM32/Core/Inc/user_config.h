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

// Configuration for which minute in the hour to start transmissions (0-59)
#define ENABLE_DELAYED_START false
#define STARTING_MINUTE 0

// Configuration for the interval between repeats
#define USE_DEFAULT_INTERVAL_BETWEEN_REPEATS true // Default is 60 seconds
#define INTERVAL_BETWEEN_REPEATS_MINUTES 10

// Configuration for values to include in the watermark
#define USER_STRING "Halla"
#define DEVICE_ID 21
#define LOCATION "-107.7749,-122.4194"
#define TEMPERATURE 20

// Configuration for toggles of values to include in the watermark
#define INCLUDE_USER_STRING true
#define INCLUDE_DEVICE_ID true
#define INCLUDE_LOCATION true
#define INCLUDE_TEMPERATURE true
#define INCLUDE_TIME true

// Configuration for FSK frequency pair
// Options:
// 1 = 20.8kHz and 22.2kHz (Default)
// 2 = 12.5kHz and 11.1kHz
// 3 = 8.3kHz and 6.9kHz
// 4 = 2.8kHz and 1.4kHz
#define FSK_FREQUENCY_PAIR 1

#endif // USER_CONFIG_H