#ifndef USER_CONFIG_H
#define USER_CONFIG_H

//------------------ USER CONFIGURATION ------------------//
// Modify these values as needed
//-------------------------------------------------------//

// Configuration for first time booting up the RTC
#define SET_INITIAL_TIME false // Set to true to set the initial time
#define INITIAL_HOUR 15
#define INITIAL_MIN 0
#define INITIAL_SEC 0
#define INITIAL_DOW 2  // Day of week (1=Mon ... 7=Sun)
#define INITIAL_DOM 30    // Day of month (1–31)
#define INITIAL_MONTH 9  // Month (1–12)
#define INITIAL_YEAR 2025 // Year (e.g. 2024)

// Configuration for which minute in the hour to start transmissions (0-59)
#define INITIAL_MINUTE 0 

// Configuration for the interval between repeats
#define USE_DEFAULT_INTERVAL_BETWEEN_REPEATS true // Default is 60 seconds
#define USE_MINUTES_INSTEAD_OF_SECONDS true
#define INTERVAL_BETWEEN_REPEATS_MINUTES 10

// Configuration for values to include in the watermark
#define USER_STRING "Halla balla, artig skalla"
#define DEVICE_ID 42
#define LOCATION "-107.7749,-122.4194" // Longitude,Latitude format
#define TEMPERATURE 20

// Configuration for toggles of values to include in the watermark
#define INCLUDE_USER_STRING true
#define INCLUDE_DEVICE_ID true
#define INCLUDE_LOCATION true
#define INCLUDE_TEMPERATURE true
#define INCLUDE_TIME true // Current time from RTC

// Configuration for FSK frequency pair
// Options:
// 1 = 21kHz and 22kHz (Default)
// 2 = 12.5kHz and 11.1kHz
// 3 = 8.3kHz and 6.9kHz
// 4 = 2.8kHz and 1.4kHz
#define FSK_FREQUENCY_PAIR 1

#endif // USER_CONFIG_H