#ifndef USER_CONFIG_H
#define USER_CONFIG_H

//------------------ USER CONFIGURATION ------------------//
// Modify these values as needed
//-------------------------------------------------------//

// Configuration for the interval between repeats
#define USE_DEFAULT_INTERVAL_BETWEEN_REPEATS true // Default is 60 seconds
#define USE_MINUTES_INSTEAD_OF_SECONDS true
#define INTERVAL_BETWEEN_REPEATS_SECONDS 5
#define INTERVAL_BETWEEN_REPEATS_MINUTES 10

// Configuration for values to include in the watermark
#define USER_STRING "Einar er kul og liker Limp Bizkit. Han er skalla"
#define DEVICE_ID 42
#define LOCATION "-107.7749,-122.4194" // Longitude,Latitude format
#define TEMPERATURE 20

// Configuration for toggles of values to include in the watermark
#define INCLUDE_USER_STRING false
#define INCLUDE_DEVICE_ID false
#define INCLUDE_LOCATION true
#define INCLUDE_TEMPERATURE true

//-------------------------------------------------------//

#endif // USER_CONFIG_H