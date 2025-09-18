#ifndef USER_CONFIG_H
#define USER_CONFIG_H

//------------------ USER CONFIGURATION ------------------//
// Modify these values as needed
// USER_STRING: The string to be encoded into the audio signal
// INTERVAL_BETWEEN_REPEATS_SECONDS: Time interval between transmissions
// USER_ID: Unique identifier for the device
// LOCATION: Location description of the device
// ALTITUDE: Altitude of the device in meters
// TEMPERATURE: Temperature in Celsius
// HUMIDITY: Humidity in Percentage
// PRESSURE: Atmospheric pressure in hPa

#define USER_STRING "Hello World!"

#define USE_DEFAULT_INTERVAL_BETWEEN_REPEATS false
#define USE_MINUTES_INSTEAD_OF_SECONDS true
#define INTERVAL_BETWEEN_REPEATS_SECONDS 5
#define INTERVAL_BETWEEN_REPEATS_MINUTES 7

#define USER_ID 12345

#define LOCATION "Living Room"
#define ALTITUDE 150.0 // in meters

#define TEMPERATURE 22.5 // in Celsius
#define HUMIDITY 45.0    // in Percentage
#define PRESSURE 1013.25 // in hPa

//-------------------------------------------------------//

#endif // USER_CONFIG_H