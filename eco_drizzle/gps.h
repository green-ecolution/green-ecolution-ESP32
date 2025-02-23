#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include "HT_TinyGPS++.h"

// Constants
const unsigned long GPS_DATA_TIMEOUT = 10 * 1000;     // 10 seconds
const unsigned long GPS_FIX_TIMEOUT = 5 * 60 * 1000;  // 5 minutes
const unsigned long GPS_CHECK_INTERVAL = 200;         // 200 ms
const uint32_t GPSBaud = 9600;

// GPS object
extern TinyGPSPlus gps;

// Function declarations
void initGPS(int rxPin, int txPin);  // Accept RX and TX pins as parameters
bool getGPSSignal(float& latitude, float& longitude, uint32_t& timeTaken);

#endif
