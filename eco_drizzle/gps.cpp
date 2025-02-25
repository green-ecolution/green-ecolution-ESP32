#include "gps.h"

// Initialize the GPS object
TinyGPSPlus gps;

// Initialize GPS module with specified RX and TX pins
void initGPS() {
  Serial2.begin(GPSBaud, SERIAL_8N1, GPSRXD, GPSTXD);  // Initialize Serial2 with custom pins
}

// Get GPS signal
bool getGPSSignal(float& latitude, float& longitude, uint32_t& timeTaken) {
  uint32_t startTime = millis();  // Record the start time
  bool validDataReceived = false;
  uint32_t lastCheckTime = 0;  // Flag to check if valid NMEA data is received

  while (millis() - startTime <= GPS_FIX_TIMEOUT) {
    // Non-blocking delay for periodic checks
    uint32_t currentTime = millis();
    if (currentTime - lastCheckTime >= GPS_CHECK_INTERVAL) {
      lastCheckTime = currentTime;  // Update the last check time

      // Check if there is data available from the GPS module
      while (Serial2.available() > 0) {
        char data = Serial2.read();
        // Print the raw NMEA data to the Serial Monitor
        // Serial.write(data);
        // Feed the data to TinyGPS++
        if (gps.encode(data)) {
          validDataReceived = true;  // Set the flag to true if valid NMEA data is received
        }
      }
    }

    // Check if a valid fix has been acquired
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      timeTaken = millis() - startTime;
      return true;  // Valid GPS signal found
    }

    // Check if no valid data has been received within the data timeout period
    if (!validDataReceived && millis() - startTime > GPS_DATA_TIMEOUT) {
      Serial.println("No valid data from GPS-module received. Check GPS-module connection. Not signal, the actual module!");
      return false;  // No valid GPS data received
    }
  }

  // Timeout, no valid signal found
  timeTaken = millis() - startTime;
  Serial.println("GPS fix timeout. No valid signal found.");
  return false;
}
