#include "helpers.h"

// Define helper functions

void addFloatToPayload(uint8_t* appData, uint8_t& appDataSize, float value, uint32_t scaleFactor) {
  if (scaleFactor > 0) {
    // Scale the float and encode it as a 4-byte integer
    uint32_t scaledValue = value * scaleFactor;
    for (int i = 0; i < 4; i++) {
      appData[appDataSize++] = (scaledValue >> (8 * (3 - i))) & 0xFF;
    }
  } else {
    // Encode the float directly as 4 bytes
    union {
      float floatValue;
      uint8_t bytes[4];
    } converter;
    converter.floatValue = value;
    for (int i = 0; i < 4; i++) {
      appData[appDataSize++] = converter.bytes[i];
    }
  }
}

void addLongToPayload(uint8_t* appData, uint8_t& appDataSize, uint32_t value) {
  // Extract bytes from the long value
  for (int i = 0; i < 4; i++) {
    appData[appDataSize++] = (value >> (8 * (3 - i))) & 0xFF;
  }
}


void addIntToPayload(uint8_t* appData, uint8_t& appDataSize, uint16_t value) {
  appData[appDataSize++] = (value >> 8) & 0xFF;  // High byte
  appData[appDataSize++] = value & 0xFF;         // Low byte
}
