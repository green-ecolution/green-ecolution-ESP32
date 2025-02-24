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

// simple to decode
/*void addFixedLengthStringToPayload(uint8_t* appData, uint8_t& appDataSize, const String& str, uint8_t maxLength, char filler = '\0') {
  // Ensure the string is not longer than the max length
  uint8_t length = min(str.length(), maxLength);

  // Add the string bytes
  for (uint8_t i = 0; i < length; i++) {
    appData[appDataSize++] = str.charAt(i);
  }

  // Pad the remaining bytes with the filler character
  for (uint8_t i = length; i < maxLength; i++) {
    appData[appDataSize++] = filler;
  }
}
*/

// allows for variable string length to be added and decoded correctly
void addCharArrayWithLengthToPayload(uint8_t* appData, uint8_t& appDataSize, const char* str) {
  // Calculate the length of the string
  uint8_t length = 0;
  while (str[length] != '\0') {
    length++;
  }

  // Add the length as the first byte
  appData[appDataSize++] = length;

  // Add the string bytes
  for (uint8_t i = 0; i < length; i++) {
    appData[appDataSize++] = str[i];
  }
}
