#ifndef HELPERS_H
#define HELPERS_H

#include <Arduino.h>

// Declare helper functions
void addFloatToPayload(uint8_t* appData, uint8_t& appDataSize, float value, uint32_t scaleFactor = 0);
void addLongToPayload(uint8_t* appData, uint8_t& appDataSize, uint32_t value);
void addIntToPayload(uint8_t* appData, uint8_t& appDataSize, uint16_t value);
#endif
