#ifndef SMT100_H
#define SMT100_H

#include <Arduino.h>

// Function declarations
void initSMT100(int rxPin, int txPin);             // Initialize Serial1 for SMT100
bool getSMT100Temperature(float& temperature);     // Get temperature from SMT100
bool getSMT100MWaterContent(float& waterContent);  // Get moisture from SMT100

#endif