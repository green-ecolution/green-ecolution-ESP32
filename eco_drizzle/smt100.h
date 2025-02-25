#ifndef SMT100_H
#define SMT100_H

#include <Arduino.h>

// connections are symmetric
#define SMT100RXD 47  // Connected to RX pin of RS485 module
#define SMT100TXD 48  // Connected to TX pin of RS485 module

// Function declarations
void initSMT100();                                 // Initialize Serial1 for SMT100
bool getSMT100Temperature(float& temperature);     // Get temperature from SMT100
bool getSMT100MWaterContent(float& waterContent);  // Get moisture from SMT100

#endif
