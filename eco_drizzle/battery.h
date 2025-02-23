// battery.h
#ifndef BATTERY_H
#define BATTERY_H

#include "Arduino.h"

// Default pin definitions
#ifndef VBAT_Read
#define VBAT_Read 1  // Default pin for reading battery voltage
#endif

#ifndef ADC_Ctrl
#define ADC_Ctrl 37  // Default pin for controlling the ADC
#endif

// Function declarations
void initBattery();
float readBatteryVoltage();

#endif
