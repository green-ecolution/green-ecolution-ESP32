#include "battery.h"

// Initialize the battery voltage reading setup
void initBattery() {
  pinMode(ADC_Ctrl, OUTPUT);  // Set ADC control pin as output
  pinMode(VBAT_Read, INPUT);  // Set battery voltage pin as input
}

// Read the battery voltage in millivolts
float readBatteryVoltage() {
  // ADC configuration
  const int resolution = 12;                  // ADC resolution (12 bits)
  const int adcMax = pow(2, resolution) - 1;  // Maximum ADC value (4095 for 12 bits)
  const float adcMaxVoltage = 3.3;            // Maximum ADC voltage (3.3V on ESP32)

  // On-board voltage divider configuration
  const int R1 = 390;  // Resistor R1 in the voltage divider
  const int R2 = 100;  // Resistor R2 in the voltage divider

  // Calibration measurements
  const float measuredVoltage = 4.2;    // Measured voltage with a multimeter
  const float reportedVoltage = 4.095;  // Reported voltage by the ADC

  // Calibration factor
  const float factor = (adcMaxVoltage / adcMax) * ((R1 + R2) / (float)R2) * (measuredVoltage / reportedVoltage);

  // Read the analog value
  digitalWrite(ADC_Ctrl, LOW);              // Enable the ADC
  delay(100);                               // Wait for stabilization
  int analogValue = analogRead(VBAT_Read);  // Read the raw ADC value
  digitalWrite(ADC_Ctrl, HIGH);             // Disable the ADC

  // Calculate the battery voltage
  float floatVoltage = factor * analogValue;  // Apply the calibration factor
  //uint16_t voltage = (int)(floatVoltage * 1000.0);  // Convert to millivolts

  return floatVoltage;  // Return the battery voltage in millivolts
}
