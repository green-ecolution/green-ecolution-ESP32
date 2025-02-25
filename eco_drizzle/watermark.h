#ifndef WATERMARK_H
#define WATERMARK_H

#include <Arduino.h>

// Pin definitions
#define S0 5       // MUX control pin S0
#define S1 6       // MUX control pin S1
#define Sensor1 4  // Sensor 1 excitation pin
#define Sensor2 2  // Sensor 2 excitation pin
#define Mux 7      // MUX enable/disable pin
#define Sense 3    // Analog input pin for reading sensor values

// Sensor tuples (S0, S1)
const uint8_t WM_30[] = { LOW, HIGH };  // S1 = LOW, S0 = HIGH // W1
const uint8_t WM_60[] = { HIGH, LOW };  // S1 = HIGH, S0 = LOW // W2
const uint8_t WM_90[] = { LOW, LOW };   // S1 = LOW, S0 = LOW // W3

const int Rx = 10000;  // fixed resistor attached in series to the sensor and ground...the same value repeated for all WM and Temp Sensor.
const long default_Mositure = 50;
const long open_resistance = 35000;  // check the open resistance value by replacing sensor with an open and replace the value here...this value might vary slightly with circuit components
const long short_resistance = 200;   // similarly check short resistance by shorting the sensor terminals and replace the value here.
const long short_CB = 240, open_CB = 255;
const int SupplyV = 5.0;     // Assuming 5V output for SupplyV, this can be measured and replaced with an exact value if required
const float cFactor = 1.1;   // correction factor optional for adjusting curves. Traditionally IRROMETER devices used a different reading method, voltage divider circuits often require this adjustment to match exactly.
const int num_of_read = 10;  // number of iterations, each is actually two reads of the sensor (both directions)

// Function declarations
void initWatermark();                                                                                                                                                // Initialize the Watermark sensor setup
void getWatermarkValues(float temperature, float& WM_30_Resistance, float& WM_60_Resistance, float& WM_90_Resistance, int& WM_30_CB, int& WM_60_CB, int& WM_90_CB);  // Read and process Watermark sensor values
float readWMsensor();                                                                                                                                                // Read the resistance of a Watermark sensor
int myCBvalue(int res, float TC, float cF);                                                                                                                          // Convert resistance to centibars/kPa
void setMuxPins(const uint8_t sensorConfig[]);
#endif
