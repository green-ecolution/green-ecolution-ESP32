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



const int Rx = 10000;           // fixed resistor attached in series to the sensor and ground...the same value repeated for all WM and Temp Sensor.
const long default_TempC = 10;  // right now fixed, but can be changed, if temperaturesensor is connected
const long default_Mositure = 50;
const long open_resistance = 35000;  // check the open resistance value by replacing sensor with an open and replace the value here...this value might vary slightly with circuit components
const long short_resistance = 200;   // similarly check short resistance by shorting the sensor terminals and replace the value here.
const long short_CB = 240, open_CB = 255;
const int SupplyV = 3.3;    // Assuming 5V output for SupplyV, this can be measured and replaced with an exact value if required
const float cFactor = 1.1;  // correction factor optional for adjusting curves. Traditionally IRROMETER devices used a different reading method, voltage divider circuits often require this adjustment to match exactly.
const int num_of_read = 1;  // number of iterations, each is actually two reads of the sensor (both directions)

// Function declarations
void initWatermark();                                                                                                                 // Initialize the Watermark sensor setup
void getWatermarkValues(float& WM1_Resistance, float& WM2_Resistance, float& WM3_Resistance, int& WM1_CB, int& WM2_CB, int& WM3_CB);  // Read and process Watermark sensor values
float readWMsensor();                                                                                                                 // Read the resistance of a Watermark sensor
int myCBvalue(int res, float TC, float cF);                                                                                           // Convert resistance to centibars/kPa

#endif
