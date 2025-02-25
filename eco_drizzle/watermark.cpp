#include "watermark.h"

// Initialize the Watermark sensor setup
void initWatermark() {
  // Set pin modes
  pinMode(S0, OUTPUT);       // MUX control pin S0
  pinMode(S1, OUTPUT);       // MUX control pin S1
  pinMode(Sensor1, OUTPUT);  // Sensor 1 excitation pin
  pinMode(Sensor2, OUTPUT);  // Sensor 2 excitation pin
  pinMode(Mux, OUTPUT);      // MUX enable/disable pin
  pinMode(Sense, INPUT);     // Analog input pin for reading sensor values

  // Disable MUX initially
  digitalWrite(Mux, HIGH);
}

void setMuxPins(const uint8_t sensorConfig[]) {
  digitalWrite(S1, sensorConfig[0]);  // Set S0
  digitalWrite(S0, sensorConfig[1]);  // Set S1
}

void getWatermarkValues(float temperature, float& WM_30_Resistance, float& WM_60_Resistance, float& WM_90_Resistance, int& WM_30_CB, int& WM_60_CB, int& WM_90_CB) {
  // Enable the MUX
  digitalWrite(Mux, LOW);

  // Read the WM_30 sensor
  delay(100);         // Wait for stabilization
  setMuxPins(WM_30);  // Set MUX pins for WM_30
  delay(10);          // Wait for MUX
  WM_30_Resistance = readWMsensor();

  // Read the WM_60 sensor
  delay(100);         // Wait for stabilization
  setMuxPins(WM_60);  // Set MUX pins for WM_60
  delay(10);          // Wait for MUX
  WM_60_Resistance = readWMsensor();

  // Read the WM_90 sensor
  delay(100);         // Wait for stabilization
  setMuxPins(WM_90);  // Set MUX pins for WM_90
  delay(10);          // Wait for MUX
  WM_90_Resistance = readWMsensor();

  // Disable the MUX
  digitalWrite(Mux, HIGH);

  // Convert resistance to centibars/kPa
  WM_30_CB = myCBvalue(WM_30_Resistance, temperature, cFactor);
  WM_60_CB = myCBvalue(WM_60_Resistance, temperature, cFactor);
  WM_90_CB = myCBvalue(WM_90_Resistance, temperature, cFactor);
}

// Read the resistance of a Watermark sensor
float readWMsensor() {
  int ARead_A1 = 0, ARead_A2 = 0;

  for (int i = 0; i < num_of_read; i++) {
    // First polarity
    digitalWrite(Sensor1, HIGH);
    delayMicroseconds(90);
    int rawValue1 = analogRead(Sense);
    ARead_A1 += rawValue1;
    digitalWrite(Sensor1, LOW);

    delay(100);  // Wait before switching polarity

    // Second polarity
    digitalWrite(Sensor2, HIGH);
    delayMicroseconds(90);
    int rawValue2 = analogRead(Sense);
    ARead_A2 += rawValue2;
    digitalWrite(Sensor2, LOW);
  }

  // Calculate average voltage
  float SenVWM1 = ((ARead_A1 / 4096.0) * SupplyV) / num_of_read;
  float SenVWM2 = ((ARead_A2 / 4096.0) * SupplyV) / num_of_read;

  // Calculate resistance
  float WM_ResistanceA = (Rx * (SupplyV - SenVWM1)) / SenVWM1;
  float WM_ResistanceB = (Rx * SenVWM2) / (SupplyV - SenVWM2);
  float WM_Resistance = (WM_ResistanceA + WM_ResistanceB) / 2.0;

  return WM_Resistance;
}

// Convert resistance to centibars/kPa
int myCBvalue(int res, float TC, float cF) {
  int WM_CB;
  float resK = res / 1000.0;
  float tempD = 1.00 + 0.018 * (TC - 24.00);

  if (res > 550.00) {
    if (res > 8000.00) {
      WM_CB = (-2.246 - 5.239 * resK * tempD - 0.06756 * resK * resK * (tempD * tempD)) * cF;
    } else if (res > 1000.00) {
      WM_CB = (-3.213 * resK - 4.093) / (1 - 0.009733 * resK - 0.01205 * TC) * cF;
    } else {
      WM_CB = (resK * 23.156 - 12.736) * tempD;
    }
  } else {
    if (res > 300.00) {
      WM_CB = 0.00;
    } else if (res < 300.00 && res >= short_resistance) {
      WM_CB = short_CB;  // Sensor short
    }
  }

  if (res >= open_resistance || res == 0) {
    WM_CB = open_CB;  // Open circuit
  }

  return WM_CB;
}
