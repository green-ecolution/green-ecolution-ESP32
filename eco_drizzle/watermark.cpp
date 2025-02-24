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

// Read and process Watermark sensor values
void getWatermarkValues(float temperature, float& WM1_Resistance, float& WM2_Resistance, float& WM3_Resistance, int& WM1_CB, int& WM2_CB, int& WM3_CB) {
  //Serial.print("Watermark correction temperature: ");
  //Serial.println(temperature);
  // Enable the MUX
  digitalWrite(Mux, LOW);

  // Read the first Watermark sensor
  delay(100);  // Wait for stabilization
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  delay(10);  // Wait for MUX
  WM1_Resistance = readWMsensor();

  // Read the second Watermark sensor
  delay(100);  // Wait for stabilization
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  delay(10);  // Wait for MUX
  WM2_Resistance = readWMsensor();

  // Read the third Watermark sensor
  delay(100);  // Wait for stabilization
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  delay(10);  // Wait for MUX
  WM3_Resistance = readWMsensor();

  // Disable the MUX
  digitalWrite(Mux, HIGH);

  // Convert resistance to centibars/kPa
  WM1_CB = myCBvalue(WM1_Resistance, temperature, cFactor);
  WM2_CB = myCBvalue(WM2_Resistance, temperature, cFactor);
  WM3_CB = myCBvalue(WM3_Resistance, temperature, cFactor);
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

    // Debug: Print raw analog values
    //Serial.printf("Raw A1: %d, Raw A2: %d\n", rawValue1, rawValue2);
  }

  // Calculate average voltage
  float SenVWM1 = ((ARead_A1 / 4096.0) * SupplyV) / num_of_read;
  float SenVWM2 = ((ARead_A2 / 4096.0) * SupplyV) / num_of_read;

  // Debug: Print intermediate voltages
  //Serial.printf("SenVWM1: %.2f V, SenVWM2: %.2f V\n", SenVWM1, SenVWM2);

  // Calculate resistance
  float WM_ResistanceA = (Rx * (SupplyV - SenVWM1)) / SenVWM1;
  float WM_ResistanceB = (Rx * SenVWM2) / (SupplyV - SenVWM2);
  float WM_Resistance = (WM_ResistanceA + WM_ResistanceB) / 2.0;

  // Debug: Print calculated resistances
  //Serial.printf("WM_ResistanceA: %.2f Ohms, WM_ResistanceB: %.2f Ohms, WM_Resistance: %.2f Ohms\n", WM_ResistanceA, WM_ResistanceB, WM_Resistance);

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