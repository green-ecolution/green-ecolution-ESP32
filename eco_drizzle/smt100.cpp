#include "smt100.h"

// Helper function to remove non-numeric characters from a string
String removeNonNumericCharacters(String str) {
  String result = "";
  for (int i = 0; i < str.length(); i++) {
    if (isdigit(str[i]) || str[i] == '-' || str[i] == '.') {
      result += str[i];
    }
  }
  return result;
}
// Initialize Serial1 for SMT100 communication
void initSMT100(int rxPin, int txPin) {
  Serial1.begin(9600, SERIAL_8N1, rxPin, txPin);  // Initialize Serial1 with custom pins
}

// Get temperature from SMT100
bool getSMT100Temperature(float& temperature) {
  // Send "GetTemperature!" command to the SMT100 sensor
  Serial1.print("GetTemperature!000000\r\n");

  // Wait for a response (adjust delay based on sensor response time)
  delay(100);

  // Check if data is available
  if (Serial1.available()) {
    String response = Serial1.readStringUntil('\n');           // Read response until newline
    String cleanedStr = removeNonNumericCharacters(response);  // Clean the response
    temperature = cleanedStr.toFloat();                        // Convert to float
    return true;                                               // Success
  }

  return false;  // No data received
}

// Get moisture from SMT100
bool getSMT100MWaterContent(float& waterContent) {
  // Send "GetWaterContent!" command to the SMT100 sensor
  Serial1.print("GetWaterContent!000000\r\n");

  // Wait for a response (adjust delay based on sensor response time)
  delay(100);

  // Check if data is available
  if (Serial1.available()) {
    String response = Serial1.readStringUntil('\n');           // Read response until newline
    String cleanedStr = removeNonNumericCharacters(response);  // Clean the response
    waterContent = cleanedStr.toFloat();                       // Convert to float
    return true;                                               // Success
  }

  return false;  // No data received
}
