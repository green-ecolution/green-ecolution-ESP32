#include "LoRaWan_APP.h"
#include <Arduino.h>
#include "lora/LoRa.h"
#include "heltec.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"
#include <math.h>
#include <string.h>
#include "HT_TinyGPS++.h"

#define VBAT_Read 1   // pin for Battery
#define	ADC_Ctrl 37
#define num_of_read 1 // number of iterations, each is actually two reads of the sensor (both directions)
#define RXD2 48       // rx-pin for SMT-100 Sensor (needs to be changed if other board is used)
#define TXD2 47       // tx-pin for SMT-100 Sensor (needs to be changed if other board is used)
#define GPSTXD 33     // connected to rx-pin of GPS module
#define GPSRXD 34     // connected to tx-pin of GPS module

// add your TTN-Credentials here
/* OTAA para */
uint8_t devEui[] = { };
uint8_t appEui[] = { };
uint8_t appKey[] = { };

const char* deviceName = "placeholder";

/* ABP para */
uint8_t nwkSKey[] = { };
uint8_t appSKey[] = { };
uint32_t devAddr = (uint32_t)0x00000000;

/* LoraWan channelsmask, default channels 0-7 */ 
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/* LoraWan region, select in Arduino IDE tools */
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/* LoraWan Class, Class A and Class C are supported */
DeviceClass_t loraWanClass = CLASS_A;

/* the application data transmission duty cycle.  value in [ms]. */
uint32_t appTxDutyCycle = 60*1000; //3600000

/* OTAA or ABP */
bool overTheAirActivation = true;

/* ADR enable */
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

extern uint8_t appDataSize;
extern uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];

/* Number of trials to transmit the frame */
uint8_t confirmedNbTrials = 4;

const float BATTERY_THRESHOLD = 3.0;

const int Rx = 10000;  // fixed resistor attached in series to the sensor and ground...the same value repeated for all WM and Temp Sensor.
const long default_TempC = 10; // right now fixed, but can be changed, if temperaturesensor is connected 
const long default_Mositure = 50;
const long open_resistance = 35000; // check the open resistance value by replacing sensor with an open and replace the value here...this value might vary slightly with circuit components
const long short_resistance = 200; // similarly check short resistance by shorting the sensor terminals and replace the value here.
const long short_CB = 240, open_CB = 255 ;
const int SupplyV = 3.3; // Assuming 5V output for SupplyV, this can be measured and replaced with an exact value if required
const float cFactor = 1.1; // correction factor optional for adjusting curves. Traditionally IRROMETER devices used a different reading method, voltage divider circuits often require this adjustment to match exactly.
int i, j = 0, WM1_CB = 0, WM2_CB = 0, WM3_CB = 0;
float SenV10K = 0, SenVTempC = 0, SenVWM1 = 0, SenVWM2 = 0, ARead_A1 = 0, ARead_A2 = 0, WM3_Resistance = 0, WM2_Resistance = 0, WM1_Resistance = 0, TempC_Resistance = 0, TempC = 0, Moisture = 0;
String receivedSMT100TempData = "";
String receivedSMT100MoistData = "";
int castedSMT100TempData = 0;
int castedSMT100MoistData = 0;
const int S0 = 5;
const int S1 = 6;
const int Sensor1 = 4;
const int Sensor2 = 2;
const int Mux = 7;
const int Sense = 3;


// GPS
// Constants
const unsigned long GPS_DATA_TIMEOUT = 10 * 1000;  // 10 seconds
const unsigned long GPS_FIX_TIMEOUT = 1 * 60 * 1000;  // 5 minutes
const unsigned long GPS_CHECK_INTERVAL = 200;  // 200 ms
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
float latitude = 0.0;
float longitude = 0.0;
unsigned long timeTaken = 0;

bool getGPSSignal() {
  unsigned long startTime = millis();  // Record the start time
  bool validDataReceived = false;
  unsigned long lastCheckTime = 0;     // Flag to check if valid NMEA data is received

  while (millis() - startTime <= GPS_FIX_TIMEOUT) {
    // Non-blocking delay for periodic checks
    unsigned long currentTime = millis();
    if (currentTime - lastCheckTime >= GPS_CHECK_INTERVAL) {
      lastCheckTime = currentTime;  // Update the last check time

      // Check if there is data available from the GPS module
      while (Serial2.available() > 0) {
        char data = Serial2.read();
        // Print the raw NMEA data to the Serial Monitor
        Serial.write(data);
        // Feed the data to TinyGPS++
        if (gps.encode(data)) {
          validDataReceived = true;  // Set the flag to true if valid NMEA data is received
        }
      }
    }

    // Check if a valid fix has been acquired
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      timeTaken = millis() - startTime;
      return true;  // Valid GPS signal found
    }

    // Check if no valid data has been received within the data timeout period
    if (!validDataReceived && millis() - startTime > GPS_DATA_TIMEOUT) {
      Serial.println("No valid data from GPS-module received. Check GPS-module connection. Not signal, the actual module!");
      return false;  // No valid GPS data received
    }
  }

  // Timeout, no valid signal found
  timeTaken = millis() - startTime;
  Serial.println("GPS fix timeout. No valid signal found.");
  return false;
}

void getWatermarkValues() {
    while (j == 0){
    // enable the MUX
    digitalWrite(Mux, LOW);

    // read the first Watermark sensor
    delay(100); // 0.1 second wait before moving to next channel or switching MUX
    // address the MUX
    digitalWrite(S0, LOW);
    digitalWrite(S1, HIGH);

    delay(10); // wait for the MUX

    WM1_Resistance = readWMsensor();

    // read the second Watermark sensor
    delay(100); // 0.1 second wait before moving to next channel or switching MUX
    // address the MUX 
    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);

    delay(10); // wait for MUX

    WM2_Resistance = readWMsensor();

    // read the third Watermark sensor
    delay(100); // 0.1 second wait before moving to next channel or switching MUX
    // address the MUX
    digitalWrite(S0, LOW);
    digitalWrite(S1, LOW);

    delay(10); // wait for the MUX

    WM3_Resistance = readWMsensor();

    delay(100); // 0.1 second wait before moving to next channel or switching MUX


    digitalWrite(Mux, HIGH);   // disable MUX 0-1 pair
    delay(100); // 0.1 second wait before moving to next channel or switching MUX

    // convert the measured reistance to kPa/centibars of soil water tension
    WM1_CB = myCBvalue(WM1_Resistance, TempC, cFactor);
    WM2_CB = myCBvalue(WM2_Resistance, TempC, cFactor);
    WM3_CB = myCBvalue(WM3_Resistance, TempC, cFactor);

    //*****************output************************************
    Serial.print("WM1 Resistance(Ohms)= ");
    Serial.println(WM1_Resistance);
    Serial.print("WM2 Resistance(Ohms)= ");
    Serial.println(WM2_Resistance);
    Serial.print("WM3 Resistance(Ohms)= ");
    Serial.println(WM3_Resistance);
    Serial.print("WM1(cb/kPa)= ");
    Serial.println(abs(WM1_CB));
    Serial.print("WM2(cb/kPa)= ");
    Serial.println(abs(WM2_CB));
    Serial.print("WM3(cb/kPa)= ");
    Serial.println(abs(WM3_CB));
    Serial.print("\n");

    j=1;
  }
}

String removeNonNumericCharacters(String str) {
  String result = "";
  for(unsigned int i = 0; i < str.length(); i++) {
    if (isDigit(str[i])) {
      result += str[i]; // add only numeric characters to the result
    }
  }
  return result;
}

void getSMT100Temperature() {
  // send "GetTemperature!" command to the RS485 device via UART1
  Serial1.print("GetTemperature!000000\r\n");  // send ASCII command to the RS485 device (UART1)
  
  // wait for a response from the RS485 device
  delay(100);  // adjust delay based on your RS485 device's response time
  // check if the RS485 device has sent back a response
  if (Serial1.available()) {
    receivedSMT100TempData = Serial1.readStringUntil('\n');  // read response until newline
    String cleanedStr = removeNonNumericCharacters(receivedSMT100TempData);
    castedSMT100TempData = cleanedStr.toInt();
    TempC = castedSMT100TempData;
    Serial.println(castedSMT100TempData);
    Serial.println("Received from RS485: " + receivedSMT100TempData);  // output to Debug Serial Monitor (UART0)
  } else {
    Serial.println("No RS485 available. Default Temp of" + String(default_TempC) + "°C is used");
    TempC = default_TempC;
  }
}

void getSMT100Moisture(){
  Serial1.print("GetWaterContent!000000\r\n");
  delay(100);

  if (Serial1.available()) {
    receivedSMT100MoistData = Serial1.readStringUntil('\n');  // read response until newline
    String cleanedStr = removeNonNumericCharacters(receivedSMT100MoistData);
    castedSMT100MoistData = cleanedStr.toInt();
    Moisture = castedSMT100MoistData;
    Serial.println(castedSMT100MoistData);
    Serial.println("Received from RS485: " + receivedSMT100MoistData);  // output to Debug Serial Monitor (UART0)
  } else {
    Serial.println("No RS485 available. Default Moisture of" + String(default_Mositure) + "% is used");
    Moisture = default_Mositure;
  }
}

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
  // converts Moisturevalue to Percentage (0-100%)
  int moistureValue = 2;
  float moisturePercentage = map(moistureValue, 1500, 3250, 100, 0);
  int int_moisture = moisturePercentage * 10; // conversion to an integer and removal of the decimal place

  // read battery
  uint16_t int_battery = readBatteryVoltage();
  float batteryVoltage = int_battery / 1000.0;
  if(batteryVoltage < BATTERY_THRESHOLD){
    esp_deep_sleep_start();
  }

  
  // reading Temperature of SMT100
  getSMT100Temperature();
  getSMT100Moisture();

  // reading Sensorvalues of Watermarksensors
  getWatermarkValues();

  // Call the method to get GPS signal
  if (getGPSSignal()) {
    Serial.println("Valid GPS signal found!");
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
    Serial.print("Time taken: ");
    Serial.print(timeTaken / 1000);  // Convert milliseconds to seconds
    Serial.println(" seconds");
  } else {
    Serial.println("Failed to get valid GPS signal.");
  }
  long lat = latitude * 1000000;  // 37.7749 → 37774900
  long lng = longitude * 1000000;
  
  appDataSize = 30;

  appData[0] = (int)WM1_Resistance >> 8;
  appData[1] = (int)WM1_Resistance;
  appData[2] = abs(WM1_CB) >> 8;
  appData[3] = abs(WM1_CB);

  appData[4] = (int)WM2_Resistance >> 8;
  appData[5] = (int)WM2_Resistance;
  appData[6] = abs(WM2_CB) >> 8;
  appData[7] = abs(WM2_CB);

  appData[8] = (int)WM3_Resistance >> 8;
  appData[9] = (int)WM3_Resistance;
  appData[10] = abs(WM3_CB) >> 8;
  appData[11] = abs(WM3_CB);

  appData[12] = (int)castedSMT100TempData >> 8; 
  appData[13] = (int)castedSMT100TempData;

  appData[14] = int_battery >> 8;
  appData[15] = int_battery;

  appData[16] = (lat >> 24); 
  appData[17] = (lat >> 16); 
  appData[18] = (lat >> 8);  
  appData[19] = lat;         

  appData[20] = (lng >> 24); 
  appData[21] = (lng >> 16); 
  appData[22] = (lng >> 8);  
  appData[23] = lng;     

  appData[24] = (int) castedSMT100MoistData >> 8;
  appData[25] = (int) castedSMT100MoistData;

  appData[26] = (timeTaken >> 24) & 0xFF;  // Most significant byte
  appData[27] = (timeTaken >> 16) & 0xFF;
  appData[28] = (timeTaken >> 8) & 0xFF;
  appData[29] = timeTaken & 0xFF;
}

/* Read ADC and get resistance of sensor */
float readWMsensor() {  
  ARead_A1 = 0;
  ARead_A2 = 0;

  for (i = 0; i < num_of_read; i++) // the num_of_read initialized above, controls the number of read successive read loops that is averaged.
  {

    digitalWrite(Sensor1, HIGH);    // set pin 5 as Vs
    delayMicroseconds(90);          // wait 90 micro seconds and take sensor read
    ARead_A1 += analogRead(Sense);  // read the analog pin and add it to the running total for this direction
    digitalWrite(Sensor1, LOW);     // set the excitation voltage to OFF/LOW

    delay(100); // 0.1 second wait before moving to next channel or switching MUX

    // swap polarity, pin 5 is already low

    digitalWrite(Sensor2, HIGH);   // set pin 11 as Vs
    delayMicroseconds(90);         // wait 90 micro seconds and take sensor read
    ARead_A2 += analogRead(Sense); // read the analog pin and add it to the running total for this direction
    digitalWrite(Sensor2, LOW);    // set the excitation voltage to OFF/LOW
  }

  SenVWM1 = ((ARead_A1 / 4096) * SupplyV) / (num_of_read); // get the average of the readings in the first direction and convert to volts
  SenVWM2 = ((ARead_A2 / 4096) * SupplyV) / (num_of_read); // get the average of the readings in the second direction and convert to volts

  float WM_ResistanceA = (Rx * (SupplyV - SenVWM1) / SenVWM1);   // do the voltage divider math, using the Rx variable representing the known resistor
  float WM_ResistanceB = Rx * SenVWM2 / (SupplyV - SenVWM2);     // reverse
  float WM_Resistance = ((WM_ResistanceA + WM_ResistanceB) / 2); // average the two directions
  return WM_Resistance;
}

/* Conversion of ohms to CB */
int myCBvalue(int res, float TC, float cF) {
  int WM_CB;
  float resK = res / 1000.0;
  float tempD = 1.00 + 0.018 * (TC - 24.00);

  if (res > 550.00) { // if in the normal calibration range
    if (res > 8000.00) { // above 8k
      WM_CB = (-2.246 - 5.239 * resK * (1 + .018 * (TC - 24.00)) - .06756 * resK * resK * (tempD * tempD)) * cF;
    } else if (res > 1000.00) { // between 1k and 8k
      WM_CB = (-3.213 * resK - 4.093) / (1 - 0.009733 * resK - 0.01205 * (TC)) * cF ;
    } else { // below 1k
      WM_CB = (resK * 23.156 - 12.736) * tempD;
    }
  } else { // below normal range but above short (new, unconditioned sensors)
    if (res > 300.00)  {
      WM_CB = 0.00;
    }
    if (res < 300.00 && res >= short_resistance) { // wire short
      WM_CB = short_CB; // 240 is a fault code for sensor terminal short
      Serial.print("Sensor Short WM \n");
    }
  }
  if (res >= open_resistance || res==0) {

    WM_CB = open_CB; // 255 is a fault code for open circuit or sensor not present

  }
  return WM_CB;
}

/* Read Batteryvoltage for TTN to monitor Batterystatus */
uint16_t readBatteryVoltage() {
  // ADC resolution
  const int resolution = 12;
  const int adcMax = pow(2,resolution) - 1;
  const float adcMaxVoltage = 3.3;
  // on-board voltage divider
  const int R1 = 390;
  const int R2 = 100;
  // calibration measurements
  const float measuredVoltage = 4.2;
  const float reportedVoltage = 4.095;
  // calibration factor
  const float factor = (adcMaxVoltage / adcMax) * ((R1 + R2)/(float)R2) * (measuredVoltage / reportedVoltage); 
  digitalWrite(ADC_Ctrl,LOW);
  delay(100);
  int analogValue = analogRead(VBAT_Read);
  digitalWrite(ADC_Ctrl,HIGH);

  float floatVoltage = factor * analogValue;
  uint16_t voltage = (int)(floatVoltage * 1000.0);

  return voltage;
}

void setup() {
  // initialize UART1 (TX = GPIO17, RX = GPIO16) for RS485 communication
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);  // RS485 communication on UART1
  Serial2.begin(GPSBaud, SERIAL_8N1, GPSRXD, GPSTXD);
  Serial.begin(115200);
  VextON();
  delay(100);

  // print initial debug message to Serial Monitor (UART0)
  Serial.println("ESP32 RS485 communication started...");
  Serial.println("Serial Txd is on pin: "+String(TXD2));
  Serial.println("Serial Rxd is on pin: "+String(RXD2));
  VextON();

  pinMode(ADC_Ctrl,OUTPUT);
  pinMode(VBAT_Read,INPUT);

  // initialize the digital pins as outputs
  pinMode(S0, OUTPUT);      // used for S0 control of the MUX 0-1
  pinMode(S1, OUTPUT);      // used for S1 control of the MUX 0-1
  pinMode(Sensor1, OUTPUT); // sensor Vs or GND
  pinMode(Mux, OUTPUT);     // enable disable MUX 0-1
  pinMode(Sensor2, OUTPUT); // sensor Vs or GND
  analogReadResolution(12); // sets the resolution to 12 bit

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  Serial.println("Starting LoRaWAN...");

  // set the TX power to maximum (14 dBm)
  LoRa.setTxPowerMax(14);
  LoRa.setSpreadingFactor(12);
}

void VextON(void) {
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

// Vext default OFF
void VextOFF(void) {
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      #if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
      #endif
      Serial.println("Initializing LoRaWAN...");
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      deviceState = DEVICE_STATE_JOIN;
      break;
    case DEVICE_STATE_JOIN:
      Serial.println("Joining LoRaWAN network...");
      LoRaWAN.join();
      deviceState = DEVICE_STATE_SEND;
      break;
    case DEVICE_STATE_SEND:
      Serial.println("Preparing and sending frame...");
      prepareTxFrame(appPort);
      Serial.print("Sending data: ");
      for (int i = 0; i < appDataSize; i++) {
        Serial.print(appData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    case DEVICE_STATE_CYCLE:
      Serial.println("Scheduling next packet transmission...");
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    case DEVICE_STATE_SLEEP:
      // Serial.println("Sleeping...");
      LoRaWAN.sleep(loraWanClass);
      break;
    default:
      Serial.println("Resetting state to INIT...");
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}
