#include "LoRaWan_APP.h"
#include <Arduino.h>
#include "lora/LoRa.h"
#include "heltec.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"

#define MOISTURE_PIN 2  // Pin für den Feuchtigkeitssensor
#define VBAT_Read 1 // Pin für die Battery
#define	ADC_Ctrl 37

/* OTAA para*/
uint8_t devEui[] = { };
uint8_t appEui[] = { };
uint8_t appKey[] = { };

/* ABP para*/
uint8_t nwkSKey[] = { };
uint8_t appSKey[] = { };
uint32_t devAddr = (uint32_t)0x00000000;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in Arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 300000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

extern uint8_t appDataSize;
extern uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];

/* Number of trials to transmit the frame */
uint8_t confirmedNbTrials = 4;

const float BATTERY_THRESHOLD = 2.7;

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
  int moistureValue = analogRead(MOISTURE_PIN); // Lesen des Feuchtigkeitswerts
  // Konvertiere den Feuchtigkeitswert zu einem Prozentsatz (0-100%)
  float moisturePercentage = map(moistureValue, 1500, 3250, 100, 0);
  int int_moisture = moisturePercentage * 10; // Umwandlung in eine Ganzzahl und entfernen der Dezimalstelle

  // Batterie auslesen
  uint16_t int_battery = readBatteryVoltage();
  float batteryVoltage = int_battery / 1000.0;
  if(batteryVoltage < BATTERY_THRESHOLD){
    esp_deep_sleep_start();
  }

  appDataSize = 6;
  appData[0] = int_moisture >> 8;
  appData[1] = int_moisture;
  appData[2] = moistureValue >> 8;
  appData[3] = moistureValue;
  appData[4] = int_battery >> 8;
  appData[5] = int_battery;
}

uint16_t readBatteryVoltage() {
  // ADC resolution
  const int resolution = 12;
  const int adcMax = pow(2,resolution) - 1;
  const float adcMaxVoltage = 3.3;
  // On-board voltage divider
  const int R1 = 390;
  const int R2 = 100;
  // Calibration measurements
  const float measuredVoltage = 4.2;
  const float reportedVoltage = 4.095;
  // Calibration factor
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
  Serial.begin(115200);
  VextON();
  delay(100);


  pinMode(ADC_Ctrl,OUTPUT);
  pinMode(VBAT_Read,INPUT);
  analogReadResolution(12); // Setzt die Auflösung auf 12 Bit

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  pinMode(MOISTURE_PIN, INPUT);
  Serial.println("Starting LoRaWAN...");

  // Set the TX power to maximum (14 dBm)
  LoRa.setTxPowerMax(14);
  LoRa.setSpreadingFactor(12);
}

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
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
