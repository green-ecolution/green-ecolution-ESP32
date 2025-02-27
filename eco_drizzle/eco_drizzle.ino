#include "heltec.h"
#include "LoRaWan_APP.h"
#include "secrets.h"
#include "helpers.h"
#include "gps.h"
#include "smt100.h"
#include "battery.h"
#include "watermark.h"

/* OTAA parameters are inside secrets.h */

/* ABP para need to be kept */
uint8_t nwkSKey[] = {};
uint8_t appSKey[] = {};
uint32_t devAddr = (uint32_t)0x00000000;
/* LoRaWAN channels mask, default channels 0-7 */
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/* LoRaWAN region, select in Arduino IDE tools */
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/* LoRaWAN Class, Class A and Class C are supported */
DeviceClass_t loraWanClass = CLASS_A;

/* Application data transmission duty cycle in milliseconds */
uint32_t appTxDutyCycle = 6 * 60 * 60 * 1000;  //6 hours <- hours*minutes*seconds*milliseconds

/* OTAA or ABP */
bool overTheAirActivation = true;

/* ADR enable */
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

const uint32_t scaleFactor = 1000000;  // used for float scaling, keeps 6 decimal places

// Battery voltage threshold
const float BATTERY_LOW_THRESHOLD = 3.3;

// Variables to store GPS data
float latitude = 0.0;
float longitude = 0.0;
uint32_t timeTaken = 0;

// Default values
const float defaultTempC = 111.0;
const float defaultWaterContent = 112.0;
float temperature = defaultTempC;
float waterContent = defaultWaterContent;

float WM_30_Resistance, WM_60_Resistance, WM_90_Resistance;
int WM_30_CB, WM_60_CB, WM_90_CB;

float batteryVoltage;

void setup() {
  Serial.begin(115200);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Print the device name to the Serial Monitor
  Serial.print("Device Name: ");
  Serial.println(deviceName);
  Serial.print("Sending frequency in ms: ");
  Serial.println(appTxDutyCycle);

  initBattery();
  initSMT100();
  initGPS();
  initWatermark();

  Serial.println("Setup complete. Starting LoRaWAN state machine...");
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
        Serial.println("State: DEVICE_STATE_INIT - Initializing LoRaWAN...");

#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();  // Generate DevEUI from chip ID (if supported)
#endif

        LoRaWAN.init(loraWanClass, loraWanRegion);
        LoRaWAN.setDefaultDR(3);  // Set default data rate

        deviceState = DEVICE_STATE_JOIN;  // Move to the JOIN state
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        Serial.println("State: DEVICE_STATE_JOIN - Attempting to join the network...");
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        Serial.println("State: DEVICE_STATE_SEND - Preparing and sending data...");

        batteryVoltage = readBatteryVoltage();
        if (batteryVoltage < BATTERY_LOW_THRESHOLD) {
          // Battery is low, send a low battery message
          prepareLowBatteryTxFrame(appPort);
          Serial.println("Battery voltage is too low! Sending final message and entering deep sleep...");
          LoRaWAN.send();
          delay(1000);
          deviceState = DEVICE_STATE_SLEEP;
        } else {
          // Battery is okay, send normal data
          prepareTxFrame(appPort);
          LoRaWAN.send();
          deviceState = DEVICE_STATE_CYCLE;
        }
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        Serial.println("State: DEVICE_STATE_CYCLE - Scheduling next transmission...");
        txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;  // Move to the SLEEP state
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass);
        break;
      }
    default:
      {
        Serial.println("State: UNKNOWN - Resetting to DEVICE_STATE_INIT...");
        deviceState = DEVICE_STATE_INIT;  // Reset to INIT state
        break;
      }
  }
}

void prepareTxFrame(uint8_t port) {

  Serial.println("Turn ON Vext");
  Heltec.VextON();
  delay(500);  // maybe unnecessary

  // Get temperature in Celsius from SMT100
  getSMT100Temperature(temperature) ? Serial.println("SMT100 Temperature okay") : Serial.println("SMT100 Temperature failed");

  // Get soil water content percentage from SMT100
  getSMT100MWaterContent(waterContent) ? Serial.println("SMT100 Water Content okay") : Serial.println("SMT100 Water Content failed");

  getGPSSignal(latitude, longitude, timeTaken) ? Serial.println("GPS okay") : Serial.println("GPS failed");

  // Read and process Watermark sensor values
  getWatermarkValues(temperature, WM_30_Resistance, WM_60_Resistance, WM_90_Resistance, WM_30_CB, WM_60_CB, WM_90_CB);

  // Prepare the payload
  appDataSize = 0;

  // Add status byte (0 = normal message)
  appData[appDataSize++] = 0;

  // Add floats to the payload
  addFloatToPayload(appData, appDataSize, temperature);  // No scaling
  addFloatToPayload(appData, appDataSize, waterContent, scaleFactor);
  addFloatToPayload(appData, appDataSize, latitude, scaleFactor);   // Scaled
  addFloatToPayload(appData, appDataSize, longitude, scaleFactor);  // Scaled
  addLongToPayload(appData, appDataSize, timeTaken);
  addFloatToPayload(appData, appDataSize, batteryVoltage, scaleFactor);

  addIntToPayload(appData, appDataSize, (int)WM_30_Resistance);
  addIntToPayload(appData, appDataSize, abs(WM_30_CB));
  addIntToPayload(appData, appDataSize, (int)WM_60_Resistance);
  addIntToPayload(appData, appDataSize, abs(WM_60_CB));
  addIntToPayload(appData, appDataSize, (int)WM_90_Resistance);
  addIntToPayload(appData, appDataSize, abs(WM_90_CB));

  // Add the device name with its length
  addCharArrayWithLengthToPayload(appData, appDataSize, deviceName);

  Serial.println("Payload prepared");
  Serial.println("Turn OFF Vext");
  Heltec.VextOFF();
}
void prepareLowBatteryTxFrame(uint8_t port) {
  // Prepare the payload
  appDataSize = 0;

  // Add status byte (1 = low battery message)
  appData[appDataSize++] = 1;

  addCharArrayWithLengthToPayload(appData, appDataSize, deviceName);
  // Add the "battery too low" message
  const char* lowBatteryMessage = "battery low, going to sleep";
  addCharArrayWithLengthToPayload(appData, appDataSize, lowBatteryMessage);

  Serial.println("Low battery payload prepared");
  Serial.println("Turn OFF Vext");
}
