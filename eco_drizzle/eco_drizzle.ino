#include "heltec.h"
#include "LoRaWan_APP.h"
#include "secrets.h"
#include "helpers.h"
#include "gps.h"
#include "smt100.h"

// connections are symmetric
#define SMT100RXD 48  // Connected to RX pin of RS485 module
#define SMT100TXD 47  // Connected to TX pin of RS485 module

// connections are crossed
#define GPSTXD 33  // Connected to RX pin of GPS module
#define GPSRXD 34  // Connected to TX pin of GPS module

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
uint32_t appTxDutyCycle = 15000;

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

// Variables to store GPS data
float latitude = 0.0;
float longitude = 0.0;
uint32_t timeTaken = 0;

// Default values
const float defaultTempC = 25.0;
const float defaultWaterContent = 50.0;
float temperature = defaultTempC;
float waterContent = defaultWaterContent;


void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Print the device name to the Serial Monitor
  Serial.print("Device Name: ");
  Serial.println(deviceName);

  initSMT100(SMT100RXD, SMT100TXD);
  initGPS(GPSRXD, GPSTXD);

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
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;  // Move to the CYCLE state
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

  // Get temperature from SMT100
  if (getSMT100Temperature(temperature)) {
    Serial.print("Temperature: ");
    Serial.println(temperature);
  } else {
    Serial.println("Failed to read temperature. Using default value.");
  }

  // Get moisture from SMT100
  if (getSMT100MWaterContent(waterContent)) {
    Serial.print("Water Content: ");
    Serial.println(waterContent);
  } else {
    Serial.println("Failed to read moisture. Using default value.");
  }

  getGPSSignal(latitude, longitude, timeTaken);

  // read the analog / millivolts value for pin 1:
  int analogValue = analogRead(1);
  int analogVolts = analogReadMilliVolts(1);
  float voltage = (analogValue * 3.3) / 4095;  // Convert to voltage
  Serial.printf("ADC analog value = %d\n",analogValue);
  Serial.printf("ADC millivolts value = %d\n",analogVolts);
  Serial.printf("calculated volts value = %d\n",voltage);

  // Prepare the payload
  appDataSize = 0;

  // Add floats to the payload
  addFloatToPayload(appData, appDataSize, temperature);  // No scaling
  addFloatToPayload(appData, appDataSize, waterContent, scaleFactor);
  addFloatToPayload(appData, appDataSize, latitude, scaleFactor);   // Scaled
  addFloatToPayload(appData, appDataSize, longitude, scaleFactor);  // Scaled
  addLongToPayload(appData, appDataSize, timeTaken);

  Serial.println("Payload prepared with temp, watercontent, lat, lng.");
  Serial.println("Turn OFF Vext");
  Heltec.VextOFF();
}
