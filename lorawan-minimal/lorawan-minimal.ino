#include "LoRaWan_APP.h"
#include "secrets.h"
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


void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Initialize RS485 communication for SMT100 sensor
  Serial1.begin(9600);

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
  String message = "Hello";
  float lat = 54.774542;
  float lng = 9.446332;
  int16_t intValue = 1234;
  uint32_t longValue = 123456789;



  // Use a union to convert float to bytes
  union {
    float floatValue;
    uint8_t floatBytes[4];
  } floatToBytes;

  floatToBytes.floatValue = lat;

  // Prepare the payload
  appDataSize = 0;

  // Add the string to the payload
  for (int i = 0; i < message.length(); i++) {
    appData[appDataSize++] = message[i];
  }

  // Add the float to the payload
  for (int i = 0; i < 4; i++) {
    appData[appDataSize++] = floatToBytes.floatBytes[i];
  }

  uint32_t scaledLong = lng * 1000000;  // Scale by 1,000,000

  // Encoding
  appData[appDataSize++] = (scaledLong >> 24) & 0xFF;
  appData[appDataSize++] = (scaledLong >> 16) & 0xFF;
  appData[appDataSize++] = (scaledLong >> 8) & 0xFF;
  appData[appDataSize++] = scaledLong & 0xFF;

  // Add the integer to the payload
  appData[appDataSize++] = (intValue >> 8) & 0xFF;  // High byte
  appData[appDataSize++] = intValue & 0xFF;         // Low byte

  // Extract bytes using a loop
  for (int i = 0; i < 4; i++) {
    appData[appDataSize++] = (longValue >> (8 * (3 - i))) & 0xFF;
  }

  Serial.println("Payload prepared with string, float, and integer16 and int32.");
}