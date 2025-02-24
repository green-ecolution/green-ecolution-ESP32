// secrets_template.h
#ifndef SECRETS_TEMPLATE_H
#define SECRETS_TEMPLATE_H

// OTAA parameters
const uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };                                                  // Replace with your DevEUI
const uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };                                                  // Replace with your AppEUI (Join EUI)
const uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // Replace with your AppKey

const char DEVICE_NAME[] = "Device123";  // Replace with your device name, max 10 chars
#endif
