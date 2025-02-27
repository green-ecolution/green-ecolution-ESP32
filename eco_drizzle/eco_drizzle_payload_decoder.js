function decodeUplink(input) {
  // max payload length is 51 bytes
  // payload length = 37 bytes + n (max 14) bytes of device name
  // payload order:
  // 0. uint8 messageType
  // 1.. depends on messageType

  const bytes = input.bytes;
  let offset = 0;

  // Read the first byte to determine the message type
  const messageType = bytes[offset++];

  if (messageType === 1) {
    // Low battery message
    return decodeLowBatteryMessage(bytes, offset);
  } else {
    // Normal message
    return decodeNormalMessage(bytes, offset);
  }
}

function decodeLowBatteryMessage(bytes, offset) {
  // 1. Device Name Length (1 byte, uint8)
  // 2. Device Name (N bytes, string)
  // 3. Low Battery Message Length (1 byte, uint8)
  // 4. Low Battery Message (N bytes, string)

  // Decode the device name
  const deviceNameLength = bytes[offset++];  // Read the length byte
  const deviceNameBytes = bytes.slice(offset, offset + deviceNameLength);  // Read the string bytes
  const deviceName = String.fromCharCode(...deviceNameBytes);
  offset += deviceNameLength;

  // Decode the "battery too low" message
  const lowBatteryMessageLength = bytes[offset++];  // Read the length byte
  const lowBatteryMessageBytes = bytes.slice(offset, offset + lowBatteryMessageLength);  // Read the string bytes
  const lowBatteryMessage = String.fromCharCode(...lowBatteryMessageBytes);

  return {
    data: {
      messageType: "low_battery",
      deviceName: deviceName,
      lowBatteryMessage: lowBatteryMessage,
    },
  };
}

function decodeNormalMessage(bytes, offset) {
  // 1. float temp 4bytes
  // 2. uint32 waterContent (scaled float) 4bytes
  // 3. uint32 lat (scaled float) 4bytes
  // 4. unit32 lng (scaled float) 4btytes
  // 5. uint32 timeTaken 4bytes
  // 6. uint32 vBat (scaled float) 4bytes
  // 7. uint16 WM30_Resistance 2bytes
  // 8. uint16 WM30_CB 2bytes
  // 9. uint16 WM60_Resistance 2bytes
  // 10. uint16 WM60_CB 2bytes
  // 11. uint16 WM90_Resistance 2bytes
  // 12. uint16 WM90_CB 2bytes
  // 13. Device Name Length (1 byte, uint8)
  // 14. Device Name (N bytes, string)
  const scalingFactor = 1000000;

  // Decode the float temp
  const tempBytes = bytes.slice(offset, offset + 4);
  const temp = new DataView(new Uint8Array(tempBytes).buffer).getFloat32(0, true);
  offset += 4;

  // Decode scaled floats waterContent, lat, and lng
  const scaledWaterContent = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  const waterContent = scaledWaterContent / scalingFactor;  // Divide by the scaling factor
  offset += 4;
  const scaledLat = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  const lat = scaledLat / scalingFactor;
  offset += 4;
  const scaledLng = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  const lng = scaledLng / scalingFactor;
  offset += 4;

  // Decode timeTaken and battery voltage
  const timeTaken = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  offset += 4;
  const scaledVBat = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  const vBat = scaledVBat / scalingFactor;
  offset += 4;

  // Decode Watermark sensor resistances and CB values (all 2-byte uint16)
  const WM30_Resistance = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;
  const WM30_CB = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;

  const WM60_Resistance = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;
  const WM60_CB = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;

  const WM90_Resistance = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;
  const WM90_CB = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;

  // Decode the device name
  const deviceNameLength = bytes[offset++];  // Read the length byte
  const deviceNameBytes = bytes.slice(offset, offset + deviceNameLength);  // Read the string bytes
  const deviceName = String.fromCharCode(...deviceNameBytes);

  return {
    data: {
      messageType: "normal",
      temperature: temp,
      waterContent: waterContent,
      latitude: lat,
      longitude: lng,
      timeTaken: timeTaken,
      batteryVoltage: vBat,
      WM30_Resistance: WM30_Resistance,
      WM30_CB: WM30_CB,
      WM60_Resistance: WM60_Resistance,
      WM60_CB: WM60_CB,
      WM90_Resistance: WM90_Resistance,
      WM90_CB: WM90_CB,
      deviceName: deviceName,
    },
  };
}
