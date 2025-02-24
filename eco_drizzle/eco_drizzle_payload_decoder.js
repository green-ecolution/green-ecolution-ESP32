function decodeUplink(input) {
  // max payload length is 51 bytes
  // payload length = 37 bytes + n (max 14) bytes of device name
  // payload order:
  // 1. float temp 4bytes
  // 2. uint32 waterContent (scaled float) 4bytes
  // 3. uint32 lat (scaled float) 4bytes
  // 4. unit32 lng (scaled float) 4btytes
  // 5. uint32 timeTaken 4bytes
  // 6. uint32 vBat (scaled float) 4bytes
  // 7. uint16 WM1_Resistance 2bytes
  // 8. uint16 WM1_CB 2bytes
  // 9. uint16 WM2_Resistance 2bytes
  // 10. uint16 WM2_CB 2bytes
  // 11. uint16 WM3_Resistance 2bytes
  // 12. uint16 WM3_CB 2bytes
  // 13 Device Name Length (1 byte, uint8)
  // 14 Device Name (N bytes, string)

  const bytes = input.bytes;
  let offset = 0;
  const scalingFactor = 1000000;

  // Decode the float temp
  const tempBytes = bytes.slice(offset, offset + 4);
  const temp = new DataView(new Uint8Array(tempBytes).buffer).getFloat32(0, true);
  offset += 4;

  //decode scaled floats waterContent, lat and lng
  const scaledWaterContent = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  const waterContent = scaledWaterContent / scalingFactor;  // Divide by the scaling factor
  offset += 4;
  const scaledLat = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  const lat = scaledLat / scalingFactor;
  offset += 4;
  const scaledLng = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  const lng = scaledLng / scalingFactor;
  offset += 4;
  const timeTaken = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  offset += 4;
  const scaledVBat = (bytes[offset] << 24) | (bytes[offset + 1] << 16) | (bytes[offset + 2] << 8) | bytes[offset + 3];
  const vBat = scaledVBat / scalingFactor;
  offset += 4;

  // Decode Watermark sensor resistances and CB values (all 2-byte uint16)
  const WM1_Resistance = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;
  const WM1_CB = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;

  const WM2_Resistance = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;
  const WM2_CB = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;

  const WM3_Resistance = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;
  const WM3_CB = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;

  // Decode the device name
  const deviceNameLength = bytes[offset++];  // Read the length byte, post increment after reading length
  const deviceNameBytes = bytes.slice(offset, offset + deviceNameLength);  // Read the string bytes
  const deviceName = String.fromCharCode(...deviceNameBytes);
  offset += deviceNameLength;



  return {
    data: {
      temperature: temp,
      waterContent: waterContent,
      latitude: lat,
      longitude: lng,
      timeTaken: timeTaken,
      batteryVoltage: vBat,
      WM1_Resistance: WM1_Resistance,
      WM1_CB: WM1_CB,
      WM2_Resistance: WM2_Resistance,
      WM2_CB: WM2_CB,
      WM3_Resistance: WM3_Resistance,
      WM3_CB: WM3_CB,
      deviceName: deviceName,
    },
  };
}
