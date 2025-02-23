function decodeUplink(input) {
  //payloard length = 24 bytes
  //payload order:
  // 1. float temp 4bytes
  // 2. uint32 waterContent (scaled float) 4bytes
  // 3. uint32 lat (scaled float) 4bytes
  // 4. unit32 lng (scaled float) 4btytes
  // 5. uint32 timeTaken 4bytes
  // 6. uint32 vBat (scaled float) 4bytes

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


  return {
    data: {
      temperature: temp,
      waterContent: waterContent,
      latitude: lat,
      longitude: lng,
      timeTaken: timeTaken,
      batteryVoltage: vBat,
    },
  };
}
