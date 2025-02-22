function decodeUplink(input) {
  const bytes = input.bytes;

  let offset = 0;

  // Decode the string
  const messageLength = 5;  // Length of "Hello"
  const message = String.fromCharCode(...bytes.slice(offset, offset + messageLength));
  offset += messageLength;

  // Decode the float
  const floatBytes = bytes.slice(offset, offset + 4);
  const floatValue = new DataView(new Uint8Array(floatBytes).buffer).getFloat32(0, true);
  offset += 4;
  const scaledValue = (bytes[offset] << 24) | (bytes[offset+1] << 16) | (bytes[offset+2] << 8) | bytes[offset+3];
  const longitude = scaledValue / 1000000;  // Divide by the scaling factor
  offset +=4;
  // Decode the integer
  const intValue = (bytes[offset] << 8) | bytes[offset + 1];
  offset += 2;
  
  let longValue = 0;
  for (let i = 0; i < 4; i++) {
      longValue = (longValue << 8) | bytes[offset + i];
  }
  offset +=4;

  return {
    data: {
      message: message,
      floatValue: floatValue,
      floatScaledUnscaledValue: longitude,
      intValue: intValue,
      longValue: longValue,
    },
  };
}
