#include "usbUplink.h"
#include "worker.h"
/* --- USB SERIAL UPLINK INJECTION FOR EXPLOIT TESTING --- */

// Copied from ruplink.cpp because the originals are static
static int usbHexCharToNibble(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return -1;
}

static size_t usbHexStringToBytes(const uint8_t *input, size_t len,
                                  uint8_t *output) {
  size_t outIndex = 0;
  for (size_t i = 0; i < len;) {
    if (input[i] == ' ') {
      i++;
      continue;
    }
    if (i + 1 >= len)
      break;
    int high = usbHexCharToNibble(input[i]);
    int low = usbHexCharToNibble(input[i + 1]);
    if (high < 0 || low < 0)
      break;
    output[outIndex++] = (high << 4) | low;
    i += 2;
  }
  return outIndex;
}

// The new USB Worker Function
void usbSerialUplinkWorker(void) {
  // Check if data is coming in over the USB cable
  if (Serial.available() > 0) {
    // Read the incoming ASCII Hex string until a newline is received
    String input = Serial.readStringUntil('\n');
    input.trim(); // Clean up any \r or whitespace

    if (input.length() > 0) {
      Serial.printf("\n[SYSTEM] USB Payload Received: %d characters\n",
                    input.length());

      // Define a buffer large enough for our exploit payload
      const size_t max_payload = 1024;
      uint8_t byteArr[max_payload];
      uint8_t parsed[max_payload];

      // Convert Arduino String to uint8_t array
      input.getBytes(byteArr, max_payload);

      // Convert ASCII Hex ("414141...") to raw bytes (0x41, 0x41, 0x41...)
      size_t parsedLen = usbHexStringToBytes(byteArr, input.length(), parsed);

      if (parsedLen > 0) {
        Serial.printf(
            "[SYSTEM] Parsed into %d raw bytes. Triggering commandHandler...\n",
            parsedLen);

        // Pass the malicious payload directly to the vulnerable handler
        commandHandler(parsed, parsedLen);
      }
    }
  }
}