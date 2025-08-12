#ifndef __TELEMETRY_H
#define __TELEMETRY_H
#include <stdint.h>
#include <Arduino.h>

#define SPACECRAFT_ID 0x01

void telemetryHandleRecvData(uint8_t* buffer, uint16_t buffer_len);
#endif