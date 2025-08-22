#ifndef __BUS_H
#define __BUS_H
#include <Arduino.h>
#include <stdint.h>

void telemetryHandleRecvData(uint8_t *buffer, uint16_t buffer_len);
#endif