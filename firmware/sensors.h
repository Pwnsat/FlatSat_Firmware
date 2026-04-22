/*  - sensors.h
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#ifndef FIRMWARE_SENSORS_H
#define FIRMWARE_SENSORS_H
#include <Arduino.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_ADDR 0x76

void sensorsConfigure(void);
bool accelerometerRead(float *x, float *y, float *z, float *temp);
bool bmeRead(float *t, float *p, float *alt, float *hum);
#endif // FIRMWARE_SENSORS_H