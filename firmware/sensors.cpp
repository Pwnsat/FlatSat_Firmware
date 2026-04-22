/*  - sensors.cpp
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "sensors.h"
#include "SparkFun_LIS2DH12.h"
#include "pins.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_BME280 bme;
SPARKFUN_LIS2DH12 accel;

typedef struct {
  bool bmeOk;
  bool accOk;
} sensor_context_t;

static sensor_context_t s_context;

static void sensorsInitAcc(void) {
  if (!accel.begin()) {
    Serial.print("[ERROR] ACC_NOT_DETECTED\r\n");
    s_context.accOk = false;
    return;
  }
  Serial.print("[INFO] ACC OK\r\n");
}

static void sensorsInitBme(void) {
  if (!bme.begin(BME_ADDR)) {
    Serial.printf("[ERROR] BMP_NOT_DETECTED. ID: 0x%02X \r\n", bme.sensorID());
    s_context.bmeOk = false;
    return;
  }
  Serial.print("[INFO] BME OK\r\n");
}

void sensorsConfigure(void) {
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();

  s_context.bmeOk = true;
  s_context.accOk = true;

  sensorsInitAcc();
  sensorsInitBme();
}

bool accelerometerRead(float *x, float *y, float *z, float *temp) {
  *x = accel.getX();
  *y = accel.getY();
  *z = accel.getZ();
  *temp = accel.getTemperature();
  return s_context.accOk;
}

bool bmeRead(float *t, float *p, float *alt, float *hum) {
  *t = bme.readTemperature();
  *p = bme.readPressure() / 100.0F;
  *alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  *hum = bme.readHumidity();
  return s_context.bmeOk;
}