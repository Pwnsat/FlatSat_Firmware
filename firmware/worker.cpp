/*  - worker.cpp
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "frame.h"
#include "led.h"
#include "mission.h"
#include "ruplink.h"
#include "sensors.h"
#include "spp.h"
#include "thruster.h"
#include "usbCDC.h"
#include <Arduino.h>

typedef struct {
  const unsigned long inverval;
  unsigned long previous;
} timeout_worker_t;

static timeout_worker_t t_tm_data = {.inverval = 1000, .previous = 0};
static timeout_worker_t t_acc_data = {.inverval = 100, .previous = 0};
static timeout_worker_t t_thruster_data = {.inverval = 100, .previous = 0};
static timeout_worker_t t_hb_thruster_data = {.inverval = 500, .previous = 0};

static timeout_worker_t t_radio_tm_data = {.inverval = 10500, .previous = 0};

static void printHexDump(const uint8_t *data, size_t len) {
  const size_t bytesPerLine = 32;
  char ascii[bytesPerLine + 1];
  ascii[bytesPerLine] = '\0';

  for (size_t i = 0; i < len; i++) {
    if (i % bytesPerLine == 0) {
      if (i != 0) {
        Serial.print("  |");
        Serial.println(ascii);
      }
      Serial.printf("%08X  ", (unsigned int)i);
    }

    Serial.printf("%02X ", data[i]);
    ascii[i % bytesPerLine] = (data[i] >= 32 && data[i] <= 126) ? data[i] : '.';
  }

  size_t remaining = len % bytesPerLine;
  if (remaining > 0) {
    for (size_t i = remaining; i < bytesPerLine; i++) {
      Serial.print("   ");
    }
  }

  Serial.print("  |");
  ascii[remaining == 0 ? bytesPerLine : remaining] = '\0';
  Serial.println(ascii);
}

static inline int16_t float_to_fixed(float val, float scale) {
  return (int16_t)(val * scale);
}

static inline float fixed_to_float(int16_t val, float scale) {
  return ((float)val) / scale;
}

static void telemetrySCSendFloatFrame(uint8_t can_id, float metric) {
  int16_t t_fixed = float_to_fixed(metric, 100.0f);
  uint8_t payload[2];
  payload[0] = CANID_TM;
  payload[1] = (uint8_t)(t_fixed & 0xFF);
  payload[2] = (uint8_t)((t_fixed >> 8) & 0xFF);

  spacecan_frame_t f;
  sc_build_reply(&f, can_id, payload, 3);
  obcWriteFrame(&f);
}

static void telemetrySCSendU16Frame(uint8_t can_id, uint16_t metric) {
  uint8_t payload[2];
  payload[0] = CANID_TM;
  payload[1] = (uint8_t)(metric & 0xFF);
  payload[2] = (uint8_t)((metric >> 8) & 0xFF);

  spacecan_frame_t f;
  sc_build_reply(&f, can_id, payload, 3);
  obcWriteFrame(&f);
}

static void telemetrySCSendU8Frame(uint8_t can_id, uint8_t metric) {
  uint8_t payload[2];
  payload[0] = CANID_TM;
  payload[1] = metric;

  spacecan_frame_t f;
  sc_build_reply(&f, can_id, payload, 2);
  obcWriteFrame(&f);
}

static void telemetrySPPPackFillBuffer(uint8_t *buffer, int *offset,
                                       float metric) {
  int16_t val = float_to_fixed(metric, 100.0f);

  buffer[(*offset)++] = (uint8_t)(val & 0xFF);        // LSB
  buffer[(*offset)++] = (uint8_t)((val >> 8) & 0xFF); // MSB
}

static void telemetrySPPPackFrame(float x, float y, float z, float t, float tm,
                                  float p, float alt, float hum) {
  int offset = 0;
  uint8_t buffer[MAX_PAYLOAD_CHUNK];
  buffer[offset++] = SPACECRAFT_ID;
  telemetrySPPPackFillBuffer(buffer, &offset, x);
  telemetrySPPPackFillBuffer(buffer, &offset, y);
  telemetrySPPPackFillBuffer(buffer, &offset, z);
  telemetrySPPPackFillBuffer(buffer, &offset, t);
  telemetrySPPPackFillBuffer(buffer, &offset, tm);
  telemetrySPPPackFillBuffer(buffer, &offset, p);
  telemetrySPPPackFillBuffer(buffer, &offset, alt);
  telemetrySPPPackFillBuffer(buffer, &offset, hum);
  buffer[offset++] = '\0';

  space_packet_t space_packet;
  const int ret = spp_tm_build_packet(&space_packet, SPP_GROUP_FLAG_UNSEGMENTED,
                                      SPP_SECHEAD_FLAG_NOPRESENT, 0,
                                      SPP_APID_TM_SEND_TM, buffer, offset);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
    Serial.println(ret);
    return;
  }

  if (uplinkRadioTransmit(
          (uint8_t *)&space_packet,
          (SPP_PRIMARY_HEADER_LEN + space_packet.header.length))) {
    printHexDump((uint8_t *)&space_packet,
                 (SPP_PRIMARY_HEADER_LEN + space_packet.header.length));
  }
}

static void telemetryWorker(void) {
  if (millis() - t_tm_data.previous > t_tm_data.inverval) {
    t_tm_data.previous = millis();
    float tm, p, alt, hum;
    bmeRead(&tm, &p, &alt, &hum);

    telemetrySCSendFloatFrame(SC_TM_ID_BME_TEMPERATURE, tm);
    telemetrySCSendFloatFrame(SC_TM_ID_BME_PRESSURE, p);
    telemetrySCSendFloatFrame(SC_TM_ID_BME_ALTITUDE, alt);
    telemetrySCSendFloatFrame(SC_TM_ID_BME_HUMIDITY, hum);
  }
}

static void accWorker(void) {
  if (millis() - t_acc_data.previous > t_acc_data.inverval) {
    t_acc_data.previous = millis();
    float x, y, z, t;
    accelerometerRead(&x, &y, &z, &t);

    telemetrySCSendFloatFrame(SC_TM_ID_ACCE_X, x);
    telemetrySCSendFloatFrame(SC_TM_ID_ACCE_Y, y);
    telemetrySCSendFloatFrame(SC_TM_ID_ACCE_Z, z);
    telemetrySCSendFloatFrame(SC_TM_ID_ACCE_TMP, t);
  }
}

static void thrusterWorker(void) {
  if (millis() - t_thruster_data.previous > t_thruster_data.inverval) {
    t_thruster_data.previous = millis();
    const uint8_t thrustert0 = thrusterGetT0Power();
    const uint8_t thrustert1 = thrusterGetT1Power();

    telemetrySCSendU8Frame(SC_TM_ID_SIM_THRUSTER0, thrustert0);
    telemetrySCSendU8Frame(SC_TM_ID_SIM_THRUSTER1, thrustert0);
  }
}

static void thrusterHBWorker(void) {
  if (millis() - t_hb_thruster_data.previous > t_hb_thruster_data.inverval) {
    t_hb_thruster_data.previous = millis();

    spacecan_frame_t f0;
    sc_build_heartbeat(&f0, SC_TM_ID_SIM_THRUSTER0, thrusterGetT0State());
    obcWriteFrame(&f0);
    spacecan_frame_t f1;
    sc_build_heartbeat(&f1, SC_TM_ID_SIM_THRUSTER1, thrusterGetT1State());
    obcWriteFrame(&f1);
  }
}

void telemetryRadioWorker(void) {
  if (millis() - t_radio_tm_data.previous > t_radio_tm_data.inverval) {
    t_radio_tm_data.previous = millis();
    float tm, p, alt, hum;
    float x, y, z, t;
    bmeRead(&tm, &p, &alt, &hum);
    delay(100);
    accelerometerRead(&x, &y, &z, &t);
    telemetrySPPPackFrame(x, y, z, t, tm, p, alt, hum);
  }
}

void telemetrySCWorker(void) {
  /* Hearth Workers */
  thrusterHBWorker();
  /* TM Workers */
  accWorker();
  telemetryWorker();
  thrusterWorker();
}

void commandHandler(uint8_t *buffer, uint16_t buffer_len) {
  space_packet_t space_packet;
  int ret = spp_unpack_packet(&space_packet, buffer, buffer_len);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Unpacking SPP: ");
    Serial.println(ret);
    return;
  }
}