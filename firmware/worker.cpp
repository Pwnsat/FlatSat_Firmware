/*  - worker.cpp
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "frame.h"
#include "hardware/watchdog.h"
#include "led.h"
#include "mission.h"
#include "rdownlink.h"
#include "sensors.h"
#include "spp.h"
#include "thruster.h"
#include "usbCDC.h"
#include <Arduino.h>

void softwareReset() { watchdog_reboot(0, 0, 0); }

typedef struct {
  unsigned long interval;
  unsigned long previous;
} timeout_worker_t;

static timeout_worker_t t_tm_data = {.interval = 1000, .previous = 0};
static timeout_worker_t t_acc_data = {.interval = 100, .previous = 0};
static timeout_worker_t t_thruster_data = {.interval = 100, .previous = 0};
static timeout_worker_t t_hb_thruster_data = {.interval = 500, .previous = 0};

static timeout_worker_t t_radio_tm_data = {.interval = 10500, .previous = 0};
static timeout_worker_t t_radio_sync = {.interval = 5000, .previous = 0};
static timeout_worker_t t_radio_beacon = {.interval = 15000, .previous = 0};

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

static void logger_spp(space_packet_t *packet) {
  uint16_t type = (packet->header.identification >> 11) & 0x1;
  uint16_t apid = packet->header.identification & 0x7FF;

  uint16_t seq_flags = (packet->header.sequence >> 14) & 0x3;
  uint16_t seq_count = packet->header.sequence & 0x3FFF;
  Serial.printf("[%s - SPP] APID=0x%04X SEQ=%d LEN=%d FLAGS=%d\n",
                type == SPP_PTYPE_TM ? "TM" : "TC", apid, seq_count,
                packet->header.length, seq_flags);
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

static void telemetrySPPPackFillFloatToBuffer(uint8_t *buffer, int *offset,
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
  telemetrySPPPackFillFloatToBuffer(buffer, &offset, x);
  telemetrySPPPackFillFloatToBuffer(buffer, &offset, y);
  telemetrySPPPackFillFloatToBuffer(buffer, &offset, z);
  telemetrySPPPackFillFloatToBuffer(buffer, &offset, t);
  telemetrySPPPackFillFloatToBuffer(buffer, &offset, tm);
  telemetrySPPPackFillFloatToBuffer(buffer, &offset, p);
  telemetrySPPPackFillFloatToBuffer(buffer, &offset, alt);
  telemetrySPPPackFillFloatToBuffer(buffer, &offset, hum);
  buffer[offset++] = thrusterGetT0Power();
  buffer[offset++] = thrusterGetT1Power();
  buffer[offset++] = '\0';

  space_packet_t space_packet;
  const int ret = spp_tm_build_packet(&space_packet, SPP_GROUP_FLAG_UNSEGMENTED,
                                      SPP_SECHEAD_FLAG_NOPRESENT, 0,
                                      SPP_APID_TM_SEND_TM, buffer, offset);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
    Serial.println(ret);
    ledBlink(8, LED_COLOR_RED);
    return;
  }

  downlinkRadioTransmitNBlock(
      (uint8_t *)&space_packet,
      (SPP_PRIMARY_HEADER_LEN + space_packet.header.length));
  logger_spp(&space_packet);
}

static void telemetrySPPTransmitVersion(void) {
  int offset = 0;
  uint8_t buffer[MAX_PAYLOAD_CHUNK];
  buffer[offset++] = SPACECRAFT_ID;
  buffer[offset++] = FIRMWARE_PATCH;
  buffer[offset++] = FIRMWARE_MINOR;
  buffer[offset++] = FIRMWARE_MAJOR;
  buffer[offset++] = '\0';

  space_packet_t space_packet;
  const int ret = spp_tm_build_packet(&space_packet, SPP_GROUP_FLAG_UNSEGMENTED,
                                      SPP_SECHEAD_FLAG_NOPRESENT, 0,
                                      SPP_APID_TM_SEND_FW, buffer, offset);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
    Serial.println(ret);

    ledBlink(8, LED_COLOR_RED);
    return;
  }

  downlinkRadioTransmitNBlock(
      (uint8_t *)&space_packet,
      (SPP_PRIMARY_HEADER_LEN + space_packet.header.length));
  logger_spp(&space_packet);
}

static void telemetrySPPTransmitPingSync(void) {
  uint8_t buffer[8] = {SPACECRAFT_ID, 0x50, 0x77, 0x6e, 0x73, 0x61, 0x74, 0x00};

  space_packet_t space_packet;
  const int ret = spp_tm_build_packet(&space_packet, SPP_GROUP_FLAG_UNSEGMENTED,
                                      SPP_SECHEAD_FLAG_NOPRESENT, 0,
                                      SPP_APID_TM_PING, buffer, 8);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
    Serial.println(ret);
    ledBlink(8, LED_COLOR_RED);
    return;
  }

  downlinkRadioTransmitNBlock(
      (uint8_t *)&space_packet,
      (SPP_PRIMARY_HEADER_LEN + space_packet.header.length));
  logger_spp(&space_packet);
}

static void telemetrySPPTransmitPingAck(void) {
  uint8_t buffer[5] = {SPACECRAFT_ID, 0x41, 0x43, 0x4b, 0x00};

  space_packet_t space_packet;
  const int ret = spp_tm_build_packet(&space_packet, SPP_GROUP_FLAG_UNSEGMENTED,
                                      SPP_SECHEAD_FLAG_NOPRESENT, 0,
                                      SPP_APID_TM_PING, buffer, 5);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
    Serial.println(ret);
    ledBlink(8, LED_COLOR_RED);
    return;
  }

  downlinkRadioTransmitNBlock(
      (uint8_t *)&space_packet,
      (SPP_PRIMARY_HEADER_LEN + space_packet.header.length));
  logger_spp(&space_packet);
}

static void telemetrySPPTransmitBeacon(void) {
  uint8_t buffer[8] = {SPACECRAFT_ID, 0x42, 0x65, 0x61, 0x63, 0x6f, 0x6e, 0x00};

  space_packet_t space_packet;
  const int ret = spp_tm_build_packet(&space_packet, SPP_GROUP_FLAG_UNSEGMENTED,
                                      SPP_SECHEAD_FLAG_NOPRESENT, 0,
                                      SPP_APID_TM_PING, buffer, 8);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
    Serial.println(ret);
    ledBlink(8, LED_COLOR_RED);
    return;
  }

  downlinkRadioTransmitNBlock(
      (uint8_t *)&space_packet,
      (SPP_PRIMARY_HEADER_LEN + space_packet.header.length));
  logger_spp(&space_packet);
}

static void telemetryWorker(void) {
  if (millis() - t_tm_data.previous > t_tm_data.interval) {
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
  if (millis() - t_acc_data.previous > t_acc_data.interval) {
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
  if (millis() - t_thruster_data.previous > t_thruster_data.interval) {
    t_thruster_data.previous = millis();
    const uint8_t thrustert0 = thrusterGetT0Power();
    const uint8_t thrustert1 = thrusterGetT1Power();

    telemetrySCSendU8Frame(SC_TM_ID_SIM_THRUSTER0, thrustert0);
    telemetrySCSendU8Frame(SC_TM_ID_SIM_THRUSTER1, thrustert1);
  }
}

static void thrusterHBWorker(void) {
  if (millis() - t_hb_thruster_data.previous > t_hb_thruster_data.interval) {
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
  if (millis() - t_radio_tm_data.previous > t_radio_tm_data.interval) {
    t_radio_tm_data.previous = millis();
    float tm, p, alt, hum;
    float x, y, z, t;
    bmeRead(&tm, &p, &alt, &hum);
    delay(100);
    accelerometerRead(&x, &y, &z, &t);
    telemetrySPPPackFrame(x, y, z, t, tm, p, alt, hum);
  }
  if (t_radio_beacon.interval != 15000 &&
      millis() - t_radio_beacon.previous > t_radio_beacon.interval) {
    t_radio_beacon.previous = millis();
    telemetrySPPTransmitBeacon();
  }
  if (millis() - t_radio_sync.previous > t_radio_sync.interval) {
    t_radio_sync.previous = millis();
    telemetrySPPTransmitPingSync();
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

static inline uint16_t to_be16(uint16_t x) { return (x >> 8) | (x << 8); }

static void spp_print_packet_details(space_packet_t *packet) {
  uint16_t version = (packet->header.identification >> 13) & 0x7;
  uint16_t type = (packet->header.identification >> 11) & 0x1;
  uint16_t sec_header = (packet->header.identification >> 10) & 0x1;
  uint16_t apid = packet->header.identification & 0x7FF;

  uint16_t seq_flags = (packet->header.sequence >> 14) & 0x3;
  uint16_t seq_count = packet->header.sequence & 0x3FFF;

  Serial.println("=== Space Packet Header ===");
  Serial.printf(" Version:             %u\n", version);
  Serial.printf(" Type:                %02X\n", type);
  Serial.printf(" Secondary Header:    %u\n", sec_header);
  Serial.printf(" APID:                0x%04X\n", apid);
  Serial.printf(" Sequence Flags:      0x%X (%s)\n", seq_flags,
                seq_flags == SPP_GROUP_FLAG_UNSEGMENTED ? "Unsegmented"
                : seq_flags == SPP_GROUP_FLAG_START     ? "Start"
                : seq_flags == SPP_GROUP_FLAG_CONT      ? "Continuation"
                                                        : "End");
  Serial.printf(" Sequence Count:      %u\n", seq_count);
  Serial.printf(" Data Length:         %u\n", packet->header.length);

  Serial.println("=== Payload Dump (Hex) ===");
  printHexDump(packet->data, packet->header.length + 1);
}

void commandApidHandler(space_packet_t *space_packet) {
  uint16_t apid = space_packet->header.identification & 0x7FF;
  if (apid == SPP_APID_TC_PING) {
    telemetrySPPTransmitPingAck();
  } else if (apid == SPP_APID_TC_RESETC) {
    ledBlink(4, LED_COLOR_RED);
    ledBlink(4, LED_COLOR_YELLOW);
    softwareReset();
  } else if (apid == SPP_APID_TC_SEND_FW) {
    telemetrySPPTransmitVersion();
  } else if (apid == SPP_APID_TM_SET_THRUSTER) {
    uint8_t thruster_id = space_packet->data[0];
    uint8_t thuster_power = space_packet->data[1];
    if (thruster_id == 0) {
      thrusterSetT0Power(thuster_power);
    } else if (thruster_id == 1) {
      thrusterSetT1Power(thuster_power);
    } else {
      Serial.println("[ERROR] Thruster not found");
    }
  } else if (apid == SPP_APID_TC_SET_BEACON_RATE) {
    uint8_t b_seconds = space_packet->data[0];
    if (b_seconds > 10) {
      Serial.println("[Error] The Beacon rate it is to high");
      return;
    }
    t_radio_beacon.interval = b_seconds * 1000;
  } else if (apid == SPP_APID_TC_BROADCAST_MSG) {
    uint16_t frequency = ((uint16_t)space_packet->data[0] << 8) |
                         (uint16_t)space_packet->data[1];
    size_t buffer_len = space_packet->header.length - 1;
    uint8_t buffer[buffer_len] = {0};
    memcpy(buffer, space_packet->data + 2, buffer_len);
    space_packet_t space_packet;
    const int ret = spp_tm_build_packet(
        &space_packet, SPP_GROUP_FLAG_UNSEGMENTED, SPP_SECHEAD_FLAG_NOPRESENT,
        0, SPP_APID_TM_BROADCAST_MSG, buffer, buffer_len);
    if (ret != SPP_ERROR_NONE) {
      Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
      Serial.println(ret);
      ledBlink(8, LED_COLOR_RED);
      return;
    }

    downlinkRadioTransmitBroadcast(
        frequency, (uint8_t *)&space_packet,
        (SPP_PRIMARY_HEADER_LEN + space_packet.header.length));
    logger_spp(&space_packet);
  } else {
    Serial.printf("[ERROR] Unknown APID: 0x%02X \n", apid);
  }
}

void commandHandler(uint8_t *buffer, uint16_t buffer_len) {
  space_packet_t space_packet;
  int ret = spp_unpack_packet(&space_packet, buffer, buffer_len);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Unpacking SPP: ");
    Serial.println(ret);
    ledBlink(8, LED_COLOR_YELLOW);
    return;
  }
  logger_spp(&space_packet);
  commandApidHandler(&space_packet);
}