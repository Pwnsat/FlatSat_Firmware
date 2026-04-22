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

#define CHUNK_SIZE 16

typedef struct {
  unsigned long interval;
  unsigned long previous;
} timeout_worker_t;

const uint8_t image_data[255] = {
    0x00, 0x1F, 0x04, 0x20, 0xEB, 0x00, 0x00, 0x00, 0x35, 0x00, 0x00, 0x00,
    0x31, 0x00, 0x00, 0x00, 0x4D, 0x75, 0x01, 0x03, 0x7A, 0x00, 0xC4, 0x00,
    0x1D, 0x00, 0x00, 0x00, 0x00, 0x23, 0x02, 0x88, 0x9A, 0x42, 0x03, 0xD0,
    0x43, 0x88, 0x04, 0x30, 0x91, 0x42, 0xF7, 0xD1, 0x18, 0x1C, 0x70, 0x47,
    0x30, 0xBF, 0xFD, 0xE7, 0xF4, 0x46, 0x00, 0xF0, 0x05, 0xF8, 0xA7, 0x48,
    0x00, 0x21, 0x01, 0x60, 0x41, 0x60, 0xE7, 0x46, 0xA5, 0x48, 0x00, 0x21,
    0xC9, 0x43, 0x01, 0x60, 0x41, 0x60, 0x70, 0x47, 0xCA, 0x9B, 0x0D, 0x5B,
    0xF9, 0x1D, 0x00, 0x00, 0x28, 0x43, 0x29, 0x20, 0x32, 0x30, 0x32, 0x30,
    0x20, 0x46, 0x6F, 0x6C, 0x6C, 0x6F, 0x20, 0x54, 0x68, 0x65, 0x20, 0x57,
    0x68, 0x74, 0x65, 0x20, 0x52, 0x61, 0x62, 0x69, 0x74, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x03, 0x4C, 0x33,
    0x57, 0x03, 0x54, 0x33, 0x8F, 0x03, 0x4D, 0x53, 0xB9, 0x26, 0x53, 0x34,
    0xAD, 0x26, 0x4D, 0x43, 0x1D, 0x26, 0x43, 0x34, 0x05, 0x26, 0x55, 0x42,
    0x91, 0x25, 0x44, 0x54, 0xA9, 0x01, 0x44, 0x45, 0xAF, 0x01, 0x57, 0x56,
    0x45, 0x01, 0x49, 0x46, 0x91, 0x24, 0x45, 0x58, 0xE5, 0x23, 0x52, 0x45,
    0x6D, 0x23, 0x52, 0x50, 0xB5, 0x23, 0x46, 0x43, 0x51, 0x23, 0x43, 0x58,
    0x21, 0x23, 0x00, 0x00, 0x47, 0x52, 0x50, 0x00, 0x43, 0x52, 0x58, 0x00,
    0x53, 0x46, 0xCC, 0x01, 0x53, 0x44, 0x4C, 0x02, 0x46, 0x5A, 0xCA, 0x01,
    0x46, 0x53, 0x34, 0x27, 0x46, 0x45, 0x28, 0x2E, 0x44, 0x53, 0x30, 0x2E,
    0x44, 0x45, 0xA4, 0x3D, 0x00, 0x00, 0x7D, 0x48, 0x01, 0x68, 0x00, 0x29,
    0x28, 0xD1, 0xFF, 0xF7, 0x9F, 0xFF, 0x7B, 0x49, 0x0A, 0x68, 0x53, 0x0E,
    0x01, 0xD3, 0x0A,
};

static timeout_worker_t t_tm_data = {.interval = 1000, .previous = 0};
static timeout_worker_t t_acc_data = {.interval = 100, .previous = 0};
static timeout_worker_t t_thruster_data = {.interval = 100, .previous = 0};
static timeout_worker_t t_hb_thruster_data = {.interval = 500, .previous = 0};

static timeout_worker_t t_radio_tm_data = {.interval = 10500, .previous = 0};
static timeout_worker_t t_radio_sync = {.interval = 5000, .previous = 0};
static timeout_worker_t t_radio_idle = {.interval = 20000, .previous = 0};
static timeout_worker_t t_radio_beacon = {.interval = 15000, .previous = 0};

static uint32_t image_data_len = 255;
static bool block_tx = false;

static uint8_t crc8_compute(const uint8_t *data, uint32_t len) {
  uint8_t crc = 0x00;

  for (uint32_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static void softwareReset() { watchdog_reboot(0, 0, 0); }

static inline int16_t float_to_fixed(float val, float scale) {
  return (int16_t)(val * scale);
}

static inline float fixed_to_float(int16_t val, float scale) {
  return ((float)val) / scale;
}

static inline uint16_t to_be16(uint16_t x) { return (x >> 8) | (x << 8); }

static void logger_spp(space_packet_t *packet) {
  uint16_t id_raw = BE_TO_HOST16(packet->header.identification);
  uint16_t seq_raw = BE_TO_HOST16(packet->header.sequence);
  uint16_t len_raw = BE_TO_HOST16(packet->header.length);

  uint8_t type = (id_raw >> 12) & 0x01;
  uint8_t sec_hdr = (id_raw >> 11) & 0x01;
  uint16_t apid = id_raw & 0x07FF;

  uint8_t seq_flags = (seq_raw >> 14) & 0x03;
  uint16_t seq_count = seq_raw & 0x3FFF;
  Serial.printf("[%s - SPP] APID=0x%03X SEQ=%d LEN=%d FLAGS=%d SEC_HDR=%s\n",
                (type == SPP_PTYPE_TM) ? "TM" : "TC", apid, seq_count,
                len_raw + 1, seq_flags, sec_hdr ? "YES" : "NO");
}

static void logger_spp_tc(space_packet_t *packet) {
  uint16_t id = packet->header.identification;
  uint16_t seq = packet->header.sequence;
  uint16_t len = packet->header.length;

  uint8_t type = (id >> 12) & 0x01;
  uint8_t sec_hdr = (id >> 11) & 0x01;
  uint16_t apid = id & 0x07FF;

  uint8_t flags = (seq >> 14) & 0x03;
  uint16_t count = seq & 0x3FFF;

  Serial.printf("[TC - SPP] APID=0x%03X SEQ=%d LEN=%d FLAGS=%d SEC_HDR=%s\n",
                apid, count, len + 1, flags, sec_hdr ? "YES" : "NO");
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
  const uint16_t total_len =
      SPP_PRIMARY_HEADER_LEN + (HOST_TO_BE16(space_packet.header.length) + 1);
  downlinkRadioTransmitNBlock((uint8_t *)&space_packet, total_len);
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

  const uint16_t total_len =
      SPP_PRIMARY_HEADER_LEN + (HOST_TO_BE16(space_packet.header.length) + 1);
  downlinkRadioTransmitNBlock((uint8_t *)&space_packet, total_len);
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

  const uint16_t total_len =
      SPP_PRIMARY_HEADER_LEN + (HOST_TO_BE16(space_packet.header.length) + 1);
  downlinkRadioTransmitNBlock((uint8_t *)&space_packet, total_len);
  logger_spp(&space_packet);
}

static void telemetrySPPTransmitIDLE(void) {
  space_packet_t space_packet;
  const int ret = spp_idle_build_packet(&space_packet);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
    Serial.println(ret);
    ledBlink(8, LED_COLOR_RED);
    return;
  }

  const uint16_t total_len =
      SPP_PRIMARY_HEADER_LEN + (HOST_TO_BE16(space_packet.header.length) + 1);
  downlinkRadioTransmitNBlock((uint8_t *)&space_packet, total_len);
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

  const uint16_t total_len =
      SPP_PRIMARY_HEADER_LEN + (HOST_TO_BE16(space_packet.header.length) + 1);
  downlinkRadioTransmitNBlock((uint8_t *)&space_packet, total_len);
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

  const uint16_t total_len =
      SPP_PRIMARY_HEADER_LEN + (HOST_TO_BE16(space_packet.header.length) + 1);
  downlinkRadioTransmitNBlock((uint8_t *)&space_packet, total_len);
  logger_spp(&space_packet);
}

static void telemetrySPPTransmitFlash(void) {
  block_tx = true;
  const uint32_t total_chunks = (image_data_len + CHUNK_SIZE - 1) / CHUNK_SIZE;

  for (uint32_t i = 0; i < total_chunks; i++) {
    uint32_t offset = i * CHUNK_SIZE;
    uint32_t remaining = image_data_len - offset;
    uint32_t size = (remaining >= CHUNK_SIZE) ? CHUNK_SIZE : remaining;

    uint8_t buffer[MAX_PAYLOAD_CHUNK];
    uint16_t buff_offset = 0;

    memset(buffer, 0, sizeof(buffer));

    buffer[buff_offset++] = SPACECRAFT_ID;

    /* Ancillary Data Field */
    // Packet Index
    buffer[buff_offset++] = (uint8_t)(i & 0xFF);
    buffer[buff_offset++] = (uint8_t)((i >> 8) & 0xFF);
    // Offset
    buffer[buff_offset++] = (uint8_t)(offset & 0xFF);
    buffer[buff_offset++] = (uint8_t)((offset >> 8) & 0xFF);
    // Remaining
    buffer[buff_offset++] = (uint8_t)(remaining & 0xFF);
    buffer[buff_offset++] = (uint8_t)((remaining >> 8) & 0xFF);

    if (buff_offset + size + 1 > MAX_PAYLOAD_CHUNK) {
      Serial.println("[ERROR] Payload overflow");
      block_tx = false;
      return;
    }

    // Data
    memcpy(&buffer[buff_offset], &image_data[offset], size);
    buff_offset += size;

    // CRC
    buffer[buff_offset++] = crc8_compute(&image_data[offset], size);
    buffer[buff_offset++] = '\0';

    space_packet_t space_packet;
    const uint8_t flag = i == 0                    ? SPP_GROUP_FLAG_START
                         : (i == total_chunks - 1) ? SPP_GROUP_FLAG_END
                                                   : SPP_GROUP_FLAG_CONT;
    int ret = spp_tm_build_packet(&space_packet, flag, SPP_SECHEAD_FLAG_PRESENT,
                                  6, SPP_APID_TM_FLASH, buffer, buff_offset);
    if (ret != SPP_ERROR_NONE) {
      Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
      Serial.println(ret);
      ledBlink(8, LED_COLOR_RED);
      block_tx = false;
      return;
    }

    const uint16_t total_len =
        SPP_PRIMARY_HEADER_LEN + (HOST_TO_BE16(space_packet.header.length) + 1);
    if (!downlinkRadioTransmit((uint8_t *)&space_packet, total_len)) {
      Serial.print("[ERROR] Transmiting: ");
      Serial.println(ret);
      ledBlink(8, LED_COLOR_RED);
      block_tx = false;
      return;
    }
    logger_spp(&space_packet);

    delay(100);
  }
  block_tx = false;
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
  if (block_tx) {
    return;
  }

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
  if (millis() - t_radio_idle.previous > t_radio_idle.interval) {
    t_radio_idle.previous = millis();
    telemetrySPPTransmitIDLE();
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
    size_t payload_total = space_packet->header.length + 1;
    size_t msg_len = payload_total - 2;
    uint8_t buffer_msg[SPP_MAX_PAYLOAD_CHUNK] = {0};

    memcpy(buffer_msg, space_packet->data + 2, msg_len);

    space_packet_t space_packet;
    const int ret = spp_tm_build_packet(
        &space_packet, SPP_GROUP_FLAG_UNSEGMENTED, SPP_SECHEAD_FLAG_NOPRESENT,
        0, SPP_APID_TM_BROADCAST_MSG, buffer_msg, msg_len);
    if (ret != SPP_ERROR_NONE) {
      Serial.print("[ERROR] Telemetry SPP Pack Frame: ");
      Serial.println(ret);
      ledBlink(8, LED_COLOR_RED);
      return;
    }
    const uint16_t total_len =
        SPP_PRIMARY_HEADER_LEN + (HOST_TO_BE16(space_packet.header.length) + 1);
    downlinkRadioTransmitBroadcast(frequency, (uint8_t *)&space_packet,
                                   total_len);
    logger_spp(&space_packet);
  } else if (apid == SPP_APID_TM_FLASH) {
    ledBlink(4, LED_COLOR_YELLOW);
    telemetrySPPTransmitFlash();
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
  logger_spp_tc(&space_packet);
  commandApidHandler(&space_packet);
}