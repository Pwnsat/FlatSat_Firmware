#include <bme_wrapper.h>
#include <commands.h>
#include <mission.h>
#include <mpu_wrapper.h>
#include <proto.h>
#include <radio_wrapper.h>
#include <spp.h>

#define MAX_PACKET_SIZE 256

static const uint8_t ping_sync[CMD_PING_SYNC_LEN] = {SPACECRAFT_ID, 0x53, 0x59,
                                                     0x4e,          0x43, 0x00};
static const uint8_t ping_ack[CMD_PING_ACK_LEN] = {SPACECRAFT_ID, 0x41, 0x43,
                                                   0x4b, 0x00};

/* COMMANDS FUNCTIONS */
/** @brief APID_TC_PING_SYNC */
static void commandSendPingSync(void) {
  space_packet_t packet;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TC_PING_SYNC,
                      ping_sync, CMD_PING_SYNC_LEN);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.println("[SYS - CMD] Response: APID_TC_PING_SYNC");
}
/** @brief APID_TM_PING_ACK */
static void commandSendPingAck(void) {
  space_packet_t packet;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TM_PING_ACK, ping_ack,
                      CMD_PING_ACK_LEN);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.println("[SYS - CMD] Response: APID_TM_PING_ACK");
}
/** @brief APID_TM_SEND_STATUS */
static void commandSendStatus(void) {
  space_packet_t packet;
  const uint8_t buffer[11] = {SPACECRAFT_ID, 0x50, 0x75, 0x72, 0x61, 0x20,
                              0x56,          0x69, 0x64, 0x61, 0x00};
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TM_SEND_STATUS,
                      buffer, 11);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.println("[SYS - CMD] Response: APID_TM_SEND_STATUS");
}
/** @brief APID_TM_SEND_TEMP */
static void commandSendTemp(void) {
  space_packet_t packet;
  uint8_t buffer[MAX_PAYLOAD_CHUNK];
  int offset = 0;
  buffer[offset++] = SPACECRAFT_ID;
  bmeReadData(buffer, offset);
  buffer[offset++] = 0x00;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TM_SEND_TEMP, buffer,
                      offset);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.println("[SYS - CMD] Response: APID_TM_SEND_TEMP");
}

/** @brief APID_TM_SEND_GYRO */
static void commandSendGyro(void) {
  space_packet_t packet;
  uint8_t buffer[MAX_PAYLOAD_CHUNK];
  int offset = 0;
  buffer[offset++] = SPACECRAFT_ID;
  mpuReadData(buffer, offset);
  buffer[offset++] = 0x00;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TM_SEND_GYRO, buffer,
                      offset);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.println("[SYS - CMD] Response: APID_TM_SEND_GYRO");
}

/** @brief APID_TM_SEND_TM */
static void commandSendTelemetry(void) {
  space_packet_t packet;
  uint8_t buffer[MAX_PAYLOAD_CHUNK];
  int offset = 0;
  buffer[offset++] = SPACECRAFT_ID;
  bmeReadData(buffer, offset);
  mpuReadData(buffer, offset);
  buffer[offset++] = 0x00;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TM_SEND_TM, buffer,
                      offset);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  radioTransmitToModem((uint8_t *)&packet,
                       (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.println("[SYS - CMD] Response: APID_TM_SEND_TM");
}

static void missionHandleData(space_packet_t *space_packet) {
  Serial.println("Packet");
  int apid = space_packet->header.identification & 0x7FF;
  if (apid == APID_TC_FIRMWARE_UPDATE) {
    Serial.print("S@");
    uint16_t seq_flags = (space_packet->header.sequence >> 14) & 0x3;
    Serial.write(seq_flags);
    Serial.write(space_packet->data, space_packet->header.length);
    Serial.println("E@");
  } else {
    Serial.print("T@");
    Serial.write(space_packet->data, space_packet->header.length);
    Serial.println("E@");
  }
}

void commandHandler(uint8_t *buffer, uint16_t buffer_len) {
  space_packet_t space_packet;
  int ret = spp_unpack_packet(&space_packet, buffer, buffer_len);
  if (ret != SPP_ERROR_NONE) {
    Serial.print("[SYS - MISSION] Error unpacking SPP: ");
    Serial.println(ret);
    Serial.write(buffer, buffer_len);
    Serial.println("");
    return;
  }

  missionHandleData(&space_packet);
}