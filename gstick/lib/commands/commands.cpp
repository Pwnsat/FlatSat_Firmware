#include <commands.h>
#include <leds.h>
#include <radio_wrapper.h>
#include <spp.h>
#define BINARY_START_BYTE 0x7E
#define MAX_PACKET_SIZE 256

static const uint8_t cmd_get[CMD_GET_LEN] = {GROUND_STICK_ID, 0x00};
static const uint8_t ping_sync[CMD_PING_SYNC_LEN] = {
    GROUND_STICK_ID, 0x53, 0x59, 0x4e, 0x43, 0x00};
static const uint8_t ping_ack[CMD_PING_ACK_LEN] = {GROUND_STICK_ID, 0x41, 0x43,
                                                   0x4b, 0x00};

SerialCommand SCmd;

static void processBinary(uint8_t *data, uint8_t len) {
  uint8_t offset = 0;
  uint8_t segment = data[offset++];
  space_packet_t packet;
  spp_tc_build_packet(&packet, segment, SPP_SECHEAD_FLAG_NOPRESENT, 0,
                      APID_TC_FIRMWARE_UPDATE, data + offset, len - offset);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
}

static void handleBinary() {
  uint8_t start = Serial.read(); // 0x7E
  uint8_t len = Serial.read();   // len
  uint8_t payload[MAX_PACKET_SIZE];
  int i = 0;
  while (i < len && Serial.available()) {
    payload[i++] = Serial.read();
  }
  if (i == len) {
    processBinary(payload, len);
  }
}

void commandHandler(void) {
  if (Serial.available()) {
    uint8_t b = Serial.peek();
    if (b == BINARY_START_BYTE || b == 49) {
      handleBinary();
    } else {
      SCmd.readSerial();
    }
  }
}

/* COMMANDS FUNCTIONS */
/** @brief APID_TC_PING_SYNC */
static void commandSendPingSync(void) {
  space_packet_t packet;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TC_PING_SYNC,
                      ping_sync, CMD_PING_SYNC_LEN);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.write(BINARY_START_BYTE);
  Serial.write(CMD_RES_PING);
  Serial.println("");
  ledsBlink(3, 500);
}
/** @brief APID_TC_PING_ACK */
static void commandSendPingAck(void) {
  space_packet_t packet;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TM_PING_ACK, ping_ack,
                      CMD_PING_ACK_LEN);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.write(BINARY_START_BYTE);
  Serial.write(CMD_RES_PING);
  Serial.println("");
  ledsBlink(3, 500);
}
/** @brief APID_TC_GET_STATUS */
static void commandSendGetStatus(void) {
  space_packet_t packet;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TC_GET_STATUS,
                      cmd_get, CMD_GET_LEN);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.write(BINARY_START_BYTE);
  Serial.write(CMD_RES_STATUS);
  Serial.println("");
  ledsBlink(3, 500);
}
/** @brief APID_TC_GET_TEMP */
static void commandSendGetTemp(void) {
  space_packet_t packet;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TC_GET_TEMP, cmd_get,
                      CMD_GET_LEN);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.write(BINARY_START_BYTE);
  Serial.write(CMD_RES_GETTEMP);
  Serial.println("");
  ledsBlink(3, 500);
}
/** @brief APID_TC_GET_GYRO */
static void commandSendGetGyro(void) {
  space_packet_t packet;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TC_GET_GYRO, cmd_get,
                      CMD_GET_LEN);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.write(BINARY_START_BYTE);
  Serial.write(CMD_RES_GETGYRO);
  Serial.println("");
  ledsBlink(3, 500);
}
/** @brief APID_TC_GET_TM */
static void commandSendGetTelemetry(void) {
  space_packet_t packet;
  spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                      SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TC_GET_TM, cmd_get,
                      CMD_GET_LEN);
  radioTransmit((uint8_t *)&packet,
                (SPP_PRIMARY_HEADER_LEN + packet.header.length));
  Serial.write(BINARY_START_BYTE);
  Serial.write(CMD_RES_GETTM);
  Serial.println("");
  ledsBlink(3, 500);
}

static void commandSendMessage(void) {
  char *arg = SCmd.next();
  if (arg != NULL) {
    space_packet_t packet;
    spp_tc_build_packet(&packet, SPP_GROUP_FLAG_UNSEGMENTED,
                        SPP_SECHEAD_FLAG_NOPRESENT, 0, APID_TC_GET_TM,
                        (const uint8_t *)arg, strlen(arg));
    radioTransmit((uint8_t *)&packet,
                  (SPP_PRIMARY_HEADER_LEN + packet.header.length));
    Serial.write(BINARY_START_BYTE);
    Serial.write(CMD_RES_GETTM);
    Serial.println("");
    ledsBlink(3, 500);
  }
}

static void serialDefaultHandler(const char *arg) { Serial.println("Unknown"); }

void commandInit(void) {
  SCmd.addCommand(CMD_PING, commandSendPingSync);
  SCmd.addCommand(CMD_STATUS, commandSendGetStatus);
  SCmd.addCommand(CMD_GETTEMP, commandSendGetTemp);
  SCmd.addCommand(CMD_GETGYRO, commandSendGetGyro);
  SCmd.addCommand(CMD_GETTM, commandSendGetTelemetry);
  SCmd.addCommand(CMD_SEND_MSG, commandSendGetTelemetry);
  SCmd.setDefaultHandler(serialDefaultHandler);
}