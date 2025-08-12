#include <telemetry.h>
#include <spp.h>
#include <leds.h>
#include <proto.h>

#define BINARY_START_BYTE 0x7E

void telemetryHandleRecvData(uint8_t* buffer, uint16_t buffer_len){
  ledsBlink(2, 50);
  space_packet_t packet;
  int ret = spp_unpack_packet(&packet, buffer, buffer_len);
  if (ret != SPP_ERROR_NONE){
    Serial.print("[SYS - MISSION] Error unpacking SPP: ");
    Serial.println(ret);
    return;
  }
  int apid = packet.header.identification & 0x7FF;
  Serial.print(apid);
  Serial.write("@");
  Serial.write(";");
  Serial.write(packet.data, packet.header.length);
  Serial.println("");
}