#include "radio_wrapper.h"
#include "proto.h"

#define SX_NSS_1 2
#define SX_BUSY_1 38
#define SX_NSS_2 3
#define SX_BUSY_2 39
#define SX_DIO1 11

#define CTF1 8
#define CTF2 9
#define CTF3 10

SX1262 radio = new Module(17, 3, 8, 2);

typedef struct {
  float frequency;
  float bandwidth;
  uint8_t spreadingFactor;
  uint8_t preambleLength;
  uint8_t codingRate;
  bool active = false;
} radio_config_t;

static radio_config_t uplink_cfg;
static radio_config_t downlink_cfg;

static radioPacketReceivedCb radi_recv_cb = NULL;

volatile bool receivedFlag = false;
volatile bool enableInterruptRadio = true;

static void radio_received_flag(void) {
  if (!enableInterruptRadio) {
    return;
  }
  receivedFlag = true;
}

static void radioSetInitialConfig(void) {
  uplink_cfg.frequency = 925;
  uplink_cfg.bandwidth = 250;
  uplink_cfg.spreadingFactor = 12;
  uplink_cfg.preambleLength = 8;
  uplink_cfg.codingRate = 5;
  uplink_cfg.active = false;

  downlink_cfg.frequency = 918;
  downlink_cfg.bandwidth = 250;
  downlink_cfg.spreadingFactor = 12;
  downlink_cfg.preambleLength = 8;
  downlink_cfg.codingRate = 5;
  downlink_cfg.active = false;
}

static void radioConfigureFSK() {
  Serial.println("[SYS] Radio Uplink init");
  int state = radio.beginFSK(925, 4.8, 5, 156.2, 10, 16, 1.6, false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[SYS] Radio Uplink OK");
    uplink_cfg.active = true;
    uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    radio.setSyncWord(syncWord, 8);
    radio.setCRC(2, 0xFFFF, 0x8005, false); // CRC igual al receptor
  } else {
    Serial.print("[SYS] Radio Uplink Error: ");
    Serial.println(state);
    uplink_cfg.active = false;
  }
}

void radioConfigure() {
  pinMode(CTF1, OUTPUT);
  pinMode(CTF2, OUTPUT);
  pinMode(CTF3, OUTPUT);

  digitalWrite(CTF1, HIGH);
  digitalWrite(CTF2, LOW);
  digitalWrite(CTF3, LOW);

  Serial.println("[SYS] Radio Uplink init");
  int state = radio.begin(925, 250, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                          10, 8, 0, false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[SYS] Radio Uplink OK");
  } else {
    Serial.print("[SYS] Radio Uplink Error: ");
    Serial.println(state);
  }
  radio.setDio1Action(radio_received_flag);
  radio.setRfSwitchPins(21, 20);
  radio.startReceive();
}

void radioCheckPacketReceived(void) {
  if (receivedFlag) {
    receivedFlag = false;
    enableInterruptRadio = false;

    int recvLen = radio.getPacketLength();
    byte byteArr[recvLen];
    int state = radio.readData(byteArr, recvLen);
    if (state == RADIOLIB_ERR_NONE) {
      if (radi_recv_cb != NULL) {
        radi_recv_cb(byteArr, recvLen);
      } else {
        Serial.print("[SYS - Radio] Recv: ");
        Serial.write(byteArr, recvLen);
        Serial.println();
        Serial.print("[SYS - Radio] RSSI: ");
        Serial.println(radio.getRSSI());
        Serial.print("[SYS - Radio] SNR: ");
        Serial.println(radio.getSNR());
      }
    } else {
      Serial.print("[SYS - Radio] Recv Error: ");
      Serial.println(state);
    }

    radio.startReceive();
    enableInterruptRadio = true;
  }
}

void radioTransmitToModem(uint8_t *buffer, uint16_t buffer_len) {
  if (!uplink_cfg.active) {
    return;
  }
  radio.setFrequency(925);
  delay(500);
  int state = radio.transmit(buffer, buffer_len);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("[SYS - Radio] Transmited: ");
    Serial.write(buffer, buffer_len);
    Serial.println();
  } else {
    Serial.print("[SYS - Radio] Transmit Error: ");
    Serial.println(state);
  }
  delay(500);
  radio.setFrequency(925);
}

void radioTransmit(uint8_t *buffer, uint16_t buffer_len) {
  if (!uplink_cfg.active) {
    return;
  }
  int state = radio.transmit(buffer, buffer_len);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("[SYS - Radio] Transmited: ");
    Serial.write(buffer, buffer_len);
    Serial.println();
  } else {
    Serial.print("[SYS - Radio] Transmit Error: ");
    Serial.println(state);
  }
}

void radioRegisterCb(radioPacketReceivedCb recv_cb) { radi_recv_cb = recv_cb; }

void radioPackConfig(uint8_t *buffer, int &offset) {
  proto_float_t freq_b;
  proto_float_t band_b;

  freq_b.f = uplink_cfg.frequency;
  band_b.f = uplink_cfg.bandwidth;

  memcpy(buffer + offset, freq_b.bytes, 4);
  offset += 4;
  memcpy(buffer + offset, band_b.bytes, 4);
  offset += 4;
  buffer[offset++] = uplink_cfg.codingRate;
  buffer[offset++] = uplink_cfg.spreadingFactor;
  buffer[offset++] = uplink_cfg.preambleLength;
}