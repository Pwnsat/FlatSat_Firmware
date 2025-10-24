#include "radio_wrapper.h"
#include "leds.h"
#include "proto.h"
typedef struct {
  float frequency;
  float bandwidth;
  uint8_t spreadingFactor;
  uint8_t preambleLength;
  uint8_t codingRate;
} radio_config_t;

#define CTF1 14
#define CTF2 11
#define CTF3 10
#define RXEN 16
#define TXEN 15

#define RADIO_NSS 17
#define RADIO_DIO1 3
#define RADIO_NRST 8
#define RADIO_BUSY 2

SX1262 radio = new Module(17, 3, 8, 2);

static radio_config_t radio_cfg;
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
  radio_cfg.frequency = 916;
  radio_cfg.bandwidth = 250;
  radio_cfg.spreadingFactor = 12;
  radio_cfg.preambleLength = 8;
  radio_cfg.codingRate = 5;
}

void radioConfigure() {
  pinMode(CTF1, OUTPUT);
  pinMode(CTF2, OUTPUT);
  pinMode(CTF3, OUTPUT);

  digitalWrite(CTF1, HIGH);
  digitalWrite(CTF2, LOW);
  digitalWrite(CTF3, LOW);

  Serial.println("[SYS] Radio Uplink init");
  int state = radio.begin(916, 250, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                          10, 8, 0, false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[SYS] Radio Uplink OK");
  } else {
    Serial.print("[SYS] Radio Uplink Error: ");
    Serial.println(state);
  }
  radio.setDio1Action(radio_received_flag);
  radio.setRfSwitchPins(16, 15);
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

void radioTransmit(uint8_t *buffer, uint16_t buffer_len) {
  // Put in a TC frequency
  radio.setFrequency(918);
  ledsBlink(4, 50);
  int state = radio.transmit(buffer, buffer_len);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("[SYS - Radio] Transmited: ");
    Serial.write(buffer, buffer_len);
    Serial.println();
  } else {
    Serial.print("[SYS - Radio] Transmit Error: ");
    Serial.println(state);
  }
  ledsBlink(4, 50);
  // Back to TM frequency
  radio.setFrequency(916);
  radio.startReceive();
}

void radioRegisterCb(radioPacketReceivedCb recv_cb) { radi_recv_cb = recv_cb; }