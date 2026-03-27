/*  - rdownlink.cpp
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "rdownlink.h"
#include "led.h"
#include "pins.h"
#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

SX1262 radio0 = new Module(5, 8, 11, 14, SPI);

static radioPacketReceivedCb radi_recv_cb = NULL;

volatile bool receivedFlag = false;
volatile bool enableInterruptRadio = true;

static void log_line(const char *type, const char *msg) {
  Serial.printf("[%s] %s\n", type, msg);
}

static void radio_received_flag(void) {
  if (!enableInterruptRadio) {
    return;
  }
  receivedFlag = true;
}

void downlinkRadioRegisterCb(radioPacketReceivedCb recv_cb) {
  radi_recv_cb = recv_cb;
}

void downlinkRadioConfigure(void) {
  SPI.setSCK(PIN_SPI_RADIO1_SCK);
  SPI.setTX(PIN_SPI_RADIO1_COPI);
  SPI.setRX(PIN_SPI_RADIO1_CIPO);
  SPI.begin();

  int state = radio0.begin();
  if (state != RADIOLIB_ERR_NONE) {
    log_line("ERROR", "Radio 0 Init Failed");
    Serial.printf("Error: %d\n", state);
    return;
  }
  state = radio0.setFrequency(DOWNLINK_FREQ);
  if (state != RADIOLIB_ERR_NONE) {
    log_line("ERROR", "Radio 0 Frequency set error");
    Serial.printf("Error: %d\n", state);
  }
  radio0.setBandwidth(DOWNLINK_BW);
  radio0.setSpreadingFactor(DOWNLINK_SF);
  radio0.setCodingRate(DOWNLINK_CR);
  radio0.setRfSwitchPins(9, 29);
  // radio0.setRfSwitchPins(28, 24);
  radio0.setPacketReceivedAction(radio_received_flag);
  radio0.explicitHeader();
  radio0.setCRC(0);
  radio0.startReceive();
  log_line("INFO", "Radio 0 Configured Successfully!");
}

void downlinkRadioCheckPacketReceived(void) {
  if (receivedFlag) {
    receivedFlag = false;
    enableInterruptRadio = false;

    int recvLen = radio0.getPacketLength();
    byte byteArr[recvLen];
    int state = radio0.readData(byteArr, recvLen);
    if (state == RADIOLIB_ERR_NONE || state == RADIOLIB_ERR_CRC_MISMATCH) {
      if (radi_recv_cb != NULL) {
        radi_recv_cb(byteArr, recvLen);
      } else {
        Serial.print("[INFO] Radio 0 Recv: ");
        Serial.write(byteArr, recvLen);
        Serial.println();
        Serial.print("[INFO] Radio 0 RSSI: ");
        Serial.println(radio0.getRSSI());
        Serial.print("[INFO] Radio 0 SNR: ");
        Serial.println(radio0.getSNR());
      }
      ledBlink(8, LED_COLOR_BLUE);
    } else {
      Serial.print("[SYS - Radio] Recv Error: ");
      Serial.println(state);
    }

    radio0.startReceive();
    enableInterruptRadio = true;
  }
}