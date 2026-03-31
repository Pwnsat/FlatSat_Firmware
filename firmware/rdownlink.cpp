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

static SPIClassRP2040 spiRadio1(spi0, PIN_SPI_RADIO1_CIPO, PIN_RADIO1_NSS,
                                PIN_SPI_RADIO1_SCK, PIN_SPI_RADIO1_COPI);
SX1262 radio1 = new Module(PIN_RADIO1_NSS, PIN_RADIO1_DIO1, PIN_RADIO1_RST,
                           PIN_RADIO1_BSY, spiRadio1);

static int transmissionState = RADIOLIB_ERR_NONE;

volatile bool transmittedFlag = false;

static void setTransmitionFlag(void) { transmittedFlag = true; }

static void log_line(const char *type, const char *msg) {
  Serial.printf("[%s] %s\n", type, msg);
}

void downlinkRadioConfigure(void) {
  spiRadio1.begin();

  int state = radio1.begin();
  if (state != RADIOLIB_ERR_NONE) {
    log_line("ERROR", "Radio 1 Init Failed");
    Serial.printf("Error: %d\n", state);
    return;
  }
  radio1.setFrequency(DOWNLINK_FREQ);
  radio1.setBandwidth(DOWNLINK_BW);
  radio1.setSpreadingFactor(DOWNLINK_SF);
  radio1.setCodingRate(DOWNLINK_CR);
  radio1.setRfSwitchPins(RADIOLIB_NC, PIN_RADIO1_ANT_SW);
  radio1.setPacketSentAction(setTransmitionFlag);
  log_line("INFO", "Radio 1 Configured Successfully!");
}

bool downlinkRadioTransmit(uint8_t *buffer, uint16_t buffer_len) {
  int state = radio1.transmit(buffer, buffer_len);
  if (state != RADIOLIB_ERR_NONE) {
    log_line("ERROR", "Radio 1 Transmition Error!");
    Serial.printf("Error: %d\n", state);
    ledBlink(8, LED_COLOR_RED);
    return false;
  }
  ledBlink(8, LED_COLOR_WHITE);
  return true;
}

void downlinkRadioTransmitNBlock(uint8_t *buffer, uint16_t buffer_len) {
  transmissionState = radio1.startTransmit(buffer, buffer_len);
}

void downlinkRadioCheckTransmition(void) {
  if (transmittedFlag) {
    transmittedFlag = false;
    if (transmissionState != RADIOLIB_ERR_NONE) {
      log_line("ERROR", "Radio 1 Transmition Error!");
      Serial.printf("Error: %d\n", transmissionState);
      ledBlink(8, LED_COLOR_RED);
    }
    radio1.finishTransmit();
  }
}