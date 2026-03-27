/*  - ruplink.cpp
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "ruplink.h"
#include "led.h"
#include "pins.h"
#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

static SPIClassRP2040 spiRadio1(spi0,
    PIN_SPI_RADIO0_CIPO,
    PIN_RADIO0_NSS,
    PIN_SPI_RADIO0_SCK,
    PIN_SPI_RADIO0_COPI);
SX1262 radio1 = new Module(PIN_RADIO0_NSS, PIN_RADIO0_DIO1, PIN_RADIO0_RST,
                            PIN_RADIO0_BSY, spiRadio1);

static void log_line(const char *type, const char *msg) {
  Serial.printf("[%s] %s\n", type, msg);
}

void uplinkRadioConfigure(void) {
  spiRadio1.begin();

  int state = radio1.begin();
  if (state != RADIOLIB_ERR_NONE) {
    log_line("ERROR", "Radio 1 Init Failed");
    Serial.printf("Error: %d\n", state);
    return;
  }
  state = radio1.setFrequency(UPLINK_FREQ);
  if (state != RADIOLIB_ERR_NONE) {
    log_line("ERROR", "Radio 1 Frequency set error");
    Serial.printf("Error: %d\n", state);
  }
  radio1.setBandwidth(UPLINK_BW);
  radio1.setSpreadingFactor(UPLINK_SF);
  radio1.setCodingRate(UPLINK_CR);
  radio1.setOutputPower(22);
  radio1.setRfSwitchPins(RADIOLIB_NC, PIN_RADIO0_ANT_SW);
  log_line("INFO", "Radio 1 Configured Successfully!");
}

bool uplinkRadioTransmit(uint8_t *buffer, uint16_t buffer_len) {
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