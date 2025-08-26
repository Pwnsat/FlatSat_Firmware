#include "bus.h"
#include "commands.h"
#include "leds.h"
#include "radio_wrapper.h"
#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  ledsConfigure();
  radioConfigure();
  commandInit();
  radioRegisterCb(telemetryHandleRecvData);
}

void loop() {
  radioCheckPacketReceived();
  commandHandler();
}