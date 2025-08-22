#include <Arduino.h>
#include <bus.h>
#include <commands.h>
#include <leds.h>
#include <radio_wrapper.h>

void setup() {
  Serial.begin(115200);
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