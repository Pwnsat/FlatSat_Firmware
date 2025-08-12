#include <Arduino.h>
#include <radio_wrapper.h>
#include <commands.h>
#include <leds.h>
#include <telemetry.h>

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  ledsConfigure();
  radioConfigure();
  commandInit();
  radioRegisterCb(telemetryHandleRecvData);
}

void loop() {
  radioCheckPacketReceived();
  commandHandler();
}