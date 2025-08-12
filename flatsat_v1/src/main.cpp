#include <Arduino.h>
#include <radio_wrapper.h>
#include <workers.h>
#include <bme_wrapper.h>
#include <mpu_wrapper.h>
#include <mission.h>
#include <commands.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);

  radioConfigure();
  telemetryConfigureSensors();
  radioRegisterCb(commandHandler);
}

void loop() {
  radioCheckPacketReceived();
  telemetryWorker();
  telemetryIdleWorker();
}