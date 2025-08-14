#include <Arduino.h>
#include <bme_wrapper.h>
#include <commands.h>
#include <mission.h>
#include <mpu_wrapper.h>
#include <radio_wrapper.h>
#include <workers.h>

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  radioConfigure();
  telemetryConfigureSensors();
  radioRegisterCb(commandHandler);
}

void loop() {
  radioCheckPacketReceived();
  telemetryWorker();
  telemetryIdleWorker();
}