#include "commands.h"
#include "mission.h"
#include "radio_wrapper.h"
#include "workers.h"

// Dont work

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.flush();
  radioConfigure();
  radioRegisterCb(commandHandler);
}

void loop() { radioCheckPacketReceived(); }