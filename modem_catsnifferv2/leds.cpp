#include "leds.h"

#define LED1 (25)
#define LED2 (27)
#define LED3 (1)

void ledsConfigure(void) {
  // pinMode(LED1, OUTPUT);
  // pinMode(LED2, OUTPUT);
  // pinMode(LED3, OUTPUT);

  void ledsTurnOff(void);
}

void ledsTurnOff(void) {
  // digitalWrite(LED1, 0);
  // digitalWrite(LED2, 0);
  // digitalWrite(LED3, 0);
}

void ledsTurnOn(void) {
  // digitalWrite(LED1, 1);
  // digitalWrite(LED2, 1);
  // digitalWrite(LED3, 1);
}

void ledsBlink(int count, int time_off) {
  for (int i = 0; i < count; i++) {
    ledsTurnOff();
    delay(time_off);
    ledsTurnOn();
    delay(time_off);
  }
  ledsTurnOff();
}