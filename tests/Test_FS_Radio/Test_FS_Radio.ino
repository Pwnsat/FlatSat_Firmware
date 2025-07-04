/*
Requirements:
RadioLib

Test Radio function and LEDS
*/

#include <RadioLib.h>

#define LED_PIN D0

// Uplink
#define SX_UL_NSS 2
#define SX_UL_BUSY 15

// Downlink
#define SX_DL_NSS 3
#define SX_DL_BUSY 16

//                                (NSS, DIO1, RESET, BUSY)
// IDK about the DIO1 and Reset pins
SX1262 radioUplink    = new Module(SX_UL_NSS, D1, D2, SX_UL_BUSY);
SX1262 radioDownlink  = new Module(SX_DL_NSS, D1, D2, SX_DL_BUSY);

void setup() {
  Serial.begin(115200);
  while(!Serial)

  // Led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Radio setup
  int state = radioUplink.begin();
  if (state == RADIOLIB_ERR_NONE){
    Serial.println("[UPLINK] OK");
  }else{
    Serial.print("[UPLINK] Error: ");
    Serial.println(state);
  }
  state = radioDownlink.begin();
  if (state == RADIOLIB_ERR_NONE){
    Serial.println("[DOWNLINK] OK");
  }else{
    Serial.print("[DOWNLINK] Error: ");
    Serial.println(state);
  }
}

void loop() {

  // Led Blink
  digitalWrite(LED_PIN, LOW);
  delay(5000);
  digitalWrite(LED_PIN, HIGH);
}
