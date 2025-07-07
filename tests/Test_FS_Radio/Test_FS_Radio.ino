/*
Requirements:
RadioLib

Test Radio function and LEDS
*/

#include <RadioLib.h>

#define LED_PIN D0

// Uplink
#define SX_UL_NSS D1
#define SX_UL_BUSY 39
#define SX_UL_IO -1
#define SX_UL_RST -1

// Downlink
#define SX_DL_NSS D2
#define SX_DL_BUSY 38
#define SX_DL_IO -1
#define SX_DL_RST -1

//                                (NSS, DIO1, RESET, BUSY)
// IDK about the DIO1 and Reset pins
SX1262 radioUplink    = new Module(SX_UL_NSS, SX_UL_IO, -1, SX_UL_BUSY);
SX1262 radioDownlink  = new Module(SX_DL_NSS, SX_DL_IO, -1, SX_DL_BUSY);

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
