BME = 0x76
ACC = 0x19


```c

void loop() {
  int ch;

  ch = Serial.read();
  if (ch > 0) {
    printAll(ch);
  }

  ch = USBSer1.read();
  if (ch > 0) {
    printAll(ch);
  }

  if (delay_without_delaying(500)) {
    LEDstate = !LEDstate;
    #ifdef LED_BUILTIN
    digitalWrite(LED_BUILTIN, LEDstate);
    #endif
  }
}

// print to all CDC ports
void printAll(int ch) {
  // always lower case
  Serial.write(tolower(ch));

  // always upper case
  USBSer1.write(toupper(ch));
}

// Helper: non-blocking "delay" alternative.
boolean delay_without_delaying(unsigned long time) {
  // return false if we're still "delaying", true if time ms has passed.
  // this should look a lot like "blink without delay"
  static unsigned long previousmillis = 0;
  unsigned long currentmillis = millis();
  if (currentmillis - previousmillis >= time) {
    previousmillis = currentmillis;
    return true;
  }
  return false;
}

```