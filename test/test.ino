#include "SparkFun_LIS2DH12.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <RadioLib.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_ADDR 0x76

Adafruit_BME280 bme;
SPARKFUN_LIS2DH12 accel;

SX1262 radio0 = new Module(17, 3, 27, 26);
SX1262 radio1 = new Module(5, 8, 11, 14);

volatile bool radio0Flag = false;
volatile bool radio1Flag = false;

bool radio0_ok = false;
bool radio1_ok = false;

void setFlag0(void) { radio0Flag = true; }
void setFlag1(void) { radio1Flag = true; }

void log_line(const char *type, const char *msg) {
  Serial.print(type);
  Serial.print("|");
  Serial.print(millis());
  Serial.print("|");
  Serial.println(msg);
}

void log_acc(float x, float y, float z, float temp) {
  Serial.print("ACC|");
  Serial.print(millis());
  Serial.print("|");
  Serial.print(x, 3);
  Serial.print("|");
  Serial.print(y, 3);
  Serial.print("|");
  Serial.print(z, 3);
  Serial.print("|");
  Serial.println(temp, 2);
}

void log_bme(float t, float p, float alt, float hum) {
  Serial.print("BME|");
  Serial.print(millis());
  Serial.print("|");
  Serial.print(t, 2);
  Serial.print("|");
  Serial.print(p, 2);
  Serial.print("|");
  Serial.print(alt, 2);
  Serial.print("|");
  Serial.println(hum, 2);
}

bool accelerometerRead(float *x, float *y, float *z, float *temp) {
  *x = accel.getX();
  *y = accel.getY();
  *z = accel.getZ();
  *temp = accel.getTemperature();

  if (isnan(*x) || isnan(*y) || isnan(*z)) {
    return false;
  }
  return true;
}

bool bmeRead(float *t, float *p, float *alt, float *hum) {
  *t = bme.readTemperature();
  *p = bme.readPressure() / 100.0F;
  *alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  *hum = bme.readHumidity();

  if (isnan(*t) || isnan(*p) || isnan(*alt)) {
    return false;
  }
  return true;
}

void testAccelerometer(void) {
  if (!accel.begin()) {
    log_line("ERR", "ACC_NOT_DETECTED");
    return;
  }

  float x, y, z, t;
  if (!accelerometerRead(&x, &y, &z, &t)) {
    log_line("ERR", "ACC_READ_FAIL");
    return;
  }

  log_acc(x, y, z, t);
}

void testBME(void) {
  if (!bme.begin(BME_ADDR)) {
    log_line("ERR", "BME_NOT_DETECTED");
    Serial.print("BME_ID|");
    Serial.println(bme.sensorID(), HEX);
    return;
  }

  float t, p, alt, hum;
  if (!bmeRead(&t, &p, &alt, &hum)) {
    log_line("ERR", "BME_READ_FAIL");
    return;
  }

  log_bme(t, p, alt, hum);
}

void initRadios() {
  int state;

  state = radio0.begin();
  if (state == RADIOLIB_ERR_NONE) {
    radio0_ok = true;
    log_line("INFO", "RADIO0_OK");
    radio0.setPacketReceivedAction(setFlag0);
  } else {
    log_line("ERR", "RADIO0_FAIL");
    Serial.println(state);
  }

  state = radio1.begin();
  if (state == RADIOLIB_ERR_NONE) {
    radio1_ok = true;
    log_line("INFO", "RADIO1_OK");
    radio1.setPacketReceivedAction(setFlag1);
  } else {
    log_line("ERR", "RADIO1_FAIL");
    Serial.println(state);
  }

  if (radio0_ok)
    radio0.startReceive();
  if (radio1_ok)
    radio1.startReceive();
}

void handleRadio0() {
  if (!radio0Flag)
    return;
  radio0Flag = false;

  String str;
  int state = radio0.readData(str);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("RAD0|");
    Serial.print(millis());
    Serial.print("|RX|");
    Serial.println(str);
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    log_line("WARN", "RAD0_CRC");
  } else {
    log_line("ERR", "RAD0_RX_FAIL");
  }

  radio0.startReceive();
}

void handleRadio1() {
  if (!radio1Flag)
    return;
  radio1Flag = false;

  String str;
  int state = radio1.readData(str);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("RAD1|");
    Serial.print(millis());
    Serial.print("|RX|");
    Serial.println(str);
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    log_line("WARN", "RAD1_CRC");
  } else {
    log_line("ERR", "RAD1_RX_FAIL");
  }

  radio1.startReceive();
}

void radioSend(SX1262 &radio, const char *name, String msg) {
  int state = radio.transmit(msg);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(name);
    Serial.print("|");
    Serial.print(millis());
    Serial.print("|TX|OK");
    Serial.println();
  } else {
    Serial.print(name);
    Serial.print("|");
    Serial.print(millis());
    Serial.print("|TX|ERR|");
    Serial.println(state);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  delay(1000);

  log_line("INFO", "BOOT");

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  log_line("INFO", "I2C_OK");

  testAccelerometer();
  testBME();

  initRadios();

  delay(1000);

  log_line("INFO", "START_TEST");

  if (radio1_ok)
    radioSend(radio1, "RAD1", "PING");
}

void loop() {
  handleRadio0();
  handleRadio1();

  static uint32_t last = 0;

  if (millis() - last > 2000) {
    last = millis();

    if (radio0_ok)
      radioSend(radio0, "RAD0", "PING");
  }
}