/* FlatSat Test firmware for LoRa and Sensors Telemetry
  July 2025
*/
#include <RadioLib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>

#define FIRMWARE_VERSION "0.0.1"

#define SX_NSS_1  D1
#define SX_BUSY_1 D11
#define SX_NSS_2  D2
#define SX_BUSY_2 D12
// Sensor Specific
#define SEALEVELPRESSURE_HPA (1013.25)
// Telemetry Structure
#define TM_SEPARATOR ","
#define SENSOR_SEPARATOR ";"

typedef enum {
  SENSOR_FAIL = -1,
  SENSOR_OK = 0
} sensor_status_t;

typedef struct {
  sensor_status_t mpuStatus;
  sensor_status_t bmeStatus;
} sensor_context_t;

typedef struct {
  int tmStatus;
  int tcStatus;
} radio_context_t;


sensor_context_t sensorContext;
radio_context_t radioContext;

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

SX1262 uplink = new Module(SX_NSS_1, SX_BUSY_1, -1, -1);
SX1262 downlink = new Module(SX_NSS_2, SX_BUSY_2, -1, -1);

const uint8_t SPACECRAFT_ID = 0x01;

static sensor_status_t initMPU(){
  Serial.println("[SYS - MPU] MPU6050 Init");
  if (!mpu.begin(0x69)) {
    Serial.println("[SYS - MPU] Failed to find MPU6050 chip");
    return SENSOR_FAIL;
  }
  Serial.println("[SYS - MPU] MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("[SYS - MPU] Accelerometer range set to: +-8G");
  Serial.println("[SYS - MPU] Gyro range set to: +- 500 deg/s");
  Serial.println("[SYS - MPU] Filter bandwidth set to: 21 Hz");
  return SENSOR_OK;
}

static sensor_status_t initBME(){
  Serial.println("[SYS - BME] Init");
  if(!bme.begin(0x76)){
    Serial.println("[SYS - BME] Failed to find BME280 chip");
    Serial.print("[SYS - BME] SensorID was: 0x"); 
    Serial.println(bme.sensorID(), 16);
    return SENSOR_FAIL;
  }
  Serial.println("[MPU] MPU6050 Found!");
  return SENSOR_OK;
}

int16_t scaleFloatToInt16(float value, float min, float max) {
  if (value < min) value = min;
  if (value > max) value = max;
  float normalized = (value - min) / (max - min);
  return (int16_t)(normalized * 65535.0f - 32768.0f);
}

uint16_t scaleFloatToUInt16(float value, float min, float max) {
  if (value < min) value = min;
  if (value > max) value = max;
  float normalized = (value - min) / (max - min);
  return (uint16_t)(normalized * 65535.0f);
}

void appendInt16(uint8_t* buffer, uint8_t& index, int16_t value) {
  buffer[index++] = (value >> 8) & 0xFF;
  buffer[index++] = value & 0xFF;
}

void appendUInt16(uint8_t* buffer, uint8_t& index, uint16_t value) {
  buffer[index++] = (value >> 8) & 0xFF;
  buffer[index++] = value & 0xFF;
}

void appendUInt8(uint8_t* buffer, uint8_t& index, uint8_t value) {
  buffer[index++] = value;
}

void readMPU_binary(uint8_t* buffer, uint8_t& index) {
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    appendUInt8(buffer, index, 0); // Status FAIL
    return;
  }

  appendUInt8(buffer, index, 1); // Status OK

  appendInt16(buffer, index, scaleFloatToInt16(a.acceleration.x, -16, 16));
  appendInt16(buffer, index, scaleFloatToInt16(a.acceleration.y, -16, 16));
  appendInt16(buffer, index, scaleFloatToInt16(a.acceleration.z, -16, 16));
  appendInt16(buffer, index, scaleFloatToInt16(g.gyro.x, -250, 250));
  appendInt16(buffer, index, scaleFloatToInt16(g.gyro.y, -250, 250));
  appendInt16(buffer, index, scaleFloatToInt16(g.gyro.z, -250, 250));
  appendInt16(buffer, index, scaleFloatToInt16(temp.temperature, -40, 85));
}

void readBME_binary(uint8_t* buffer, uint8_t& index) {
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F; // hPa
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();

  appendInt16(buffer, index, scaleFloatToInt16(temp, -40, 85));
  appendUInt16(buffer, index, scaleFloatToUInt16(pressure, 300, 1100));
  appendInt16(buffer, index, scaleFloatToInt16(altitude, -500, 8000));
  appendUInt8(buffer, index, (uint8_t)(humidity));
}

static String readMPU(){
  String telemetry = "";
  sensors_event_t a, g, temp;
  if(!mpu.getEvent(&a, &g, &temp)){
    telemetry += "MPU:FAIL";
    sensorContext.mpuStatus = SENSOR_FAIL;
    return telemetry;
  }
  telemetry += "MPU:OK=";
  telemetry += String(a.acceleration.x, 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(a.acceleration.y, 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(a.acceleration.z, 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(g.gyro.x, 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(g.gyro.y, 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(g.gyro.z, 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(temp.temperature, 1);
  return telemetry;
}

static String readBME(){
  String telemetry = "";
  // TODO: Write a good check for the integrity
  telemetry += "BME:OK=";
  telemetry += String(bme.readTemperature(), 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(bme.readPressure() / 100.0F, 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(bme.readAltitude(SEALEVELPRESSURE_HPA), 1);
  telemetry += String(TM_SEPARATOR);
  telemetry += String(bme.readHumidity(), 1);
  return telemetry;
}

static String getSensorsTelemetry(){
  String mpuTelemetry = readMPU();
  String bmeTelemetry = readBME();
  String telemetry = mpuTelemetry;// + SENSOR_SEPARATOR + bmeTelemetry;
  return telemetry;
}

static void printHexDump(const uint8_t* data, size_t len) {
  const size_t bytesPerLine = 32;
  char ascii[bytesPerLine + 1];
  ascii[bytesPerLine] = '\0';

  for (size_t i = 0; i < len; i++) {
    if (i % bytesPerLine == 0) {
      if (i != 0) {
        Serial.print("  |");
        Serial.println(ascii);
      }
      Serial.printf("%08X  ", (unsigned int)i);
    }

    Serial.printf("%02X ", data[i]);
    ascii[i % bytesPerLine] = (data[i] >= 32 && data[i] <= 126) ? data[i] : '.';
  }

  size_t remaining = len % bytesPerLine;
  if (remaining > 0) {
    for (size_t i = remaining; i < bytesPerLine; i++) {
      Serial.print("   ");
    }
  }

  Serial.print("  |");
  ascii[remaining == 0 ? bytesPerLine : remaining] = '\0';
  Serial.println(ascii);
}

static void printStringHexDump(const String& input) {
  const size_t bytesPerLine = 32;
  char ascii[bytesPerLine + 1];
  ascii[bytesPerLine] = '\0';

  size_t len = input.length();
  const char* data = input.c_str();

  for (size_t i = 0; i < len; i++) {
    if (i % bytesPerLine == 0) {
      if (i != 0) {
        Serial.print("  |");
        Serial.println(ascii);
      }
      Serial.printf("%08X  ", (unsigned int)i);
    }

    Serial.printf("%02X ", (uint8_t)data[i]);
    ascii[i % bytesPerLine] = (data[i] >= 32 && data[i] <= 126) ? data[i] : '.';
  }

  size_t remaining = len % bytesPerLine;
  if (remaining > 0) {
    for (size_t i = remaining; i < bytesPerLine; i++) {
      Serial.print("   ");
    }
  }

  Serial.print("  |");
  ascii[remaining == 0 ? bytesPerLine : remaining] = '\0';
  Serial.println(ascii);
}


void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.print("\nFirmware version: ");
  Serial.println(FIRMWARE_VERSION);
  

  Serial.println("[SYS] Radio Uplink init");
  int state = uplink.begin();
  if (state== RADIOLIB_ERR_NONE){
    Serial.println("[SYS] Radio Uplink OK");
  }else{
    Serial.print("[SYS] Radio Uplink Error: ");
    Serial.println(state);
  }
  uplink.setFrequency(916);
  uplink.setBandwidth(250);
  uplink.setSpreadingFactor(12);
  uplink.setPreambleLength(8);
  uplink.setCodingRate(5);

  Serial.println("[SYS] Radio Downlink init");
  state = downlink.begin();
  if (state== RADIOLIB_ERR_NONE){
    Serial.println("[SYS] Radio Downlink OK");
  }else{
    Serial.print("[SYS] Radio Downlink Error: ");
    Serial.println(state);
  }
  downlink.setFrequency(916);
  downlink.setBandwidth(250);
  downlink.setSpreadingFactor(12);
  downlink.setPreambleLength(8);
  downlink.setCodingRate(5);

  downlink.startReceive();

  Serial.println("[SYS] Init sensors...");
  sensorContext.mpuStatus = initMPU();
  sensorContext.bmeStatus = initBME();
}

void loop() {
  // String telemetry = getSensorsTelemetry();

  uint8_t payload[64]; // o el tamaño que se necesite
  uint8_t idx = 0;

  payload[idx++] = SPACECRAFT_ID;

  readMPU_binary(payload, idx);
  readBME_binary(payload, idx);

  int state = uplink.transmit(payload, idx);
  if (state == RADIOLIB_ERR_NONE){
    Serial.print("[SYS - Radio] Transmited: ");
    Serial.write(payload, idx);
    Serial.println();
  }else{
    Serial.print("[SYS - Radio] Transmit Error: ");
    Serial.println(state);
  }
  delay(2000);
  int recvLen = downlink.getPacketLength();
  byte byteArr[recvLen];
  state = downlink.readData(byteArr, recvLen);
  if (state == RADIOLIB_ERR_NONE){
    if (byteArr[0] != SPACECRAFT_ID){
      Serial.println();
      Serial.print("[SYS - Radio] Recv: ");
      Serial.write(byteArr, recvLen);
      Serial.println();
      Serial.print("[SYS - Radio] RSSI: ");
      Serial.println(downlink.getRSSI());
      Serial.print("[SYS - Radio] SNR: ");
      Serial.println(downlink.getSNR());
      // printStringHexDump(data);
      printHexDump(byteArr, recvLen);
      Serial.println();
    }
  }else{
    Serial.print("[SYS - Radio] Recv Error: ");
    Serial.println(state);
  }
  delay(2000);
}