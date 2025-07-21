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
#define SPACECRAFT_ID "COSMOS1"
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
    Serial.println("[SYS] Radio Uplink OK");
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
  String telemetry = getSensorsTelemetry();
  Serial.print("[SYS - Radio] Transmiting: ");
  Serial.println(telemetry);
  int state = uplink.transmit(telemetry);
  if (state == RADIOLIB_ERR_NONE){
    Serial.print("[SYS - Radio] Transmit: ");
    Serial.println(telemetry);
  }else{
    Serial.print("[SYS - Radio] Transmit Error: ");
    Serial.println(state);
  }
  delay(2000);
  String data;
  state = downlink.readData(data);
  if (state == RADIOLIB_ERR_NONE){
    Serial.println();
    Serial.print("[SYS - Radio] Recv: ");
    Serial.println(data);
    Serial.print("[SYS - Radio] RSSI: ");
    Serial.println(downlink.getRSSI());
    Serial.print("[SYS - Radio] SNR: ");
    Serial.println(downlink.getSNR());
    printStringHexDump(data);
    Serial.println();
  }else{
    Serial.print("[SYS - Radio] Recv Error: ");
    Serial.println(state);
  }
  delay(2000);
}