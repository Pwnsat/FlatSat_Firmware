/*
FLatSat V1 - Firmware with Telemetry collection and LoRa communication
  PWNSAT 2025
  Kevin Leon
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <RadioLib.h>
#include <Wire.h>

#define FIRMWARE_VERSION "0.0.1"

// Telemetry Radio
#define FSAT_TM_NSS   D2
#define FSAT_TM_RST   -1
#define FSAT_TM_IO    -1
#define FSAT_TM_BUSY  D12
// Telecommands Radio
#define FSAT_TC_NSS   D1
#define FSAT_TC_RST   -1
#define FSAT_TC_IO    -1
#define FSAT_TC_BUSY  D11
// Board Common Comms
#define FSAT_SCK      D8
#define FSAT_MISO     D9
#define FSAT_MOSI     D10
// Sensor Specific
#define SEALEVELPRESSURE_HPA (1013.25)
// Radio Config
#define RADIO_FREQ  916  // Frequency
#define RADIO_SP    12   // Spreading Factor
#define RADIO_BW    250  // Bandwidth
#define RADIO_CR    5    // Coding Rate
#define RADIO_PL    8    // Preamble Length
#define RADIO_SW    0x12 // SyncWord
#define RADIO_OP    4    // Output Power
#define RADIO_RECV_TIMEOUT 500
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

SPIClass spiRadio(HSPI);
SX1262 radioTm = new Module(FSAT_TM_NSS, FSAT_TM_IO, FSAT_TM_RST, FSAT_TM_BUSY, spiRadio);
SX1262 radioTc = new Module(FSAT_TC_NSS, FSAT_TC_IO, FSAT_TC_RST, FSAT_TC_BUSY, spiRadio);


static void checkRadioTmSPI(){
  Serial.println("[SYS - Radio - SPI] Testing Telemetry Communication");
  pinMode(FSAT_TM_NSS, OUTPUT);
  digitalWrite(FSAT_TM_NSS, HIGH);
  spiRadio.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(FSAT_TM_NSS, LOW);
  uint8_t response = spiRadio.transfer(0x42);
  digitalWrite(FSAT_TM_NSS, HIGH);
  spiRadio.endTransaction();

  Serial.print("[SYS - Radio - SPI] Response: 0x");
  Serial.println(response, HEX);
}

static void checkRadioTcSPI(){
  Serial.println("[SYS - Radio - SPI] Testing Telecommand Communication");
  pinMode(FSAT_TC_NSS, OUTPUT);
  digitalWrite(FSAT_TC_NSS, HIGH);
  spiRadio.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(FSAT_TC_NSS, LOW);
  uint8_t response = spiRadio.transfer(0x42);
  digitalWrite(FSAT_TC_NSS, HIGH);
  spiRadio.endTransaction();

  Serial.print("[SYS - Radio - SPI] Response: 0x");
  Serial.println(response, HEX);
}

static int initRadioTM(){
  Serial.println("[SYS - Radio] Telemetry Radio Init");

  int state = radioTm.begin();
  radioTm.setFrequency(RADIO_FREQ);
  radioTm.setSpreadingFactor(RADIO_SP);
  radioTm.setBandwidth(RADIO_BW);
  radioTm.setCodingRate(RADIO_CR);
  radioTm.setPreambleLength(RADIO_PL);
  radioTm.setSyncWord(RADIO_SW);
  radioTm.setOutputPower(RADIO_OP);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[SYS - Radio] Telemetry OK");
    return RADIOLIB_ERR_NONE;
  } else {
    Serial.print("[SYS - Radio] Telemetry Error radio: ");
    Serial.println(state);
    return state;
  }
}

static int initRadioTC(){
  Serial.println("[SYS - Radio] Telecommand Radio Init");
  
  int state = radioTc.begin();
  radioTc.setFrequency(RADIO_FREQ);
  radioTc.setSpreadingFactor(RADIO_SP);
  radioTc.setBandwidth(RADIO_BW);
  radioTc.setCodingRate(RADIO_CR);
  radioTc.setPreambleLength(RADIO_PL);
  radioTc.setSyncWord(RADIO_SW);
  radioTc.setOutputPower(RADIO_OP);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[SYS - Radio] Telecommand OK");
    return RADIOLIB_ERR_NONE;
  } else {
    Serial.print("[SYS - Radio] Telecommand Error radio: ");
    Serial.println(state);
    return state;
  }
}

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
  // String bmeTelemetry = readBME();
  String telemetry = mpuTelemetry; // + SENSOR_SEPARATOR + bmeTelemetry;
  return telemetry;
}

static void radioTelemetryTransmit(String payload){
  String telemetry = String(SPACECRAFT_ID) + SENSOR_SEPARATOR + payload;

  Serial.print("[SYS - Radio] Transmiting: ");
  Serial.println(telemetry);
  int state = radioTm.transmit("Hello");
  if (state != RADIOLIB_ERR_NONE){
    Serial.print("[SYS - Radio] Transmition Failed: ");
    Serial.println(state);
    return;
  }
  Serial.println("[SYS - Radio] Transmition OK");
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


static void radioTelecommandListen(){
  radioTc.startReceive(RADIO_RECV_TIMEOUT);
  String data;
  int state = radioTc.readData(data);
  if (state == RADIOLIB_ERR_NONE) {
    if (data.length() == 0) return;
    
    Serial.println("[SYS - Radio] Telecommand Packet Received: ");
    Serial.print("[SYS - Radio] Packet RSSI: ");
    Serial.println(radioTc.getRSSI());
    Serial.print("[SYS - Radio] Packet SNR: ");
    Serial.println(radioTc.getSNR());
    printStringHexDump(data);
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[SYS - Radio] Telecommand Packet CRC Error");
  } else {
    Serial.print("[SYS - Radio] Telecommand Error:");
    Serial.println(state);
  }
}

static void sensorIntegrityCheck(){
  if(sensorContext.mpuStatus == SENSOR_FAIL){
    Serial.println("[SYS - SIC] MPU failed");
    sensorContext.mpuStatus = initMPU();
  }
  if(sensorContext.bmeStatus == SENSOR_FAIL){
    Serial.println("[SYS - SIC] BME failed");
    sensorContext.bmeStatus = initBME();
  }

  if(radioContext.tmStatus == RADIOLIB_ERR_NONE){
    Serial.println("[SYS - SIC] Radio TM failed");
    radioContext.tmStatus = initRadioTM();
  }
  if(radioContext.tcStatus == RADIOLIB_ERR_NONE){
    Serial.println("[SYS - SIC] Radio TC failed");
    radioContext.tcStatus = initRadioTC();
  }
}

void setup(){
  Serial.begin(115200);
  while (!Serial);

  Serial.print("\nFirmware version: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("[SYS] Init sensors...");

  checkRadioTmSPI();
  checkRadioTcSPI();

  spiRadio.begin(FSAT_SCK, FSAT_MISO, FSAT_MOSI);
  
  sensorContext.mpuStatus = initMPU();
  sensorContext.bmeStatus = initBME();

  radioContext.tmStatus = initRadioTM();
  radioContext.tcStatus = initRadioTC();
}

void loop(){
  // sensorIntegrityCheck();
  radioTelemetryTransmit(getSensorsTelemetry());
  delay(1000);
  radioTelecommandListen();
  delay(5000);
}