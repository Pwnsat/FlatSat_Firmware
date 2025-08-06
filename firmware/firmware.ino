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
#define SX_DIO1 D19
// Sensor Specific
#define SEALEVELPRESSURE_HPA (1013.25)
// Telemetry Structure
#define TM_SEPARATOR ","
#define SENSOR_SEPARATOR ";"

// SPP
#define CCSDS_VERSION 0
#define APID_WEATHER 100
#define APID_COMMAND 200
#define APID_SENSORS 300
#define APID_IDLE 0xFFF
// TC
#define TC_VERSION 0
#define VCID_TC 0
#define VCID_ROOT 2
#define UNLOCK_COMMAND 0x01
#define SET_VR 0x02
// TM
#define TM_VERSION 0
#define VCID_TM 1
// CLCW
#define STATUS_FIELD 123
#define COP1 0b01
// Common
#define MAX_SPP_PACKET_SIZE 64
#define MAX_TC_FRAME_SIZE 128
#define MAX_TM_FRAME_SIZE 128
// CLTU
#define CLTU_TAIL_LENGTH 8
#define CLTU_START_SEQUENCE 0xEB90

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


typedef enum {
  PACKET_TYPE_TM = 0,
  PACKET_TYPE_TC = 1
} packet_type_t;

typedef enum {
  GROUPING_FLAG_CONT = 0b00,
  GROUPING_FLAG_START = 0b01,
  GROUPING_FLAG_END = 0b10,
  GROUPING_FLAG_UNSEGMENTED = 0b11
} grouping_flag_t;

typedef struct {
  // Primary Header (6 bytes)
  uint16_t packetid;          // Version(3) + Type(1) + SecHdr(1) + APID(11)
  uint16_t packet_sequence;   // Seq flags(2) + Seq count(14)
  uint16_t length;            // Packet length = (len(payload) + len(secondary_header)) - 1
} __attribute__((packed)) spp_primary_header_t;

typedef struct {
  spp_primary_header_t header;
  uint8_t data[MAX_SPP_PACKET_SIZE-6];  // Secondary header + payload
} __attribute__((packed)) spp_packet_t;


typedef enum {
  FRAME_OK = 0,
  FRAME_ERROR_TC_VERSION,
  FRAME_ERROR_TM_VERSION,
  FRAME_ERROR_VCID,
  FRAME_ERROR_FRAME_LEN,
  FRAME_ERROR_BUFF_SIZE,
  FRAME_ERROR_CRC,
  FRAME_ERROR_PARSE_SPP,
  FRAME_ERROR_LOCKOUT,
  SPP_ERROR_PACKET_LEN,
  SPP_ERROR_INVALID_VERSION,
} tc_error_t;

sensor_context_t sensorContext;
radio_context_t radioContext;

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

SX1262 uplink = new Module(SX_NSS_1, SX_BUSY_1, -1, -1);

// SX1262 downlink = new Module(SX_NSS_2, SX_DIO1, -1, SX_BUSY_2);
SX1262 downlink = new Module(SX_NSS_2, SX_BUSY_2, -1, -1);

const uint8_t SPACECRAFT_ID = 0x01;

static uint16_t counter_tc = 0;
static uint16_t counter_tm = 0;

const unsigned long interval = 10000;    // 30 s interval to send message
unsigned long previousMillis = 0;  // will store last time message sent

static bool send_tm = false;

volatile bool receivedFlag = false;
volatile bool enableInterruptRadio = true;

void setFlag(void) {
  if(!enableInterruptRadio) {
    return;
  }
  receivedFlag = true;
}

static int build_packet(spp_packet_t* space_packet, packet_type_t type, grouping_flag_t flag, uint16_t apid, uint16_t sequence_count, const uint8_t* payload, uint16_t payload_len) {
  memset(space_packet, 0, sizeof(spp_packet_t));
  space_packet->header.packetid = (CCSDS_VERSION << 13) | (type << 11) | (0 << 10) | (apid & 0x7FF); // Secondary Header Flag = 0; By now hardcoded
  space_packet->header.packet_sequence = (flag << 14) | (sequence_count & 0x3FFF);
  uint16_t total_payload_len = payload_len;
  space_packet->header.length = total_payload_len;
  
  if (payload_len > 0 && payload != NULL){
    if (total_payload_len > MAX_SPP_PACKET_SIZE) return SPP_ERROR_PACKET_LEN;
    memcpy(space_packet->data, payload, payload_len);
  }
  return FRAME_OK;
}

void spp_build_tc_packet(spp_packet_t* space_packet, packet_type_t type, grouping_flag_t flag, uint8_t* payload, uint16_t payload_len) {
  counter_tc = (counter_tc + 1) & 0x3FFF;
  build_packet(space_packet, type, flag, APID_COMMAND, counter_tc, payload, payload_len);
}

void spp_build_tm_packet(spp_packet_t* space_packet, uint16_t apid, uint8_t* payload, uint16_t payload_len) {
  counter_tm = (counter_tm + 1) & 0x3FFF;
  build_packet(space_packet, PACKET_TYPE_TM, GROUPING_FLAG_UNSEGMENTED, apid, counter_tm, payload, payload_len);
}

tc_error_t spp_parse_packet(spp_packet_t* space_packet, uint8_t* buffer, uint16_t buffer_len){
  if (buffer_len < 6) {
    return FRAME_ERROR_FRAME_LEN;
  }

  memset(space_packet, 0, sizeof(spp_packet_t));

  space_packet->header.packetid = (buffer[1] << 8) | buffer[0];
  space_packet->header.packet_sequence = (buffer[3] << 8) | buffer[2];
  space_packet->header.length = (buffer[5] << 8) | buffer[4];

  uint16_t payload_len = space_packet->header.length + 1; // CCSDS length es (tamaño real - 1)
  if (payload_len > MAX_SPP_PACKET_SIZE || payload_len > buffer_len - 6) {
      return FRAME_ERROR_FRAME_LEN;
  }
  if (payload_len > 0) {
      memcpy(space_packet->data, buffer + 6, payload_len);
  }
  uint8_t version = (space_packet->header.packetid >> 13) & 0x07;
  if (version != CCSDS_VERSION) {
      return SPP_ERROR_INVALID_VERSION;
  }

  return FRAME_OK;
}


void spp_print_hex(uint8_t* packet, uint16_t packet_length){
  for (int i = 0; i < packet_length + 1; ++i) {
    Serial.printf("%02X ", packet[i]);
    if ((i + 1) % 16 == 0) Serial.println();
  }
  if ((packet_length + 1) % 16 != 0) Serial.println();
}

void spp_print_packet_details(spp_packet_t* packet){
  uint16_t version     = (packet->header.packetid >> 13) & 0x7;
  uint16_t type        = (packet->header.packetid >> 11) & 0x1;
  uint16_t sec_header  = (packet->header.packetid >> 10) & 0x1;
  uint16_t apid        = packet->header.packetid & 0x7FF;
  
  uint16_t seq_flags   = (packet->header.packet_sequence >> 14) & 0x3;
  uint16_t seq_count   = packet->header.packet_sequence & 0x3FFF;

  Serial.println("=== Space Packet Header ===");
  Serial.printf(" Version:             %u\n", version);
  Serial.printf(" Type:                %s\n", type == PACKET_TYPE_TM ? "Telemetry" : "Telecommand");
  Serial.printf(" Secondary Header:    %u\n", sec_header);
  Serial.printf(" APID:                0x%03X (%s)\n", apid,
                apid == APID_COMMAND ? "Telecommand" :
                apid == APID_WEATHER ? "Data" :
                apid == APID_IDLE ? "IDLE" : "Unknown");

  Serial.printf(" Sequence Flags:      0x%X (%s)\n", seq_flags,
                seq_flags == GROUPING_FLAG_UNSEGMENTED ? "Unsegmented" :
                seq_flags == GROUPING_FLAG_START ? "Start" :
                seq_flags == GROUPING_FLAG_CONT ? "Continuation" : "End");

  Serial.printf(" Sequence Count:      %u\n", seq_count);
  Serial.printf(" Data Length:         %u\n", packet->header.length);

  Serial.println("=== Payload Dump (Hex) ===");
  spp_print_hex(packet->data, packet->header.length);

  Serial.println("=== Packet Dump (Hex) ===");
  uint8_t packet_buffer[70]; // Max header (6) + data (64)

  packet_buffer[0] = packet->header.packetid >> 8;
  packet_buffer[1] = packet->header.packetid & 0xFF;
  packet_buffer[2] = packet->header.packet_sequence >> 8;
  packet_buffer[3] = packet->header.packet_sequence & 0xFF;
  packet_buffer[4] = packet->header.length >> 8;
  packet_buffer[5] = packet->header.length & 0xFF;
  memcpy(packet_buffer + 6, packet->data, packet->header.length);

  spp_print_hex(packet_buffer, 6 + packet->header.length);
  Serial.println();
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

  delay(1000);

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

  downlink.setPacketReceivedAction(setFlag);
  downlink.startReceive();

  Serial.println("[SYS] Init sensors...");
  sensorContext.mpuStatus = initMPU();
  sensorContext.bmeStatus = initBME();
}

void loop() {
  if(millis() - previousMillis > interval){
    previousMillis = millis();
    uint8_t payload[40];
    uint8_t idx = 0;

    payload[idx++] = SPACECRAFT_ID;
    spp_packet_t spp_tc;
    if(send_tm){
      readMPU_binary(payload, idx);
      delay(500);
      readBME_binary(payload, idx);
      delay(500);
      spp_build_tm_packet(&spp_tc, APID_SENSORS, payload, idx);
      send_tm = false;
    }else{
      // 48 61 70 70 79 48 61 63 6b 69 6e 67 44 65 66 63 6f 6e
      payload[idx++] = 0x48;
      payload[idx++] = 0x61;
      payload[idx++] = 0x70;
      payload[idx++] = 0x70;
      payload[idx++] = 0x79;
      payload[idx++] = 0x48;
      payload[idx++] = 0x61;
      payload[idx++] = 0x63;
      payload[idx++] = 0x6B;
      payload[idx++] = 0x69;
      payload[idx++] = 0x6E;
      payload[idx++] = 0x67;
      payload[idx++] = 0x44;
      payload[idx++] = 0x65;
      payload[idx++] = 0x66;
      payload[idx++] = 0x63;
      payload[idx++] = 0x6F;
      payload[idx++] = 0x6E;
      spp_build_tc_packet(&spp_tc, PACKET_TYPE_TC, GROUPING_FLAG_UNSEGMENTED, payload, idx);
      send_tm = true;
    }

    int state = downlink.transmit((uint8_t*)&spp_tc, (6 + spp_tc.header.length + 1));
    if (state == RADIOLIB_ERR_NONE){
      Serial.print("[SYS - Radio] Transmited: ");
      Serial.write(payload, idx);
      Serial.println();
      // spp_print_packet_details(&spp_tc);
    }else{
      Serial.print("[SYS - Radio] Transmit Error: ");
      Serial.println(state);
    }
  }
  // String telemetry = getSensorsTelemetry();
  // uint8_t payload[30];
  // uint8_t idx = 0;

  // payload[idx++] = SPACECRAFT_ID;
  // spp_packet_t spp_tc;
  // if(send_tm){
  //   readMPU_binary(payload, idx);
  //   delay(500);
  //   readBME_binary(payload, idx);
  //   delay(500);
  //   spp_build_tm_packet(&spp_tc, APID_SENSORS, payload, idx);
  //   send_tm = false;
  // }else{
  //   payload[idx++] = 0x01;
  //   payload[idx++] = 0x02;
  //   payload[idx++] = 0x03;
  //   spp_build_tc_packet(&spp_tc, PACKET_TYPE_TC, GROUPING_FLAG_UNSEGMENTED, payload, idx);
  //   send_tm = true;
  // }

  // int state = downlink.transmit((uint8_t*)&spp_tc, (6 + spp_tc.header.length + 1));
  // if (state == RADIOLIB_ERR_NONE){
  //   Serial.print("[SYS - Radio] Transmited: ");
  //   Serial.write(payload, idx);
  //   Serial.println();
  //   // spp_print_packet_details(&spp_tc);
  // }else{
  //   Serial.print("[SYS - Radio] Transmit Error: ");
  //   Serial.println(state);
  // }
  // delay(2000);
  if(receivedFlag){
    receivedFlag = false;
    enableInterruptRadio = false;
    int recvLen = downlink.getPacketLength();
    byte byteArr[recvLen];
    int state = downlink.readData(byteArr, recvLen);
    if (state == RADIOLIB_ERR_NONE){
      Serial.print("[SYS - Radio] Recv: ");
      Serial.write(byteArr, recvLen);
      Serial.println();
      spp_packet_t spp_tc;
      spp_parse_packet(&spp_tc, byteArr, recvLen);
      Serial.println();
      Serial.print("[SYS - Radio] RSSI: ");
      Serial.println(downlink.getRSSI());
      Serial.print("[SYS - Radio] SNR: ");
      Serial.println(downlink.getSNR());
      // printStringHexDump(data);
      printHexDump(byteArr, recvLen);
      // spp_print_packet_details(&spp_tc);
      if(spp_tc.data[2] == 0x33 || spp_tc.data[2] == 2 || spp_tc.data[3] == 0x33 || spp_tc.data[3] == 2 || spp_tc.data[2] == 0x32 || spp_tc.data[3] == 0x32){
        Serial.println("===========================================");
        Serial.println("PWNSAT!");
        Serial.println("===========================================");
      }
    }else{
      Serial.print("[SYS - Radio] Recv Error: ");
      Serial.println(state);
    }
    downlink.startReceive();
    enableInterruptRadio = true;
  }
  
}
