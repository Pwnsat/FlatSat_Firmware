#include <RadioLib.h>

#define SX_UL_NSS   D2  // NSS1 - NSS2=D2
#define SX_UL_RST   D19
#define SX_UL_IO    -1
#define SX_UL_BUSY  D12 // BUSY1 - BUSY2=D12
#define SX_DL_NSS   D1  // NSS1 - NSS2=D2
#define SX_DL_RST   D19
#define SX_DL_IO    -1
#define SX_DL_BUSY  D11 // BUSY1 - BUSY2=D12
#define SX_SCK      D8
#define SX_MISO     D9
#define SX_MOSI     D10

SPIClass spiRadio(FSPI);
SX1262 radioUplink = new Module(SX_UL_NSS, SX_UL_IO, SX_UL_RST, SX_UL_BUSY, spiRadio);
SX1262 radioDownlink = new Module(SX_DL_NSS, SX_DL_IO, SX_DL_RST, SX_DL_BUSY, spiRadio);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(SX_UL_NSS, OUTPUT);
  digitalWrite(SX_UL_NSS, HIGH);

  pinMode(SX_DL_NSS, OUTPUT);
  digitalWrite(SX_DL_NSS, HIGH);

  Serial.print("Probing UPL SPI...");
  spiRadio.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SX_UL_NSS, LOW);
  uint8_t response = spiRadio.transfer(0x42); // comando inválido intencional
  digitalWrite(SX_UL_NSS, HIGH);
  spiRadio.endTransaction();
  Serial.print(" Got: 0x");
  Serial.println(response, HEX);

  Serial.print("Probing DWL SPI...");
  spiRadio.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SX_DL_NSS, LOW);
  response = spiRadio.transfer(0x42); // comando inválido intencional
  digitalWrite(SX_DL_NSS, HIGH);
  spiRadio.endTransaction();
  Serial.print(" Got: 0x");
  Serial.println(response, HEX);


  Serial.println("Iniciando SPI...");
  spiRadio.begin(SX_SCK, SX_MISO, SX_MOSI);

  Serial.println("Iniciando UPL radio...");
  int state = radioUplink.begin();
  radioUplink.setFrequency(915);
  radioUplink.setSpreadingFactor(11);
  radioUplink.setBandwidth(250);
  radioUplink.setCodingRate(5);
  radioUplink.setPreambleLength(8);
  radioUplink.setSyncWord(0x12);
  radioUplink.setOutputPower(8);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("¡Radio OK!");
  } else {
    Serial.print("Error al iniciar UPL radio: ");
    Serial.println(state);
  }

  Serial.println("Iniciando DPL radio...");
  state = radioDownlink.begin();
  radioDownlink.setFrequency(915);
  radioDownlink.setSpreadingFactor(11);
  radioDownlink.setBandwidth(250);
  radioDownlink.setCodingRate(5);
  radioDownlink.setPreambleLength(8);
  radioDownlink.setSyncWord(0x12);
  radioDownlink.setOutputPower(8);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("¡Radio OK!");
  } else {
    Serial.print("Error al iniciar DPL radio: ");
    Serial.println(state);
  }

  radioDownlink.startReceive();
}

void loop() {
  radioDownlink.transmit("Flatsat alive!");
  String data;
  int err = radioDownlink.readData(data);
  if (err == RADIOLIB_ERR_NONE){
    Serial.println("Data:");
    Serial.println(data);
  } else if (err == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("CRC Error");
  }else{
    Serial.print("Err: ");
    Serial.println(err);
  }
  delay(1000);
}
