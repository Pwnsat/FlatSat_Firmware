# FlatSat Firmware v1.0.0 Public

The firmware utilizes a **Asymmetric Multiprocessing (AMP)** approach on the RP2040 microcontroller. By splitting tasks across two cores, the system ensures that high-speed data links do not interfere with time-critical radio operations.

## Dual-Core Distribution
- **Core 0 (Mission Control & RF):** Manages the physical radio interfaces (Uplink/Downlink), sensor data acquisition, and the primary telemetry state machine.
- **Core 1 (OBC Data Link):** Dedicated to the TinyUSB stack, handling the `usbCDC` interface to provide a high-speed command-and-control link for the On-Board Computer (OBC) or ground station simulation.

> The OBC Data Link is used in case you don't have access to a SDR device to collect and send packets.

## Hardware Infrastructure

The infrastructure is designed for **FlatSat** testing, where hardware components are laid out for accessibility and auditing.

|**Component**|**Description**|**Interface**|
|---|---|---|
|**MCU**|Raspberry Pi RP2040|Dual Core ARM Cortex-M0+|
|**Radio 0**|SX1262 LoRa (Uplink)|SPI0 / NSS Pin 17|
|**Radio 1**|SX1262 LoRa (Downlink)|SPI0 / NSS Pin 5|
|**IMU**|LIS2DH12 Accelerometer|I2C (SDA 20, SCL 21)|
|**ENV**|BME280 Environment Sensor|I2C (SDA 20, SCL 21)|
|**Status**|WS2812B NeoPixel|GPIO 15|

# Communication Protocol: Space Packet Protocol (SPP)

The core of the communication system is the **CCSDS Space Packet Protocol**. This allows the spacecraft to route data using **Application Process Identifiers (APIDs)**, enabling modular subsystem addressing.

## Packet Encapsulation

Every packet consists of a **Primary Header** (6 bytes) and a **Data Field**.
1. **Packet Identification (2 bytes):** Contains the Version, Type (0 for TM, 1 for TC), and the APID.
2. **Packet Sequence Control (2 bytes):** Contains Segmentation Flags and the 14-bit Sequence Count.
3. **Packet Length (2 bytes):** Total length of the data field minus one.

# Attack Vectors

|**Attack Method**|**Layer**|**Tooling**|**Real-World Impact**|
|---|---|---|---|
|**Bit-Slipping**|Physical|SDR / GNU Radio|Desynchronization of the RF link.|
|**APID Brute-forcing**|Link|LoRa Transceiver|Discovery of hidden "Debug" commands.|
|**Telemetry Hijacking**|Data|Antenna / LNA|Eavesdropping on sensitive mission data.|
|**Logic Bombing**|Application|Custom Python Script|Triggering the `softwareReset()` loop.|


# Requirements for Arduino IDLE

**Board components**
- [Arduino Pico boards](https://github.com/earlephilhower/arduino-pico)

**Libraries**
- NeoPixelBus
- RadioLib
- SparkFun_LIS2DH12
- Adafruit_BME280
- Adafruit_TinyUSB
#### Board configuration
- **Board Type**: Generic RP20240
- Select the **Tools**:
	- **Board Stage**: IS25LP080 QSPI /4
	- **Flash Size**: 4 MB (No FS)
	- **USB Stack**: Adafruit TinyUSB

#### Flash firmware
- Sketch -> Verify/Compile
- Sketch -> Upload

> [!IMPORTANT] Once the firmware successfully flashed the board, you will have now two serial endpoints.

# Scripts
## Requirements
- pwntools
- spacepackets

## TcUSB
This script show how to send Telecomands over the USB interface insted of using a SDR for communication.

```python
PORT = '/dev/cu.usbmodemfsat3' # Change this line with the second endpoint of your board
```

```shell
python3 tcUSB.py

# Expected output

[+] Final RF Payload (ASCII Hex): 1006c0050000
[*] Sending 6 characters over /dev/cu.usbmodemfsat3...
[+] Payload delivered successfully!
```