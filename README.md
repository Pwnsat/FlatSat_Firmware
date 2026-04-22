# FlatSat_Firmware

Firmware for the FlatSat v0.1

## Overview

This firmware is a workshop version for testing on the FlatSat board. With this firmware you can test:
- Packet interception
- Packet injection

## Setup

### Arduino IDE

#### 1. Install RP2040 Board Support

Install the [arduino-pico](https://github.com/earlephilhower/arduino-pico) core from source:

```bash
mkdir -p ~/Arduino/hardware/pico
git clone https://github.com/earlephilhower/arduino-pico.git ~/Arduino/hardware/pico/rp2040
cd ~/Arduino/hardware/pico/rp2040
git submodule update --init --recursive
cd tools
python3 ./get.py
```

Restart the Arduino IDE after installation.

#### 2. Install Required Libraries

Install the following via **Sketch > Include Library > Manage Libraries**:

- **RadioLib**
- **SparkFun LIS2DH12**
- **Adafruit BME280 Library**
- **Adafruit Unified Sensor**

#### 3. Board and Tool Settings

Under the **Tools** menu, set:

| Setting | Value |
|---------|-------|
| Board | Generic RP2040 |
| Boot Stage 2 | IS25LP080 QSPI /4 |
| Flash Size | 4MB - No FS |
| USB Stack | Adafruit TinyUSB |

#### 4. Flash the Firmware

Open `firmware/firmware.ino` in the Arduino IDE and upload.

## Linux Serial Permissions

If you get a permission error accessing `/dev/ttyACM*`, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

# Test

With arduino upload with the `Default with spiffs` partition scheme

## RADIO

This code will initialize the radio and blink the LEDS.
> **Note:** I don't know if this will work, due to the DIO1 and RESET pins are not connected in diagram

## MPU

Test for the MPU-6050.

## BME

Test for the BME-280
