# FlatSat_Firmware

Firmware for the FlatSat v0.1

# Flatsat V1
This firmware is just a workshop version for testing. The final firmware will release in the next days.
With this firmware you can test:
- Packet interception
- Packet injection


# Test

With arduino upload with the `Default with spiffs` partition scheme

## RADIO

This code will initialize the radio and blink the LEDS.
> **Note:** I don't know if this will work, due to the DIO1 and RESET pins are not connected in diagram

## MPU

Test for the MPU-6050.

## BME

Test for the BME-280

# Requirements 

- Platformio
- Arduino