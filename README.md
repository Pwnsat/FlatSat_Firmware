# FlatSat_Firmware

Firmware for the FlatSat v0.1

# Test

With arduino upload with the `Default with spiffs` partition scheme

## RADIO

This code will initialize the radio and blink the LEDS.
> **Note:** I don't know if this will work, due to the DIO1 and RESET pins are not connected in diagram

## MPU

Test for the MPU-6050.

## BME

Test for the BME-280


# Test inicial de workshop

## Catsniffer v2
1. Abre el `gstick_catsnifferv2.ino` con Arduino IDE.
2. Carga el firmware a la catsniffer
3. Ya puedes usar el script del repo de pwnsatc3 para la recepcion de paquetes


## Flatsat
1. Requires VSCode instalado
2. A vscode le debes intalar la extension `platformio`
3. Una vez instalado todo, cargas el firmware de `flatsat_v1`
4. Abres una consola serial para ver los logs y saber si todo inicio bien.


### Notas:
- Si el flatsat no tiene el puente en el pin 11 modificar linea 15 del archivo `lib/radio/radio_wrapper.cpp` por:
`SX1262 downlink = new Module(SX_NSS_2, SX_BUSY_2, -1, -1);`
- Si al momento de abrir el serial el radio de uplink da error, busca todas las variables que usen `uplink` y cambialas por `downlink`
