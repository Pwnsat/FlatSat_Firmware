import serial
import struct

ser = serial.Serial('/dev/cu.usbmodemfsat3', 115200)

FRAME_SIZE = 13

SYNC = 0xAA

while True:
    # buscar sync
    if ser.read(1)[0] != SYNC:
        continue

    id_l = ser.read(1)[0]
    id_h = ser.read(1)[0]
    can_id = id_l | (id_h << 8)

    dlc = ser.read(1)[0]
    payload = ser.read(dlc)

    print(f"ID: {hex(can_id)} DLC: {dlc} DATA: {payload.hex()}", flush=True)