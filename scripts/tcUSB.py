from pwn import *
from spacepackets.ccsds.spacepacket import SpHeader
import serial
import time

PORT = '/dev/cu.usbmodemfsat3'
BAUD = 921600

tc_header = SpHeader.tc(apid=6, seq_count=5, data_len=0)
telecommand = tc_header.pack()

success(f"Final RF Payload (ASCII Hex): {telecommand.hex()}")

info(f"Sending {len(telecommand)} characters over {PORT}...")

try:
  with serial.Serial(PORT, BAUD, timeout=1) as ser:
    time.sleep(2)

    ser.write(telecommand + b'\n')
    ser.flush()

    time.sleep(0.5)
    success("Payload delivered successfully!")

except Exception as e:
  print(f"Error opening serial port: {e}")