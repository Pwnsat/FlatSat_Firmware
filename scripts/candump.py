import serial
import time

ser = serial.Serial('/dev/cu.usbmodemfsat3', 115200)

SYNC = 0xAA

last_frames = {}

def format_data(data, prev):
  if prev is None:
    return data.hex(' ')

  out = []
  for i in range(len(data)):
    if i < len(prev) and data[i] != prev[i]:
      out.append(f"\x1b[31m{data[i]:02X}\x1b[0m")  # rojo
    else:
      out.append(f"{data[i]:02X}")
  return ' '.join(out)


while True:
  if ser.read(1)[0] != SYNC:
    continue

  id_l = ser.read(1)[0]
  id_h = ser.read(1)[0]
  can_id = id_l | (id_h << 8)

  dlc = ser.read(1)[0]
  payload = ser.read(dlc)

  prev = last_frames.get(can_id)

  data_str = format_data(payload, prev)

  print(f"ID: {hex(can_id)} DLC: {dlc} DATA: {data_str}", flush=True)

  last_frames[can_id] = payload