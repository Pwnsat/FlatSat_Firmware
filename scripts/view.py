import serial
import time
from collections import defaultdict
from rich.live import Live
from rich.table import Table

PORT = '/dev/cu.usbmodemfsat3'
BAUD = 115200

SYNC = 0xAA

ser = serial.Serial(PORT, BAUD)

# Estado de frames
frames = {}

def read_frame():
  while True:
    b = ser.read(1)
    if not b:
      continue

    if b[0] != SYNC:
      continue

    id_l = ser.read(1)[0]
    id_h = ser.read(1)[0]
    can_id = id_l | (id_h << 8)

    dlc = ser.read(1)[0]
    data = ser.read(dlc)

    return can_id, dlc, data


def build_table():
  table = Table(title="SpaceCAN Viewer", expand=True)

  table.add_column("ID", justify="right")
  table.add_column("DLC")
  table.add_column("DATA")
  table.add_column("COUNT")
  table.add_column("LAST (s)")

  now = time.time()

  for can_id, info in sorted(frames.items()):
    age = now - info["last_time"]

    table.add_row(
      f"{can_id:03X}",
      str(info["dlc"]),
      info["data"].hex(" "),
      str(info["count"]),
      f"{age:0.2f}"
    )

  return table


start = time.time()

with Live(build_table(), refresh_per_second=10) as live:
  while True:
    can_id, dlc, data = read_frame()

    now = time.time()

    if can_id not in frames:
      frames[can_id] = {
          "dlc": dlc,
          "data": data,
          "count": 0,
          "last_time": now
      }

    frames[can_id]["dlc"] = dlc
    frames[can_id]["data"] = data
    frames[can_id]["count"] += 1
    frames[can_id]["last_time"] = now

    live.update(build_table())