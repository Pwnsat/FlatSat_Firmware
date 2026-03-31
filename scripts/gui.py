from rich.layout import Layout
from rich.panel import Panel
from rich.live import Live
from rich import box
from rich.text import Text
from rich.table import Table
import serial
import struct

from collections import deque

MAX_LOG_LINES = 50
log_buffer = deque(maxlen=MAX_LOG_LINES)

PORT = '/dev/cu.usbmodemfsat3'
BAUD = 115200

SYNC = 0xAA

SC_TM_ID_ACCE_X = 0x01
SC_TM_ID_ACCE_Y = 0x02
SC_TM_ID_ACCE_Z = 0x03
SC_TM_ID_ACCE_TMP = 0x04

SC_TM_ID_BME_TEMPERATURE = 0x05
SC_TM_ID_BME_PRESSURE = 0x06
SC_TM_ID_BME_ALTITUDE = 0x07
SC_TM_ID_BME_HUMIDITY = 0x08

SC_TM_ID_SIM_THRUSTER0 = 0x09
SC_TM_ID_SIM_THRUSTER1 = 0x0A

ser = serial.Serial(PORT, BAUD)

ser.flush()

# Estado de frames
frames = {}

def format_hexdump(can_id, dlc, data):
  hex_part = " ".join(f"{b:02X}" for b in data)
  ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
  return f"{can_id:03X}  [{dlc}]  {hex_part:<24}  |{ascii_part}|"

def decode_frame(frame_id, data, state):
  if frame_id > 0x300 and frame_id < 0x400:
    can_id = (frame_id - 0x300) & 0x7F

    if len(data) < 2:
      return

    frame_type = data[0]
    payload = data[1:]

    if can_id <= 0x08:
      if len(payload) < 2:
        return

      value = struct.unpack("<h", payload[:2])[0] / 100.0

      if can_id == SC_TM_ID_ACCE_X:
        state["accel"]["x"] = value
      elif can_id == SC_TM_ID_ACCE_Y:
        state["accel"]["y"] = value
      elif can_id == SC_TM_ID_ACCE_Z:
        state["accel"]["z"] = value
      elif can_id == SC_TM_ID_ACCE_TMP:
        state["accel"]["tmp"] = value

      elif can_id == SC_TM_ID_BME_TEMPERATURE:
        state["bme"]["temp"] = value
      elif can_id == SC_TM_ID_BME_PRESSURE:
        state["bme"]["press"] = value
      elif can_id == SC_TM_ID_BME_ALTITUDE:
        state["bme"]["alt"] = value
      elif can_id == SC_TM_ID_BME_HUMIDITY:
        state["bme"]["hum"] = value
    
    elif can_id in (SC_TM_ID_SIM_THRUSTER0, SC_TM_ID_SIM_THRUSTER1):
      if len(payload) < 1:
        return

      value = payload[0]

      if can_id == SC_TM_ID_SIM_THRUSTER0:
        state["thrusters"]["t0"] = value
      else:
        state["thrusters"]["t1"] = value
  elif frame_id >= 0x700 and frame_id < 0x800:
    can_id = (frame_id - 0x700) & 0x7F

    if len(data) < 1:
      return

    state_val = data[0]

    if can_id == SC_TM_ID_SIM_THRUSTER0:
      state["thrusters"]["t0_state"] = state_val

    elif can_id == SC_TM_ID_SIM_THRUSTER1:
      state["thrusters"]["t1_state"] = state_val

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

def get_thruster_state(thruster):
  if thruster == 0:
    return "[white]IDLE[/white]"
  elif thruster == 1:
    return "[yellow]RUNNING[/yellow]"
  else:
    return "[red]UNKNOWN[/red]"
    

def build_ui(state):

    layout = Layout(name="PWNSAT - SpaceCAN Telemetry Viewer")

    layout.split(
        Layout(name="top", size=3),
        Layout(name="main", size=15),
        Layout(name="bottom")
    )

    layout["main"].split_row(
        Layout(name="left"),
        Layout(name="right")
    )
    
    layout["bottom"].split(
        Layout(name="thrusters", size=10),
        Layout(name="log")
    )
    
    header = Panel(
      Text("PWNSAT - SpaceCAN Telemetry Viewer", justify="center", style="bold cyan"),
      border_style="cyan"
    )

    # =========================
    # BME PANEL
    # =========================
    bme = state["bme"]
    bme_table = Table(box=box.SIMPLE)
    bme_table.add_column("Metric")
    bme_table.add_column("Value")

    bme_table.add_row("Temp", f"{bme['temp']:.2f} °C")
    bme_table.add_row("Humidity", f"{bme['hum']:.2f} %")
    bme_table.add_row("Pressure", f"{bme['press']:.2f} hPa")
    bme_table.add_row("Altitude", f"{bme['alt']:.2f} m")

    # =========================
    # ACCEL PANEL
    # =========================
    acc = state["accel"]
    acc_table = Table(box=box.SIMPLE)
    acc_table.add_column("Axis")
    acc_table.add_column("Value")

    acc_table.add_row("X", str(acc["x"]))
    acc_table.add_row("Y", str(acc["y"]))
    acc_table.add_row("Z", str(acc["z"]))
    acc_table.add_row("TEMP", str(acc["tmp"]))

    # =========================
    # THRUSTERS PANEL
    # =========================
    thr = state["thrusters"]
    t0_state = get_thruster_state(thr['t0_state'])
    t1_state = get_thruster_state(thr['t1_state'])
    thruster_text = f"""
    
    T1 ({t0_state}): [{'█'*thr['t0']}{' '*(100-thr['t0'])}] {thr['t0']}
    
    
    T2 ({t1_state}): [{'█'*thr['t1']}{' '*(100-thr['t1'])}] {thr['t1']}
    """
    
    log_text = "\n".join(log_buffer)

    layout["top"].update(header)
    layout["left"].update(Panel(bme_table, title="BME Sensor"))
    layout["right"].update(Panel(acc_table, title="Accelerometer"))
    layout["bottom"]["thrusters"].update(Panel(thruster_text, title="Thrusters"))
    layout["bottom"]["log"].update(Panel(log_text, title="Hexdump Log", border_style="green"))

    return layout

state = {
    "bme": {"temp":0,"hum":0,"press":0,"alt":0, "state": 0},
    "accel": {"x":0,"y":0,"z":0, "tmp": 0, "state": 0},
    "thrusters": {"t0":0,"t1":0, "t0_state": 0, "t1_state": 0}
}

with Live(build_ui(state), refresh_per_second=10) as live:
  while True:
    try:
      can_id, dlc, data = read_frame()

      decode_frame(can_id, data, state)
      line = format_hexdump(can_id, dlc, data)
      log_buffer.append(line)

      live.update(build_ui(state))
    except KeyboardInterrupt:
      ser.flush()
      ser.close()
      break