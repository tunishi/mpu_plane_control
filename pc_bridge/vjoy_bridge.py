"""Reads "roll,pitch,yaw" CSV lines from the STM32 over serial and drives a
vJoy virtual joystick with them, so FlightGear (or any sim) can use it as a
normal joystick for aileron/elevator/rudder.

Setup (Windows):
  1. Install the vJoy driver (https://sourceforge.net/projects/vjoystick/).
  2. Run "Configure vJoy" and enable X, Y and Rz axes on device #1.
  3. pip install -r requirements.txt
  4. python vjoy_bridge.py [COM_PORT]
     If COM_PORT is omitted, the script lists available ports and asks.

While running, press 'c' + Enter in this console to recenter (zero) roll,
pitch and yaw on the STM32 -- useful to cancel yaw drift.
"""

import sys
import threading
import time

import serial
import serial.tools.list_ports
import pyvjoy

BAUD_RATE = 115200
VJOY_DEVICE_ID = 1

# Degrees of MPU rotation that map to full joystick deflection.
ROLL_RANGE_DEG = 45.0
PITCH_RANGE_DEG = 45.0
YAW_RANGE_DEG = 180.0

# Exponential smoothing on the incoming values (0 = no smoothing, closer to
# 1 = smoother but laggier). Cuts down single-sample noise spikes that would
# otherwise snap a control surface to a large deflection for one frame.
SMOOTHING = 0.6

VJOY_MIN = 1
VJOY_MAX = 32768
VJOY_MID = (VJOY_MIN + VJOY_MAX) // 2


def pick_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        sys.exit(1)
    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    idx = input("Select port index: ").strip()
    return ports[int(idx)].device


def deg_to_axis(value_deg: float, range_deg: float) -> int:
    normalized = max(-1.0, min(1.0, value_deg / range_deg))
    axis = VJOY_MID + normalized * (VJOY_MAX - VJOY_MID)
    return int(max(VJOY_MIN, min(VJOY_MAX, axis)))


def stdin_watcher(ser: serial.Serial):
    while True:
        line = sys.stdin.readline()
        if not line:
            return
        if line.strip().lower() == "c":
            ser.write(b"c")
            print("-> recenter sent")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else pick_port()

    j = pyvjoy.VJoyDevice(VJOY_DEVICE_ID)

    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    print(f"Connected to {port} @ {BAUD_RATE}, driving vJoy device {VJOY_DEVICE_ID}.")
    print("Type 'c' + Enter any time to recenter roll/pitch/yaw.")

    threading.Thread(target=stdin_watcher, args=(ser,), daemon=True).start()

    smooth_roll = smooth_pitch = smooth_yaw = 0.0
    first = True

    while True:
        raw = ser.readline().decode("ascii", errors="ignore").strip()
        if not raw:
            continue
        parts = raw.split(",")
        if len(parts) != 3:
            continue
        try:
            roll, pitch, yaw = (float(p) for p in parts)
        except ValueError:
            continue

        if first:
            smooth_roll, smooth_pitch, smooth_yaw = roll, pitch, yaw
            first = False
        else:
            smooth_roll = SMOOTHING * smooth_roll + (1 - SMOOTHING) * roll
            smooth_pitch = SMOOTHING * smooth_pitch + (1 - SMOOTHING) * pitch
            smooth_yaw = SMOOTHING * smooth_yaw + (1 - SMOOTHING) * yaw

        j.set_axis(pyvjoy.HID_USAGE_X, deg_to_axis(smooth_roll, ROLL_RANGE_DEG))
        j.set_axis(pyvjoy.HID_USAGE_Y, deg_to_axis(smooth_pitch, PITCH_RANGE_DEG))
        j.set_axis(pyvjoy.HID_USAGE_RZ, deg_to_axis(smooth_yaw, YAW_RANGE_DEG))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
