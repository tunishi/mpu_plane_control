"""Reads "roll,pitch,yaw" CSV lines from the STM32 over serial and sends
normalized aileron/elevator/rudder values to FlightGear's generic protocol
over UDP (see ../README.md and Protocol/mpu-input.xml in FG_ROOT).

This replaces the vJoy path: FlightGear's joystick/SDL input subsystem
turned out to be unreliable with this vJoy setup (axis data reached vJoy
fine but never reached FlightGear's control properties), whereas writing
straight to the property tree over UDP is direct and reliable.

Launch FlightGear with (already added to the project's launch command):
  --generic=socket,in,45,,5501,udp,mpu-input

Usage: python fg_udp_bridge.py [COM_PORT]
Press 'c' + Enter to recenter (zero) roll/pitch/yaw on the STM32.
"""

import sys
import threading
import time
import socket

import serial
import serial.tools.list_ports

BAUD_RATE = 115200
FG_HOST = "127.0.0.1"
FG_PORT = 5501

# Degrees of MPU rotation that map to full control deflection (-1..1).
ROLL_RANGE_DEG = 45.0
PITCH_RANGE_DEG = 45.0
YAW_RANGE_DEG = 180.0

# Exponential smoothing (0 = none, closer to 1 = smoother but laggier).
SMOOTHING = 0.6

# Flip to -1.0 if a control moves the wrong way in the sim.
ROLL_SIGN = -1.0
PITCH_SIGN = -1.0
YAW_SIGN = -1.0


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


def deg_to_norm(value_deg: float, range_deg: float) -> float:
    return max(-1.0, min(1.0, value_deg / range_deg))


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

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    print(f"Connected to {port} @ {BAUD_RATE}, sending to FlightGear UDP {FG_HOST}:{FG_PORT}.")
    print("Type 'c' + Enter any time to recenter roll/pitch/yaw.")

    # Always start from a known-level state: recenter on every launch so a
    # stale/tilted reading from a previous session can never get sent to
    # the sim before the user remembers to do it manually.
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"c")
    print("Auto-recenter sent, keep the board still and level for a moment...")
    time.sleep(0.3)
    ser.reset_input_buffer()

    threading.Thread(target=stdin_watcher, args=(ser,), daemon=True).start()

    smooth_roll = smooth_pitch = smooth_yaw = 0.0
    first = True
    last_print = 0.0

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

        aileron = ROLL_SIGN * deg_to_norm(smooth_roll, ROLL_RANGE_DEG)
        elevator = PITCH_SIGN * deg_to_norm(smooth_pitch, PITCH_RANGE_DEG)
        rudder = YAW_SIGN * deg_to_norm(smooth_yaw, YAW_RANGE_DEG)

        udp.sendto(f"{aileron},{elevator},{rudder}\n".encode(), (FG_HOST, FG_PORT))

        now = time.time()
        if now - last_print >= 0.2:
            last_print = now
            print(f"\rroll:{roll:7.2f} pitch:{pitch:7.2f} yaw:{yaw:7.2f}  ->  aileron:{aileron:5.2f} elevator:{elevator:5.2f} rudder:{rudder:5.2f}   ", end="", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
