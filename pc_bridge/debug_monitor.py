"""Human-readable, throttled view of the STM32's roll/pitch/yaw stream.

The firmware sends raw CSV at ~100 Hz for the vJoy bridge; this script just
reads it and reprints a labeled line a few times per second so it's
readable, without touching the firmware's wire format.

Usage: python debug_monitor.py [COM_PORT]
Press 'c' + Enter to recenter (zero) roll/pitch/yaw. Ctrl+C to quit.
"""

import sys
import threading
import time

import serial
import serial.tools.list_ports

BAUD_RATE = 115200
PRINT_INTERVAL_S = 0.2  # 5 updates/sec, easy to read


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


def stdin_watcher(ser: serial.Serial):
    while True:
        line = sys.stdin.readline()
        if not line:
            return
        if line.strip().lower() == "c":
            ser.write(b"c")
            print("\n-> recenter sent")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else pick_port()
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    print(f"Connected to {port} @ {BAUD_RATE}. Type 'c' + Enter to recenter. Ctrl+C to quit.\n")

    threading.Thread(target=stdin_watcher, args=(ser,), daemon=True).start()

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

        now = time.time()
        if now - last_print >= PRINT_INTERVAL_S:
            last_print = now
            print(f"\rroll: {roll:7.2f}   pitch: {pitch:7.2f}   yaw: {yaw:7.2f}   ", end="", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
