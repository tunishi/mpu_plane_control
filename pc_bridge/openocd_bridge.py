"""Reads roll/pitch/yaw straight from the STM32's RAM over ST-Link (SWD),
using OpenOCD, and sends normalized aileron/elevator/rudder to FlightGear's
generic protocol over UDP (see Protocol/mpu-input.xml in FG_ROOT).

No serial/USB-C connection is needed: only the ST-Link (SWDIO, SWCLK, GND,
3V3) is used. The firmware publishes g_roll/g_pitch/g_yaw in RAM and reads
g_recenter_req; this script writes 1 to g_recenter_req to recenter.

Usage: python openocd_bridge.py
Press 'c' + Enter to recenter. Ctrl+C to quit.
"""

import atexit
import re
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

OPENOCD_DIR = Path.home() / ".platformio" / "packages" / "tool-openocd"
OPENOCD_BIN = OPENOCD_DIR / "bin" / "openocd.exe"
OPENOCD_SCRIPTS = OPENOCD_DIR / "openocd" / "scripts"
NM_BIN = Path.home() / ".platformio" / "packages" / "toolchain-gccarmnoneeabi" / "bin" / "arm-none-eabi-nm.exe"
ELF_PATH = Path(__file__).resolve().parent.parent / "firmware" / ".pio" / "build" / "blackpill_f411ce_stlink" / "firmware.elf"

TELNET_PORT = 4444
FG_HOST = "127.0.0.1"
FG_PORT = 5501

ROLL_RANGE_DEG = 45.0
PITCH_RANGE_DEG = 45.0
YAW_RANGE_DEG = 180.0
SMOOTHING = 0.6

ROLL_SIGN = -1.0
PITCH_SIGN = -1.0
YAW_SIGN = -1.0


def read_symbols() -> dict:
    out = subprocess.run([str(NM_BIN), str(ELF_PATH)], capture_output=True, text=True, check=True).stdout
    symbols = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in ("g_roll", "g_pitch", "g_yaw", "g_recenter_req"):
            symbols[parts[2]] = int(parts[0], 16)
    missing = {"g_roll", "g_pitch", "g_yaw", "g_recenter_req"} - symbols.keys()
    if missing:
        sys.exit(f"Symbols not found in ELF (rebuild firmware?): {missing}")
    return symbols


class OpenOcdSession:
    def __init__(self):
        self.lock = threading.Lock()
        self.proc = subprocess.Popen(
            [
                str(OPENOCD_BIN), "-s", str(OPENOCD_SCRIPTS),
                "-f", "interface/stlink.cfg",
                "-c", "transport select swd",
                "-c", "adapter speed 1800",
                "-f", "target/stm32f4x.cfg",
                "-c", f"telnet_port {TELNET_PORT}",
                "-c", "gdb_port disabled",
                "-c", "tcl_port disabled",
                "-c", "init",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.sock = None
        for _ in range(50):
            time.sleep(0.2)
            if self.proc.poll() is not None:
                sys.exit("OpenOCD exited:\n" + self.proc.stderr.read())
            try:
                self.sock = socket.create_connection(("127.0.0.1", TELNET_PORT), timeout=2)
                break
            except OSError:
                continue
        if self.sock is None:
            sys.exit("Could not connect to OpenOCD telnet interface.")
        self.sock.settimeout(2)
        self._read_until_prompt()
        self.command("resume")
        atexit.register(self.close)

    def _read_until_prompt(self) -> str:
        data = b""
        while not data.endswith(b"> "):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("OpenOCD closed the connection")
            data += chunk
        return data.decode(errors="ignore")

    def command(self, cmd: str) -> str:
        with self.lock:
            self.sock.sendall((cmd + "\n").encode())
            return self._read_until_prompt()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        self.proc.terminate()


def read_floats(session: OpenOcdSession, addr: int, count: int) -> list:
    text = session.command(f"mdw 0x{addr:08x} {count}")
    words = re.findall(r"0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)", text)
    if not words:
        raise ValueError(f"unexpected mdw output: {text!r}")
    hex_words = words[0].split()
    return [struct.unpack(">f", bytes.fromhex(w))[0] for w in hex_words]


def deg_to_norm(value_deg: float, range_deg: float) -> float:
    return max(-1.0, min(1.0, value_deg / range_deg))


def stdin_watcher(request_recenter):
    while True:
        line = sys.stdin.readline()
        if not line:
            return
        if line.strip().lower() == "c":
            request_recenter()
            print("\n-> recenter requested")


def main():
    symbols = read_symbols()
    session = OpenOcdSession()
    print("Connected to STM32 over ST-Link.")

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def request_recenter():
        session.command(f"mwb 0x{symbols['g_recenter_req']:08x} 1")

    request_recenter()
    print("Auto-recenter sent. Keep the board still and level for a moment...")
    time.sleep(0.3)

    threading.Thread(target=stdin_watcher, args=(request_recenter,), daemon=True).start()

    smooth_roll = smooth_pitch = smooth_yaw = 0.0
    first = True
    last_print = 0.0
    base = min(symbols["g_roll"], symbols["g_pitch"], symbols["g_yaw"])
    count = (max(symbols["g_roll"], symbols["g_pitch"], symbols["g_yaw"]) - base) // 4 + 1
    offsets = {name: (symbols[name] - base) // 4 for name in ("g_roll", "g_pitch", "g_yaw")}

    while True:
        words = read_floats(session, base, count)
        roll = words[offsets["g_roll"]]
        pitch = words[offsets["g_pitch"]]
        yaw = words[offsets["g_yaw"]]
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
            print(
                f"\rroll:{roll:7.2f} pitch:{pitch:7.2f} yaw:{yaw:7.2f}  ->  "
                f"aileron:{aileron:5.2f} elevator:{elevator:5.2f} rudder:{rudder:5.2f}   ",
                end="", flush=True,
            )


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, ConnectionError):
        pass
