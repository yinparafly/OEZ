# Serial link and auto-flow for AbiMonitor_TJX
# Usage: python abi_tjx.py [--port COM23] [--auto]

import serial, time, sys, struct, os
from dataclasses import dataclass
from abi_tjx import *

@dataclass
class TelemetryLine:
    rpm: int; gear: int; ms: int; points: int; rec: bool; alive: bool; armed: bool; full: bool

def parse_telemetry(line: str) -> TelemetryLine | None:
    if not line.startswith("L,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 9:
        return None
    try:
        return TelemetryLine(
            rpm=int(parts[1]), gear=int(parts[2]), ms=int(parts[3]),
            points=int(parts[4]), rec=parts[6]=="1", alive=parts[7]=="1",
            armed=parts[8]=="1", full=parts[9]=="1")
    except (ValueError, IndexError):
        return None

class SerialLink:
    def __init__(self, port="COM23", baud=921600):
        self.ser = serial.Serial(port, baud, timeout=3)
        self.buf = b""
    
    def send(self, cmd: str):
        self.ser.write((cmd + "\r\n").encode())
    
    def read_line(self) -> str | None:
        if b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            return line.decode(errors="replace").strip()
        return None
    
    def poll(self):
        if self.ser.in_waiting:
            self.buf += self.ser.read(self.ser.in_waiting)
    
    def read_until(self, prefix: str, timeout_ms: float = 3000) -> str | None:
        t0 = time.monotonic()
        while (time.monotonic() - t0) * 1000 < timeout_ms:
            self.poll()
            line = self.read_line()
            if line and line.startswith(prefix):
                return line
            time.sleep(0.01)
        return None
    
    def close(self):
        self.ser.close()

def auto_capture(port="COM23", rec_ms=2000):
    """Auto-flow: MONITOR START → wait trigger → wait DONE → DUMP BIN → save"""
    s = SerialLink(port)
    print(f"[AUTO] Armed on {port}")
    s.send("MONITOR START")
    time.sleep(0.2)
    
    # Wait for trigger (recording starts)
    print("[AUTO] Waiting for trigger...")
    for _ in range(100):  # 10s timeout
        s.poll()
        line = s.read_line()
        if line and "rec=1" in line:
            print(f"[AUTO] Recording: {line}")
            break
        time.sleep(0.1)
    
    # Wait for DONE
    print(f"[AUTO] Recording {rec_ms}ms...")
    t0 = time.monotonic()
    snap_count = 0
    while (time.monotonic() - t0) * 1000 < rec_ms + 5000:
        s.poll()
        line = s.read_line()
        if line and "done=1" in line:
            print(f"[AUTO] SNAP DONE: {line}")
            break
        if line and line.startswith("L,"):
            t = parse_telemetry(line)
            if t:
                snap_count = t.points
        time.sleep(0.05)
    
    print(f"[AUTO] SNAP complete: {snap_count} points")
    s.send("SNAP?")
    time.sleep(0.3)
    s.poll()
    for _ in range(10):
        line = s.read_line()
        if line and line.startswith("# SNAP"):
            print(line)
    
    s.close()
    return snap_count

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM23")
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--rec-ms", type=int, default=2000)
    ap.add_argument("--cmd", default=None, help="Send single command and read response")
    args = ap.parse_args()
    
    if args.auto:
        auto_capture(args.port, args.rec_ms)
    elif args.cmd:
        s = SerialLink(args.port)
        s.send(args.cmd)
        time.sleep(0.5)
        s.poll()
        for _ in range(10):
            line = s.read_line()
            if line:
                print(line)
        s.close()
    else:
        print("Usage: python abi_tjx_auto.py --auto [--rec-ms=2000] [--port=COM23]")
        print("       python abi_tjx_auto.py --cmd=PING [--port=COM23]")
