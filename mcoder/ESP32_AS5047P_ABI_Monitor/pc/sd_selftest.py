#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""COM6 SD 自检：SD? / SD TEST / SD LIST（过滤 L, 遥测）"""
from __future__ import annotations

import pathlib
import sys
import time

import serial

PORT = "COM6"
BAUD = 921600
LOG = pathlib.Path(__file__).resolve().parent / "serial_logs" / "sd_selftest_last.txt"


def main() -> int:
    print(f"=== open {PORT} @ {BAUD} ===", flush=True)
    try:
        ser = serial.Serial(
            PORT, BAUD, timeout=0.15, write_timeout=2.0, dsrdtr=False, rtscts=False
        )
    except Exception as exc:
        print(f"OPEN FAIL: {exc}", flush=True)
        return 2

    try:
        ser.setDTR(False)
        ser.setRTS(True)
        time.sleep(0.05)
        ser.setRTS(False)
        time.sleep(1.2)
    except Exception:
        time.sleep(0.5)

    ser.reset_input_buffer()
    kept: list[str] = []

    def read_for(sec: float) -> None:
        t0 = time.time()
        buf = b""
        while time.time() - t0 < sec:
            chunk = ser.read(4096)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", errors="ignore").strip("\r")
                    if not line or line.startswith("L,"):
                        continue
                    kept.append(line)
                    print(line, flush=True)
            else:
                time.sleep(0.01)

    def send(cmd: str) -> None:
        print(f">> {cmd}", flush=True)
        ser.write((cmd + "\n").encode("utf-8"))
        ser.flush()

    print("=== boot 3s ===", flush=True)
    read_for(3.0)

    for cmd, wait in (("SD?", 1.0), ("SD TEST", 2.5), ("SD LIST", 3.0)):
        send(cmd)
        read_for(wait)

    ser.close()
    text = "\n".join(kept)
    ok_fw = "monitor-v20-sd" in text
    ok_sd = ("# SD OK" in text) or ("SD ready=1" in text)
    ok_test = "SD TEST OK" in text
    ok_file = "oez_sd_test" in text.lower()

    print("=== SUMMARY ===", flush=True)
    print(f"FW v20-sd : {ok_fw}", flush=True)
    print(f"SD mount  : {ok_sd}", flush=True)
    print(f"SD TEST   : {ok_test}", flush=True)
    print(f"test file : {ok_file}", flush=True)

    LOG.parent.mkdir(parents=True, exist_ok=True)
    LOG.write_text(text + "\n", encoding="utf-8")
    print(f"log saved : {LOG}", flush=True)
    return 0 if (ok_sd and ok_test) else 1


if __name__ == "__main__":
    sys.exit(main())
