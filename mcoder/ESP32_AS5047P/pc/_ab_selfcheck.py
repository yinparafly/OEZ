# -*- coding: utf-8 -*-
"""阶段 A/B 自检：PING/ENC?/CTRL?/TELEM?/BLE? + 短时 CLOSED RPM2000（保活）后停车。"""
from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("PYTHONIOENCODING", "utf-8")

from auto_learn_test import open_port, send, read_lines  # noqa: E402
from com_port_guard import acquire  # noqa: E402

acquire("ab_selfcheck")
ser = open_port("COM10")


def drain(sec: float = 0.6):
    return [ln for ln in read_lines(ser, sec)]


def cmd(c: str, wait: float = 0.5):
    send(ser, c)
    time.sleep(wait)
    lines = [ln for ln in drain(wait) if ln.startswith("#")]
    for ln in lines:
        print(f"[{c}] {ln}", flush=True)
    return lines


try:
    time.sleep(0.8)
    print("--- banner ---", flush=True)
    for ln in drain(1.2):
        if ln.startswith("#"):
            print(ln, flush=True)

    for c in ("PING", "ENC?", "CTRL?", "TELEM?", "BLE?", "CORE?"):
        cmd(c)

    print("--- short CLOSED RPM 2000 ---", flush=True)
    cmd("SOFT ON", 0.3)
    cmd("MODE CLOSED", 0.3)
    cmd("FREQ 400", 0.3)
    cmd("START", 0.3)
    t0 = time.time()
    last_rpm = None
    while time.time() - t0 < 4.0:
        send(ser, "RPM 2000")  # 保活防 HOST_TIMEOUT
        time.sleep(0.4)
        for ln in drain(0.35):
            if ln.startswith("#"):
                print(ln, flush=True)
            elif ln and not ln.startswith("#"):
                cols = ln.split(",")
                if len(cols) >= 5:
                    try:
                        last_rpm = float(cols[4])
                    except ValueError:
                        pass
                    print(
                        f"telem cols={len(cols)} rpm={cols[4]} pulse={cols[9] if len(cols)>9 else '?'} "
                        f"enc_hz={cols[-1]}",
                        flush=True,
                    )

    cmd("ENC?", 0.4)
    cmd("CTRL?", 0.4)
    cmd("TELEM?", 0.4)

    print("--- stop ---", flush=True)
    cmd("ESTOP", 0.3)
    cmd("STOP", 0.3)
    cmd("RPM 0", 0.3)
    send(ser, "PWM 1000")
    time.sleep(0.5)
    for ln in drain(0.8):
        if ln and (ln.startswith("#") or ("," in ln and not ln.startswith("#"))):
            print(ln, flush=True)

    print(f"=== last_rpm_seen={last_rpm} ===", flush=True)
finally:
    try:
        send(ser, "ESTOP")
        send(ser, "STOP")
    except Exception:
        pass
    ser.close()
