#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal HOLD PHASE host usage (COM10). Early-brake + optional Hall REFINE: 0 / 90 / 180.

Example:
  python hold_phase_smoke.py --port COM10 --target 0 --rpm 80 --dry-run
  python hold_phase_smoke.py --port COM10 --target 90 --rpm 80
  python hold_phase_smoke.py --port COM10 --target 0 --rpm 80 --refine on --tol 5
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

from com_port_guard import acquire, release


def read_for(ser: serial.Serial, sec: float, pred=None):
    end = time.time() + sec
    lines: list[str] = []
    buf = b""
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("utf-8", "replace").strip()
                if s:
                    lines.append(s)
                    print(s)
                    if pred and pred(s):
                        return lines, True
        else:
            time.sleep(0.02)
    return lines, False


def send(ser: serial.Serial, cmd: str) -> None:
    print(f">> {cmd}")
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description="HOLD PHASE smoke / usage example")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--target", type=float, default=0.0, help="disc deg: 0/90/180/…")
    ap.add_argument("--rpm", type=float, default=80.0, help="motor crawl RPM")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument(
        "--tol",
        type=float,
        default=5.0,
        help="PHASE TOL deg (pass band)",
    )
    ap.add_argument(
        "--refine",
        choices=("on", "off", "leave"),
        default="on",
        help="PHASE REFINE for 0/180 Hall crawl after first settle",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="only send query/lead cmds, do not start motor",
    )
    ap.add_argument(
        "--hall-cal",
        action="store_true",
        help="send HALL CAL before HOLD (sets disc 0 at current pose; for bench only)",
    )
    args = ap.parse_args()

    acquire("hold_phase_smoke")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
        time.sleep(0.6)
        ser.reset_input_buffer()

        send(ser, "TELEM OFF")
        read_for(ser, 0.4)
        send(ser, "HOST OFF")
        read_for(ser, 0.6, lambda s: "HOST OFF" in s)
        send(ser, "HALL?")
        hall_lines, _ = read_for(ser, 1.0)
        send(ser, "PHASE?")
        read_for(ser, 0.8)
        send(ser, "PHASE LEAD?")
        read_for(ser, 0.6)
        send(ser, f"PHASE TOL {args.tol:g}")
        read_for(ser, 0.5, lambda s: "PHASE TOL" in s)
        if args.refine != "leave":
            send(ser, f"PHASE REFINE {args.refine.upper()}")
            read_for(ser, 0.5, lambda s: "PHASE REFINE" in s)
        send(ser, "PHASE REFINE?")
        read_for(ser, 0.5)
        send(ser, "PHASE TOL?")
        read_for(ser, 0.5)

        if args.dry_run:
            print("--- dry-run: not starting HOLD PHASE ---")
            print("Usage when ready:")
            print(f"  PHASE TOL {args.tol:g}")
            print(f"  PHASE REFINE {args.refine.upper() if args.refine != 'leave' else 'ON'}")
            print(f"  HOLD PHASE {args.target:g} {args.rpm:g}")
            print("If cal=0: pass GPIO4 once, or: HALL CAL  then HOLD PHASE …")
            print("0/180: first settle miss → REFINE crawl to Hall; 90°: enc-only")
            return 0

        cal_ok = any("cal=1" in s for s in hall_lines)
        if not cal_ok and not args.hall_cal:
            print("FAIL: flap cal=0 — pass GPIO4 or re-run with --hall-cal (bench)")
            return 3
        if args.hall_cal:
            send(ser, "HALL CAL")
            read_for(ser, 0.8)

        send(ser, "ESTOP")
        read_for(ser, 0.3)
        send(ser, "STOP")
        read_for(ser, 0.4)

        cmd = f"HOLD PHASE {args.target:g} {args.rpm:g}"
        send(ser, cmd)
        lines, ok = read_for(
            ser,
            args.timeout,
            lambda s: ("HOLD PHASE DONE" in s)
            or s.startswith("# ESTOP")
            or ("ERR HOLD PHASE" in s),
        )
        if not ok:
            print("FAIL: no DONE/ESTOP within timeout — sending ESTOP")
            send(ser, "ESTOP")
            read_for(ser, 0.5)
            return 2

        last = next(
            (
                s
                for s in reversed(lines)
                if "HOLD PHASE DONE" in s or s.startswith("# ESTOP") or "ERR HOLD" in s
            ),
            "",
        )
        if "DONE" in last:
            print("PASS:", last)
            if "refine=1" in last or "refine_hall" in last:
                print("  (Hall REFINE path used)")
            return 0
        print("FAIL:", last)
        send(ser, "ESTOP")
        return 1
    finally:
        release()


if __name__ == "__main__":
    sys.exit(main())
