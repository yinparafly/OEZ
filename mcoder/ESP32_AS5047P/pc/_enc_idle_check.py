# -*- coding: utf-8 -*-
"""空转(不给油)快速自检：读 banner、ENC?、几行遥测，确认 enc_sample_hz 与列数。"""
from __future__ import annotations
import sys, time
from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire

acquire("enc_idle_check")
ser = open_port("COM10")
try:
    time.sleep(0.4)
    print("--- banner ---", flush=True)
    for ln in read_lines(ser, 1.0):
        if ln.startswith("#"):
            print(ln, flush=True)
    for c in ("PING", "ENC?"):
        send(ser, c); time.sleep(0.3)
        for ln in read_lines(ser, 0.8):
            if ln.startswith("#"):
                print(f"[{c}] {ln}", flush=True)
    print("--- 3 telem lines (count cols) ---", flush=True)
    n = 0
    end = time.time() + 2.0
    while time.time() < end and n < 3:
        for ln in read_lines(ser, 0.2):
            if ln and not ln.startswith("#"):
                cols = ln.split(",")
                print(f"cols={len(cols)} last={cols[-1]} rpm={cols[4] if len(cols)>4 else '?'}", flush=True)
                n += 1
    send(ser, "ENC?"); time.sleep(0.3)
    for ln in read_lines(ser, 0.8):
        if ln.startswith("# ENC"):
            print(f"[ENC2] {ln}", flush=True)
finally:
    ser.close()
