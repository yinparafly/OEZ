# -*- coding: utf-8 -*-
"""恢复后安全确认：连 COM10，发 STOP/RPM0/PWM1000，读 ENC? 与几行遥测确认已停车。"""
from __future__ import annotations
import sys, time
from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire

acquire("enc_safecheck")
ser = open_port("COM10")
try:
    time.sleep(0.3); read_lines(ser, 0.5)
    for c in ("ESTOP", "STOP", "RPM 0", "PWM 1000"):
        send(ser, c); time.sleep(0.25)
        for ln in read_lines(ser, 0.35):
            if ln.startswith("#"): print(f"[{c}] {ln}", flush=True)
    send(ser, "ENC?"); time.sleep(0.3)
    for ln in read_lines(ser, 0.8):
        if ln.startswith("# ENC"): print(f"[ENC?] {ln}", flush=True)
    print("--- 5 telem lines (rpm=col4, pulse=col9) ---", flush=True)
    n = 0; end = time.time() + 2.5
    while time.time() < end and n < 5:
        for ln in read_lines(ser, 0.2):
            if ln and not ln.startswith("#"):
                p = ln.split(",")
                if len(p) >= 25:
                    print(f"rpm={p[4]} pulse={p[9]} run={p[16]} enc_hz={p[24]}", flush=True); n += 1
finally:
    ser.close()
