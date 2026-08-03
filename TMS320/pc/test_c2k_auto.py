#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全自动端到端：MOTOR 启动 → SNAP 自动触发 → DUMP BIN → 停止电机。
用法: python test_c2k_auto.py [COM口] [转速0-9999] [时长秒]
"""
import sys, time, struct, zlib
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM23"
SPEED = int(sys.argv[2]) if len(sys.argv) > 2 else 3000
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
SNAP_PREAMBLE = bytes([0xAA] * 10 + [0x55])
SNAP_MAGIC_V2 = 0xAB1C0002

s = serial.Serial(PORT, 921600, timeout=0.3)
time.sleep(0.3)
s.reset_input_buffer()

def cmd(c, wait=0.4):
    s.reset_input_buffer()
    s.write(c.encode() + b"\n")
    time.sleep(wait)
    out = []
    for l in s.read(500).decode(errors="replace").splitlines():
        if l.startswith("#"):
            out.append(l)
    return out

print("[1] PING:", cmd("PING"))
print("[2] MONITOR START:", cmd("MONITOR START"))
print(f"[3] 启动电机 MOTOR 1 {SPEED}")
print("    ", cmd(f"MOTOR 1 {SPEED}"))
print("[4] 等待触发并记录 (max %.1fs)..." % (DUR + 3))
s.reset_input_buffer()
done = False
t0 = time.monotonic()
last_snap = None
while time.monotonic() - t0 < DUR + 3:
    for l in s.read(4096).decode(errors="replace").splitlines():
        if l.startswith("# SNAP") and l != last_snap:
            last_snap = l
            print("    ", l, flush=True)
            if "done=1" in l:
                done = True
    if done:
        break
print("[5] 停止电机:", cmd("MOTOR 0 0"))
if not done:
    print("    FAIL: 未触发记录。SNAP?=", cmd("SNAP?"))
    cmd("MOTOR 0 0")
    s.close()
    sys.exit(1)

print("[6] DUMP BIN ...")
s.reset_input_buffer()
s.write(b"DUMP BIN\n")
time.sleep(0.3)
buf = bytearray()
t0 = time.monotonic()
while time.monotonic() - t0 < 5:
    buf.extend(s.read(8192))
    if buf.find(SNAP_PREAMBLE) >= 0 and len(buf) - buf.find(SNAP_PREAMBLE) > 100:
        break
pre_i = buf.find(SNAP_PREAMBLE)
if pre_i < 0:
    print("    FAIL: no preamble")
    cmd("MOTOR 0 0")
    s.close()
    sys.exit(1)
magic, n, hz = struct.unpack("<IHH", bytes(buf[pre_i + 11: pre_i + 19]))
print(f"    magic=0x{magic:08X} n={n} hz={hz}")
need = 8 + n * 16 + 4
while len(buf) - pre_i < 11 + need:
    buf.extend(s.read(8192))
    if len(buf) - pre_i >= 11 + need:
        break
raw = bytes(buf[pre_i + 11: pre_i + 11 + need])
magic2, n2, hz2 = struct.unpack("<IHH", raw[0:8])
payload = raw[8:8 + n2 * 16]
(crc,) = struct.unpack("<I", raw[8 + n2 * 16:])
got = zlib.crc32(payload) & 0xFFFFFFFF
print(f"    CRC: got=0x{got:08X} expect=0x{crc:08X} {'OK' if got == crc else 'FAIL'}")
if got == crc and n2 > 0:
    t0u, c0, i0 = struct.unpack("<IqI", payload[0:16])
    tNu, cN, iN = struct.unpack("<IqI", payload[-16:])
    dur_ms = (tNu - t0u) / 1000.0
    print(f"    首点 t={t0u}us counts={c0} idx={i0}")
    print(f"    尾点 t={tNu}us counts={cN} idx={iN}")
    d = (cN - c0)
    rpm_avg = d * 1000.0 / dur_ms / 4000 * 60 if dur_ms > 0 else 0
    print(f"    时长 {dur_ms:.0f} ms  Δcounts={d} → 平均 {rpm_avg:.0f} RPM")
    print("    [PASS] 全链路验证通过")
print("[7] LOG CLEAR:", cmd("LOG CLEAR"))
cmd("MOTOR 0 0")
s.close()
