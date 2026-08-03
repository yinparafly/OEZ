#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""收全 DUMP BIN 帧存盘 + 完整性分析"""
import sys, time, struct, zlib
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM23"
OUT = sys.argv[2] if len(sys.argv) > 2 else r"D:\temp\opencode\dump_frame.bin"
SNAP_PREAMBLE = bytes([0xAA] * 10 + [0x55])

s = serial.Serial(PORT, 921600, timeout=0.3)
time.sleep(0.3)
s.reset_input_buffer()
# 确保有数据可拉
s.write(b"MONITOR START\n"); time.sleep(0.2)
s.write(b"MOTOR 1 3000\n"); time.sleep(0.5)
# 等 done 事件
t0 = time.monotonic()
while time.monotonic() - t0 < 8:
    txt = s.read(4096).decode(errors="replace")
    if "done=1" in txt:
        print("done event seen")
        break
s.write(b"MOTOR 0 0\n"); time.sleep(0.2)
# 拉帧
s.reset_input_buffer()
s.write(b"DUMP BIN\n")
time.sleep(0.4)
buf = bytearray()
t0 = time.monotonic()
while time.monotonic() < t0 + 4:
    buf.extend(s.read(16384))
    if buf.find(SNAP_PREAMBLE) >= 0 and len(buf) - buf.find(SNAP_PREAMBLE) > 5000:
        break
s.write(b"LOG CLEAR\n")
s.close()
open(OUT, "wb").write(bytes(buf))
print("saved %d bytes to %s" % (len(buf), OUT))
pre_i = buf.find(SNAP_PREAMBLE)
print("preamble at %d" % pre_i)
if pre_i < 0:
    sys.exit(1)
magic, n, hz = struct.unpack("<IHH", bytes(buf[pre_i+11:pre_i+19]))
print("magic=0x%08X n=%d hz=%d" % (magic, n, hz))
need = 8 + n*16 + 4
got = len(buf) - pre_i
print("frame len got=%d expect=%d %s" % (got, 11+need, "OK" if got >= 11+need else "SHORT"))
if got >= 11+need:
    raw = bytes(buf[pre_i+11:pre_i+11+need])
    payload = raw[8:8+n*16]
    crc_exp = struct.unpack("<I", raw[8+n*16:])[0]
    crc_calc = zlib.crc32(payload) & 0xFFFFFFFF
    print("CRC expect=0x%08X calc=0x%08X %s" % (crc_exp, crc_calc, "OK" if crc_exp == crc_calc else "FAIL"))
    # 数据规律检查
    import collections
    dts, cts = [], []
    for i in range(0, n*16, 16):
        t, c, idx = struct.unpack("<IqI", payload[i:i+16])
        dts.append(t); cts.append(c)
    mono = sum(1 for i in range(1, len(dts)) if dts[i] > dts[i-1])
    print("t monotonic: %d/%d ; t first=%d last=%d" % (mono, n, dts[0], dts[-1]))
    print("counts range: %d .. %d" % (min(cts), max(cts)))
