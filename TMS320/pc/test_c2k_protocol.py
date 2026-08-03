#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C2000 ESP32-兼容协议端到端验证（需手动转动电机）：
MONITOR START → 转电机 → 自动触发记录 → SNAP DONE → DUMP BIN → 解析校验。
用法: python test_c2k_protocol.py [COM口]
"""
import sys, time, struct, zlib
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM23"
SNAP_PREAMBLE = bytes([0xAA] * 10 + [0x55])
SNAP_MAGIC_V2 = 0xAB1C0002

def main():
    s = serial.Serial(PORT, 921600, timeout=0.3)
    time.sleep(0.3)
    s.reset_input_buffer()

    def cmd(c, wait=0.4):
        s.reset_input_buffer()
        s.write(c.encode() + b"\n")
        time.sleep(wait)
        return s.read(400).decode(errors="replace")

    print("[1] PING:", cmd("PING").splitlines()[0])
    print("[2] MONITOR START:", cmd("MONITOR START").splitlines()[0])
    print("[3] 请在 15 秒内转动电机（手动快速转动即可）...")
    s.reset_input_buffer()
    done = False
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        data = s.read(2000)
        if not data:
            continue
        txt = data.decode(errors="replace")
        if "SNAP DONE" in txt or "SNAP" in txt:
            pass
        for line in txt.splitlines():
            if line.startswith("# SNAP") and "done=1" in line:
                done = True
        if done:
            break
    print("[4] SNAP done:", done)
    if not done:
        print("    (未触发 — 转快一点或检查编码器接线)")
        s.close()
        return

    # DUMP BIN 拉取
    print("[5] DUMP BIN ...")
    s.reset_input_buffer()
    s.write(b"DUMP BIN\n")
    time.sleep(0.3)
    # 读 preamble+header
    buf = bytearray()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        buf.extend(s.read(4096))
        if len(buf) > 20:
            break
    pre_i = buf.find(SNAP_PREAMBLE)
    if pre_i < 0:
        print("    FAIL: no preamble, head=", bytes(buf[:20]).hex(" "))
        s.close()
        return
    hdr = bytes(buf[pre_i + 11: pre_i + 19])
    magic, n, hz = struct.unpack("<IHH", hdr)
    print(f"    magic=0x{magic:08X} n={n} hz={hz}")
    need = 8 + n * 16 + 4
    # 继续收集直到收满
    while len(buf) - pre_i < 11 + need:
        buf.extend(s.read(4096))
        if len(buf) - pre_i >= 11 + need:
            break
    raw = bytes(buf[pre_i + 11: pre_i + 11 + need])
    magic2, n2, hz2 = struct.unpack("<IHH", raw[0:8])
    payload = raw[8:8 + n2 * 16]
    (crc,) = struct.unpack("<I", raw[8 + n2 * 16:])
    got = zlib.crc32(payload) & 0xFFFFFFFF
    print(f"    CRC: got=0x{got:08X} expect=0x{crc:08X} {'OK' if got == crc else 'FAIL'}")
    if got == crc and n2 > 0:
        # 首尾采样点
        t0, c0, i0 = struct.unpack("<IqI", payload[0:16])
        tN, cN, iN = struct.unpack("<IqI", payload[-16:])
        dur_ms = (tN - t0) / 1000.0
        print(f"    首点 t={t0}us counts={c0} idx={i0}")
        print(f"    尾点 t={tN}us counts={cN} idx={iN}")
        print(f"    时长 {dur_ms:.0f} ms  Δcounts={cN - c0} → 平均 {(cN - c0) / dur_ms * 1000 / 4000 * 60:0f} RPM")
        print("    [PASS] 全链路验证通过")
    s.close()

if __name__ == "__main__":
    main()
