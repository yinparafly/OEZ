# -*- coding: utf-8 -*-
"""离线：构造假 snap BIN，校验 parse_snap_bindump（无需硬件）。"""
from __future__ import annotations

import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from abi_monitor import SNAP_MAGIC, SNAP_POINT_SIZE, parse_snap_bindump, snap_crc32


def _make_point(t_us: int, rpm_x10: int, direc: int, index_n: int) -> bytes:
    return struct.pack("<IhbBI", t_us, rpm_x10, direc, 0, index_n)


def main() -> int:
    n, hz = 100, 2000
    payload = b"".join(
        _make_point(i * 500, 1200 + (i % 7), 1 if i % 2 == 0 else -1, i // 10)
        for i in range(n)
    )
    assert len(payload) == n * SNAP_POINT_SIZE
    crc = snap_crc32(payload)
    assert crc == (zlib.crc32(payload) & 0xFFFFFFFF)
    blob = struct.pack("<IHH", SNAP_MAGIC, n, hz) + payload + struct.pack("<I", crc)
    got_n, got_hz, rows = parse_snap_bindump(blob)
    assert got_n == n and got_hz == hz and len(rows) == n
    assert abs(rows[0][1] - 120.0) < 0.01
    print(f"OK offline parse n={got_n} hz={got_hz} rows={len(rows)} crc=0x{crc:08X}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
