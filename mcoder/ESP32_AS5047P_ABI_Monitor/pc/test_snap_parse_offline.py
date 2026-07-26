# -*- coding: utf-8 -*-
"""离线：构造假 snap BIN v1/v2，校验 parse_snap_bindump。"""
from __future__ import annotations

import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from abi_monitor import (
    SNAP_MAGIC_V1,
    SNAP_MAGIC_V2,
    SNAP_POINT_SIZE_V1,
    SNAP_POINT_SIZE_V2,
    parse_snap_bindump,
    snap_crc32,
)


def _make_v1(t_us: int, rpm_x10: int, direc: int, index_n: int) -> bytes:
    return struct.pack("<IhbBI", t_us, rpm_x10, direc, 0, index_n)


def _make_v2(t_us: int, counts: int, index_n: int) -> bytes:
    return struct.pack("<IqI", t_us, counts, index_n)


def main() -> int:
    # v1 回归
    n, hz = 100, 2000
    payload = b"".join(
        _make_v1(i * 500, 1200 + (i % 7), 1 if i % 2 == 0 else -1, i // 10)
        for i in range(n)
    )
    assert len(payload) == n * SNAP_POINT_SIZE_V1
    crc = snap_crc32(payload)
    assert crc == (zlib.crc32(payload) & 0xFFFFFFFF)
    blob = struct.pack("<IHH", SNAP_MAGIC_V1, n, hz) + payload + struct.pack("<I", crc)
    got_n, got_hz, rows = parse_snap_bindump(blob)
    assert got_n == n and got_hz == hz and len(rows) == n
    assert abs(rows[0][1] - 120.0) < 0.01

    # v2：匀速约 30 RPM → 每 500us +1 count @4000 steps
    # rpm = 1/4000 * 1e6/500 * 60 = 30
    n2 = 64
    payload2 = b"".join(_make_v2(i * 500, i, 0) for i in range(n2))
    assert len(payload2) == n2 * SNAP_POINT_SIZE_V2
    crc2 = snap_crc32(payload2)
    blob2 = struct.pack("<IHH", SNAP_MAGIC_V2, n2, hz) + payload2 + struct.pack("<I", crc2)
    gn, ghz, rows2 = parse_snap_bindump(blob2)
    assert gn == n2 and ghz == hz
    # 满窗后应接近 30 RPM
    assert abs(rows2[-1][1] - 30.0) < 0.5, rows2[-1][1]

    print(f"OK offline parse v1+v2 n={got_n}/{gn} last_rpm≈{rows2[-1][1]:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
