import struct, sys, zlib
sys.path.insert(0, r"D:\oezcon\TMS320\pc")
from abi_tjx import (SNAP_MAGIC_V3, SNAP_POINT_SIZE_V3, snap_crc32,
                     parse_bin_end, parse_snap_bindump, parse_bin_frame,
                     rpm_from_counts_series)

def _make_v3(t_us, counts, idx, erpm=0):
    return struct.pack("<IqIi", t_us, counts, idx, erpm)

def main() -> int:
    assert snap_crc32(b"") == 0
    assert snap_crc32(b"123456789") == 0xCBF43926
    assert snap_crc32(b"abc") == zlib.crc32(b"abc")

    m = parse_bin_end("# BIN END n=10 hz=0 steps=4000 mode=event bytes=212 crc=0x12345678")
    assert m and m["n"] == 10 and m["steps"] == 4000 and m["mode"] == "event"

    n = 64
    payload = b"".join(_make_v3(i * 500, i, 0, i * 30) for i in range(n))
    crc = snap_crc32(payload)
    blob = struct.pack("<IHH", SNAP_MAGIC_V3, n, 0) + payload + struct.pack("<I", crc)
    gn, ghz, rows = parse_snap_bindump(blob)
    assert gn == n and ghz == 0 and len(rows) == n
    assert abs(rows[-1][1] - 30.0) < 0.5, rows[-1][1]   # 1/4000*1e6/500*60=30
    assert rows[-1][7] == n - 1                          # counts 透传
    assert rows[-1][8] == (n - 1) * 30                   # event_rpm 透传

    rpms = rpm_from_counts_series([i*500 for i in range(n)], list(range(n)), steps=4000)
    assert abs(rpms[-1] - 30.0) < 0.5

    frame = b"\xAA"*10 + b"\x55" + blob
    r2 = parse_bin_frame(frame)
    assert r2 and r2[0] == n

    try:
        blob_bad = blob[:-1]
        parse_snap_bindump(blob_bad)
        return 1
    except ValueError:
        pass
    print("test_abi_tjx: ALL PASS")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
