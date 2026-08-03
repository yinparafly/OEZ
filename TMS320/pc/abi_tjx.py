import re, struct
SNAP_PREAMBLE = b"\xAA" * 10 + b"\x55"
SNAP_MAGIC_V2 = 0xAB1C0002
SNAP_POINT_SIZE_V2 = 16
SNAP_POINT_FMT = "<IqI"          # t_us u32, counts i64, index_n u32 (LE)

def snap_crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1))
    return ~crc & 0xFFFFFFFF

_BIN_END_RE = re.compile(
    r"# BIN END n=(\d+) hz=(\d+) steps=(-?\d+) mode=(\w+) bytes=(\d+) crc=0x([0-9A-Fa-f]+)")

def parse_bin_end(line: str) -> dict | None:
    m = _BIN_END_RE.search(line)
    if not m:
        return None
    return {"n": int(m.group(1)), "hz": int(m.group(2)), "steps": int(m.group(3)),
            "mode": m.group(4), "bytes": int(m.group(5)), "crc": int(m.group(6), 16)}

def rpm_from_counts_series(t_us, counts, *, steps=4000, vel_win=16):
    out = [0.0] * len(counts)
    w = max(1, int(vel_win))
    for i in range(len(counts)):
        j = i - w if i >= w else 0
        dc = counts[i] - counts[j]
        dt = t_us[i] - t_us[j]
        if dt <= 0:
            out[i] = out[i - 1] if i else 0.0
            continue
        out[i] = (dc / steps) * (1_000_000.0 / dt) * 60.0
    return out

def parse_snap_bindump(raw: bytes, steps: int = 4000):
    if len(raw) < 12:
        raise ValueError(f"bin too short: {len(raw)}")
    magic, n, hz = struct.unpack_from("<IHH", raw, 0)
    if magic != SNAP_MAGIC_V2:
        raise ValueError(f"bad magic 0x{magic:08X}")
    need = 8 + n * SNAP_POINT_SIZE_V2 + 4
    if len(raw) < need:
        raise ValueError(f"bin len {len(raw)} < need {need}")
    payload = raw[8:8 + n * SNAP_POINT_SIZE_V2]
    (crc,) = struct.unpack_from("<I", raw, 8 + n * SNAP_POINT_SIZE_V2)
    got = snap_crc32(payload)
    if got != crc:
        raise ValueError(f"CRC mismatch got=0x{got:08X} expect=0x{crc:08X}")
    t_list, c_list, idx_list = [], [], []
    for i in range(n):
        t_us, counts, index_n = struct.unpack_from(SNAP_POINT_FMT, payload, i * SNAP_POINT_SIZE_V2)
        t_list.append(int(t_us)); c_list.append(int(counts)); idx_list.append(int(index_n))
    rpms = rpm_from_counts_series(t_list, c_list, steps=steps)
    rows = []
    for i in range(n):
        rpm = rpms[i]
        direc = 1 if rpm > 0.5 else (-1 if rpm < -0.5 else 0)
        rows.append((t_list[i] / 1000.0, abs(rpm), direc, 1, idx_list[i], 0, 0, c_list[i]))
    return n, hz, rows

def parse_bin_frame(buf: bytes):
    if buf.startswith(SNAP_PREAMBLE):
        buf = buf[len(SNAP_PREAMBLE):]
    if len(buf) < 12:
        return None
    return parse_snap_bindump(buf)
