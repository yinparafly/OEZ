#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行：snap_*.bin → CSV（裁剪 / 转速过滤 / I0 / 距离 S）。"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from abi_monitor import parse_snap_file  # noqa: E402
from snap_edit import (  # noqa: E402
    delete_time_range,
    distance_series,
    filter_by_rpm,
    index_baseline,
    integrate_rpm_distance,
    keep_time_range,
)


def main() -> int:
    ap = argparse.ArgumentParser(description="snap_*.bin → CSV + S")
    ap.add_argument("bin", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=None)
    ap.add_argument("--t0", type=float, default=None, help="保留起点秒")
    ap.add_argument("--t1", type=float, default=None, help="保留终点秒")
    ap.add_argument("--delete-t0", type=float, default=None)
    ap.add_argument("--delete-t1", type=float, default=None)
    ap.add_argument("--rpm-min", type=float, default=0.0, help="丢掉 |RPM|<此值")
    ap.add_argument("--drop-zero", action="store_true")
    ap.add_argument("--i0", type=str, default="auto", help="I 基准，auto=首点")
    ap.add_argument("--len-i", type=float, default=1.0, help="每 I 对应长度")
    args = ap.parse_args()

    raw = args.bin.read_bytes()
    n, hz, rows = parse_snap_file(raw)
    if args.t0 is not None or args.t1 is not None:
        rows = keep_time_range(rows, args.t0 or 0.0, args.t1 if args.t1 is not None else 1e9)
    if args.delete_t0 is not None and args.delete_t1 is not None:
        rows = delete_time_range(rows, args.delete_t0, args.delete_t1)
    if args.rpm_min > 0 or args.drop_zero:
        rows = filter_by_rpm(rows, rpm_min=args.rpm_min, drop_zero=args.drop_zero)

    if args.i0.strip().lower() in ("", "auto"):
        i0 = index_baseline(rows)
    else:
        i0 = int(float(args.i0))
    series = distance_series(rows, i0=i0, length_per_i=args.len_i)
    s_rpm = integrate_rpm_distance(rows, length_per_rev=args.len_i)

    out = args.out or args.bin.with_suffix(".csv")
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_ms",
                "t_rel_s",
                "rpm",
                "dir",
                "index_n",
                "i_rel",
                "S_I",
                "S_rpm_integ",
                "I0",
                "length_per_I",
                "event_rpm",
            ]
        )
        for i, r in enumerate(rows):
            w.writerow(
                [
                    f"{float(r[0]):.3f}",
                    f"{series[i][0]:.6f}",
                    f"{float(r[1]):.2f}",
                    int(r[2]),
                    int(r[4]),
                    series[i][4],
                    f"{series[i][2]:.6f}",
                    f"{s_rpm[i][2]:.6f}",
                    i0,
                    args.len_i,
                    int(r[8]) if len(r) > 8 else 0,
                ]
            )
    s_end = series[-1][2] if series else 0.0
    print(f"OK n_in={n} hz={hz} n_out={len(rows)} I0={i0} S_end={s_end:.3f} → {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
