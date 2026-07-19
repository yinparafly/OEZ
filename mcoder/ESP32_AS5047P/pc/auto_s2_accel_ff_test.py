#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S2 自动验证：加速度前馈 Ka（备忘）。

对比同一轨迹 A1(1000→2000)、A2(3000→2000)：
  Ka=0 vs Ka>0（默认 0.05 us/(rpm/s)）
看入带时间 / 是否仍 PASS。

用法:
  python auto_s2_accel_ff_test.py --port COM10
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

from auto_learn_test import BAUD, DEFAULT_PORT, LOG_DIR, open_port, send, wait_banner
from auto_s1_accel_decel_test import (
    S1Report,
    TrajResult,
    drain,
    keep_alive,
    probe_soft_up_down,
    run_traj,
    safe_stop,
    set_soft,
)
from com_port_guard import acquire


def set_ka(ser, ka: float) -> None:
    send(ser, f"KA {ka:.4f}")
    keep_alive(ser)
    time.sleep(0.05)
    send(ser, "KA?")
    metas, _ = drain(ser, 0.2)
    for m in metas:
        if "KA" in m:
            print(" ", m[:100])


def main() -> int:
    ap = argparse.ArgumentParser(description="S2 Ka 加速度前馈对比")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--ka", type=float, default=0.05, help="试验 Ka，单位 us/(rpm/s)")
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    ap.add_argument("--up", type=float, default=800.0)
    ap.add_argument("--down", type=float, default=300.0)
    args = ap.parse_args()

    print(f"打开 {args.port} @ {BAUD} …")
    try:
        acquire("auto_s2_accel_ff_test")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        return 1

    try:
        info = wait_banner(ser)
        print("板子:", info)
        if float(info.get("ctrl_hz") or 0) < 200:
            print("ctrl_hz 过低")
            return 2

        send(ser, "STOP")
        send(ser, "MODE CLOSED")
        send(ser, f"RPMMAX {args.rpmmax:.0f}")
        if not probe_soft_up_down(ser):
            print("需要已烧录 S1 的固件（SOFT RATE UP/DOWN）")
            return 6
        # 探测 KA
        send(ser, "KA?")
        time.sleep(0.1)
        metas, _ = drain(ser, 0.25)
        if not any("KA=" in m for m in metas):
            print("固件无 KA 命令 — 请烧录含 S2 的固件")
            for m in metas[:5]:
                print(" ", m[:100])
            return 6

        report = S1Report(port=args.port, soft_up_down=True)
        jobs = [
            ("A1_ka0", 0.0, args.up, args.up, 1000.0, 2000.0),
            ("A2_ka0", 0.0, args.up, args.down, 3000.0, 2000.0),
            ("A1_ka", args.ka, args.up, args.up, 1000.0, 2000.0),
            ("A2_ka", args.ka, args.up, args.down, 3000.0, 2000.0),
        ]

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = LOG_DIR / f"s2_accel_ff_{ts}.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(
                [
                    "name", "ka", "up", "down", "from", "to", "t_in_band",
                    "mean_tail", "peak", "verdict", "note",
                ]
            )
            for name, ka, up, down, f, t in jobs:
                print(f"\n>>> {name}: Ka={ka}  {f:.0f}→{t:.0f} …")
                set_ka(ser, ka)
                set_soft(ser, up, down)
                res = run_traj(
                    ser,
                    name=name,
                    up=up,
                    down=down,
                    rpm_from=f,
                    rpm_to=t,
                    settle_s=args.settle,
                    rpmmax=args.rpmmax,
                )
                report.results.append(res)
                w.writerow(
                    [
                        name,
                        f"{ka:.4f}",
                        f"{up:.0f}",
                        f"{down:.0f}",
                        f"{f:.0f}",
                        f"{t:.0f}",
                        f"{res.t_in_band_s:.3f}" if res.t_in_band_s is not None else "",
                        f"{res.mean_tail:.2f}",
                        f"{res.peak:.2f}",
                        res.verdict,
                        res.note,
                    ]
                )
                fp.flush()
                print(f"    → {res.verdict}  t_band={res.t_in_band_s}  {res.note}")
                if res.verdict == "ABORT":
                    break

        set_ka(ser, 0.0)
        safe_stop(ser)

        # 自定义对比摘要
        def find(n: str) -> TrajResult | None:
            return next((r for r in report.results if r.name == n), None)

        a1_0, a1_k = find("A1_ka0"), find("A1_ka")
        a2_0, a2_k = find("A2_ka0"), find("A2_ka")
        lines = [
            f"port={args.port} ka_test={args.ka}",
            "S2 加速度前馈对比（A1 升 / A2 降）",
            "",
            report.summary(),
            "",
            "—— Ka 效果 ——",
        ]
        if a1_0 and a1_k and a1_0.t_in_band_s and a1_k.t_in_band_s:
            d = a1_k.t_in_band_s - a1_0.t_in_band_s
            lines.append(
                f"A1 升速入带: Ka=0 → {a1_0.t_in_band_s:.2f}s ; "
                f"Ka={args.ka} → {a1_k.t_in_band_s:.2f}s (Δ={d:+.2f}s)"
            )
        if a2_0 and a2_k and a2_0.t_in_band_s and a2_k.t_in_band_s:
            d = a2_k.t_in_band_s - a2_0.t_in_band_s
            lines.append(
                f"A2 降速入带: Ka=0 → {a2_0.t_in_band_s:.2f}s ; "
                f"Ka={args.ka} → {a2_k.t_in_band_s:.2f}s (Δ={d:+.2f}s)"
            )
        fails = [r for r in report.results if r.verdict.startswith("FAIL")]
        if not fails and all(r.verdict == "PASS" for r in report.results):
            lines.append("OVERALL: PASS（S2 Ka 可保留；再视跟踪误差微调）")
            code = 0
        elif not fails:
            lines.append("OVERALL: PASS_MARGINAL")
            code = 0
        else:
            lines.append("OVERALL: FAIL")
            code = 4

        text = "\n".join(lines) + f"\n\ncsv={csv_path}\n"
        rep = csv_path.with_suffix(".report.txt")
        rep.write_text(text, encoding="utf-8")
        print("\n======== S2 报告 ========")
        print(text)
        print(f"报告: {rep}")
        return code
    except KeyboardInterrupt:
        safe_stop(ser)
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"异常: {exc}")
        try:
            send(ser, "ESTOP")
            safe_stop(ser)
        except Exception:  # noqa: BLE001
            pass
        return 5
    finally:
        try:
            safe_stop(ser)
        except Exception:  # noqa: BLE001
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
