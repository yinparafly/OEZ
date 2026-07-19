#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
T4 停转启动抽检（修订版）：1000μs 停稳 → 闭环到 1000 RPM，重复 N 次。

判据（方案）：启动可靠入带 → 维持「停转+斜坡」，无需微转怠速。

用法:
  python auto_t4_idle_start_test.py --port COM10
  python auto_t4_idle_start_test.py --repeats 3 --settle 8
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from auto_learn_test import BAUD, DEFAULT_PORT, LOG_DIR, open_port, send, wait_banner
from auto_s1_accel_decel_test import drain, keep_alive, run_traj, safe_stop
from com_port_guard import acquire

TARGET_RPM = 1000.0


@dataclass
class Trial:
    n: int
    t_in_band: float
    mean_tail: float
    peak: float
    verdict: str
    note: str


def main() -> int:
    ap = argparse.ArgumentParser(description="T4 停转→1000RPM 启动抽检")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--up", type=float, default=800.0)
    ap.add_argument("--down", type=float, default=300.0)
    args = ap.parse_args()

    print(f"打开 {args.port} @ {BAUD} …")
    try:
        acquire("auto_t4_idle_start_test")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        return 1

    trials: list[Trial] = []
    try:
        info = wait_banner(ser)
        print("板子:", info)
        ctrl = float(info.get("ctrl_hz") or 0)
        if ctrl < 200:
            print(f"FAIL: ctrl_hz={ctrl} < 200")
            return 2

        send(ser, "PROTO PWM")
        send(ser, "FREQ 50")
        send(ser, "STOP")
        send(ser, "MODE CLOSED")
        send(ser, "RPMMAX 6000")
        send(ser, f"SOFT RATE UP {args.up:.0f}")
        send(ser, f"SOFT RATE DOWN {args.down:.0f}")
        send(ser, "KA 0.05")
        drain(ser, 0.3)

        for i in range(1, max(1, args.repeats) + 1):
            print(f"\n>>> T4#{i}: 停转 3s → 1000 RPM …")
            send(ser, "RPM 0")
            send(ser, "PWM 1000")
            send(ser, "STOP")
            t0 = time.time()
            while time.time() - t0 < 3.0:
                keep_alive(ser)
                drain(ser, 0.1)

            tr = run_traj(
                ser,
                name=f"T4_{i}",
                up=args.up,
                down=args.down,
                rpm_from=0.0,
                rpm_to=TARGET_RPM,
                settle_s=args.settle,
                rpmmax=6000.0,
            )
            t_in = tr.t_in_band_s if tr.t_in_band_s is not None else -1.0
            trials.append(
                Trial(i, t_in, tr.mean_tail, tr.peak, tr.verdict, tr.note)
            )
            print(
                f"    → {tr.verdict}  t_band={t_in:.2f}s  "
                f"tail={tr.mean_tail:.0f}  {tr.note}"
            )
            safe_stop(ser)
            time.sleep(1.0)

        passes = sum(1 for t in trials if t.verdict == "PASS")
        overall = "PASS" if passes == len(trials) else ("MARGINAL" if passes else "FAIL")
        decision = (
            "维持停转+斜坡（无需微转怠速）"
            if overall == "PASS"
            else "启动不稳，考虑可配置 IDLE_US=1050 再测"
        )

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = LOG_DIR / f"t4_idle_start_{ts}.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(["n", "t_in_band", "mean_tail", "peak", "verdict", "note"])
            for t in trials:
                w.writerow(
                    [
                        t.n,
                        f"{t.t_in_band:.3f}",
                        f"{t.mean_tail:.1f}",
                        f"{t.peak:.1f}",
                        t.verdict,
                        t.note,
                    ]
                )

        lines = [
            "T4 停转启动抽检（空载）",
            f"port={args.port} target={TARGET_RPM:.0f} repeats={len(trials)}",
            f"SOFT UP/DOWN={args.up:.0f}/{args.down:.0f}  FREQ=50  Ka=0.05",
            "",
            f"{'n':>3} {'t_band':>8} {'tail':>8} {'peak':>8}  verdict  note",
        ]
        for t in trials:
            lines.append(
                f"{t.n:3d} {t.t_in_band:8.2f} {t.mean_tail:8.1f} {t.peak:8.1f}  "
                f"{t.verdict:<7} {t.note}"
            )
        lines += ["", f"OVERALL: {overall}", f"决策: {decision}", f"csv={csv_path}"]
        text = "\n".join(lines)
        rep = csv_path.with_suffix(".report.txt")
        rep.write_text(text, encoding="utf-8")
        root = Path(__file__).resolve().parent.parent / f"T4停转启动报告_{ts}.txt"
        root.write_text(text + "\n", encoding="utf-8")

        print("\n======== T4 报告 ========")
        print(text)
        print(f"报告: {rep}")
        return 0 if overall == "PASS" else 4
    except KeyboardInterrupt:
        print("\n中断")
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"异常: {exc}")
        return 5
    finally:
        try:
            safe_stop(ser)
            send(ser, "PROTO PWM")
            send(ser, "FREQ 50")
        except Exception:  # noqa: BLE001
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
