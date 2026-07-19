#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S3 自动对比：减速积分限制 / 加减速双增益（ADG）。

固定 S1 斜坡 + Ka，对比三组配置的 A1(升) / A2(降)：
  0) BASE   — ADG OFF（基线）
  1) SCALE  — ADG ON，减速 Ki×0.35，积分限 250
  2) DUAL   — ADG DUAL，加速/减速两套 kp/ki

用法:
  python auto_s3_adg_test.py --port COM10
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
    TrajResult,
    drain,
    keep_alive,
    probe_soft_up_down,
    run_traj,
    safe_stop,
)
from com_port_guard import acquire


def adg_cmd(ser, *parts: str) -> list[str]:
    line = " ".join(parts)
    send(ser, line)
    keep_alive(ser)
    time.sleep(0.08)
    metas, _ = drain(ser, 0.25)
    for m in metas:
        if "ADG" in m or "ERR" in m:
            print(" ", m[:120])
    return metas


def configure_mode(ser, mode: str, *, kp_a: float, ki_a: float, kp_d: float, ki_d: float) -> None:
    """mode: BASE | SCALE | DUAL"""
    if mode == "BASE":
        adg_cmd(ser, "ADG", "OFF")
        adg_cmd(ser, "ADG", "DUAL", "OFF")
    elif mode == "SCALE":
        adg_cmd(ser, "ADG", "DUAL", "OFF")
        adg_cmd(ser, "ADG", "KI_SCALE", "0.35")
        adg_cmd(ser, "ADG", "KP_SCALE", "1.0")
        adg_cmd(ser, "ADG", "ILIM", "250", "800")
        adg_cmd(ser, "ADG", "ON")
    elif mode == "DUAL":
        adg_cmd(ser, "ADG", "ACCEL", f"{kp_a:.4f}", f"{ki_a:.4f}")
        adg_cmd(ser, "ADG", "DECEL", f"{kp_d:.4f}", f"{ki_d:.4f}")
        adg_cmd(ser, "ADG", "ILIM", "250", "800")
        adg_cmd(ser, "ADG", "DUAL", "ON")
    else:
        raise ValueError(mode)
    adg_cmd(ser, "ADG?")


def main() -> int:
    ap = argparse.ArgumentParser(description="S3 ADG 减速积分/双增益对比")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    ap.add_argument("--up", type=float, default=800.0)
    ap.add_argument("--down", type=float, default=300.0)
    ap.add_argument("--ka", type=float, default=0.05)
    ap.add_argument("--kp-accel", type=float, default=0.08)
    ap.add_argument("--ki-accel", type=float, default=0.25)
    ap.add_argument("--kp-decel", type=float, default=0.06)
    ap.add_argument("--ki-decel", type=float, default=0.08)
    args = ap.parse_args()

    print(f"打开 {args.port} @ {BAUD} …")
    try:
        acquire("auto_s3_adg_test")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        return 1

    results: list[TrajResult] = []
    try:
        info = wait_banner(ser)
        print("板子:", info)
        if float(info.get("ctrl_hz") or 0) < 200:
            print("ctrl_hz 过低")
            return 2

        send(ser, "PROTO PWM")
        send(ser, "FREQ 50")
        send(ser, "STOP")
        send(ser, "MODE CLOSED")
        send(ser, f"RPMMAX {args.rpmmax:.0f}")
        send(ser, f"KA {args.ka:.4f}")
        if not probe_soft_up_down(ser):
            print("需要 S1 固件")
            return 6

        send(ser, "ADG?")
        time.sleep(0.1)
        metas, _ = drain(ser, 0.3)
        if not any("ADG" in m for m in metas):
            print("固件无 ADG — 请烧录含 S3 的固件")
            for m in metas[:6]:
                print(" ", m[:100])
            return 6

        modes = ("BASE", "SCALE", "DUAL")
        trajs = (
            ("A1", args.up, args.up, 1000.0, 2000.0),
            ("A2", args.up, args.down, 3000.0, 2000.0),
        )

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = LOG_DIR / f"s3_adg_{ts}.csv"

        with csv_path.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(
                [
                    "mode", "traj", "up", "down", "from", "to",
                    "t_in_band", "mean_tail", "peak", "pulse_end",
                    "overshoot_pct", "verdict", "note",
                ]
            )

            for mode in modes:
                print(f"\n======== 配置 {mode} ========")
                configure_mode(
                    ser,
                    mode,
                    kp_a=args.kp_accel,
                    ki_a=args.ki_accel,
                    kp_d=args.kp_decel,
                    ki_d=args.ki_decel,
                )
                for tname, up, down, f, t in trajs:
                    name = f"{mode}_{tname}"
                    print(f"\n>>> {name}: {f:.0f}→{t:.0f} UP={up:.0f} DOWN={down:.0f} …")
                    tr = run_traj(
                        ser,
                        name=name,
                        up=up,
                        down=down,
                        rpm_from=f,
                        rpm_to=t,
                        settle_s=args.settle,
                        rpmmax=args.rpmmax,
                    )
                    # 附带 mode 信息到 note
                    tr.note = f"[{mode}] {tr.note}"
                    results.append(tr)
                    t_in = tr.t_in_band_s if tr.t_in_band_s is not None else -1.0
                    print(
                        f"    → {tr.verdict}  t_band={t_in:.2f}s  "
                        f"tail={tr.mean_tail:.0f} over={tr.overshoot_pct:.1f}%  "
                        f"pulse={tr.pulse_end}"
                    )
                    w.writerow(
                        [
                            mode, tname, f"{up:.0f}", f"{down:.0f}",
                            f"{f:.0f}", f"{t:.0f}",
                            f"{t_in:.3f}", f"{tr.mean_tail:.1f}", f"{tr.peak:.1f}",
                            tr.pulse_end, f"{tr.overshoot_pct:.2f}",
                            tr.verdict, tr.note,
                        ]
                    )
                    fp.flush()
                    safe_stop(ser)
                    time.sleep(0.8)

        # 对比表（重点 A2 降速）
        def find(mode: str, traj: str) -> TrajResult | None:
            for r in results:
                if r.name == f"{mode}_{traj}":
                    return r
            return None

        lines = [
            "S3 ADG 对比报告（空载）",
            f"port={args.port} Ka={args.ka} SOFT UP/DOWN={args.up:.0f}/{args.down:.0f}",
            f"SCALE: KI_SCALE=0.35 KP_SCALE=1.0 ILIM_DECEL=250",
            f"DUAL: ACCEL kp/ki={args.kp_accel}/{args.ki_accel}  "
            f"DECEL kp/ki={args.kp_decel}/{args.ki_decel}",
            "",
            f"{'mode':<8} {'traj':<4} {'t_band':>8} {'tail':>8} {'peak':>8} "
            f"{'over%':>7} {'pulse':>6}  verdict",
        ]
        for mode in modes:
            for tname in ("A1", "A2"):
                r = find(mode, tname)
                if not r:
                    continue
                t_in = r.t_in_band_s if r.t_in_band_s is not None else -1.0
                lines.append(
                    f"{mode:<8} {tname:<4} {t_in:8.2f} {r.mean_tail:8.1f} {r.peak:8.1f} "
                    f"{r.overshoot_pct:7.1f} {r.pulse_end:6d}  {r.verdict}"
                )

        lines.append("")
        lines.append("—— A2 降速对比（相对 BASE）——")
        base_a2 = find("BASE", "A2")
        for mode in ("SCALE", "DUAL"):
            r = find(mode, "A2")
            if not r or not base_a2 or base_a2.t_in_band_s is None or r.t_in_band_s is None:
                lines.append(f"{mode}: 缺数据")
                continue
            dt = r.t_in_band_s - base_a2.t_in_band_s
            do = r.overshoot_pct - base_a2.overshoot_pct
            lines.append(
                f"{mode}: t_band {base_a2.t_in_band_s:.2f}→{r.t_in_band_s:.2f}s (Δ={dt:+.2f}s); "
                f"overshoot {base_a2.overshoot_pct:.1f}→{r.overshoot_pct:.1f}% (Δ={do:+.1f}%)"
            )

        lines.append("")
        lines.append("—— A1 升速对比（相对 BASE）——")
        base_a1 = find("BASE", "A1")
        for mode in ("SCALE", "DUAL"):
            r = find(mode, "A1")
            if not r or not base_a1 or base_a1.t_in_band_s is None or r.t_in_band_s is None:
                lines.append(f"{mode}: 缺数据")
                continue
            dt = r.t_in_band_s - base_a1.t_in_band_s
            do = r.overshoot_pct - base_a1.overshoot_pct
            lines.append(
                f"{mode}: t_band {base_a1.t_in_band_s:.2f}→{r.t_in_band_s:.2f}s (Δ={dt:+.2f}s); "
                f"overshoot {base_a1.overshoot_pct:.1f}→{r.overshoot_pct:.1f}% (Δ={do:+.1f}%)"
            )

        # 决策建议
        lines.append("")
        scale_a2 = find("SCALE", "A2")
        dual_a2 = find("DUAL", "A2")
        decision = "维持 ADG OFF（相对基线无明显收益）"
        if base_a2 and scale_a2 and base_a2.t_in_band_s and scale_a2.t_in_band_s:
            # 减速段：更跟指令/更少假超调算收益；空载常被 DOWN 斜坡主导
            better_scale = (
                scale_a2.verdict == "PASS"
                and scale_a2.overshoot_pct + 0.2 < base_a2.overshoot_pct
                and (scale_a2.t_in_band_s or 99) <= (base_a2.t_in_band_s or 99) + 0.05
            )
            better_dual = False
            if dual_a2 and dual_a2.t_in_band_s is not None:
                better_dual = (
                    dual_a2.verdict == "PASS"
                    and dual_a2.overshoot_pct + 0.2 < base_a2.overshoot_pct
                )
            if better_dual and dual_a2 and (
                not better_scale
                or (scale_a2 and dual_a2.overshoot_pct <= scale_a2.overshoot_pct)
            ):
                decision = "建议 ADG DUAL ON（减速超调/积分更干净），并 ADG SAVE"
            elif better_scale:
                decision = "建议 ADG ON + SCALE（KI_SCALE=0.35 ILIM=250），并 ADG SAVE"
            elif (
                scale_a2
                and scale_a2.verdict == "PASS"
                and base_a2.verdict == "PASS"
            ):
                # 空载常见：时间略变、超调略升/降，收益不足以默认打开
                if (
                    scale_a2.overshoot_pct > base_a2.overshoot_pct + 0.5
                    and (scale_a2.t_in_band_s or 0) >= (base_a2.t_in_band_s or 0) - 0.2
                ):
                    decision = (
                        "维持 ADG OFF：SCALE/DUAL 未降低超调；"
                        "空载减速由 DOWN 斜坡主导，扑翼后再开 ADG"
                    )
                else:
                    decision = (
                        "三组均可 PASS；空载减速主要由 DOWN 斜坡主导，"
                        "ADG 收益有限——默认 ADG OFF，扑翼后再开"
                    )

        all_pass = all(r.verdict == "PASS" for r in results)
        lines += [
            f"决策: {decision}",
            f"OVERALL: {'PASS' if all_pass else 'CHECK'}",
            f"csv={csv_path}",
        ]
        text = "\n".join(lines)
        rep = csv_path.with_suffix(".report.txt")
        rep.write_text(text, encoding="utf-8")
        root = Path(__file__).resolve().parent.parent / f"S3_ADG对比报告_{ts}.txt"
        root.write_text(text + "\n", encoding="utf-8")

        # 测完回到安全：ADG OFF（除非决策要开——这里不自动 SAVE，避免改掉用户 NVS）
        adg_cmd(ser, "ADG", "OFF")
        safe_stop(ser)

        print("\n======== S3 报告 ========")
        print(text)
        print(f"报告: {rep}")
        print(f"副本: {root}")
        return 0 if all_pass else 4
    except KeyboardInterrupt:
        print("\n中断")
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"异常: {exc}")
        return 5
    finally:
        try:
            send(ser, "ADG OFF")
            safe_stop(ser)
            send(ser, "FREQ 50")
        except Exception:  # noqa: BLE001
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
