#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空载多点闭环转速测试（含 6000 RPM）。

默认测点: 1000, 2000, 3000, 4000, 5000, 6000
每点: 从停转/上一档斜坡到目标 → 停留 settle 秒 → 记均值/峰值/入带时间 → 停稳再下一点

用法:
  python auto_noload_rpm_ladder.py --port COM10
  python auto_noload_rpm_ladder.py --freq 400
  python auto_noload_rpm_ladder.py --targets 1000,2000,3000,4000,5000,6000
"""

from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

from auto_learn_test import BAUD, DEFAULT_PORT, LOG_DIR, open_port, send, wait_banner
from auto_s1_accel_decel_test import drain, keep_alive, run_traj, safe_stop, set_soft
from com_port_guard import acquire

MIN_SPIN = 40.0
BAND_FRAC = 0.10
DEFAULT_TARGETS = (1000, 2000, 3000, 4000, 5000, 6000)


def set_ka(ser, ka: float) -> None:
    send(ser, f"KA {ka:.4f}")
    keep_alive(ser)
    drain(ser, 0.15)


def write_report(
    path: Path,
    *,
    port: str,
    info: dict,
    up: float,
    down: float,
    ka: float,
    settle: float,
    rpmmax: float,
    esc_pwm_hz: int,
    rows: list[dict],
) -> str:
    lines: list[str] = []
    lines.append("# 空载闭环多点转速测试报告")
    lines.append("")
    lines.append("**工况标注：空载（noload，电机未接外部负载，仅转子惯量）**")
    lines.append("")
    lines.append("## 1. 试验条件")
    lines.append("")
    lines.append(f"| 项 | 值 |")
    lines.append(f"|----|----|")
    lines.append(f"| 日期时间 | {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} |")
    lines.append(f"| 串口 | {port} @ {BAUD} |")
    lines.append(f"| ctrl_hz | {info.get('ctrl_hz', '?')} |")
    lines.append(f"| **ESC PWM FREQ** | **{esc_pwm_hz} Hz** |")
    lines.append(f"| 模式 | CLOSED + 前馈图 + GainMap（若有效） |")
    lines.append(f"| RPMMAX | {rpmmax:.0f} |")
    lines.append(f"| SOFT RATE UP / DOWN | {up:.0f} / {down:.0f} RPM/s |")
    lines.append(f"| Ka（加速度前馈） | {ka:.4f} μs/(rpm/s) |")
    lines.append(f"| 每点观察时间 | {settle:.1f} s |")
    lines.append(f"| 入带判据 | ±{BAND_FRAC*100:.0f}% 目标转速 |")
    lines.append(f"| Profile | noload（空载） |")
    lines.append("")
    lines.append("## 2. 测点结果")
    lines.append("")
    lines.append(
        "| 目标 RPM | 入带时间 s | 末段均值 RPM | 峰值 RPM | 末脉宽 μs | "
        "稳态误差 % | 结论 | 备注 |"
    )
    lines.append(
        "|----------|------------|--------------|----------|-----------|"
        "------------|------|------|"
    )

    n_pass = 0
    n_fail = 0
    for r in rows:
        tgt = r["target"]
        mean = r["mean"]
        err_pct = (abs(mean - tgt) / max(tgt, 1.0)) * 100.0 if mean > MIN_SPIN else 999.0
        t_in = r["t_in"]
        t_s = f"{t_in:.2f}" if t_in is not None else "—"
        lines.append(
            f"| {tgt:.0f} | {t_s} | {mean:.1f} | {r['peak']:.1f} | {r['pulse']} | "
            f"{err_pct:.1f} | **{r['verdict']}** | {r['note']} |"
        )
        if r["verdict"] == "PASS":
            n_pass += 1
        elif r["verdict"].startswith("FAIL"):
            n_fail += 1

    lines.append("")
    lines.append("## 3. 汇总")
    lines.append("")
    lines.append(f"- ESC PWM 刷新率: **{esc_pwm_hz} Hz**")
    lines.append(f"- 测点数: {len(rows)}")
    lines.append(f"- PASS: {n_pass}")
    lines.append(f"- FAIL: {n_fail}")
    lines.append(f"- 其它: {len(rows) - n_pass - n_fail}")
    lines.append("")

    # 最高通过点
    passed = [r for r in rows if r["verdict"] == "PASS"]
    if passed:
        hi = max(passed, key=lambda x: x["target"])
        lines.append(
            f"- 空载下最高稳定测点: **{hi['target']:.0f} RPM** "
            f"（均值 {hi['mean']:.0f}，入带 {hi['t_in']:.2f}s）"
        )
    failed = [r for r in rows if r["verdict"].startswith("FAIL")]
    if failed:
        lines.append(
            "- 失败测点: "
            + ", ".join(f"{r['target']:.0f}({r['verdict']})" for r in failed)
        )

    lines.append("")
    lines.append("## 4. 说明")
    lines.append("")
    lines.append("- 本报告仅反映 **空载** 台架；接扑翼/负载后须用 `flap` profile 重学并重测。")
    lines.append("- 6000 RPM 为电机端目标上限；超速保护约 1.10×RPMMAX。")
    lines.append("- 入带时间受 SOFT 升斜率与 Ka 影响，不代表电调电气极限。")
    lines.append(f"- 本轮强制 `FREQ {esc_pwm_hz}`；结束后回安全默认 50 Hz。")
    lines.append("")
    if n_fail == 0 and n_pass == len(rows) and rows:
        lines.append(f"**总评：PASS（空载多点闭环 @ {esc_pwm_hz} Hz）**")
    elif n_pass > 0:
        lines.append(f"**总评：PARTIAL（@{esc_pwm_hz} Hz 部分测点通过，见上表）**")
    else:
        lines.append(f"**总评：FAIL（@{esc_pwm_hz} Hz）**")

    text = "\n".join(lines) + "\n"
    path.write_text(text, encoding="utf-8")
    return text


def main() -> int:
    ap = argparse.ArgumentParser(description="空载多点闭环至 6000")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--targets", default="", help="逗号分隔，默认 1000…6000")
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    ap.add_argument("--up", type=float, default=800.0)
    ap.add_argument("--down", type=float, default=300.0)
    ap.add_argument("--ka", type=float, default=0.05)
    ap.add_argument(
        "--freq",
        type=int,
        default=50,
        help="ESC PWM 刷新率 Hz（50..600），本测强制设置",
    )
    args = ap.parse_args()

    if args.targets.strip():
        targets = [float(x) for x in args.targets.split(",") if x.strip()]
    else:
        targets = [float(x) for x in DEFAULT_TARGETS]
    targets = [t for t in targets if 100 <= t <= args.rpmmax]
    if not targets:
        print("无有效测点")
        return 1
    esc_hz = max(50, min(600, int(args.freq)))

    print(f"打开 {args.port} @ {BAUD} …")
    try:
        acquire("auto_noload_rpm_ladder")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        return 1

    try:
        info = wait_banner(ser, timeout_s=4.0)
        print("板子:", info)
        if float(info.get("ctrl_hz") or 0) < 200:
            print("ctrl_hz 过低，中止")
            return 2

        print(
            f"空载多点测试 @ FREQ={esc_hz} Hz: {targets}\n"
            f"  UP={args.up:.0f} DOWN={args.down:.0f} Ka={args.ka} settle={args.settle}s"
        )
        send(ser, "STOP")
        send(ser, "PROTO PWM")
        send(ser, f"FREQ {esc_hz}")
        time.sleep(0.2)
        metas, _ = drain(ser, 0.3)
        for m in metas:
            if "FREQ" in m or "ERR" in m:
                print(" ", m[:100])
        send(ser, "PROFILE noload")
        send(ser, "MODE CLOSED")
        send(ser, "GAIN ON")
        send(ser, "ADG OFF")
        send(ser, f"RPMMAX {args.rpmmax:.0f}")
        set_ka(ser, args.ka)
        set_soft(ser, args.up, args.down)
        drain(ser, 0.3)

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = LOG_DIR / f"noload_ladder_f{esc_hz}_{ts}.csv"
        rows: list[dict] = []

        with csv_path.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(
                [
                    "load",
                    "esc_pwm_hz",
                    "target",
                    "t_in_band_s",
                    "mean_rpm",
                    "peak_rpm",
                    "pulse_us",
                    "verdict",
                    "note",
                    "up",
                    "down",
                    "ka",
                ]
            )

            prev = 0.0
            for tgt in targets:
                # 从上一档（或 0）斜坡到本档；用 run_traj(from→to)
                rpm_from = prev if prev > 0 else min(1000.0, tgt)
                if tgt <= 1000:
                    rpm_from = 0.0
                    # 从停转起步：先 START 到 tgt
                    name = f"T{tgt:.0f}"
                    print(f"\n>>> 空载测点 {tgt:.0f} RPM（自停转）…")
                    # 特殊：from≈0 时 run_traj 的起点等待可能失败；用 from=tgt*0.3 或直接
                    res = run_traj(
                        ser,
                        name=name,
                        up=args.up,
                        down=args.down,
                        rpm_from=max(300.0, tgt * 0.3) if tgt >= 1000 else 300.0,
                        rpm_to=tgt,
                        settle_s=args.settle,
                        rpmmax=args.rpmmax,
                    )
                    # 若 300 起点对 1000 不合适，上面仍可；对 1000 from=300 OK
                else:
                    name = f"T{tgt:.0f}"
                    print(f"\n>>> 空载测点 {prev:.0f}→{tgt:.0f} RPM …")
                    res = run_traj(
                        ser,
                        name=name,
                        up=args.up,
                        down=args.down,
                        rpm_from=prev,
                        rpm_to=tgt,
                        settle_s=args.settle,
                        rpmmax=args.rpmmax,
                    )

                row = {
                    "target": tgt,
                    "t_in": res.t_in_band_s,
                    "mean": res.mean_tail,
                    "peak": res.peak,
                    "pulse": res.pulse_end,
                    "verdict": res.verdict,
                    "note": res.note,
                }
                rows.append(row)
                w.writerow(
                    [
                        "noload",
                        esc_hz,
                        f"{tgt:.0f}",
                        f"{res.t_in_band_s:.3f}" if res.t_in_band_s is not None else "",
                        f"{res.mean_tail:.2f}",
                        f"{res.peak:.2f}",
                        res.pulse_end,
                        res.verdict,
                        res.note,
                        f"{args.up:.0f}",
                        f"{args.down:.0f}",
                        f"{args.ka:.4f}",
                    ]
                )
                fp.flush()
                print(
                    f"    → {res.verdict}  t_band={res.t_in_band_s}  "
                    f"mean={res.mean_tail:.0f}  {res.note}"
                )
                if res.verdict == "ABORT":
                    break
                if res.verdict.startswith("FAIL"):
                    # 继续更高点仍可测，但记录；若完全转不起来则停
                    if res.mean_tail < MIN_SPIN:
                        print("  几乎无转速，停止后续更高测点")
                        break
                prev = tgt
                # 点间停稳，避免连续爬升过热观感；仍保持空载阶梯清晰
                safe_stop(ser)
                time.sleep(1.5)

        set_ka(ser, args.ka)  # 保持推荐 Ka
        safe_stop(ser)
        send(ser, "FREQ 50")

        rep_path = csv_path.with_suffix(".report.md")
        text = write_report(
            rep_path,
            port=args.port,
            info=info,
            up=args.up,
            down=args.down,
            ka=args.ka,
            settle=args.settle,
            rpmmax=args.rpmmax,
            esc_pwm_hz=esc_hz,
            rows=rows,
        )
        # 同步一份到项目根备忘旁，便于查找
        copy = (
            Path(__file__).resolve().parents[1]
            / f"空载多点测试报告_F{esc_hz}_{ts}.md"
        )
        copy.write_text(text, encoding="utf-8")
        print("\n" + text)
        print(f"报告: {rep_path}")
        print(f"副本: {copy}")

        if any(r["verdict"] == "ABORT" for r in rows):
            return 3
        if any(r["verdict"].startswith("FAIL") for r in rows):
            return 4
        return 0
    except KeyboardInterrupt:
        safe_stop(ser)
        try:
            send(ser, "FREQ 50")
        except Exception:  # noqa: BLE001
            pass
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"异常: {exc}")
        try:
            send(ser, "ESTOP")
            safe_stop(ser)
            send(ser, "FREQ 50")
        except Exception:  # noqa: BLE001
            pass
        return 5
    finally:
        try:
            safe_stop(ser)
            send(ser, "FREQ 50")
        except Exception:  # noqa: BLE001
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
