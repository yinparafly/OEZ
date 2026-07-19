#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FREQ 50 Hz vs 400 Hz 闭环控制效果 A/B 对比。

依据: ../FREQ50vs400_闭环对比方案_2026-07-19.md

用法:
  python auto_freq_ab_compare.py --port COM10
  python auto_freq_ab_compare.py --port COM10 --order 400,50
  python auto_freq_ab_compare.py --port COM10 --skip-curve
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
TBAND_EPS = 0.15  # s：小于此视为噪声
LADDER = (1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0)
CURVE = (1000.0, 2000.0, 4000.0, 2000.0, 1000.0)


def set_ka(ser, ka: float) -> None:
    send(ser, f"KA {ka:.4f}")
    keep_alive(ser)
    drain(ser, 0.15)


def set_freq(ser, hz: int) -> None:
    send(ser, "PROTO PWM")
    send(ser, f"FREQ {hz}")
    time.sleep(0.2)
    metas, _ = drain(ser, 0.3)
    for m in metas:
        if "FREQ" in m or "ERR" in m:
            print(" ", m[:100])


def apply_fixed(ser, *, up: float, down: float, ka: float, rpmmax: float) -> None:
    send(ser, "STOP")
    send(ser, "PROFILE noload")
    send(ser, "MODE CLOSED")
    send(ser, "GAIN ON")
    send(ser, "ADG OFF")
    send(ser, f"RPMMAX {rpmmax:.0f}")
    set_ka(ser, ka)
    set_soft(ser, up, down)
    drain(ser, 0.25)


def restore_safe(ser) -> None:
    try:
        send(ser, "PWM 1000")
        send(ser, "MEASURE ABORT")
        send(ser, "STOP")
        send(ser, "FREQ 50")
        send(ser, "ADG OFF")
        time.sleep(0.15)
        drain(ser, 0.2)
    except Exception:  # noqa: BLE001
        pass


def run_ladder_side(
    ser,
    *,
    freq: int,
    settle: float,
    up: float,
    down: float,
    ka: float,
    rpmmax: float,
) -> list[dict]:
    print(f"\n======== C1 阶梯 @ FREQ={freq} ========")
    set_freq(ser, freq)
    apply_fixed(ser, up=up, down=down, ka=ka, rpmmax=rpmmax)
    rows: list[dict] = []
    prev = 0.0
    for tgt in LADDER:
        if tgt <= 1000:
            print(f"  >>> {tgt:.0f}（自低点）…")
            res = run_traj(
                ser,
                name=f"L{freq}_{tgt:.0f}",
                up=up,
                down=down,
                rpm_from=max(300.0, tgt * 0.3),
                rpm_to=tgt,
                settle_s=settle,
                rpmmax=rpmmax,
            )
        else:
            print(f"  >>> {prev:.0f}→{tgt:.0f} …")
            res = run_traj(
                ser,
                name=f"L{freq}_{tgt:.0f}",
                up=up,
                down=down,
                rpm_from=prev,
                rpm_to=tgt,
                settle_s=settle,
                rpmmax=rpmmax,
            )
        err = (
            abs(res.mean_tail - tgt) / max(tgt, 1.0) * 100.0
            if res.mean_tail > MIN_SPIN
            else 999.0
        )
        t_in = res.t_in_band_s
        row = {
            "block": "C1",
            "freq": freq,
            "name": f"{tgt:.0f}",
            "from": prev if prev > 0 else 300.0,
            "to": tgt,
            "t_band": t_in,
            "mean": res.mean_tail,
            "peak": res.peak,
            "pulse": res.pulse_end,
            "over_pct": res.overshoot_pct,
            "err_pct": err,
            "verdict": res.verdict,
        }
        rows.append(row)
        tb = f"{t_in:.2f}" if t_in is not None else "—"
        print(f"    → {res.verdict} t={tb} mean={res.mean_tail:.0f} err={err:.1f}%")
        if res.verdict == "ABORT" or (
            res.verdict.startswith("FAIL") and res.mean_tail < MIN_SPIN
        ):
            break
        prev = tgt
        safe_stop(ser)
        time.sleep(1.2)
    return rows


def run_dyn_side(
    ser,
    *,
    freq: int,
    settle: float,
    up: float,
    down: float,
    ka: float,
    rpmmax: float,
) -> list[dict]:
    print(f"\n======== C2 加减速 @ FREQ={freq} ========")
    set_freq(ser, freq)
    apply_fixed(ser, up=up, down=down, ka=ka, rpmmax=rpmmax)
    rows: list[dict] = []
    cases = (
        ("A1", up, down, 1000.0, 2000.0),
        ("A2", up, down, 3000.0, 2000.0),
    )
    for name, u, d, f, t in cases:
        print(f"  >>> {name} {f:.0f}→{t:.0f} …")
        res = run_traj(
            ser,
            name=f"{name}_{freq}",
            up=u,
            down=d,
            rpm_from=f,
            rpm_to=t,
            settle_s=settle,
            rpmmax=rpmmax,
        )
        t_in = res.t_in_band_s
        row = {
            "block": "C2",
            "freq": freq,
            "name": name,
            "from": f,
            "to": t,
            "t_band": t_in,
            "mean": res.mean_tail,
            "peak": res.peak,
            "pulse": res.pulse_end,
            "over_pct": res.overshoot_pct,
            "err_pct": abs(res.mean_tail - t) / max(t, 1.0) * 100.0,
            "verdict": res.verdict,
        }
        rows.append(row)
        tb = f"{t_in:.2f}" if t_in is not None else "—"
        print(f"    → {res.verdict} t={tb} over={res.overshoot_pct:.1f}%")
        safe_stop(ser)
        time.sleep(1.2)
        if res.verdict == "ABORT":
            break
    return rows


def run_curve_side(
    ser,
    *,
    freq: int,
    settle: float,
    up: float,
    down: float,
    ka: float,
    rpmmax: float,
) -> list[dict]:
    print(f"\n======== C3 多段曲线 @ FREQ={freq} ========")
    set_freq(ser, freq)
    apply_fixed(ser, up=up, down=down, ka=ka, rpmmax=rpmmax)
    rows: list[dict] = []
    prev = CURVE[0]
    # 先稳住起点
    print(f"  >>> 起点 {prev:.0f} …")
    res0 = run_traj(
        ser,
        name=f"C{freq}_base",
        up=up,
        down=down,
        rpm_from=max(300.0, prev * 0.3),
        rpm_to=prev,
        settle_s=min(settle, 6.0),
        rpmmax=rpmmax,
    )
    if res0.verdict.startswith("FAIL") or res0.verdict == "ABORT":
        rows.append(
            {
                "block": "C3",
                "freq": freq,
                "name": f"base→{prev:.0f}",
                "from": 300.0,
                "to": prev,
                "t_band": res0.t_in_band_s,
                "mean": res0.mean_tail,
                "peak": res0.peak,
                "pulse": res0.pulse_end,
                "over_pct": res0.overshoot_pct,
                "err_pct": 999.0,
                "verdict": res0.verdict,
            }
        )
        return rows

    for tgt in CURVE[1:]:
        print(f"  >>> 曲线 {prev:.0f}→{tgt:.0f}（连续）…")
        res = run_traj(
            ser,
            name=f"C{freq}_{prev:.0f}_{tgt:.0f}",
            up=up,
            down=down,
            rpm_from=prev,
            rpm_to=tgt,
            settle_s=settle,
            rpmmax=rpmmax,
        )
        err = (
            abs(res.mean_tail - tgt) / max(tgt, 1.0) * 100.0
            if res.mean_tail > MIN_SPIN
            else 999.0
        )
        row = {
            "block": "C3",
            "freq": freq,
            "name": f"{prev:.0f}→{tgt:.0f}",
            "from": prev,
            "to": tgt,
            "t_band": res.t_in_band_s,
            "mean": res.mean_tail,
            "peak": res.peak,
            "pulse": res.pulse_end,
            "over_pct": res.overshoot_pct,
            "err_pct": err,
            "verdict": res.verdict,
        }
        rows.append(row)
        tb = f"{res.t_in_band_s:.2f}" if res.t_in_band_s is not None else "—"
        print(f"    → {res.verdict} t={tb} over={res.overshoot_pct:.1f}% err={err:.1f}%")
        if res.verdict == "ABORT" or (
            res.verdict.startswith("FAIL") and res.mean_tail < MIN_SPIN
        ):
            break
        prev = tgt
        # 连续曲线：不停转，仅短歇
        keep_alive(ser)
        time.sleep(0.4)

    safe_stop(ser)
    time.sleep(1.0)
    return rows


def _pair_map(rows: list[dict], block: str) -> dict[tuple[int, str], dict]:
    return {(r["freq"], r["name"]): r for r in rows if r["block"] == block}


def judge(all_rows: list[dict], order: list[int]) -> list[str]:
    """按方案第 4 节给出结案标签。"""
    lines: list[str] = []
    if len(order) < 2:
        return ["数据不足，无法对比"]

    f_lo, f_hi = 50, 400
    # C1
    c1 = _pair_map(all_rows, "C1")
    faster = slower = noise = 0
    err_worse = 0
    for name in [f"{x:.0f}" for x in LADDER]:
        a = c1.get((f_lo, name))
        b = c1.get((f_hi, name))
        if not a or not b:
            continue
        if a["t_band"] is None or b["t_band"] is None:
            continue
        d = b["t_band"] - a["t_band"]
        if d <= -TBAND_EPS:
            faster += 1
        elif d >= TBAND_EPS:
            slower += 1
        else:
            noise += 1
        if b["err_pct"] - a["err_pct"] > 1.0:
            err_worse += 1

    lines.append(
        f"C1 阶梯: 400更快点数={faster}, 更慢={slower}, 噪声内={noise}, "
        f"稳态误差明显变差点数={err_worse}"
    )

    # C2
    c2 = _pair_map(all_rows, "C2")
    dyn_notes = []
    dyn_better = dyn_worse = 0
    for name in ("A1", "A2"):
        a = c2.get((f_lo, name))
        b = c2.get((f_hi, name))
        if not a or not b or a["t_band"] is None or b["t_band"] is None:
            dyn_notes.append(f"{name}: 缺数据")
            continue
        dt = b["t_band"] - a["t_band"]
        do = b["over_pct"] - a["over_pct"]
        if dt <= -TBAND_EPS and do <= 1.0:
            dyn_better += 1
            tag = "400更快"
        elif dt >= TBAND_EPS or do > 2.0:
            dyn_worse += 1
            tag = "400更差/超调增"
        else:
            tag = "相当"
        dyn_notes.append(
            f"{name}: t50={a['t_band']:.2f} t400={b['t_band']:.2f} "
            f"Δt={dt:+.2f}s  over50={a['over_pct']:.1f}% over400={b['over_pct']:.1f}% → {tag}"
        )
    lines.extend(dyn_notes)

    # C3
    c3_50 = [r for r in all_rows if r["block"] == "C3" and r["freq"] == f_lo]
    c3_400 = [r for r in all_rows if r["block"] == "C3" and r["freq"] == f_hi]
    if c3_50 and c3_400:
        def _mean_t(rs):
            vals = [r["t_band"] for r in rs if r["t_band"] is not None]
            return sum(vals) / len(vals) if vals else None

        def _mean_err(rs):
            vals = [r["err_pct"] for r in rs if r["err_pct"] < 900]
            return sum(vals) / len(vals) if vals else None

        mt50, mt400 = _mean_t(c3_50), _mean_t(c3_400)
        me50, me400 = _mean_err(c3_50), _mean_err(c3_400)
        if mt50 is not None and mt400 is not None:
            lines.append(
                f"C3 曲线: 平均t_band 50={mt50:.2f}s 400={mt400:.2f}s Δ={mt400 - mt50:+.2f}s; "
                f"平均末段误差 50={me50:.2f}% 400={me400:.2f}%"
                if me50 is not None and me400 is not None
                else f"C3 曲线: 平均t_band 50={mt50:.2f}s 400={mt400:.2f}s"
            )
        fail50 = any(r["verdict"].startswith("FAIL") or r["verdict"] == "ABORT" for r in c3_50)
        fail400 = any(r["verdict"].startswith("FAIL") or r["verdict"] == "ABORT" for r in c3_400)
        if fail50 or fail400:
            lines.append(f"C3 FAIL/ABORT: 50={'Y' if fail50 else 'N'} 400={'Y' if fail400 else 'N'}")

    # 总标签
    if faster >= 4 and err_worse == 0 and dyn_worse == 0 and (dyn_better >= 1 or faster >= 5):
        label = "**400 明显更好**（跟踪/入带）"
    elif slower >= 4 or dyn_worse >= 2:
        label = "**400 更差**（本轮指标）"
    elif faster >= 1 and slower == 0 and dyn_worse == 0:
        label = "**400 略好 / 倾向更好**（未达「明显」阈值）"
    else:
        label = "**相当 / 噪声内**（跟踪未证明系统性更好；400 仍可因脉宽余量作工作默认）"

    lines.append("")
    lines.append(f"### 结案：{label}")
    lines.append(
        f"判据阈值：|Δt_band|≥{TBAND_EPS:.2f}s 视为有差异；"
        "速度环仍为 ctrl_hz=250，本对比只反映 PWM 载波差异。"
    )
    return lines


def write_report(
    path: Path,
    *,
    port: str,
    info: dict,
    order: list[int],
    up: float,
    down: float,
    ka: float,
    settle: float,
    curve_settle: float,
    rows: list[dict],
) -> str:
    lines: list[str] = []
    lines.append("# FREQ 50 Hz vs 400 Hz 闭环对比报告")
    lines.append("")
    lines.append("**同会话 A/B；仅改 FREQ；空载 noload**")
    lines.append("")
    lines.append("## 1. 条件")
    lines.append("")
    lines.append("| 项 | 值 |")
    lines.append("|----|----|")
    lines.append(f"| 时间 | {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} |")
    lines.append(f"| 串口 | {port} @ {BAUD} |")
    lines.append(f"| ctrl_hz | {info.get('ctrl_hz', '?')} |")
    lines.append(f"| 顺序 | {' → '.join(str(x) for x in order)} Hz |")
    lines.append(f"| SOFT UP/DOWN | {up:.0f} / {down:.0f} |")
    lines.append(f"| Ka | {ka:.4f} |")
    lines.append("| ADG | OFF |")
    lines.append(f"| C1/C2 settle | {settle:.1f} s |")
    lines.append(f"| C3 settle | {curve_settle:.1f} s |")
    lines.append("")

    def emit_block(title: str, block: str) -> None:
        lines.append(f"## {title}")
        lines.append("")
        names = []
        for r in rows:
            if r["block"] == block and r["name"] not in names:
                names.append(r["name"])
        if not names:
            lines.append("（未测）")
            lines.append("")
            return
        lines.append(
            "| 项目 | FREQ50 t_band | FREQ400 t_band | Δt(400−50) | "
            "err50% | err400% | over50% | over400% | v50 | v400 |"
        )
        lines.append(
            "|------|---------------|----------------|------------|"
            "-------|---------|---------|----------|-----|------|"
        )
        pm = _pair_map(rows, block)
        for name in names:
            a = pm.get((50, name))
            b = pm.get((400, name))

            def fmt_t(r):
                if not r or r["t_band"] is None:
                    return "—"
                return f"{r['t_band']:.2f}"

            def fmt_n(r, key, nd=1):
                if not r or r.get(key) is None:
                    return "—"
                return f"{r[key]:.{nd}f}"

            dt = "—"
            if a and b and a["t_band"] is not None and b["t_band"] is not None:
                dt = f"{b['t_band'] - a['t_band']:+.2f}"
            lines.append(
                f"| {name} | {fmt_t(a)} | {fmt_t(b)} | {dt} | "
                f"{fmt_n(a, 'err_pct')} | {fmt_n(b, 'err_pct')} | "
                f"{fmt_n(a, 'over_pct')} | {fmt_n(b, 'over_pct')} | "
                f"{a['verdict'] if a else '—'} | {b['verdict'] if b else '—'} |"
            )
        lines.append("")

    emit_block("2. C1 阶梯对比", "C1")
    emit_block("3. C2 加减速对比", "C2")
    emit_block("4. C3 多段曲线对比", "C3")

    lines.append("## 5. 自动判读")
    lines.append("")
    for s in judge(rows, order):
        if not s:
            lines.append("")
        elif s.startswith("###"):
            lines.append(s)
        else:
            lines.append(f"- {s}")
    lines.append("")
    lines.append("原始 CSV 同名 `.csv`。")
    text = "\n".join(lines) + "\n"
    path.write_text(text, encoding="utf-8")
    return text


def main() -> int:
    ap = argparse.ArgumentParser(description="FREQ 50 vs 400 闭环 A/B 对比")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--order", default="50,400", help="逗号分隔，如 50,400 或 400,50")
    ap.add_argument("--up", type=float, default=800.0)
    ap.add_argument("--down", type=float, default=300.0)
    ap.add_argument("--ka", type=float, default=0.05)
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--curve-settle", type=float, default=6.0)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    ap.add_argument("--skip-curve", action="store_true")
    ap.add_argument("--skip-dyn", action="store_true")
    args = ap.parse_args()

    order = [int(x.strip()) for x in args.order.split(",") if x.strip()]
    for hz in order:
        if hz not in (50, 400):
            print(f"本脚本仅对比 50/400，收到 {hz}")
            return 2
    if sorted(order) != [50, 400]:
        print("order 须恰好包含 50 与 400 各一次")
        return 2

    acquire("auto_freq_ab_compare")
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        return 1

    all_rows: list[dict] = []
    try:
        info = wait_banner(ser, timeout_s=4.0)
        print("板子:", info)
        if float(info.get("ctrl_hz") or 0) < 200:
            print("ctrl_hz 过低，中止")
            return 2

        print(
            f"A/B 对比顺序: {' → '.join(str(x) for x in order)} Hz\n"
            f"  固定 UP/DOWN={args.up:.0f}/{args.down:.0f} Ka={args.ka} "
            f"settle={args.settle}s curve_settle={args.curve_settle}s"
        )

        for hz in order:
            all_rows.extend(
                run_ladder_side(
                    ser,
                    freq=hz,
                    settle=args.settle,
                    up=args.up,
                    down=args.down,
                    ka=args.ka,
                    rpmmax=args.rpmmax,
                )
            )
            if not args.skip_dyn:
                all_rows.extend(
                    run_dyn_side(
                        ser,
                        freq=hz,
                        settle=args.settle,
                        up=args.up,
                        down=args.down,
                        ka=args.ka,
                        rpmmax=args.rpmmax,
                    )
                )
            if not args.skip_curve:
                all_rows.extend(
                    run_curve_side(
                        ser,
                        freq=hz,
                        settle=args.curve_settle,
                        up=args.up,
                        down=args.down,
                        ka=args.ka,
                        rpmmax=args.rpmmax,
                    )
                )

        restore_safe(ser)

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = LOG_DIR / f"freq_ab_compare_{ts}.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(
                [
                    "block",
                    "freq",
                    "name",
                    "from",
                    "to",
                    "t_band",
                    "mean",
                    "peak",
                    "pulse",
                    "over_pct",
                    "err_pct",
                    "verdict",
                ]
            )
            for r in all_rows:
                w.writerow(
                    [
                        r["block"],
                        r["freq"],
                        r["name"],
                        r["from"],
                        r["to"],
                        f"{r['t_band']:.3f}" if r["t_band"] is not None else "",
                        f"{r['mean']:.2f}",
                        f"{r['peak']:.2f}",
                        r["pulse"],
                        f"{r['over_pct']:.2f}",
                        f"{r['err_pct']:.2f}",
                        r["verdict"],
                    ]
                )

        rep_path = csv_path.with_suffix(".report.md")
        text = write_report(
            rep_path,
            port=args.port,
            info=info,
            order=order,
            up=args.up,
            down=args.down,
            ka=args.ka,
            settle=args.settle,
            curve_settle=args.curve_settle,
            rows=all_rows,
        )
        copy = Path(__file__).resolve().parents[1] / f"FREQ50vs400对比报告_{ts}.md"
        copy.write_text(text, encoding="utf-8")
        print("\n" + text)
        print(f"报告: {rep_path}")
        print(f"副本: {copy}")
        print(f"CSV:  {csv_path}")

        if any(r["verdict"] == "ABORT" for r in all_rows):
            return 3
        if any(r["verdict"].startswith("FAIL") for r in all_rows):
            return 4
        return 0
    except KeyboardInterrupt:
        restore_safe(ser)
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"异常: {exc}")
        restore_safe(ser)
        return 5
    finally:
        restore_safe(ser)
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
