#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FREQ=400 Hz 分步验证（疑问→小实验→最终实验）。

依据: ../FREQ400_分步实测方案_2026-07-19.md

用法:
  python auto_freq400_staged.py --port COM10 --phase V1
  python auto_freq400_staged.py --port COM10 --upto V3
  python auto_freq400_staged.py --port COM10 --phase F
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

from auto_freq_test import MIN_SPIN_RPM, run_one_freq, wait_stopped
from auto_learn_test import BAUD, DEFAULT_PORT, LOG_DIR, open_port, send, wait_banner
from auto_s1_accel_decel_test import drain, keep_alive, run_traj, safe_stop, set_soft
from com_port_guard import acquire

TARGET_HZ = 400
PHASES = ("V1", "V2", "V3", "V4", "V5", "V6", "F")
STATE_PATH = LOG_DIR / "freq400_staged_state.json"


def load_state() -> dict:
    if STATE_PATH.exists():
        try:
            return json.loads(STATE_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            pass
    return {"passed": [], "notes": []}


def save_state(state: dict) -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    STATE_PATH.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


def mark_pass(state: dict, phase: str, note: str) -> None:
    if phase not in state["passed"]:
        state["passed"].append(phase)
    state["notes"].append(f"{phase}: PASS — {note}")
    save_state(state)


def require_prior(state: dict, need: list[str]) -> str | None:
    missing = [p for p in need if p not in state["passed"]]
    if missing:
        return f"缺少前置 PASS: {missing}（可先 --upto 做到该步，或 --force 强行）"
    return None


def set_freq400(ser) -> bool:
    send(ser, "PROTO PWM")
    send(ser, f"FREQ {TARGET_HZ}")
    time.sleep(0.2)
    metas, _ = drain(ser, 0.3)
    ok = any(f"FREQ {TARGET_HZ}" in m or f"ACK FREQ {TARGET_HZ}" in m for m in metas)
    for m in metas:
        if "FREQ" in m or "ERR" in m:
            print(" ", m[:100])
    return ok or True  # 部分固件只回 ok=1


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


def phase_v1(ser) -> tuple[bool, str]:
    print("\n======== V1 回答 Q1：400 Hz 开环执行器 ========")
    set_freq400(ser)
    # run_one_freq 结束会 FREQ 50；我们要保持逻辑清晰，再设回
    res = run_one_freq(ser, TARGET_HZ, 1100, 3.0)
    set_freq400(ser)
    restore_safe(ser)
    ok = res.verdict == "PASS"
    note = f"mean={res.mean_rpm:.1f} peak={res.peak_rpm:.1f} {res.note}"
    print(f"  → {res.verdict} {note}")
    return ok, note


def phase_v2(ser) -> tuple[bool, str]:
    print("\n======== V2 回答 Q2：同脉宽 50 vs 400 RPM ========")
    wait_stopped(ser)
    r50 = run_one_freq(ser, 50, 1100, 3.0)
    wait_stopped(ser)
    r400 = run_one_freq(ser, TARGET_HZ, 1100, 3.0)
    restore_safe(ser)
    if r50.verdict != "PASS" or r400.verdict != "PASS":
        note = f"50={r50.verdict}/{r50.mean_rpm:.1f} 400={r400.verdict}/{r400.mean_rpm:.1f}"
        print(f"  → FAIL {note}")
        return False, note
    tol = max(50.0, abs(r50.mean_rpm) * 0.10)
    delta = abs(r400.mean_rpm - r50.mean_rpm)
    if delta <= tol:
        note = f"mean50={r50.mean_rpm:.1f} mean400={r400.mean_rpm:.1f} Δ={delta:.1f}≤{tol:.1f} → map可复用"
        print(f"  → PASS {note}")
        return True, note
    note = (
        f"mean50={r50.mean_rpm:.1f} mean400={r400.mean_rpm:.1f} Δ={delta:.1f}>{tol:.1f} "
        f"→ MARGINAL（可继续但盯稳态）"
    )
    print(f"  → MARGINAL {note}")
    # MARGINAL 仍放行后续，但标记
    return True, note


def _closed_prep(ser, up: float, down: float, ka: float, rpmmax: float) -> None:
    set_freq400(ser)
    send(ser, "PROFILE noload")
    send(ser, "MODE CLOSED")
    send(ser, "GAIN ON")
    send(ser, "ADG OFF")
    send(ser, f"RPMMAX {rpmmax:.0f}")
    send(ser, f"KA {ka:.4f}")
    set_soft(ser, up, down)
    keep_alive(ser)
    drain(ser, 0.2)


def phase_v3(ser, settle: float, up: float, down: float, ka: float, rpmmax: float) -> tuple[bool, str]:
    print("\n======== V3 回答 Q3：闭环 1000 @400 ========")
    _closed_prep(ser, up, down, ka, rpmmax)
    tr = run_traj(
        ser,
        name="V3_1000",
        up=up,
        down=down,
        rpm_from=300.0,
        rpm_to=1000.0,
        settle_s=settle,
        rpmmax=rpmmax,
    )
    restore_safe(ser)
    t_in = tr.t_in_band_s if tr.t_in_band_s is not None else -1.0
    note = f"{tr.verdict} t_band={t_in:.2f} mean={tr.mean_tail:.1f} pulse={tr.pulse_end}"
    print(f"  → {note}")
    return tr.verdict == "PASS", note


def phase_ladder(
    ser,
    targets: list[float],
    settle: float,
    up: float,
    down: float,
    ka: float,
    rpmmax: float,
    title: str,
) -> tuple[bool, str]:
    print(f"\n======== {title} ========")
    _closed_prep(ser, up, down, ka, rpmmax)
    notes: list[str] = []
    all_ok = True
    prev = 0.0
    for tgt in targets:
        if tgt <= 1000:
            rpm_from = 300.0
        else:
            rpm_from = prev if prev > 0 else tgt - 1000.0
        print(f"  >>> {rpm_from:.0f}→{tgt:.0f} …")
        tr = run_traj(
            ser,
            name=f"T{tgt:.0f}",
            up=up,
            down=down,
            rpm_from=rpm_from,
            rpm_to=tgt,
            settle_s=settle,
            rpmmax=rpmmax,
        )
        t_in = tr.t_in_band_s if tr.t_in_band_s is not None else -1.0
        line = f"{tgt:.0f}:{tr.verdict}/t={t_in:.2f}/mean={tr.mean_tail:.0f}/p={tr.pulse_end}"
        notes.append(line)
        print(f"    → {line}")
        if tr.verdict != "PASS":
            all_ok = False
            if tr.mean_tail < MIN_SPIN_RPM:
                break
        prev = tgt
        safe_stop(ser)
        set_freq400(ser)
        time.sleep(0.8)
    restore_safe(ser)
    return all_ok, "; ".join(notes)


def phase_v4(ser, settle: float, up: float, down: float, ka: float, rpmmax: float) -> tuple[bool, str]:
    return phase_ladder(
        ser, [2000.0, 3000.0], settle, up, down, ka, rpmmax, "V4 回答 Q4：中点 2000/3000"
    )


def phase_v5(ser, settle: float, up: float, down: float, ka: float, rpmmax: float) -> tuple[bool, str]:
    return phase_ladder(
        ser,
        [4000.0, 5000.0, 6000.0],
        settle,
        up,
        down,
        ka,
        rpmmax,
        "V5 回答 Q4：高点 4000/5000/6000",
    )


def phase_v6(ser, settle: float, up: float, down: float, ka: float, rpmmax: float) -> tuple[bool, str]:
    print("\n======== V6 回答 Q5：加减速 A1/A2 @400 ========")
    _closed_prep(ser, up, down, ka, rpmmax)
    jobs = [
        ("A1", up, up, 1000.0, 2000.0),
        ("A2", up, down, 3000.0, 2000.0),
    ]
    notes: list[str] = []
    all_ok = True
    for name, u, d, f, t in jobs:
        tr = run_traj(
            ser, name=name, up=u, down=d, rpm_from=f, rpm_to=t, settle_s=settle, rpmmax=rpmmax
        )
        t_in = tr.t_in_band_s if tr.t_in_band_s is not None else -1.0
        line = f"{name}:{tr.verdict}/t={t_in:.2f}/over={tr.overshoot_pct:.1f}%"
        notes.append(line)
        print(f"  → {line}")
        if tr.verdict != "PASS":
            all_ok = False
        safe_stop(ser)
        set_freq400(ser)
        time.sleep(0.5)
    restore_safe(ser)
    return all_ok, "; ".join(notes)


def phase_f(ser, settle: float, up: float, down: float, ka: float, rpmmax: float) -> tuple[bool, str]:
    print("\n======== F 最终实验：1000…6000 @400 ========")
    ok, note = phase_ladder(
        ser,
        [1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0],
        settle,
        up,
        down,
        ka,
        rpmmax,
        "F 全量程阶梯",
    )
    return ok, note


def main() -> int:
    ap = argparse.ArgumentParser(description="FREQ400 疑问验证分步实验")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--phase", default="", help="单步: V1..V6 或 F")
    ap.add_argument("--upto", default="", help="连续做到该步（含）")
    ap.add_argument("--force", action="store_true", help="忽略前置 PASS 检查")
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--up", type=float, default=800.0)
    ap.add_argument("--down", type=float, default=300.0)
    ap.add_argument("--ka", type=float, default=0.05)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    ap.add_argument("--reset-state", action="store_true", help="清空分步 PASS 状态")
    args = ap.parse_args()

    if args.reset_state and STATE_PATH.exists():
        STATE_PATH.unlink()
        print("已清空分步状态")

    if args.phase:
        todo = [args.phase.upper()]
    elif args.upto:
        end = args.upto.upper()
        if end not in PHASES:
            print(f"--upto 须为 {PHASES}")
            return 1
        todo = list(PHASES[: PHASES.index(end) + 1])
    else:
        print("请指定 --phase V1 或 --upto V3")
        return 1

    for p in todo:
        if p not in PHASES:
            print(f"未知 phase {p}")
            return 1

    state = load_state()
    print(f"打开 {args.port} @ {BAUD} … 已通过: {state.get('passed')}")
    try:
        acquire("auto_freq400_staged")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        return 1

    log_lines = [
        f"# FREQ400 分步验证 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"phases={todo}",
        f"fixed: UP/DOWN={args.up}/{args.down} Ka={args.ka} ADG=OFF",
        "",
    ]
    try:
        info = wait_banner(ser)
        print("板子:", info)
        if float(info.get("ctrl_hz") or 0) < 200:
            print("ctrl_hz 过低")
            return 2

        priors = {
            "V1": [],
            "V2": ["V1"],
            "V3": ["V1", "V2"],
            "V4": ["V1", "V2", "V3"],
            "V5": ["V1", "V2", "V3", "V4"],
            "V6": ["V1", "V2", "V3"],  # 动态可不强依赖 V5
            "F": ["V1", "V2", "V3", "V4", "V5"],
        }

        runners = {
            "V1": lambda: phase_v1(ser),
            "V2": lambda: phase_v2(ser),
            "V3": lambda: phase_v3(ser, args.settle, args.up, args.down, args.ka, args.rpmmax),
            "V4": lambda: phase_v4(ser, args.settle, args.up, args.down, args.ka, args.rpmmax),
            "V5": lambda: phase_v5(ser, args.settle, args.up, args.down, args.ka, args.rpmmax),
            "V6": lambda: phase_v6(ser, args.settle, args.up, args.down, args.ka, args.rpmmax),
            "F": lambda: phase_f(ser, args.settle, args.up, args.down, args.ka, args.rpmmax),
        }

        for phase in todo:
            if not args.force:
                err = require_prior(state, priors[phase])
                if err:
                    print(err)
                    log_lines.append(f"{phase}: BLOCKED {err}")
                    return 3
            ok, note = runners[phase]()
            log_lines.append(f"## {phase}\n{note}\n")
            if ok:
                mark_pass(state, phase, note)
            else:
                restore_safe(ser)
                log_lines.append(f"**{phase} FAIL — 停止更高步**\n")
                print(f"{phase} FAIL — 按方案停止更高步")
                _write_log(log_lines)
                return 4

        restore_safe(ser)
        log_lines.append("\n**本段全部 PASS**\n")
        _write_log(log_lines)
        print("\n本段全部 PASS。状态:", state["passed"])
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


def _write_log(lines: list[str]) -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = LOG_DIR / f"freq400_staged_{ts}.report.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    copy = Path(__file__).resolve().parents[1] / f"FREQ400分步记录_{ts}.md"
    copy.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"记录: {path}")
    print(f"副本: {copy}")


if __name__ == "__main__":
    raise SystemExit(main())
