#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""6000RPM → phase-0 stop strategy comparison (COM10, float, 2k/400).

Strategies:
  A timed soft-down: HOLD PHASE stop_ms sweep 2000→800
  B stepped RPM: 6000→4000→2000 → HOLD PHASE
  C aggressive soft+crawl: SOFT DOWN to low RPM then HOLD PHASE stop_ms=0
  D lead-heavy: larger PHASE LEAD + shorter stop_ms
  E baseline early-brake profile (stock lead + mid stop_ms)

Usage:
  set PYTHONIOENCODING=utf-8
  python hold_phase_strategy_cmp.py --port COM10 --rpm 6000
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("PYTHONIOENCODING", "utf-8")

from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire, release
from motor_spin_detect import detect_spin

PORT = "COM10"
LOGDIR = Path(__file__).resolve().parent / "logs"
GEAR_NOTE = "4661/168"
PASS_TOL = 10.0
TOL_GOOD = 8.0

_out: list[str] = []


def log(s: str) -> None:
    print(s, flush=True)
    _out.append(s)


def drain(ser, budget: float = 0.15) -> list[str]:
    lines = read_lines(ser, budget)
    for ln in lines:
        if ln.startswith("#") or (ln and not ln[0:1].isdigit()):
            log(ln)
    return lines


def wait_ack(ser, pred, timeout: float = 2.0) -> list[str]:
    end = time.time() + timeout
    got: list[str] = []
    while time.time() < end:
        for ln in drain(ser, 0.12):
            got.append(ln)
            if pred(ln):
                return got
        time.sleep(0.02)
    return got


def wrap180(d: float) -> float:
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def phase_err_abs(disc: float | None, target: float = 0.0) -> float | None:
    if disc is None:
        return None
    return abs(wrap180(disc - target))


def parse_kv(line: str, key: str):
    m = re.search(rf"{re.escape(key)}=([\-\d.]+)", line)
    return float(m.group(1)) if m else None


def parse_telem(line: str):
    if not line or line.startswith("#"):
        return None
    p = line.split(",")
    if len(p) < 17:
        return None
    try:
        return {
            "rpm": float(p[4]),
            "pulse": int(float(p[9])),
            "target": float(p[10]),
            "run": int(float(p[16])),
        }
    except (ValueError, IndexError):
        return None


def safe_stop(ser) -> None:
    for c in ("ESTOP", "STOP", "RPM 0", "PWM 1000"):
        send(ser, c)
        drain(ser, 0.12)


def hold_until_rpm(ser, target: float, settle_s: float, band: float = 350.0, timeout: float = 50.0) -> dict:
    t0 = time.time()
    last_ka = 0.0
    win: list[float] = []
    ok_since: float | None = None
    while time.time() - t0 < timeout:
        now = time.time()
        if now - last_ka >= 0.45:
            send(ser, f"RPM {target:.0f}")
            last_ka = now
        for ln in drain(ser, 0.08):
            t = parse_telem(ln)
            if not t:
                continue
            rpm = abs(t["rpm"])
            win.append(rpm)
            if len(win) > 40:
                win.pop(0)
            if abs(rpm - target) <= band:
                if ok_since is None:
                    ok_since = now
                elif now - ok_since >= settle_s and len(win) >= 8:
                    return {
                        "ok": True,
                        "mean": sum(win) / len(win),
                        "last": rpm,
                        "pulse": t["pulse"],
                        "elapsed": now - t0,
                    }
            else:
                ok_since = None
        time.sleep(0.02)
    return {
        "ok": False,
        "mean": sum(win) / len(win) if win else 0.0,
        "last": win[-1] if win else 0.0,
        "elapsed": time.time() - t0,
    }


def soft_down_to_rpm(ser, target: float, settle_s: float = 1.2, band: float = 200.0, timeout: float = 25.0) -> dict:
    """Closed-loop soft down using current SOFT RATE DOWN; wait near target."""
    send(ser, "MODE CLOSED")
    send(ser, "SOFT ON")
    send(ser, "TELEM ON")
    send(ser, f"RPM {target:.0f}")
    return hold_until_rpm(ser, target, settle_s, band=band, timeout=timeout)


def run_hold_phase(ser, target_deg: float, crawl: float, stop_ms: int, timeout: float = 30.0) -> dict:
    cmd = f"HOLD PHASE {target_deg:g} {crawl:g} {stop_ms}"
    log(f">> {cmd}")
    t_cmd = time.time()
    send(ser, cmd)
    done = ""
    brake = ""
    soft = ""
    overshoot = False
    end = time.time() + timeout
    while time.time() < end:
        for ln in drain(ser, 0.12):
            if "SOFTDONE" in ln:
                soft = ln
            if "BRAKE" in ln and "HOLD PHASE" in ln:
                brake = ln
                if "before_lead" in ln:
                    overshoot = True
            if "HOLD PHASE DONE" in ln or ln.startswith("# ESTOP") or "ERR HOLD PHASE" in ln:
                done = ln
                return {
                    "ok": "DONE" in ln and "ERR" not in ln,
                    "done": done,
                    "brake": brake,
                    "soft": soft,
                    "overshoot": overshoot,
                    "wall_s": time.time() - t_cmd,
                }
        time.sleep(0.02)
    return {
        "ok": False,
        "done": "TIMEOUT",
        "brake": brake,
        "soft": soft,
        "overshoot": overshoot,
        "wall_s": time.time() - t_cmd,
    }


def parse_done(line: str) -> dict:
    out: dict = {"raw": line}
    for key in (
        "disc",
        "lead",
        "hall_confirm",
        "elapsed_ms",
        "brake_to_stop_ms",
        "profile_ms",
        "soft_ms",
        "rpm0",
        "target",
    ):
        v = parse_kv(line, key)
        if v is not None:
            out[key] = v
    m = re.search(r"reason=(\S+)", line)
    if m:
        out["reason"] = m.group(1)
    return out


def set_lead(ser, deg: float, ms: float = 100.0, k: float = 0.03) -> None:
    send(ser, f"PHASE LEAD {deg:.2f} {ms:.0f} {k:.4f}")
    wait_ack(ser, lambda s: "PHASE LEAD" in s, 1.2)


def prep_closed(ser, soft_up: float = 900.0, soft_down: float = 2500.0) -> None:
    for c in (
        "MODE CLOSED",
        "SOFT ON",
        f"SOFT RATE UP {soft_up:.0f}",
        f"SOFT RATE DOWN {soft_down:.0f}",
        "RPMMAX 12000",
        "FREQ 400",
        "TELEM ON",
    ):
        send(ser, c)
        drain(ser, 0.12)


def reaccel_to(ser, rpm: float, settle_s: float = 3.5) -> dict:
    safe_stop(ser)
    time.sleep(0.45)
    send(ser, "STOP")
    drain(ser, 0.2)
    prep_closed(ser)
    send(ser, "RPM 2000")
    gate = detect_spin(ser, seconds=2.2, keepalive="RPM 2000")
    log(f"reaccel gate {gate.summary_line()}")
    send(ser, f"RPM {rpm:.0f}")
    st = hold_until_rpm(ser, rpm, settle_s, band=350.0)
    log(f"reaccel stable ok={st['ok']} mean={st.get('mean', 0):.1f} last={st.get('last', 0):.1f}")
    return st


def eval_trial(
    name: str,
    wall_s: float,
    hold: dict,
    lead_cmd: float,
    note: str = "",
    tol: float | None = None,
) -> dict:
    if tol is None:
        tol = PASS_TOL
    parsed = parse_done(hold["done"]) if "DONE" in hold.get("done", "") else {}
    disc = parsed.get("disc")
    err = phase_err_abs(disc)
    hc = parsed.get("hall_confirm")
    ok_fw = bool(hold.get("ok"))
    overshoot = bool(hold.get("overshoot"))
    pass_tol = ok_fw and (not overshoot) and err is not None and err <= tol
    row = {
        "name": name,
        "wall_s": wall_s,
        "hold_wall_s": hold.get("wall_s"),
        "ok_fw": ok_fw,
        "pass": pass_tol,
        "disc": disc,
        "err": err,
        "overshoot": overshoot,
        "hall_confirm": hc,
        "lead_cmd": lead_cmd,
        "lead_done": parsed.get("lead"),
        "elapsed_ms": parsed.get("elapsed_ms"),
        "brake_to_stop_ms": parsed.get("brake_to_stop_ms"),
        "soft_ms": parsed.get("soft_ms"),
        "profile_ms": parsed.get("profile_ms"),
        "rpm0": parsed.get("rpm0"),
        "brake": hold.get("brake", ""),
        "done": hold.get("done", ""),
        "note": note,
    }
    log(
        f"RESULT {name}: pass={pass_tol} wall={wall_s:.3f}s hold_wall={hold.get('wall_s', float('nan')):.3f}s "
        f"|err|={err if err is not None else float('nan'):.2f} disc={disc} "
        f"overshoot={overshoot} hall_confirm={hc} soft_ms={parsed.get('soft_ms')} "
        f"brake_to_stop={parsed.get('brake_to_stop_ms')}"
    )
    return row



def chinese_join_model_conclusion(rows: list[dict]) -> None:
    """Explain approach + terminal-invariant join model from measured rows."""
    log("")
    log("======== 截取同一刹车策略后半段（模型结论）========")
    log("假设: 任意转速停到相位，可共用同一「终端不变式」；高速只是更长的进场/降速段，")
    log("      到捕获带后动作相同（early-brake lead + HOLD PHASE 到目标）。")
    log("结构:")
    log("  1) 进场/软降（随起始RPM变长）: 当前RPM → 捕获带（低RPM + remain在窗内）")
    log("  2) 终端不变式（与起始RPM无关）: PHASE LEAD + crawl/HOLD PHASE → BRAKE → SETTLE")
    timed = [r for r in rows if r["name"].startswith("A_timed_") or r["name"].startswith("E_")]
    stepped = [r for r in rows if r["name"].startswith("B")]
    term = [r for r in rows if r["name"].startswith("C") or "hold0" in r["name"] or "hold800" in r["name"]]
    passes = [r for r in rows if r.get("pass")]
    if timed:
        ok_t = [r for r in timed if r["pass"]]
        log(f"定时软降A/E: 共{len(timed)}次, 通过{len(ok_t)}次")
        if ok_t:
            best_t = min(ok_t, key=lambda x: x["wall_s"])
            log(f"  最快定时通过: {best_t['name']} wall={best_t['wall_s']:.3f}s |err|={best_t['err']:.2f}")
    if stepped:
        ok_s = [r for r in stepped if r["pass"]]
        log(f"阶梯降速B(先到中低速再同一HOLD终端): 共{len(stepped)}次, 通过{len(ok_s)}次")
        for r in stepped:
            hw = r.get("hold_wall_s")
            log(
                f"  {r['name']}: pass={r['pass']} 总壁钟={r['wall_s']:.3f}s "
                f"HOLD段={hw if hw is not None else float('nan'):.3f}s |err|={r['err']}"
            )
        # Compare HOLD-only wall of stepped vs timed-from-similar join RPM if available
        hold_walls = [r.get("hold_wall_s") for r in stepped if r.get("pass") and r.get("hold_wall_s")]
        if hold_walls and timed:
            # A_timed from full 6k includes soft from 6k; B HOLD starts already low — HOLD段应接近「截取后半段」
            log(
                "  解读: 若B的HOLD段壁钟 ≈ 低起点定时软降的HOLD壁钟，则支持"
                "「阶梯软降=截入更长软降剖面中途」。"
            )
            log(f"  B通过项 HOLD段壁钟: " + ", ".join(f"{x:.3f}s" for x in hold_walls))
    if passes:
        # Recommend terminal recipe from fastest pass with smallest hold wall among similar errs
        by_hold = sorted(
            [r for r in passes if r.get("hold_wall_s") is not None],
            key=lambda x: x["hold_wall_s"],
        )
        rec = by_hold[0] if by_hold else min(passes, key=lambda x: x["wall_s"])
        log("推荐可复用终端配方（与进场RPM解耦）:")
        log(f"  参考试验: {rec['name']}")
        log(f"  lead≈{rec.get('lead_cmd')}° (DONE lead={rec.get('lead_done')})")
        log(f"  crawl 沿用脚本默认; soft_ms/profile 来自 stop_ms（固件 soft=0.75*profile, 下限800）")
        log(f"  soft_ms={rec.get('soft_ms')} profile_ms={rec.get('profile_ms')} brake_to_stop_ms={rec.get('brake_to_stop_ms')}")
        log("  仅随起始RPM变化的量: 进场软降时长/SOFT RATE DOWN，直到进入捕获带后再发同一 HOLD PHASE。")
    else:
        log("本次无通过项，终端配方待下次有PASS后固化；模型仍按「进场可变 + 终端不变」设计后续试验。")
    log("坐标系注意: 全部指令目标均为 HOLD PHASE 0（非180）。若目视物理~180而固件disc~0，")
    log("  属标架/霍尔对调问题，应 PHASE OFFSET 180 或核对 GPIO4=0/GPIO5=180，而非改终端策略本身。")


def main() -> int:
    ap = argparse.ArgumentParser(description="6000RPM phase-0 stop strategy compare")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--rpm", type=float, default=6000.0)
    ap.add_argument("--crawl", type=float, default=80.0)
    ap.add_argument("--tol", type=float, default=PASS_TOL)
    ap.add_argument("--settle", type=float, default=3.5)
    ap.add_argument(
        "--strategies",
        default="A,B,C,D,E",
        help="comma list: A=timed_sweep B=stepped C=agg_crawl D=lead_heavy E=baseline",
    )
    args = ap.parse_args()
    globals()['PASS_TOL'] = args.tol

    LOGDIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOGDIR / f"hold_phase_strategy_cmp_{stamp}.txt"

    want = {x.strip().upper() for x in args.strategies.split(",") if x.strip()}
    rows: list[dict] = []

    acquire("hold_phase_strategy_cmp")
    ser = None
    rc = 1
    try:
        log(f"=== HOLD PHASE strategy compare {datetime.now().isoformat(timespec='seconds')} ===")
        log(f"port={args.port} rpm={args.rpm:.0f} crawl={args.crawl} gear={GEAR_NOTE} float 2k/400 HOST OFF")
        log(f"strategies={sorted(want)} tol=|{args.tol}|° prefer ≤{TOL_GOOD}°")
        ser = open_port(args.port)
        time.sleep(0.7)
        ser.reset_input_buffer()

        send(ser, "TELEM OFF")
        drain(ser, 0.25)
        send(ser, "HOST OFF")
        wait_ack(ser, lambda s: "HOST OFF" in s, 1.5)
        send(ser, "FIXED?")
        wait_ack(ser, lambda s: "FIXED" in s or "float" in s.lower() or "ACK" in s, 1.0)
        send(ser, "ENC?")
        wait_ack(ser, lambda s: "ENC" in s or "collect" in s.lower() or "ACK" in s, 1.2)
        send(ser, "CTRL?")
        wait_ack(ser, lambda s: "CTRL" in s or "ACK" in s, 1.0)
        send(ser, "HALL?")
        wait_ack(ser, lambda s: "cal=" in s, 1.5)
        send(ser, "PHASE?")
        wait_ack(ser, lambda s: "PHASE" in s, 1.0)
        send(ser, "PHASE LEAD?")
        wait_ack(ser, lambda s: "PHASE LEAD" in s, 1.0)

        # stock lead for most trials
        set_lead(ser, 10.0, 100, 0.03)
        send(ser, "PHASE OFFSET 0")
        drain(ser, 0.15)

        safe_stop(ser)
        time.sleep(0.3)
        prep_closed(ser)

        log("--- gate RPM 2000 (Hall latch) ---")
        send(ser, "RPM 2000")
        gate = detect_spin(ser, seconds=3.2, keepalive="RPM 2000")
        log(f"gate {gate.summary_line()}")
        if not gate.spinning:
            log("FAIL: gate not spinning")
            safe_stop(ser)
            log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
            return 4
        send(ser, "HALL?")
        hl = wait_ack(ser, lambda s: "cal=" in s, 1.5)
        if not any("cal=1" in s for s in hl):
            log("WARN cal=0 after gate — continue (may fail HOLD)")

        log(f"--- stabilize {args.rpm:.0f} RPM ---")
        send(ser, f"RPM {args.rpm:.0f}")
        st0 = hold_until_rpm(ser, args.rpm, args.settle, band=350.0)
        log(f"stable0 ok={st0['ok']} mean={st0.get('mean', 0):.1f}")
        if not st0["ok"]:
            log("FAIL: cannot stabilize at target RPM")
            safe_stop(ser)
            log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
            return 5

        # -------- E baseline: stop_ms=1500 lead=10 --------
        if "E" in want:
            log("\n======== E baseline soft 1500 lead=10 ========")
            set_lead(ser, 10.0)
            send(ser, "TELEM OFF")
            drain(ser, 0.15)
            t0 = time.time()
            h = run_hold_phase(ser, 0.0, args.crawl, 1500)
            rows.append(eval_trial("E_base_1500_lead10", time.time() - t0, h, 10.0))
            reaccel_to(ser, args.rpm, args.settle)

        # -------- A timed soft-down sweep --------
        if "A" in want:
            for ms in (2000, 1500, 1200, 1000, 800):
                # skip duplicate of E if already done
                if ms == 1500 and "E" in want:
                    log(f"\n======== A timed skip {ms} (same as E) — use E row ========")
                    continue
                log(f"\n======== A timed soft-down stop_ms={ms} lead=10 ========")
                set_lead(ser, 10.0)
                send(ser, "TELEM OFF")
                drain(ser, 0.15)
                t0 = time.time()
                h = run_hold_phase(ser, 0.0, args.crawl, ms)
                rows.append(eval_trial(f"A_timed_{ms}", time.time() - t0, h, 10.0, note=f"stop_ms={ms}"))
                # if fail/overshoot at this ms, still try shorter once to see fail mode; but always reaccel
                reaccel_to(ser, args.rpm, args.settle)

        # -------- B stepped RPM then HOLD --------
        if "B" in want:
            log("\n======== B stepped 6000→4000→2000 → HOLD 1000 ========")
            set_lead(ser, 10.0)
            send(ser, "TELEM ON")
            prep_closed(ser, soft_up=900, soft_down=2800)
            t0 = time.time()
            send(ser, "RPM 4000")
            s1 = hold_until_rpm(ser, 4000, 1.0, band=400.0, timeout=20.0)
            log(f"step4000 ok={s1['ok']} mean={s1.get('mean', 0):.1f}")
            send(ser, "RPM 2000")
            s2 = hold_until_rpm(ser, 2000, 1.0, band=350.0, timeout=20.0)
            log(f"step2000 ok={s2['ok']} mean={s2.get('mean', 0):.1f}")
            send(ser, "TELEM OFF")
            drain(ser, 0.12)
            h = run_hold_phase(ser, 0.0, args.crawl, 1000)
            wall = time.time() - t0
            rows.append(
                eval_trial(
                    "B_step_6k_4k_2k_hold1000",
                    wall,
                    h,
                    10.0,
                    note=f"step_ok={s1['ok']}/{s2['ok']}",
                )
            )
            reaccel_to(ser, args.rpm, args.settle)

            log("\n======== B2 stepped 6000→3000 → HOLD 800 ========")
            set_lead(ser, 10.0)
            send(ser, "TELEM ON")
            prep_closed(ser, soft_up=900, soft_down=3000)
            t0 = time.time()
            send(ser, "RPM 3000")
            s3 = hold_until_rpm(ser, 3000, 1.0, band=400.0, timeout=22.0)
            log(f"step3000 ok={s3['ok']} mean={s3.get('mean', 0):.1f}")
            send(ser, "TELEM OFF")
            drain(ser, 0.12)
            h = run_hold_phase(ser, 0.0, args.crawl, 800)
            rows.append(
                eval_trial("B2_step_6k_3k_hold800", time.time() - t0, h, 10.0, note=f"step_ok={s3['ok']}")
            )
            reaccel_to(ser, args.rpm, args.settle)

        # -------- C aggressive soft then crawl (stop_ms=0) --------
        if "C" in want:
            log("\n======== C agg soft→crawl200 then HOLD stop_ms=0 ========")
            set_lead(ser, 10.0)
            send(ser, "TELEM ON")
            prep_closed(ser, soft_up=900, soft_down=4500)
            t0 = time.time()
            sd = soft_down_to_rpm(ser, 200.0, settle_s=0.8, band=180.0, timeout=20.0)
            log(f"soft→200 ok={sd['ok']} mean={sd.get('mean', 0):.1f}")
            send(ser, "TELEM OFF")
            drain(ser, 0.12)
            h = run_hold_phase(ser, 0.0, args.crawl, 0)  # immediate crawl+lead
            rows.append(
                eval_trial(
                    "C_agg_soft200_hold0",
                    time.time() - t0,
                    h,
                    10.0,
                    note=f"pre_soft_ok={sd['ok']}",
                )
            )
            reaccel_to(ser, args.rpm, args.settle)

            log("\n======== C2 agg soft→crawl400 then HOLD 800 ========")
            set_lead(ser, 10.0)
            send(ser, "TELEM ON")
            prep_closed(ser, soft_up=900, soft_down=4500)
            t0 = time.time()
            sd = soft_down_to_rpm(ser, 400.0, settle_s=0.6, band=220.0, timeout=18.0)
            log(f"soft→400 ok={sd['ok']} mean={sd.get('mean', 0):.1f}")
            send(ser, "TELEM OFF")
            drain(ser, 0.12)
            h = run_hold_phase(ser, 0.0, args.crawl, 800)
            rows.append(
                eval_trial(
                    "C2_agg_soft400_hold800",
                    time.time() - t0,
                    h,
                    10.0,
                    note=f"pre_soft_ok={sd['ok']}",
                )
            )
            reaccel_to(ser, args.rpm, args.settle)

        # -------- D lead-heavy early cut --------
        if "D" in want:
            for lead, ms in ((18.0, 1200), (22.0, 1000), (28.0, 800)):
                log(f"\n======== D lead-heavy lead={lead:.0f} stop_ms={ms} ========")
                set_lead(ser, lead, 100, 0.04)
                send(ser, "TELEM OFF")
                drain(ser, 0.15)
                t0 = time.time()
                h = run_hold_phase(ser, 0.0, args.crawl, ms)
                rows.append(
                    eval_trial(
                        f"D_lead{lead:.0f}_ms{ms}",
                        time.time() - t0,
                        h,
                        lead,
                        note="lead_heavy",
                    )
                )
                reaccel_to(ser, args.rpm, args.settle)

        # restore stock lead
        set_lead(ser, 10.0, 100, 0.03)
        safe_stop(ser)
        send(ser, "PWM 1000")
        drain(ser, 0.2)

        # -------- Chinese comparison table --------
        log("\n======== 中文策略对比表 ========")
        log(
            f"{'策略':<28} {'成功':>4} {'壁钟s':>7} {'|err|°':>7} {'disc°':>8} "
            f"{'冲偏':>4} {'hall':>4} {'soft_ms':>7} {'brk→停ms':>8} {'lead':>5}"
        )
        passes = [r for r in rows if r["pass"]]
        fails = [r for r in rows if not r["pass"]]
        for r in rows:
            err_s = f"{r['err']:.2f}" if r["err"] is not None else "-"
            disc_s = f"{r['disc']:.2f}" if r["disc"] is not None else "-"
            soft_s = f"{r['soft_ms']:.0f}" if r.get("soft_ms") is not None else "-"
            bts = f"{r['brake_to_stop_ms']:.0f}" if r.get("brake_to_stop_ms") is not None else "-"
            hc = int(r["hall_confirm"]) if r.get("hall_confirm") is not None else -1
            log(
                f"{r['name']:<28} {'Y' if r['pass'] else 'N':>4} {r['wall_s']:7.3f} {err_s:>7} {disc_s:>8} "
                f"{'Y' if r['overshoot'] else 'N':>4} {hc:4d} {soft_s:>7} {bts:>8} {r['lead_cmd']:5.1f}"
            )

        if passes:
            best = min(passes, key=lambda x: x["wall_s"])
            log("")
            log(f"最快可靠成功: {best['name']}")
            log(f"  壁钟(减速起点→DONE): {best['wall_s']:.3f} s")
            log(f"  HOLD指令→DONE: {best.get('hold_wall_s', float('nan')):.3f} s")
            log(f"  最终盘相位 disc={best['disc']:.2f}° |err|={best['err']:.2f}°")
            log(f"  lead_cmd={best['lead_cmd']:.1f} lead_done={best.get('lead_done')}")
            log(f"  soft_ms={best.get('soft_ms')} profile_ms={best.get('profile_ms')} brake_to_stop_ms={best.get('brake_to_stop_ms')}")
            log(f"  hall_confirm={best.get('hall_confirm')} overshoot={best['overshoot']}")
            log(f"  DONE: {best['done']}")
            rc = 0
        else:
            log("无通过容差的策略（|err|≤tol 且无冲偏）")
            rc = 1

        if fails:
            log("失败/冲偏条目:")
            for r in fails:
                log(
                    f"  FAIL {r['name']}: wall={r['wall_s']:.3f} err={r['err']} "
                    f"overshoot={r['overshoot']} done={r['done'][:120]}"
                )

        chinese_join_model_conclusion(rows)
        log(f"日志: {log_path}")
        log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
        return rc
    except Exception as e:
        log(f"EXCEPTION: {e}")
        try:
            if ser:
                safe_stop(ser)
        except Exception:
            pass
        log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
        return 99
    finally:
        try:
            if ser:
                try:
                    send(ser, "PWM 1000")
                    drain(ser, 0.15)
                except Exception:
                    pass
                ser.close()
        except Exception:
            pass
        try:
            release()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
