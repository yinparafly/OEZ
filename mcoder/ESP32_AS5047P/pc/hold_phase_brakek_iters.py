#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Multi-iter HOLD PHASE distance-planner / brake_k EMA check (COM10).

Flow:
  HOST OFF -> Hall gate spin -> PHASE PLAN?/BRAKEK?/REFINE/TOL
  -> N soft-down HOLD PHASE 0 from start RPM(s)
  -> record DONE disc/err/reason/refine/hall_confirm + brake_k before/after

Usage:
  set PYTHONIOENCODING=utf-8
  python hold_phase_brakek_iters.py --port COM10 --iters 5 --rpm 2500
  python hold_phase_brakek_iters.py --port COM10 --iters 4 --rpm 2500 --rpm-hi 5000
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


def parse_kv(line: str, key: str):
    m = re.search(rf"{re.escape(key)}=([\-\d.]+(?:[eE][\-+]?\d+)?)", line)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def parse_kv_str(line: str, key: str):
    m = re.search(rf"{re.escape(key)}=([^\s]+)", line)
    return m.group(1) if m else None


def wrap180(d: float) -> float:
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def in_band(disc: float, lo: float = 355.0, hi: float = 5.0) -> bool:
    d = disc % 360.0
    return d >= lo or d <= hi


def safe_stop(ser) -> None:
    for c in ("ESTOP", "STOP", "RPM 0", "PWM 1000"):
        send(ser, c)
        drain(ser, 0.12)


def parse_telem(line: str):
    if not line or line.startswith("#"):
        return None
    p = line.split(",")
    if len(p) < 17:
        return None
    try:
        return {"rpm": float(p[4]), "pulse": int(float(p[9])), "run": int(float(p[16]))}
    except (ValueError, IndexError):
        return None


def read_hall(ser) -> dict:
    send(ser, "HALL?")
    # Prefer summary line with irq_dn=/cal= (avoid early return on async HALL DOWN/UP)
    lines = wait_ack(ser, lambda s: "irq_dn=" in s or ("cal=" in s and "gear=" in s), 2.0)
    extra = drain(ser, 0.25)
    lines = list(lines) + list(extra)
    out = {"cal": 0, "irq_dn": 0, "irq_up": 0, "raw": " ".join(lines)}
    for ln in lines:
        for k in ("irq_dn", "irq_up", "cal"):
            v = parse_kv(ln, k)
            if v is not None:
                out[k] = int(v)
    return out


def read_brakek(ser) -> dict:
    send(ser, "PHASE BRAKEK?")
    lines = wait_ack(ser, lambda s: "BRAKEK" in s or "brake_k=" in s, 1.2)
    blob = " ".join(lines)
    return {
        "k": parse_kv(blob, "k") or parse_kv(blob, "brake_k"),
        "est": parse_kv(blob, "est@rpm") or parse_kv(blob, "est_brake"),
        "learn_n": parse_kv(blob, "learn_n"),
        "raw": blob,
    }


def read_plan(ser) -> str:
    send(ser, "PHASE PLAN?")
    lines = wait_ack(ser, lambda s: "PHASE PLAN" in s or "brake_k=" in s, 1.2)
    return " | ".join(lines) if lines else "(no ACK)"


def hold_until_rpm(ser, target: float, settle_s: float, band: float = 350.0) -> dict:
    t0 = time.time()
    last_ka = 0.0
    win: list[float] = []
    ok_since: float | None = None
    while time.time() - t0 < 45.0:
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
                    }
            else:
                ok_since = None
        time.sleep(0.01)
    mean = sum(win) / len(win) if win else 0.0
    return {"ok": False, "mean": mean, "last": win[-1] if win else 0.0}


def run_hold(ser, target: float, crawl: float, stop_ms: int, timeout: float = 35.0) -> dict:
    cmd = f"HOLD PHASE {target:g} {crawl:g} {stop_ms}"
    send(ser, cmd)
    log(f">> {cmd}")
    t0 = time.time()
    brake = ""
    learn = ""
    done = ""
    latches: list[str] = []
    while time.time() - t0 < timeout:
        for ln in drain(ser, 0.12):
            if ("HALL DOWN" in ln or "HALL UP" in ln) and "motor_cnt=" in ln:
                latches.append(ln)
                log(f"latch: {ln}")
            if "BRAKE" in ln and "HOLD PHASE" in ln and "BRAKEK" not in ln:
                brake = ln
            if "BRAKEK learn" in ln:
                learn = ln
            if "HOLD PHASE DONE" in ln or ln.startswith("# ESTOP") or "ERR HOLD PHASE" in ln:
                done = ln
                return {
                    "ok": "DONE" in ln,
                    "done": done,
                    "brake": brake,
                    "learn": learn,
                    "latches": latches,
                    "wall_s": time.time() - t0,
                }
        time.sleep(0.02)
    return {
        "ok": False,
        "done": "TIMEOUT",
        "brake": brake,
        "learn": learn,
        "latches": latches,
        "wall_s": time.time() - t0,
    }



def main() -> int:
    ap = argparse.ArgumentParser(description="HOLD PHASE brake_k multi-iter learn check")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--target", type=float, default=0.0)
    ap.add_argument("--rpm", type=float, default=2500.0, help="primary start RPM")
    ap.add_argument("--rpm-hi", type=float, default=0.0, help="optional higher RPM after low iters")
    ap.add_argument("--iters", type=int, default=5, help="iters at --rpm")
    ap.add_argument("--iters-hi", type=int, default=2, help="iters at --rpm-hi if set")
    ap.add_argument("--crawl", type=float, default=80.0)
    ap.add_argument("--stop-ms", type=int, default=2000)
    ap.add_argument("--tol", type=float, default=5.0)
    ap.add_argument("--settle", type=float, default=3.0)
    args = ap.parse_args()

    LOGDIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOGDIR / f"hold_phase_brakek_iters_{stamp}.txt"

    schedule: list[tuple[float, int]] = [(args.rpm, args.iters)]
    if args.rpm_hi > 0:
        schedule.append((args.rpm_hi, args.iters_hi))

    acquire("hold_phase_brakek_iters")
    ser = None
    rc = 1
    trend: list[dict] = []
    try:
        log(f"=== HOLD PHASE BRAKEK ITERS {datetime.now().isoformat(timespec='seconds')} ===")
        log(
            f"port={args.port} target={args.target} schedule={schedule} "
            f"stop_ms={args.stop_ms} crawl={args.crawl} tol={args.tol}"
        )
        ser = open_port(args.port)
        time.sleep(0.8)
        ser.reset_input_buffer()

        send(ser, "TELEM OFF")
        drain(ser, 0.3)
        send(ser, "HOST OFF")
        wait_ack(ser, lambda s: "HOST OFF" in s, 1.5)

        # Architecture / FIXED probe (flash only if cmds missing)
        send(ser, "FIXED?")
        fixed_lines = wait_ack(ser, lambda s: "FIXED" in s or "ACK" in s or "ERR" in s, 1.2)
        log(f"FIXED?: {' | '.join(fixed_lines) if fixed_lines else '(no reply)'}")

        for c in (
            "MODE CLOSED",
            "SOFT ON",
            "SOFT RATE UP 900",
            "SOFT RATE DOWN 2000",
            "RPMMAX 12000",
            "FREQ 400",
            "TELEM ON",
        ):
            send(ser, c)
            drain(ser, 0.12)

        # Planner presence
        plan = read_plan(ser)
        log(f"PHASE PLAN?: {plan}")
        if "ERR" in plan and "PHASE PLAN" in plan and "brake_k" not in plan:
            log("BLOCK: PHASE PLAN? missing on board — need flash with distance planner")
            safe_stop(ser)
            log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
            return 3

        bk0 = read_brakek(ser)
        log(f"PHASE BRAKEK? init k={bk0['k']} est={bk0['est']} learn_n={bk0['learn_n']}")
        if bk0["k"] is None and "ERR" in bk0["raw"]:
            log("BLOCK: PHASE BRAKEK? missing — need flash")
            safe_stop(ser)
            log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
            return 3

        send(ser, "PHASE REFINE ON")
        wait_ack(ser, lambda s: "REFINE" in s, 1.0)
        send(ser, "PHASE REFINE?")
        wait_ack(ser, lambda s: "REFINE" in s, 1.0)
        send(ser, f"PHASE TOL {args.tol:g}")
        wait_ack(ser, lambda s: "TOL" in s, 1.0)
        send(ser, "PHASE TOL?")
        wait_ack(ser, lambda s: "TOL" in s, 1.0)

        # Hall gate while spinning
        log("--- gate RPM 2000 (Hall latch) ---")
        send(ser, "RPM 2000")
        gate = detect_spin(ser, seconds=3.5, keepalive="RPM 2000")
        log(f"gate {gate.summary_line()}")
        h = read_hall(ser)
        log(f"after gate cal={h['cal']} irq_dn={h['irq_dn']} irq_up={h['irq_up']}")
        if h["irq_dn"] == 0 and h["irq_up"] == 0:
            # gate.summary already proved edges if spinning+hall; retry once
            time.sleep(0.2)
            h = read_hall(ser)
            log(f"retry HALL? cal={h['cal']} irq_dn={h['irq_dn']} irq_up={h['irq_up']}")
        if h["irq_dn"] == 0 and h["irq_up"] == 0 and not getattr(gate, "spinning", False):
            log("BLOCK: Hall dead (irq_dn=irq_up=0) — check GPIO4/5 / magnet")
            safe_stop(ser)
            log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
            return 2
        if h["irq_dn"] == 0 and h["irq_up"] == 0:
            log("WARN: HALL? parse still 0 but gate saw motion — continue (async edge race)")
        if h["cal"] != 1:
            log("WARN: cal!=1 after spin — may lack absolute 0° latch")
        if h["irq_dn"] == 0:
            log("WARN: GPIO4 irq_dn=0 — physical 0° cannot Hall-confirm")

        it_global = 0
        for rpm_set, n_iters in schedule:
            for local_i in range(1, n_iters + 1):
                it_global += 1
                log(
                    f"\n======== ITER {it_global} "
                    f"(rpm={rpm_set:.0f} #{local_i}/{n_iters}) ========"
                )
                safe_stop(ser)
                time.sleep(0.35)
                send(ser, "STOP")
                drain(ser, 0.2)
                send(ser, "MODE CLOSED")
                send(ser, "SOFT ON")
                send(ser, "SOFT RATE DOWN 2000")
                send(ser, f"PHASE TOL {args.tol:g}")
                send(ser, "PHASE REFINE ON")
                send(ser, "TELEM ON")
                drain(ser, 0.2)

                # warm then target
                send(ser, "RPM 2000")
                detect_spin(ser, seconds=1.8, keepalive="RPM 2000")
                send(ser, f"RPM {rpm_set:.0f}")
                st = hold_until_rpm(ser, rpm_set, args.settle, band=350.0)
                log(f"stable ok={st['ok']} mean={st.get('mean', 0):.1f} last={st.get('last', 0):.1f}")
                if not st["ok"]:
                    log("FAIL stabilize — abort schedule")
                    safe_stop(ser)
                    break

                bk_before = read_brakek(ser)
                send(ser, "PHASE PLAN?")
                plan_b = wait_ack(ser, lambda s: "PHASE PLAN" in s or "brake_k=" in s, 1.0)
                est_before = None
                for ln in plan_b:
                    v = parse_kv(ln, "est_brake")
                    if v is not None:
                        est_before = v
                log(
                    f"before: brake_k={bk_before['k']} learn_n={bk_before['learn_n']} "
                    f"est_brake={est_before}"
                )

                h_before = read_hall(ser)
                send(ser, "TELEM OFF")
                drain(ser, 0.12)

                r = run_hold(ser, args.target, args.crawl, args.stop_ms)
                log(f"done: {r['done']}")
                cnts = [parse_kv(x, "motor_cnt") for x in r.get("latches", [])]
                cnts = [c for c in cnts if c is not None]
                log(f"latches_during_hold={len(r.get('latches', []))} motor_cnt_seq={cnts}")
                if len(set(cnts)) < len(cnts):
                    log("WARN: duplicate motor_cnt latch (debounce?)")
                elif len(cnts) >= 2:
                    log("OK: disc_ref motor_cnt updated across Hall edges")
                if r["brake"]:
                    log(f"brake: {r['brake']}")
                if r["learn"]:
                    log(f"learn: {r['learn']}")

                bk_after = read_brakek(ser)
                h_after = read_hall(ser)
                send(ser, "PHASE?")
                ph_lines = wait_ack(ser, lambda s: "disc=" in s or "PHASE" in s, 1.0)
                ph_blob = " ".join(ph_lines)

                done = r["done"]
                disc = parse_kv(done, "disc")
                if disc is None:
                    disc = parse_kv(ph_blob, "disc") or parse_kv(ph_blob, "disc_deg")
                err = parse_kv(done, "err")
                if err is None and disc is not None:
                    err = wrap180(disc - args.target)
                reason = parse_kv_str(done, "reason")
                refine = parse_kv(done, "refine")
                hall_confirm = parse_kv(done, "hall_confirm")
                k_done = parse_kv(done, "brake_k")
                est_brake_evt = parse_kv(r["brake"], "est_brake") if r["brake"] else None
                brake_reason = parse_kv_str(r["brake"], "reason") if r["brake"] else None

                abs_err = abs(err) if err is not None else None
                band_ok = in_band(disc) if disc is not None else False
                row = {
                    "iter": it_global,
                    "rpm": rpm_set,
                    "disc": disc,
                    "err": err,
                    "abs_err": abs_err,
                    "reason": reason,
                    "refine": refine,
                    "hall_confirm": hall_confirm,
                    "k_before": bk_before["k"],
                    "k_after": bk_after["k"] if bk_after["k"] is not None else k_done,
                    "learn_n_before": bk_before["learn_n"],
                    "learn_n_after": bk_after["learn_n"],
                    "est_before": est_before,
                    "est_brake_evt": est_brake_evt,
                    "brake_reason": brake_reason,
                    "band_ok": band_ok,
                    "ok": r["ok"],
                    "delta_dn": h_after["irq_dn"] - h_before["irq_dn"],
                    "delta_up": h_after["irq_up"] - h_before["irq_up"],
                    "wall_s": r["wall_s"],
                }
                trend.append(row)
                log(
                    f"ITER{it_global}: disc={disc} err={err} |err|={abs_err} "
                    f"reason={reason} refine={refine} hall_confirm={hall_confirm} "
                    f"band[355,5]={band_ok} "
                    f"k {bk_before['k']}->{row['k_after']} "
                    f"learn_n {bk_before['learn_n']}->{bk_after['learn_n']} "
                    f"est_evt={est_brake_evt} brake_reason={brake_reason}"
                )

                safe_stop(ser)
                time.sleep(0.5)

                if abs_err is not None and abs_err <= args.tol and band_ok:
                    rc = 0

        # summary
        send(ser, "HOST OFF")
        drain(ser, 0.3)
        safe_stop(ser)

        log("\n======== 中文结果摘要 ========")
        log(f"迭代次数: {len(trend)}")
        for row in trend:
            log(
                f"  #{row['iter']} @{row['rpm']:.0f}: disc={row['disc']} "
                f"err={row['err']} |err|={row['abs_err']} reason={row['reason']} "
                f"refine={row['refine']} hall_c={row['hall_confirm']} "
                f"k={row['k_before']}→{row['k_after']} "
                f"learn_n={row['learn_n_before']}→{row['learn_n_after']} "
                f"band={row['band_ok']}"
            )
        if trend:
            ks = [r["k_before"] for r in trend if r["k_before"] is not None]
            ke = [r["k_after"] for r in trend if r["k_after"] is not None]
            errs = [r["abs_err"] for r in trend if r["abs_err"] is not None]
            log(
                f"brake_k: {ks[0] if ks else None} → {ke[-1] if ke else None} "
                f"(Δ={(ke[-1]-ks[0]) if ks and ke else None})"
            )
            log("误差趋势 |err|: " + " → ".join(f"{e:.2f}" for e in errs))
            refine_hits = [r for r in trend if (r.get("reason") == "refine_hall") or (r.get("refine") == 1)]
            log(f"REFINE 触发: {len(refine_hits)}/{len(trend)}" + (
                " reasons=" + ",".join(str(r.get("reason")) for r in refine_hits) if refine_hits else ""
            ))
            # fastest reliable RPM: all iters at that rpm with |err|<=tol or band
            by_rpm: dict[float, list] = {}
            for r in trend:
                by_rpm.setdefault(r["rpm"], []).append(r)
            reliable = []
            for rpm, rows in sorted(by_rpm.items()):
                ok_n = sum(
                    1
                    for r in rows
                    if r["abs_err"] is not None
                    and r["abs_err"] <= args.tol
                    and r.get("band_ok")
                )
                if ok_n == len(rows) and ok_n > 0:
                    reliable.append(rpm)
            if reliable:
                log(f"可靠起始转速(该档全部达标): {reliable} — 最快可靠={max(reliable):.0f} RPM")
            else:
                partial = []
                for rpm, rows in sorted(by_rpm.items()):
                    ok_n = sum(
                        1
                        for r in rows
                        if r["abs_err"] is not None and r["abs_err"] <= args.tol
                    )
                    partial.append(f"{rpm:.0f}:{ok_n}/{len(rows)}")
                log(f"无一档全部达标; 分档命中: {', '.join(partial)}")

        log(f"日志: {log_path}")
        log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
        return rc if trend else 1
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
            if ser is not None:
                try:
                    send(ser, "HOST OFF")
                    drain(ser, 0.2)
                    safe_stop(ser)
                except Exception:
                    pass
                try:
                    ser.close()
                except Exception:
                    pass
        finally:
            release()


if __name__ == "__main__":
    sys.exit(main())
