# -*- coding: utf-8 -*-
"""[METHODOLOGY-INVALID for calc accuracy] 双烧闭环跟踪对比。

对「Q16.16 vs float **计算精度**」无效：两次 flash、不同时刻样本上的闭环
跟踪均值差，混入台架/扰动/控制，不是同一 raw 上的双算法差。

正确方法见: compare_q16_same_sample.py（同 (t_us,counts) 双算）。
本脚本仅可作次要「系统可切/闭环稳」检查，勿引用其 Δmean≈47 为定点精度。

用法（次要系统检查）:
  python compare_float_q16.py --tag float
  python compare_float_q16.py --tag q16
  python compare_float_q16.py --analyze float.json q16.json
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import statistics as st
import sys
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("PYTHONIOENCODING", "utf-8")

from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire, release
from motor_spin_detect import detect_spin
from _load_compare import parse_load, fmt_row

PORT = "COM10"
TARGETS = [2000.0, 3000.0, 4000.0]
RPMMAX = 12000
SOFT_UP = 900
SOFT_DOWN = 700
SETTLE_S = 8.0
HOLD_S = 11.0          # 总保持; 末段 MEAN_TAIL_S 用于均值
MEAN_TAIL_S = 3.0
KEEPALIVE_S = 0.5
POLL_S = 2.5
ABS_DIFF_PASS = 0.5    # |μ_q16 − μ_float| < 0.5 RPM
PP_ABS_OK = 5.0
PP_REL_OK = 0.10

LOGDIR = Path(__file__).resolve().parent / "logs"

_out: list[str] = []


def log(s: str) -> None:
    print(s, flush=True)
    _out.append(s)


def drain(ser, budget=0.15):
    return read_lines(ser, budget)


def parse_telem(line: str):
    if not line or line.startswith("#"):
        return None
    p = line.split(",")
    if len(p) < 25:
        return None
    try:
        return {
            "t_ms": float(p[0]),
            "rpm": float(p[4]),
            "pulse": int(float(p[9])),
            "target": float(p[10]),
            "run": int(float(p[16])),
            "out_hz": float(p[22]),
            "enc_hz": float(p[24]),
        }
    except (ValueError, IndexError):
        return None


def query_line(ser, cmd: str, prefix: str, timeout=1.2) -> str:
    drain(ser, 0.05)
    send(ser, cmd)
    end = time.time() + timeout
    while time.time() < end:
        for ln in drain(ser, 0.15):
            if ln.startswith(prefix):
                return ln
    return ""


def enc_query(ser):
    ln = query_line(ser, "ENC?", "# ENC")
    m = re.search(r"meas_hz=([\d.]+).*?cpu%=([\d.]+).*?loop_hz=([\d.]+)", ln)
    if m:
        return float(m.group(1)), float(m.group(2)), float(m.group(3)), ln
    return None, None, None, ln


def ctrl_query(ser):
    return query_line(ser, "CTRL?", "# CTRL")


def fixed_query(ser):
    return query_line(ser, "FIXED?", "# FIXED")


def load_query(ser, tag: str) -> dict | None:
    drain(ser, 0.05)
    send(ser, "LOAD?")
    end = time.time() + 1.2
    while time.time() < end:
        for ln in drain(ser, 0.15):
            if ln.startswith("# LOAD"):
                d = parse_load(ln)
                if d is not None:
                    m = re.search(r"enc_ring_overrun=(\d+)", ln, re.I)
                    d["enc_ring_overrun"] = int(m.group(1)) if m else None
                log(fmt_row(tag, d, ln))
                return d
    log(f"{tag}: LOAD? 无响应")
    return None


def safe_stop(ser):
    for c in ("RPM 0", "STOP", "PWM 1000"):
        send(ser, c)
        time.sleep(0.2)
    drain(ser, 0.5)


def estop(ser):
    for c in ("ESTOP", "PWM 1000"):
        send(ser, c)
        time.sleep(0.15)
    drain(ser, 0.4)


def ramp_down(ser):
    for r in (3500, 2500, 1500):
        for _ in range(4):
            send(ser, f"RPM {r}")
            drain(ser, 0.12)
            time.sleep(0.25)
    safe_stop(ser)


def hold_target(ser, target: float, csv_writer, t0: float, tag: str) -> dict:
    """Hold TARGET for HOLD_S; return stats from last MEAN_TAIL_S."""
    send(ser, "LOAD RESET")
    time.sleep(0.1)
    drain(ser, 0.2)

    t_seg = time.time()
    last_ka = 0.0
    last_poll = 0.0
    aborted = None
    rpm_all: list[float] = []
    pulse_all: list[int] = []
    enchz_all: list[float] = []
    peak = 0.0
    load_mid = None

    while time.time() - t_seg < HOLD_S:
        noww = time.time()
        tseg = noww - t_seg
        if noww - last_ka >= KEEPALIVE_S:
            last_ka = noww
            send(ser, f"RPM {target:.0f}")
        for ln in drain(ser, 0.12):
            if ln.startswith("#"):
                if "ESTOP" in ln and "cleared" not in ln and "ACK ESTOP" not in ln:
                    aborted = ln[:120]
                continue
            te = parse_telem(ln)
            if not te:
                continue
            arpm = abs(te["rpm"])
            rpm_all.append(arpm)
            pulse_all.append(te["pulse"])
            enchz_all.append(te["enc_hz"])
            peak = max(peak, arpm)
            csv_writer.writerow([
                f"{noww - t0:.2f}", f"{tseg:.2f}", int(te["t_ms"]), tag,
                f"{target:.0f}", f"{arpm:.2f}", te["pulse"], f"{te['target']:.0f}",
                te["run"], f"{te['out_hz']:.5f}", f"{te['enc_hz']:.1f}",
            ])
        if aborted:
            break
        if load_mid is None and tseg >= SETTLE_S + 1.0:
            load_mid = load_query(ser, f"[LOAD @{target:.0f} mid]")
        if noww - last_poll >= POLL_S:
            last_poll = noww
            hz, cpu, _, _ = enc_query(ser)
            cur = rpm_all[-1] if rpm_all else 0.0
            log(f"  [{tag} t={tseg:4.1f}s] target={target:.0f} enc~{cur:.1f} "
                f"meas_hz={hz} cpu%={cpu}")
        time.sleep(0.02)

    # last MEAN_TAIL_S of samples (approx by time fraction of HOLD_S)
    n = len(rpm_all)
    if n == 0:
        return {
            "target": target, "n": 0, "mean": None, "pp": None, "peak": peak,
            "pulse_mean": None, "aborted": aborted or "no_samples", "load": load_mid,
        }
    # Use last fraction of samples corresponding to MEAN_TAIL_S / HOLD_S
    frac = MEAN_TAIL_S / HOLD_S
    start = max(0, int(n * (1.0 - frac)))
    # Prefer samples after SETTLE: if we have enough, take last MEAN_TAIL portion of post-settle
    settle_idx = int(n * (SETTLE_S / HOLD_S))
    post = rpm_all[settle_idx:]
    if len(post) >= 3:
        tail_n = max(3, int(len(post) * (MEAN_TAIL_S / max(HOLD_S - SETTLE_S, 0.1))))
        win = post[-tail_n:]
        pwin = pulse_all[settle_idx:][-tail_n:]
        ewin = enchz_all[settle_idx:][-tail_n:]
    else:
        win = rpm_all[start:]
        pwin = pulse_all[start:]
        ewin = enchz_all[start:]

    mean = st.mean(win)
    pp = max(win) - min(win)
    return {
        "target": target,
        "n": len(win),
        "n_all": n,
        "mean": round(mean, 3),
        "std": round(st.pstdev(win), 3) if len(win) > 1 else 0.0,
        "min": round(min(win), 2),
        "max": round(max(win), 2),
        "pp": round(pp, 2),
        "peak": round(peak, 2),
        "overshoot_pct": round(max(0.0, (peak - target) / target * 100.0), 3),
        "static_err": round(mean - target, 3),
        "pulse_mean": round(st.mean(pwin), 2) if pwin else None,
        "enc_hz_mean": round(st.mean(ewin), 1) if ewin else None,
        "aborted": aborted,
        "load": load_mid,
    }


def run_ladder(tag: str) -> int:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOGDIR / f"q16_cmp_{tag}_{stamp}.txt"
    csv_path = LOGDIR / f"q16_cmp_{tag}_{stamp}.csv"
    json_path = LOGDIR / f"q16_cmp_{tag}_{stamp}.json"
    meta_path = LOGDIR / f"q16_cmp_{tag}_{stamp}.meta.txt"

    acquire(f"q16_cmp_{tag}")
    LOGDIR.mkdir(parents=True, exist_ok=True)
    ser = open_port(PORT)
    t0 = time.time()
    results: list[dict] = []
    aborted = None
    arch_ok = False
    fixed_raw = ""
    ctrl_raw = ""
    enc_raw = ""
    load_idle = None
    load_post = None

    csvf = open(csv_path, "w", newline="", encoding="utf-8")
    cw = csv.writer(csvf)
    cw.writerow([
        "wall_s", "t_seg_s", "t_ms", "tag", "step_target", "enc_rpm", "pulse",
        "target_ramped", "run", "out_hz", "enc_sample_hz",
    ])

    try:
        drain(ser, 0.8)
        safe_stop(ser)
        time.sleep(0.3)

        log(f"=== Q16 compare ladder tag={tag} {datetime.now().isoformat(timespec='seconds')} ===")
        log(f"targets={TARGETS} settle>={SETTLE_S}s hold={HOLD_S}s mean_tail={MEAN_TAIL_S}s")
        log("arch lock: collect/use=2000Hz out=400Hz usb_telem=20Hz")

        send(ser, "HOST OFF")
        time.sleep(0.15)
        for ln in drain(ser, 0.4):
            if ln.startswith("#"):
                log(f"  {ln[:160]}")

        fixed_raw = fixed_query(ser)
        log(f"FIXED? {fixed_raw[:200]}")
        hz, cpu, loop, enc_raw = enc_query(ser)
        log(f"ENC? meas_hz={hz} cpu%={cpu} loop_hz={loop} raw={enc_raw[:180]}")
        ctrl_raw = ctrl_query(ser)
        log(f"CTRL? {ctrl_raw[:200]}")

        m_out = re.search(r"out_hz=([\d.]+)", ctrl_raw)
        m_usb = re.search(r"usb_telem_hz=([\d.]+)", ctrl_raw)
        out_hz = float(m_out.group(1)) if m_out else None
        usb_hz = float(m_usb.group(1)) if m_usb else None
        arch_ok = (
            hz is not None and abs(hz - 2000) < 50
            and out_hz is not None and abs(out_hz - 400) < 5
            and (usb_hz is None or abs(usb_hz - 20) < 2)
        )
        log(f"arch_ok={arch_ok} (meas≈2000 out≈400 usb≈20)")
        if not arch_ok:
            log("ARCH FAIL — abort before motor")
            raise SystemExit(4)

        m_fx = re.search(r"use_fixed_point=(\d)", fixed_raw)
        fixed_val = int(m_fx.group(1)) if m_fx else None
        expect_fx = 1 if tag.lower().startswith("q16") else 0
        if fixed_val is not None and fixed_val != expect_fx:
            log(f"FIXED mismatch: got {fixed_val} expect {expect_fx} for tag={tag}")
            raise SystemExit(5)

        send(ser, "LOAD RESET")
        time.sleep(0.1)
        drain(ser, 0.2)
        load_idle = load_query(ser, "[LOAD idle]")

        for c in (
            "STOP", "MODE CLOSED", "SOFT ON",
            f"SOFT RATE UP {SOFT_UP}", f"SOFT RATE DOWN {SOFT_DOWN}",
            "FREQ 400", f"RPMMAX {RPMMAX}", "TELEM ON", "START",
        ):
            send(ser, c)
            time.sleep(0.15)
            acks = [x for x in drain(ser, 0.25) if x.startswith("#")]
            log(f"CMD {c} -> {acks[:2]}")

        # gate @2000
        log("\n=== spin_detect gate @2000 ===")
        send(ser, "RPM 2000")
        t_gate = time.time()
        while time.time() - t_gate < 2.5:
            send(ser, "RPM 2000")
            drain(ser, 0.2)
            time.sleep(0.3)
        gate = detect_spin(ser, seconds=3.0, keepalive="RPM 2000")
        log(gate.summary_line())
        if not gate.spinning:
            log("SPIN_DETECT FAIL — ESTOP")
            estop(ser)
            safe_stop(ser)
            raise SystemExit(3)

        for tgt in TARGETS:
            log(f"\n=== step {tgt:.0f} RPM ===")
            # soft approach: brief keepalive at target (SOFT handles ramp)
            send(ser, f"RPM {tgt:.0f}")
            # allow soft ramp time if jumping up
            if tgt > 2000:
                ramp_need = (tgt - 2000) / float(SOFT_UP) + 0.5
                t_r = time.time()
                while time.time() - t_r < min(ramp_need, 8.0):
                    send(ser, f"RPM {tgt:.0f}")
                    drain(ser, 0.15)
                    time.sleep(0.35)
            st_res = hold_target(ser, tgt, cw, t0, tag)
            results.append(st_res)
            log(
                f"  RESULT @{tgt:.0f}: mean={st_res['mean']} pp={st_res['pp']} "
                f"err={st_res.get('static_err')} n={st_res['n']} aborted={st_res['aborted']}"
            )
            if st_res.get("aborted"):
                aborted = st_res["aborted"]
                break
            if st_res.get("mean") is None:
                aborted = "no_mean"
                break

        log("\n--- ramp down & stop ---")
        if aborted:
            estop(ser)
            safe_stop(ser)
        else:
            ramp_down(ser)
        load_post = load_query(ser, "[LOAD post]")

        meta = (
            f"ts={stamp}\nUSE_FIXED_POINT_expect={expect_fx}\nFIXED_raw={fixed_raw}\n"
            f"CTRL={ctrl_raw}\nENC={enc_raw}\nSOFT_UP={SOFT_UP} SOFT_DOWN={SOFT_DOWN}\n"
            f"settle_s={SETTLE_S} hold_s={HOLD_S} mean_tail_s={MEAN_TAIL_S}\n"
            f"targets={TARGETS}\ntag={tag}\nnote=scheme_4A_ladder\n"
        )
        meta_path.write_text(meta, encoding="utf-8")

        summary = {
            "stamp": stamp,
            "tag": tag,
            "fixed_expect": expect_fx,
            "fixed_raw": fixed_raw,
            "ctrl_raw": ctrl_raw,
            "enc_raw": enc_raw,
            "arch_ok": arch_ok,
            "aborted": aborted,
            "targets": TARGETS,
            "settle_s": SETTLE_S,
            "hold_s": HOLD_S,
            "mean_tail_s": MEAN_TAIL_S,
            "points": results,
            "load_idle": load_idle,
            "load_post": load_post,
            "csv": str(csv_path),
            "log": str(log_path),
            "meta": str(meta_path),
        }
        json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        log_path.write_text("\n".join(_out), encoding="utf-8")
        log(f"\njson={json_path}")
        log(f"csv={csv_path}")
        log(f"log={log_path}")
        log(f"meta={meta_path}")
        return 0 if (not aborted and arch_ok) else 2
    except SystemExit:
        log_path.write_text("\n".join(_out), encoding="utf-8")
        raise
    except Exception as exc:
        try:
            estop(ser)
            safe_stop(ser)
        except Exception:
            pass
        log(f"ERROR: {exc}")
        log_path.write_text("\n".join(_out), encoding="utf-8")
        raise
    finally:
        try:
            csvf.flush()
            csvf.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        release()


def analyze(float_json: Path, q16_json: Path, out_path: Path | None = None) -> int:
    a = json.loads(float_json.read_text(encoding="utf-8"))
    b = json.loads(q16_json.read_text(encoding="utf-8"))
    lines = [
        f"COMPARE float={float_json.name}  q16={q16_json.name}",
        f"float aborted={a.get('aborted')} arch_ok={a.get('arch_ok')}",
        f"q16   aborted={b.get('aborted')} arch_ok={b.get('arch_ok')}",
        "",
        f"{'target':>8} {'n_f':>5} {'n_q':>5} {'mean_f':>10} {'mean_q':>10} "
        f"{'abs_diff':>10} {'pp_f':>8} {'pp_q':>8} {'pass_0p5':>8}",
    ]
    max_abs = 0.0
    all_pass = True
    by_t_f = {p["target"]: p for p in a.get("points", [])}
    by_t_q = {p["target"]: p for p in b.get("points", [])}
    for t in sorted(set(by_t_f) | set(by_t_q)):
        pf, pq = by_t_f.get(t), by_t_q.get(t)
        if not pf or not pq or pf.get("mean") is None or pq.get("mean") is None:
            lines.append(f"{t:8.0f}  MISSING POINT")
            all_pass = False
            continue
        diff = abs(pq["mean"] - pf["mean"])
        max_abs = max(max_abs, diff)
        pp_ok = True
        if pf.get("pp") is not None and pq.get("pp") is not None:
            dpp = abs(pq["pp"] - pf["pp"])
            rel = dpp / max(pf["pp"], 1e-6)
            pp_ok = (dpp < PP_ABS_OK) or (rel < PP_REL_OK) or (pq["pp"] < pf["pp"] * 2)
        ok = diff < ABS_DIFF_PASS and pp_ok
        if not ok:
            all_pass = False
        lines.append(
            f"{t:8.0f} {pf['n']:5d} {pq['n']:5d} {pf['mean']:10.3f} {pq['mean']:10.3f} "
            f"{diff:10.3f} {pf.get('pp') or 0:8.2f} {pq.get('pp') or 0:8.2f} "
            f"{'PASS' if ok else 'FAIL':>8}"
        )

    # load deltas
    def _busy(side, key="load_idle"):
        d = side.get(key) or {}
        return d.get("enc_busy_avg_us"), d.get("ctrl_overrun"), d.get("enc_ring_overrun")

    lines.append("")
    for label, key in (("idle", "load_idle"),):
        bf, of, rf = _busy(a, key)
        bq, oq, rq = _busy(b, key)
        lines.append(
            f"LOAD {label}: float enc_busy_avg={bf} overrun={of} ring={rf} | "
            f"q16 enc_busy_avg={bq} overrun={oq} ring={rq}"
        )
    # per-point load from points
    for t in sorted(by_t_f):
        lf = (by_t_f[t].get("load") or {})
        lq = (by_t_q.get(t) or {}).get("load") or {}
        lines.append(
            f"LOAD @{t:.0f}: float busy={lf.get('enc_busy_avg_us')} ov={lf.get('ctrl_overrun')} "
            f"ring={lf.get('enc_ring_overrun')} | q16 busy={lq.get('enc_busy_avg_us')} "
            f"ov={lq.get('ctrl_overrun')} ring={lq.get('enc_ring_overrun')}"
        )

    verdict = "PASS" if all_pass and not a.get("aborted") and not b.get("aborted") else "FAIL"
    lines.append("")
    lines.append(f"SUMMARY: {verdict}  max_abs_diff={max_abs:.4f}  criterion=<{ABS_DIFF_PASS}")
    text = "\n".join(lines)
    print(text, flush=True)
    if out_path is None:
        out_path = LOGDIR / f"q16_cmp_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    out_path.write_text(text + "\n", encoding="utf-8")
    print(f"wrote {out_path}", flush=True)
    return 0 if verdict == "PASS" else 1


def main() -> int:
    global PORT
    ap = argparse.ArgumentParser(description="Float vs Q16.16 compare ladder")
    ap.add_argument("--tag", choices=("float", "q16"), help="run ladder under this firmware tag")
    ap.add_argument("--analyze", nargs=2, metavar=("FLOAT_JSON", "Q16_JSON"))
    ap.add_argument("--out", default="", help="analyze summary path")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()
    if args.port:
        PORT = args.port

    if args.analyze:
        return analyze(Path(args.analyze[0]), Path(args.analyze[1]),
                       Path(args.out) if args.out else None)
    if not args.tag:
        ap.error("need --tag float|q16 or --analyze")
    return run_ladder(args.tag)


if __name__ == "__main__":
    sys.exit(main())
