# -*- coding: utf-8 -*-
"""同采样双算法：float vs Q16.16 求速精度对比（正确方法）。

方法（与「双烧闭环跟踪误差」无关）：
  同一 (t_us, counts) 序列 → 浮点窗差+EMA → Q16.16 窗差+EMA → 逐点 |Δrpm|

用法:
  python compare_q16_same_sample.py --synthetic          # 合成 2kHz 序列（无电机）
  python compare_q16_same_sample.py --csv raw.csv        # 离线 CSV: t_us,counts
  python compare_q16_same_sample.py --hw                 # 需已烧 Q16 影子固件; COM10 阶梯+Q16CMP?

旧脚本 compare_float_q16.py（双烧闭环 Δmean）对「计算精度」方法论无效，勿作主证据。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import struct
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

os.environ.setdefault("PYTHONIOENCODING", "utf-8")

LOGDIR = Path(__file__).resolve().parent / "logs"
ENC_CPR = 16384
ENC_VEL_WINDOW = 8
ENC_SAMPLE_HZ = 2000
ENC_PERIOD_US = 500
EMA_ALPHA = 0.10
SIGN_FLIP_SAMPLES = ENC_SAMPLE_HZ // 12  # 166 @2k
PASS_ABS_MEAN = 0.5
PASS_ABS_MAX = 2.0  # 同采样瞬时尖刺允许略宽；均值仍 <0.5
Q16_SCALE = 1 << 16


def float_to_q16(x: float) -> int:
    return int(x * Q16_SCALE + (0.5 if x >= 0 else -0.5))


def q16_to_float(x: int) -> float:
    return float(x) / float(Q16_SCALE)


def q16_mul(a: int, b: int) -> int:
    return (a * b) >> 16


def q16_abs(x: int) -> int:
    return -x if x < 0 else x


def calc_rpm_window_q16(dc: int, dt_us: int, cpr: int = ENC_CPR) -> int:
    if dt_us == 0 or cpr == 0:
        return 0
    num = dc * 60_000_000
    den = cpr * dt_us
    if den == 0:
        return 0
    rpm_q16 = (num << 16) // den
    if rpm_q16 > 0x7FFFFFFF:
        return 0x7FFFFFFF
    if rpm_q16 < -0x80000000:
        return -0x80000000
    return int(rpm_q16)


def calc_rpm_window_float(dc: int, dt_us: int, cpr: int = ENC_CPR) -> float:
    if dt_us <= 0 or cpr == 0:
        return 0.0
    return float(dc) * 60_000_000.0 / (float(cpr) * float(dt_us))


@dataclass
class DualState:
    hist_counts: list
    hist_us: list
    head: int
    fill: int
    rpm_filt_f: float
    rpm_stable_f: float
    sign_lock_f: int
    flip_f: int
    rpm_filt_q: int
    rpm_stable_q: int
    sign_lock_q: int
    flip_q: int
    rpm_max_lim: float = 12000.0 * 1.5 + 500.0


def dual_reset(lim: float = 18500.0) -> DualState:
    return DualState(
        hist_counts=[0] * 16,
        hist_us=[0] * 16,
        head=0,
        fill=0,
        rpm_filt_f=0.0,
        rpm_stable_f=0.0,
        sign_lock_f=0,
        flip_f=0,
        rpm_filt_q=0,
        rpm_stable_q=0,
        sign_lock_q=0,
        flip_q=0,
        rpm_max_lim=lim,
    )


def _apply_sign(mag: float, sgn: int, lock: int, flip: int):
    if lock == 0:
        if sgn != 0:
            lock = sgn
    elif sgn != 0 and sgn != lock:
        flip += 1
        if flip >= SIGN_FLIP_SAMPLES:
            lock = sgn
            flip = 0
    else:
        flip = 0
    use = lock if lock != 0 else sgn
    if use == 0:
        use = 1
    return float(use) * mag, lock, flip


def dual_step(st: DualState, t_us: int, counts: int) -> tuple[float, float, float]:
    """One sample: returns (rpm_float_stable, rpm_q16_stable, abs_delta)."""
    cur = st.head
    st.hist_counts[cur] = counts
    st.hist_us[cur] = t_us & 0xFFFFFFFF
    st.head = (st.head + 1) % 16
    if st.fill < 16:
        st.fill += 1

    w = ENC_VEL_WINDOW
    if w > st.fill - 1:
        w = st.fill - 1

    rpm_inst_f = st.rpm_filt_f
    rpm_inst_q = st.rpm_filt_q
    if w >= 1:
        idx = (cur - w + 16) % 16
        dc = counts - st.hist_counts[idx]
        dt_us = (t_us - st.hist_us[idx]) & 0xFFFFFFFF
        if dt_us > 0:
            rpm_inst_f = calc_rpm_window_float(dc, dt_us)
            rpm_inst_q = calc_rpm_window_q16(dc, dt_us)

    lim = st.rpm_max_lim
    if abs(rpm_inst_f) > lim:
        rpm_inst_f = st.rpm_filt_f
    lim_q = float_to_q16(lim)
    if q16_abs(rpm_inst_q) > lim_q:
        rpm_inst_q = st.rpm_filt_q

    st.rpm_filt_f = 0.90 * st.rpm_filt_f + 0.10 * rpm_inst_f
    alpha_q = float_to_q16(EMA_ALPHA)
    one_m = float_to_q16(1.0) - alpha_q
    st.rpm_filt_q = q16_mul(one_m, st.rpm_filt_q) + q16_mul(alpha_q, rpm_inst_q)

    mag_f = abs(st.rpm_filt_f)
    sgn_f = 1 if st.rpm_filt_f > 0 else (-1 if st.rpm_filt_f < 0 else 0)
    if mag_f < 80.0:
        st.sign_lock_f = 0
        st.flip_f = 0
        st.rpm_stable_f = st.rpm_filt_f
    else:
        st.rpm_stable_f, st.sign_lock_f, st.flip_f = _apply_sign(
            mag_f, sgn_f, st.sign_lock_f, st.flip_f
        )

    mag_q = q16_to_float(q16_abs(st.rpm_filt_q))
    sgn_q = 1 if st.rpm_filt_q > 0 else (-1 if st.rpm_filt_q < 0 else 0)
    if mag_q < 80.0:
        st.sign_lock_q = 0
        st.flip_q = 0
        st.rpm_stable_q = st.rpm_filt_q
    else:
        stable_f, st.sign_lock_q, st.flip_q = _apply_sign(
            mag_q, sgn_q, st.sign_lock_q, st.flip_q
        )
        st.rpm_stable_q = float_to_q16(stable_f)

    rf = st.rpm_stable_f
    rq = q16_to_float(st.rpm_stable_q)
    return rf, rq, abs(rf - rq)


def run_series(samples: list[tuple[int, int]], lim: float = 18500.0) -> dict:
    st = dual_reset(lim)
    deltas: list[float] = []
    rpm_f_list: list[float] = []
    rpm_q_list: list[float] = []
    # skip first window fill for stats (warmup)
    warmup = ENC_VEL_WINDOW + 2
    for i, (t_us, counts) in enumerate(samples):
        rf, rq, d = dual_step(st, int(t_us), int(counts))
        if i >= warmup:
            deltas.append(d)
            rpm_f_list.append(rf)
            rpm_q_list.append(rq)

    n = len(deltas)
    if n == 0:
        return {"n": 0, "mean_abs": None, "max_abs": None, "pass": False}

    mean_abs = sum(deltas) / n
    max_abs = max(deltas)
    mean_f = sum(rpm_f_list) / n
    mean_q = sum(rpm_q_list) / n
    ok = (mean_abs < PASS_ABS_MEAN) and (max_abs < PASS_ABS_MAX)
    return {
        "n": n,
        "mean_abs_drpm": round(mean_abs, 6),
        "max_abs_drpm": round(max_abs, 6),
        "p99_abs_drpm": round(sorted(deltas)[int(0.99 * (n - 1))], 6) if n > 1 else round(max_abs, 6),
        "mean_rpm_float": round(mean_f, 3),
        "mean_rpm_q16": round(mean_q, 3),
        "mean_signed_diff": round(mean_q - mean_f, 6),
        "pass_mean_lt_0p5": mean_abs < PASS_ABS_MEAN,
        "pass_max_lt_2": max_abs < PASS_ABS_MAX,
        "pass": ok,
        "criterion": f"mean|Δ|<{PASS_ABS_MEAN} and max|Δ|<{PASS_ABS_MAX}",
    }


def synth_series(rpm: float, seconds: float = 1.0, jitter_us: int = 0) -> list[tuple[int, int]]:
    """Ideal constant-RPM unwrapped counts @2kHz."""
    n = int(seconds * ENC_SAMPLE_HZ)
    out: list[tuple[int, int]] = []
    counts = 0.0
    t = 0
    # counts per sample = rpm/60 * CPR / fs
    dcounts = (rpm / 60.0) * ENC_CPR / ENC_SAMPLE_HZ
    import random
    for i in range(n):
        dt = ENC_PERIOD_US
        if jitter_us:
            dt += random.randint(-jitter_us, jitter_us)
            if dt < 100:
                dt = 100
        t += dt
        counts += dcounts
        out.append((t, int(round(counts))))
    return out


def load_csv(path: Path) -> list[tuple[int, int]]:
    rows: list[tuple[int, int]] = []
    with path.open(newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        if r.fieldnames and "t_us" in r.fieldnames and "counts" in r.fieldnames:
            for row in r:
                rows.append((int(float(row["t_us"])), int(float(row["counts"]))))
            return rows
    # fallback: two columns
    with path.open(newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        for row in r:
            if not row or row[0].startswith("#") or row[0] == "t_us":
                continue
            rows.append((int(float(row[0])), int(float(row[1]))))
    return rows


def write_report(tag: str, results: list[dict], extra: dict | None = None) -> Path:
    LOGDIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = LOGDIR / f"q16_same_sample_{tag}_{stamp}"
    summary = {
        "method": "same_raw_samples_dual_algorithm",
        "invalid_method_note": (
            "dual-flash closed-loop tracking Δmean (compare_float_q16.py / "
            "q16_cmp_*_20260722_174*) is NOT valid for Q16 vs float calc accuracy"
        ),
        "stamp": stamp,
        "tag": tag,
        "points": results,
        "extra": extra or {},
    }
    all_pass = all(p.get("pass") for p in results if p.get("n", 0) > 0)
    max_mean = max((p.get("mean_abs_drpm") or 0) for p in results) if results else None
    max_max = max((p.get("max_abs_drpm") or 0) for p in results) if results else None
    summary["verdict"] = "PASS" if all_pass and results else "FAIL"
    summary["worst_mean_abs"] = max_mean
    summary["worst_max_abs"] = max_max

    lines = [
        f"=== Q16 same-sample compare tag={tag} {stamp} ===",
        "method: SAME (t_us,counts) → float window+EMA vs Q16.16 window+EMA",
        "NOT dual-flash closed-loop tracking error",
        f"criterion: {PASS_ABS_MEAN=} {PASS_ABS_MAX=}",
        "",
        f"{'label':>12} {'n':>8} {'mean|Δ|':>12} {'max|Δ|':>12} {'mean_f':>10} {'mean_q':>10} {'pass':>6}",
    ]
    for p in results:
        lines.append(
            f"{p.get('label','?'):>12} {p.get('n',0):8d} "
            f"{p.get('mean_abs_drpm') or 0:12.6f} {p.get('max_abs_drpm') or 0:12.6f} "
            f"{p.get('mean_rpm_float') or 0:10.2f} {p.get('mean_rpm_q16') or 0:10.2f} "
            f"{'PASS' if p.get('pass') else 'FAIL':>6}"
        )
    lines.append("")
    lines.append(
        f"SUMMARY: {summary['verdict']}  worst_mean|Δ|={max_mean}  worst_max|Δ|={max_max}"
    )
    text = "\n".join(lines) + "\n"
    print(text, flush=True)
    (base.with_suffix(".txt")).write_text(text, encoding="utf-8")
    (base.with_suffix(".json")).write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"wrote {base}.txt / .json", flush=True)
    return base


def run_synthetic(rpms=(2000.0, 3000.0, 4000.0), seconds=1.0) -> Path:
    results = []
    for rpm in rpms:
        samples = synth_series(rpm, seconds=seconds, jitter_us=0)
        r = run_series(samples)
        r["label"] = f"synth_{int(rpm)}"
        r["target_rpm"] = rpm
        results.append(r)
        # with timing jitter
        samples_j = synth_series(rpm, seconds=seconds, jitter_us=5)
        rj = run_series(samples_j)
        rj["label"] = f"jitter_{int(rpm)}"
        rj["target_rpm"] = rpm
        results.append(rj)
    return write_report("synth", results)


def run_csv(path: Path) -> Path:
    samples = load_csv(path)
    r = run_series(samples)
    r["label"] = path.stem
    r["source"] = str(path)
    return write_report("csv", [r], extra={"csv": str(path)})


def run_hw(port: str = "COM10", targets=(2000.0, 3000.0, 4000.0)) -> Path:
    from auto_learn_test import open_port, send, read_lines
    from com_port_guard import acquire, release
    from motor_spin_detect import detect_spin

    acquire("q16_same_sample_hw")
    ser = open_port(port)
    results = []
    log_lines: list[str] = []

    def drain(budget=0.15):
        return read_lines(ser, budget)

    def qline(cmd, prefix, timeout=1.5):
        drain(0.05)
        send(ser, cmd)
        end = time.time() + timeout
        while time.time() < end:
            for ln in drain(0.15):
                if ln.startswith(prefix):
                    return ln
        return ""

    try:
        drain(0.5)
        send(ser, "HOST OFF")
        time.sleep(0.15)
        drain(0.3)
        fx = qline("FIXED?", "# FIXED")
        cmp0 = qline("Q16CMP?", "# Q16CMP")
        log_lines.append(fx)
        log_lines.append(cmp0)
        if "shadow=1" not in cmp0 and "shadow=on" not in cmp0.lower():
            # also accept n= with mean if present
            if "mean_abs" not in cmp0 and "mean|d|=" not in cmp0.replace(" ", ""):
                raise SystemExit(
                    f"Board lacks Q16CMP shadow (got: {cmp0!r}). "
                    "Flash firmware with Q16_SHADOW_COMPARE=1 first."
                )

        for c in (
            "STOP", "MODE CLOSED", "SOFT ON",
            "SOFT RATE UP 900", "SOFT RATE DOWN 700",
            "FREQ 400", "RPMMAX 12000", "TELEM ON", "START",
        ):
            send(ser, c)
            time.sleep(0.12)
            drain(0.2)

        send(ser, "RPM 2000")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            send(ser, "RPM 2000")
            drain(0.15)
            time.sleep(0.25)
        gate = detect_spin(ser, seconds=3.0, keepalive="RPM 2000")
        log_lines.append(gate.summary_line())
        if not gate.spinning:
            send(ser, "ESTOP")
            raise SystemExit("spin_detect fail")

        for tgt in targets:
            # ramp/hold first so float & Q16 shadow EMA stay locked together
            send(ser, f"RPM {tgt:.0f}")
            t_hold = time.time()
            while time.time() - t_hold < 10.0:
                send(ser, f"RPM {tgt:.0f}")
                drain(0.12)
                time.sleep(0.35)
            # stats-only reset (must NOT zero shadow EMA — that desyncs vs float)
            send(ser, "Q16CMP RESET")
            time.sleep(0.05)
            drain(0.1)
            t_m = time.time()
            while time.time() - t_m < 3.0:
                send(ser, f"RPM {tgt:.0f}")
                drain(0.12)
                time.sleep(0.3)
            ln = qline("Q16CMP?", "# Q16CMP")
            log_lines.append(ln)
            # parse: # Q16CMP shadow=1 n=... mean_abs=... max_abs=... rpm_f=... rpm_q=...
            m_n = re.search(r"\bn=(\d+)", ln)
            m_mean = re.search(r"mean_abs=([0-9.eE+-]+)", ln)
            m_max = re.search(r"max_abs=([0-9.eE+-]+)", ln)
            m_rf = re.search(r"rpm_f=([0-9.eE+-]+)", ln)
            m_rq = re.search(r"rpm_q=([0-9.eE+-]+)", ln)
            mean_abs = float(m_mean.group(1)) if m_mean else None
            max_abs = float(m_max.group(1)) if m_max else None
            n = int(m_n.group(1)) if m_n else 0
            ok = (
                mean_abs is not None
                and max_abs is not None
                and n > 100
                and mean_abs < PASS_ABS_MEAN
                and max_abs < PASS_ABS_MAX
            )
            results.append({
                "label": f"hw_{int(tgt)}",
                "target_rpm": tgt,
                "n": n,
                "mean_abs_drpm": mean_abs,
                "max_abs_drpm": max_abs,
                "mean_rpm_float": float(m_rf.group(1)) if m_rf else None,
                "mean_rpm_q16": float(m_rq.group(1)) if m_rq else None,
                "pass": ok,
                "raw": ln,
            })

        for r in (3500, 2500, 1500):
            send(ser, f"RPM {r}")
            time.sleep(0.4)
            drain(0.15)
        send(ser, "RPM 0")
        send(ser, "STOP")
        send(ser, "PWM 1000")
        drain(0.4)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        release()

    base = write_report("hw", results, extra={"log_lines": log_lines})
    (base.with_name(base.name + "_hwlog.txt")).write_text(
        "\n".join(log_lines) + "\n", encoding="utf-8"
    )
    return base


def main() -> int:
    ap = argparse.ArgumentParser(description="Same-sample float vs Q16.16 compare")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--synthetic", action="store_true")
    g.add_argument("--csv", type=str, default="")
    g.add_argument("--hw", action="store_true")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--seconds", type=float, default=1.0)
    args = ap.parse_args()
    if args.synthetic:
        p = run_synthetic(seconds=args.seconds)
        data = json.loads(p.with_suffix(".json").read_text(encoding="utf-8"))
        return 0 if data.get("verdict") == "PASS" else 1
    if args.csv:
        p = run_csv(Path(args.csv))
        data = json.loads(p.with_suffix(".json").read_text(encoding="utf-8"))
        return 0 if data.get("verdict") == "PASS" else 1
    if args.hw:
        p = run_hw(port=args.port)
        data = json.loads(p.with_suffix(".json").read_text(encoding="utf-8"))
        return 0 if data.get("verdict") == "PASS" else 1
    return 2


if __name__ == "__main__":
    sys.exit(main())
