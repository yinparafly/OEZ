# -*- coding: utf-8 -*-
"""
第2步：前馈图 / PID 在新负载（齿轮台架 i≈27.744）下的评估闭环 + 霍尔定点减速比精度采集。

串行复用 COM10（com_port_guard 占口），保活 0.7s < HOST_TIMEOUT 1.5s，异常 ESTOP→PWM1000。

阶段（分次运行，每次干净占/放锁）：
  --stage diag     捕获 OLD 状态(PID/GAIN/KA/ADG) + 旧前馈(从 learn CSV) + 闭环诊断(800/1500/2000/2500)
                   同时采集霍尔 0°/180° 定点 gear 多样本
  --stage relearn  LEARN RAMP 重建前馈(RPMMAX 3000) + 应用建议 PID(PID SAVE) + 记录新状态
  --stage verify   用新前馈+PID 闭环验证(800/1500/2000/2500) + 霍尔定点 gear 采集
  --stage report   汇总 diag/relearn/verify JSON → pc/logs/ff_pid_对比_<ts>.md

输出：pc/logs/ 下 JSON / CSV / MD。安全：RPMMAX≤6000。
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import statistics as st
import sys
import time
from datetime import datetime
from pathlib import Path

from auto_learn_test import open_port, send, read_lines, wait_banner
from com_port_guard import acquire
from analyze_learn_log import (
    latest_log,
    parse_learn_points,
    interp_pulse_for_rpm,
    suggest_pid_from_points,
    suggest_pid_zones_from_points,
)

LOG_DIR = Path(__file__).resolve().parent / "logs"
PORT = "COM10"
KEEPALIVE_S = 0.5  # < HOST_TIMEOUT 1.5s，留足余量
SAT_PULSE = 1985

TELEM_MIN_FIELDS = 17


def now_ts() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_telem(line: str) -> dict | None:
    if not line or line.startswith("#"):
        return None
    p = line.split(",")
    if len(p) < TELEM_MIN_FIELDS:
        return None
    try:
        d = {
            "t_ms": float(p[0]),
            "rpm": float(p[4]),
            "pulse": int(float(p[9])),
            "target": float(p[10]),
            "mode": int(float(p[11])),
            "kp": float(p[12]),
            "ki": float(p[13]),
            "kd": float(p[14]),
            "run": int(float(p[16])),
        }
        if len(p) >= 24:
            d["out_hz_f"] = float(p[22])
            d["gear_f"] = float(p[23])
        return d
    except (ValueError, IndexError):
        return None


HALL_DN_RE = re.compile(r"# HALL DOWN .*out_hz=(\S+) gear_meas=(\S+) .*count=(\d+)")
HALL_UP_RE = re.compile(r"# HALL UP .*out_hz=(\S+) gear_meas=(\S+) .*count=(\d+)")
HALL_MEAS_RE = re.compile(
    r"# HALL MEAS out_hz=(\S+) out_rpm=(\S+) gear_meas=(\S+) design=(\S+) "
    r"err_ppm=(\S+) src=(\S+) dn_hz=(\S+) dn_i=(\S+) up_hz=(\S+) up_i=(\S+)"
)


def fnum(s: str) -> float | None:
    try:
        v = float(s)
        return v
    except ValueError:
        return None


def drain(ser, budget=0.15):
    metas, telems, halls = [], [], []
    for ln in read_lines(ser, budget):
        if ln.startswith("#"):
            metas.append(ln)
            m = HALL_DN_RE.search(ln)
            if m:
                g = fnum(m.group(2))
                oh = fnum(m.group(1))
                if g and g > 1.0:
                    halls.append({"src": "dn", "gear": g, "out_hz": oh, "count": int(m.group(3))})
            m = HALL_UP_RE.search(ln)
            if m:
                g = fnum(m.group(2))
                oh = fnum(m.group(1))
                if g and g > 1.0:
                    halls.append({"src": "up", "gear": g, "out_hz": oh, "count": int(m.group(3))})
        else:
            t = parse_telem(ln)
            if t:
                telems.append(t)
    return metas, telems, halls


def query(ser, cmd, budget=0.6):
    drain(ser, 0.05)
    send(ser, cmd)
    metas, _, _ = drain(ser, budget)
    return metas


def safe_stop(ser):
    for c in ("RPM 0", "STOP", "PWM 1000"):
        send(ser, c)
        time.sleep(0.15)
    drain(ser, 0.4)


def estop(ser):
    for c in ("ESTOP", "PWM 1000"):
        send(ser, c)
        time.sleep(0.15)
    drain(ser, 0.4)


def capture_state(ser) -> dict:
    """查询 PID?/GAIN?/KA?/ADG?/SOFT?/HALL? 并解析。"""
    state: dict = {}
    pid = query(ser, "PID?", 0.6)
    state["pid_raw"] = pid
    for ln in pid:
        m = re.search(r"kp=([\d.]+) ki=([\d.]+) kd=([\d.]+)", ln)
        if m:
            state["kp"], state["ki"], state["kd"] = (
                float(m.group(1)),
                float(m.group(2)),
                float(m.group(3)),
            )
    ka = query(ser, "KA?", 0.5)
    state["ka_raw"] = ka
    for ln in ka:
        m = re.search(r"KA=([\d.]+)", ln)
        if m:
            state["ka"] = float(m.group(1))
    adg = query(ser, "ADG?", 0.5)
    state["adg_raw"] = adg
    for ln in adg:
        m = re.search(r"# ADG (\w+) dual=(\w+)", ln)
        if m:
            state["adg_on"] = m.group(1)
            state["adg_dual"] = m.group(2)
    soft = query(ser, "SOFT?", 0.5)
    state["soft_raw"] = soft
    gain = query(ser, "GAIN?", 0.8)
    state["gain_raw"] = gain
    zones = []
    for ln in gain:
        m = re.search(r"# GAIN point rpm=([\d.]+) kp=([\d.]+) ki=([\d.]+) kd=([\d.]+)", ln)
        if m:
            zones.append(
                {
                    "rpm": float(m.group(1)),
                    "kp": float(m.group(2)),
                    "ki": float(m.group(3)),
                    "kd": float(m.group(4)),
                }
            )
        m2 = re.search(r"# GAIN BEGIN .*valid=(\d+) schedule=(\w+)", ln)
        if m2:
            state["gain_valid"] = int(m2.group(1))
            state["gain_schedule"] = m2.group(2)
    state["gain_zones"] = zones
    hall = query(ser, "HALL?", 1.2)
    state["hall_raw"] = hall
    return state


def _seg_metrics(target, hold_s, series, ff_pulse):
    """series: [(el_seg, rpm, pulse, oh, gf)] 单段闭环稳态指标。"""
    if not series:
        return {"target": target, "aborted": "no telem"}
    rpms = [s[1] for s in series]
    pulses = [s[2] for s in series]
    half = [s for s in series if s[0] >= hold_s * 0.5] or series[-max(5, len(series) // 3):]
    mean_rpm = st.mean([s[1] for s in half])
    std_rpm = st.pstdev([s[1] for s in half]) if len(half) > 1 else 0.0
    settle_pulse = st.mean([s[2] for s in half])
    peak_rpm = max(rpms)
    overshoot_pct = max(0.0, (peak_rpm - target) / target * 100.0) if target > 0 else 0.0
    static_err = mean_rpm - target
    u = settle_pulse - ff_pulse if ff_pulse else None
    sat = settle_pulse >= SAT_PULSE or max(pulses) >= 1995
    tol = 0.05 * target
    settle_time = None
    for i, s in enumerate(series):
        if abs(s[1] - target) <= tol and all(abs(series[j][1] - target) <= tol for j in range(i, len(series))):
            settle_time = s[0]
            break
    return {
        "target": target,
        "hold_s": hold_s,
        "n_samples": len(series),
        "mean_rpm": round(mean_rpm, 2),
        "std_rpm": round(std_rpm, 2),
        "peak_rpm": round(peak_rpm, 2),
        "overshoot_pct": round(overshoot_pct, 2),
        "static_err": round(static_err, 2),
        "static_err_pct": round(static_err / target * 100.0, 2) if target > 0 else None,
        "settle_pulse": round(settle_pulse, 1),
        "ff_pulse": round(ff_pulse, 1) if ff_pulse else None,
        "u": round(u, 1) if u is not None else None,
        "saturated": bool(sat),
        "settle_time_s": round(settle_time, 2) if settle_time is not None else None,
    }


def continuous_run(ser, segments, rpmmax, ff_points, phase, w, fp=None, keepalive=KEEPALIVE_S,
                   soft_up=800, soft_down=400):
    """一次连续转动：SOFT 平滑升到首段目标，段间只改 RPM 设定（不停车、不停顿挫点），
    末尾一次性 STOP SOFT 缓降收尾。返回 (每段指标, 全部霍尔样本, aborted)。"""
    safe_stop(ser)
    time.sleep(0.3)
    drain(ser, 0.3)
    for c in ("MODE CLOSED", "SOFT ON", f"SOFT RATE UP {soft_up:.0f}",
              f"SOFT RATE DOWN {soft_down:.0f}", f"RPMMAX {rpmmax:.0f}"):
        send(ser, c)
        time.sleep(0.1)
    drain(ser, 0.2)
    send(ser, "START")
    results = []
    all_halls = []
    aborted = None
    t_run0 = time.time()
    last_ka = t_run0
    last_flush = t_run0
    for tgt, hold in segments:
        send(ser, f"RPM {tgt:.0f}")
        print(f"  -> RPM {tgt:.0f} hold {hold:.0f}s", flush=True)
        seg0 = time.time()
        series = []
        while time.time() - seg0 < hold:
            if time.time() - last_ka >= keepalive:
                last_ka = time.time()
                send(ser, f"RPM {tgt:.0f}")
            if fp is not None and time.time() - last_flush >= 1.0:
                last_flush = time.time()
                fp.flush()
            metas, telems, hs = drain(ser, 0.12)
            for m in metas:
                if "ESTOP" in m and "cleared" not in m and "ACK ESTOP" not in m:
                    aborted = m[:120]
                    break
            if aborted:
                break
            el_run = time.time() - t_run0
            el_seg = time.time() - seg0
            for te in telems:
                series.append((el_seg, abs(te["rpm"]), te["pulse"], te.get("out_hz_f"), te.get("gear_f")))
                w.writerow([phase, f"{tgt:.0f}", f"{el_run:.2f}", f"{abs(te['rpm']):.1f}", te["pulse"],
                            "" if te.get("out_hz_f") is None else f"{te['out_hz_f']:.5f}",
                            "" if te.get("gear_f") is None else f"{te['gear_f']:.5f}"])
            for h in hs:
                all_halls.append((el_run, h))
            time.sleep(0.02)
        ff = interp_pulse_for_rpm(ff_points, tgt) if ff_points else None
        res = _seg_metrics(tgt, hold, series, ff)
        # 段内霍尔定点汇总（本段 out_hz 对应转速稳态）
        res["hall_meas"] = next((ln for ln in query(ser, "HALL?", 1.3) if HALL_MEAS_RE.search(ln)), None)
        results.append(res)
        print("   ", {k: res.get(k) for k in ("mean_rpm", "peak_rpm", "overshoot_pct",
              "static_err", "settle_pulse", "ff_pulse", "u", "saturated", "settle_time_s")})
        if aborted:
            print("   ABORTED:", aborted)
            break
    # 一次性缓降收尾（STOP SOFT → OPEN 斜坡降到 0，不再 RUNNING 故无 host_timeout 风险）
    if not aborted:
        send(ser, "STOP SOFT")
        t = time.time()
        while time.time() - t < 8.0:
            drain(ser, 0.2)
    safe_stop(ser)
    time.sleep(0.4)
    return results, all_halls, aborted


def hall_gear_stats(halls):
    dn = [h["gear"] for _, h in halls if h["src"] == "dn"]
    up = [h["gear"] for _, h in halls if h["src"] == "up"]
    dn_hz = [h["out_hz"] for _, h in halls if h["src"] == "dn" and h["out_hz"]]
    up_hz = [h["out_hz"] for _, h in halls if h["src"] == "up" and h["out_hz"]]

    def stats(v):
        if not v:
            return None
        return {
            "n": len(v),
            "mean": round(st.mean(v), 6),
            "std": round(st.pstdev(v), 6) if len(v) > 1 else 0.0,
            "min": round(min(v), 6),
            "max": round(max(v), 6),
        }

    return {"dn": stats(dn), "up": stats(up), "dn_hz": stats(dn_hz), "up_hz": stats(up_hz),
            "dn_vals": [round(x, 6) for x in dn], "up_vals": [round(x, 6) for x in up]}


# ---------------- stages ----------------
def stage_diag(args):
    acquire("ff_pid_diag")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = now_ts()
    ff_csv = Path(args.ff_csv) if args.ff_csv else latest_log(LOG_DIR)
    old_points = []
    if ff_csv and ff_csv.exists():
        rows = list(csv.DictReader(ff_csv.open(encoding="utf-8")))
        old_points = parse_learn_points(rows)
    print(f"OLD ff CSV: {ff_csv}  points={len(old_points)}")

    ser = open_port(PORT)
    out = {"ts": ts, "stage": "diag", "ff_csv": str(ff_csv), "old_ff_points": old_points}
    try:
        print("banner:", wait_banner(ser))
        safe_stop(ser)
        out["state_old"] = capture_state(ser)
        print("OLD PID:", out["state_old"].get("kp"), out["state_old"].get("ki"),
              out["state_old"].get("kd"), "GAIN sched:", out["state_old"].get("gain_schedule"),
              "zones:", len(out["state_old"].get("gain_zones", [])))

        targets = [float(x) for x in args.targets.split(",")]
        holds = [float(x) for x in args.holds.split(",")]
        segments = list(zip(targets, holds))
        print(f"\n>>> [OLD] 连续转动 segments={segments} (SOFT 平滑, 不停顿挫点)")
        telem_csv = LOG_DIR / f"ff_diag_telem_{ts}.csv"
        with telem_csv.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(["phase", "target", "t_s", "rpm", "pulse", "out_hz_f", "gear_f"])
            probes, all_halls, aborted = continuous_run(ser, segments, args.rpmmax, old_points, "old", w, fp=fp)
        out["probes_old"] = probes
        out["aborted"] = aborted
        out["hall_gear"] = hall_gear_stats(all_halls)
        # 霍尔原始样本 CSV
        hall_csv = LOG_DIR / f"hall_gear_samples_{ts}.csv"
        with hall_csv.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(["t_s", "src", "gear_fixed", "out_hz_fixed", "count"])
            for el, h in all_halls:
                w.writerow([f"{el:.2f}", h["src"], f"{h['gear']:.6f}",
                            "" if h["out_hz"] is None else f"{h['out_hz']:.6f}", h["count"]])
        out["telem_csv"] = str(telem_csv)
        out["hall_csv"] = str(hall_csv)
        safe_stop(ser)
    except Exception as exc:
        estop(ser)
        print("ERROR:", exc)
        out["error"] = str(exc)
        raise
    finally:
        try:
            ser.close()
        except Exception:
            pass
    jp = LOG_DIR / f"ff_eval_diag_{ts}.json"
    jp.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nDIAG JSON: {jp}")
    print("HALL gear:", json.dumps(out["hall_gear"], ensure_ascii=False))
    return 0


def stage_relearn(args):
    acquire("ff_pid_relearn")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = now_ts()
    ser = open_port(PORT)
    out = {"ts": ts, "stage": "relearn", "rpmmax": args.rpmmax}
    learn_csv = LOG_DIR / f"learn_relearn_{ts}.csv"
    try:
        print("banner:", wait_banner(ser))
        safe_stop(ser)
        # LEARN RAMP
        send(ser, "STOP")
        send(ser, "RPM 0")
        send(ser, f"RPMMAX {args.rpmmax:.0f}")
        send(ser, "PROFILE noload")
        time.sleep(0.2)
        drain(ser, 0.3)
        send(ser, "MEASURE RAMP")
        print("LEARN RAMP started, waiting for DONE...")
        fp = learn_csv.open("w", newline="", encoding="utf-8")
        w = csv.writer(fp)
        w.writerow(["wall_s", "t_ms", "kind", "pulse_us", "rpm", "target", "mode", "run", "note"])
        t0 = time.time()
        done = ""
        suggest = {}
        gain_pts = []
        while time.time() - t0 < 120:
            send(ser, "PING")
            metas, telems, _ = drain(ser, 0.15)
            for ln in metas:
                note = ln.lstrip("# ").strip()
                if any(k in ln for k in ("LEARN", "MEASURE", "SUGGEST", "EVAL", "GAIN", "MAP")):
                    kind = "settle" if "LEARN RAMP settle" in ln else "meta"
                    w.writerow([f"{time.time():.3f}", "", kind, "", "", "", "", "", note[:200]])
                    if "LEARN RAMP sample" in ln:
                        mm = re.search(r"pulse=(\d+) rpm=([\d.]+)", ln)
                        if mm:
                            print("  sample", mm.group(1), mm.group(2))
                    m = re.search(r"# SUGGEST gain=([\d.]+) rpm/us  PID kp=([\d.]+) ki=([\d.]+)", ln)
                    if m:
                        suggest = {"gain": float(m.group(1)), "kp": float(m.group(2)), "ki": float(m.group(3))}
                    mg = re.search(r"# GAIN point rpm=([\d.]+) kp=([\d.]+) ki=([\d.]+)", ln)
                    if mg:
                        gain_pts.append({"rpm": float(mg.group(1)), "kp": float(mg.group(2)), "ki": float(mg.group(3))})
                    if "LEARN DONE" in ln:
                        done = note
                if "MAP point" in ln:
                    mp = re.search(r"pulse=(\d+) rpm=([\d.]+)", ln)
                    if mp:
                        w.writerow([f"{time.time():.3f}", "", "meta", "", "", "", "", "",
                                    f"ACK LEARN point pulse={mp.group(1)} rpm={mp.group(2)}"])
            for te in telems:
                w.writerow([f"{time.time():.3f}", te["t_ms"], "telem", te["pulse"], te["rpm"],
                            te["target"], te["mode"], te["run"], ""])
            if done:
                break
        fp.close()
        print("LEARN DONE:", done[:140])
        out["learn_done"] = done
        out["suggest"] = suggest
        out["gain_points_fw"] = gain_pts

        # 应用建议 PID
        if suggest.get("kp"):
            send(ser, f"PID {suggest['kp']:.4f} {suggest['ki']:.4f} 0")
            time.sleep(0.2)
            send(ser, "PID SAVE")
            time.sleep(0.2)
            drain(ser, 0.4)
            print(f"applied PID {suggest['kp']:.4f} {suggest['ki']:.4f} 0 + SAVE")
        # GAIN 固件已自动建表+保存+ON；确认
        out["state_new"] = capture_state(ser)
        out["learn_csv"] = str(learn_csv)
        safe_stop(ser)
    except Exception as exc:
        estop(ser)
        print("ERROR:", exc)
        out["error"] = str(exc)
        try:
            fp.close()
        except Exception:
            pass
        raise
    finally:
        try:
            ser.close()
        except Exception:
            pass
    jp = LOG_DIR / f"ff_eval_relearn_{ts}.json"
    jp.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"RELEARN JSON: {jp}")
    return 0


def stage_verify(args):
    acquire("ff_pid_verify")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = now_ts()
    ff_csv = Path(args.ff_csv) if args.ff_csv else None
    if ff_csv is None:
        cands = sorted(LOG_DIR.glob("learn_relearn_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        ff_csv = cands[0] if cands else latest_log(LOG_DIR)
    new_points = []
    if ff_csv and ff_csv.exists():
        rows = list(csv.DictReader(ff_csv.open(encoding="utf-8")))
        new_points = parse_learn_points(rows)
    print(f"NEW ff CSV: {ff_csv}  points={len(new_points)}")

    ser = open_port(PORT)
    out = {"ts": ts, "stage": "verify", "ff_csv": str(ff_csv), "new_ff_points": new_points}
    try:
        print("banner:", wait_banner(ser))
        safe_stop(ser)
        out["state_new"] = capture_state(ser)
        targets = [float(x) for x in args.targets.split(",")]
        holds = [float(x) for x in args.holds.split(",")]
        segments = list(zip(targets, holds))
        print(f"\n>>> [NEW] 连续转动 segments={segments} (SOFT 平滑, 不停顿挫点)")
        telem_csv = LOG_DIR / f"ff_verify_telem_{ts}.csv"
        with telem_csv.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(["phase", "target", "t_s", "rpm", "pulse", "out_hz_f", "gear_f"])
            probes, all_halls, aborted = continuous_run(ser, segments, args.rpmmax, new_points, "new", w, fp=fp)
        out["probes_new"] = probes
        out["aborted"] = aborted
        out["hall_gear"] = hall_gear_stats(all_halls)
        hall_csv = LOG_DIR / f"hall_gear_samples_verify_{ts}.csv"
        with hall_csv.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(["t_s", "src", "gear_fixed", "out_hz_fixed", "count"])
            for el, h in all_halls:
                w.writerow([f"{el:.2f}", h["src"], f"{h['gear']:.6f}",
                            "" if h["out_hz"] is None else f"{h['out_hz']:.6f}", h["count"]])
        out["telem_csv"] = str(telem_csv)
        out["hall_csv"] = str(hall_csv)
        safe_stop(ser)
    except Exception as exc:
        estop(ser)
        print("ERROR:", exc)
        out["error"] = str(exc)
        raise
    finally:
        try:
            ser.close()
        except Exception:
            pass
    jp = LOG_DIR / f"ff_eval_verify_{ts}.json"
    jp.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"VERIFY JSON: {jp}")
    print("HALL gear:", json.dumps(out["hall_gear"], ensure_ascii=False))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, choices=["diag", "relearn", "verify", "report"])
    ap.add_argument("--targets", default="2000,3000,4000,6000")
    ap.add_argument("--holds", default="35,30,25,20")
    ap.add_argument("--rpmmax", type=float, default=7000.0)
    ap.add_argument("--ff-csv", default="")
    args = ap.parse_args()
    if args.stage == "diag":
        return stage_diag(args)
    if args.stage == "relearn":
        return stage_relearn(args)
    if args.stage == "verify":
        return stage_verify(args)
    print("report stage handled by ff_pid_report.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
