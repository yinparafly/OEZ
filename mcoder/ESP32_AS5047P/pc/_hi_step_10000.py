# -*- coding: utf-8 -*-
"""高速阶梯：闭环平滑升到 10000RPM，稳定保持 ~20s 后安全停车。

- HOST OFF 防卡滞；MODE CLOSED + SOFT ON + RPMMAX 12000；目标 10000；FREQ 400。
- 架构：采集/使用 2kHz、输出 400Hz（勿改 1200）。
- 先门闩 motor_spin_detect @2000；FAIL → ESTOP，禁止升 10000。
- 每 0.5s 重发 RPM 保活（双保险；HOST OFF 已关超时）。
- 稳态采：enc 均值/spread/超调、pulse、饱和、霍尔交叉、LOAD?(overrun/enc_busy)。
- 稳态中抽检 spin_detect；到时或异常 → 平滑降速 → RPM0/STOP/PWM1000；释放锁。
- 只做本档，不自动再升；汇报后问用户。

用法: PYTHONIOENCODING=utf-8 python _hi_step_10000.py
"""
from __future__ import annotations

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
from analyze_learn_log import parse_learn_points, interp_pulse_for_rpm
from motor_spin_detect import detect_spin, require_spinning
from _load_compare import parse_load, fmt_row

PORT = "COM10"
GEAR = 27.744
TARGET = 10000.0
RPMMAX = 12000
SOFT_UP = 900          # rpm/s；2000→10000 ≈ 8.9s
SOFT_DOWN = 700
SETTLE_S = 12.0        # 到该秒后计入稳态
HOLD_S = 33.0          # 段总时长；稳态 [12,33] ≈ 21s ≥ 20s
KEEPALIVE_S = 0.5
POLL_S = 3.0
SAT_PULSE = 1985
SPIN_SPOT_AT = 20.0

LOGDIR = Path(__file__).resolve().parent / "logs"
STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG = LOGDIR / f"hi_step_10000_{STAMP}.txt"
CSVP = LOGDIR / f"hi_step_10000_{STAMP}.csv"
JSONP = LOGDIR / f"hi_step_10000_{STAMP}.json"

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


def enc_query(ser):
    drain(ser, 0.05)
    send(ser, "ENC?")
    end = time.time() + 1.0
    while time.time() < end:
        for ln in drain(ser, 0.15):
            if ln.startswith("# ENC"):
                m = re.search(r"meas_hz=([\d.]+).*?cpu%=([\d.]+).*?loop_hz=([\d.]+)", ln)
                if m:
                    return float(m.group(1)), float(m.group(2)), float(m.group(3))
    return None, None, None


def hall_meas_query(ser):
    drain(ser, 0.05)
    send(ser, "HALL?")
    end = time.time() + 1.4
    hit = None
    while time.time() < end:
        for ln in drain(ser, 0.15):
            if ln.startswith("# HALL MEAS"):
                m = re.search(r"out_hz=([\-\d.]+) out_rpm=([\-\d.]+)", ln)
                if m:
                    hit = (ln, float(m.group(1)), float(m.group(2)))
    return hit


def load_query(ser, tag: str) -> dict | None:
    drain(ser, 0.05)
    send(ser, "LOAD?")
    end = time.time() + 1.2
    raw = ""
    while time.time() < end:
        for ln in drain(ser, 0.15):
            if ln.startswith("# LOAD"):
                raw = ln
                d = parse_load(ln)
                if d is not None:
                    m = re.search(r"enc_ring_overrun=(\d+)", ln, re.I)
                    if m:
                        d["enc_ring_overrun"] = int(m.group(1))
                    else:
                        d["enc_ring_overrun"] = None
                log(fmt_row(tag, d, raw))
                if d is None:
                    log(f"{tag} raw: {raw[:200]}")
                return d
    log(f"{tag}: LOAD? 无响应")
    return None


def safe_stop(ser):
    for c in ("RPM 0", "STOP", "PWM 1000"):
        send(ser, c)
        time.sleep(0.2)
    drain(ser, 0.6)


def estop(ser):
    for c in ("ESTOP", "PWM 1000"):
        send(ser, c)
        time.sleep(0.15)
    drain(ser, 0.4)


def ramp_down(ser):
    """平滑降速再停：不在低速停留、不硬停。"""
    for r in (8500, 6500, 4500, 2500, 1000):
        for _ in range(5):
            send(ser, f"RPM {r}")
            drain(ser, 0.15)
            time.sleep(0.3)
    safe_stop(ser)


def load_ff_points():
    cands = sorted(LOGDIR.glob("learn_relearn_*.csv"),
                   key=lambda p: p.stat().st_mtime, reverse=True)
    # 也认 learn_ramp_*.csv（本轮 LEARN RAMP）
    ramp = sorted(LOGDIR.glob("learn_ramp_*.csv"),
                  key=lambda p: p.stat().st_mtime, reverse=True)
    all_c = list(cands) + list(ramp)
    if not all_c:
        return [], None
    # 取最新修改时间
    all_c.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    rows = list(csv.DictReader(all_c[0].open(encoding="utf-8")))
    return parse_learn_points(rows), all_c[0]


def main():
    acquire("hi_step_10000")
    LOGDIR.mkdir(parents=True, exist_ok=True)

    ff_points, ff_csv = load_ff_points()
    ff_pulse = interp_pulse_for_rpm(ff_points, TARGET) if ff_points else None
    ff_max_rpm = max((r for _, r in ff_points), default=None)
    ff_max_pulse = None
    if ff_points:
        ff_max_pulse = max(ff_points, key=lambda x: x[1])[0]
    ff_11000 = None  # ceiling: no auto next-step

    log(f"=== 高速阶梯 10000RPM  {datetime.now().isoformat(timespec='seconds')} ===")
    log(f"port={PORT} target={TARGET:.0f} rpmmax={RPMMAX} soft_up={SOFT_UP} soft_down={SOFT_DOWN}")
    log(f"hold={HOLD_S:.0f}s settle>={SETTLE_S:.0f}s keepalive={KEEPALIVE_S}s gear={GEAR}")
    log(f"arch: collect/use=2000Hz out=400Hz (FREQ 400)")
    log(f"FF csv={ff_csv} points={len(ff_points)} ff_max={ff_max_rpm}RPM@{ff_max_pulse}us "
        f"ff_pulse@10000={ff_pulse}")
    if ff_max_rpm is not None and ff_max_rpm < TARGET:
        log(f"注意: 前馈图最高仅 {ff_max_rpm:.0f}RPM，9000 超出学习范围（PID 积分补偿），"
            f"ff_pulse 为末点钳位值")

    ser = open_port(PORT)
    csvf = open(CSVP, "w", newline="", encoding="utf-8")
    cw = csv.writer(csvf)
    cw.writerow(["wall_s", "t_seg_s", "t_ms", "enc_rpm", "pulse", "target_ramped",
                 "run", "out_hz", "enc_sample_hz"])
    t_start = time.time()
    last_flush = t_start

    aborted = None
    gate_ok = False
    spot_spin = None
    load_pre = load_idle = load_run = load_post = None
    try:
        drain(ser, 0.6)
        safe_stop(ser)
        time.sleep(0.3)

        log("--- HOST OFF + idle ENC? / LOAD RESET / LOAD? ---")
        send(ser, "HOST OFF")
        time.sleep(0.15)
        for ln in drain(ser, 0.4):
            if ln.startswith("#"):
                log(f"  {ln[:160]}")

        hz0, cpu0, loop0 = enc_query(ser)
        log(f"[IDLE] enc_sample_hz={hz0} cpu%={cpu0} loop_hz={loop0}")
        send(ser, "LOAD RESET")
        time.sleep(0.15)
        drain(ser, 0.3)
        load_idle = load_query(ser, "[LOAD idle]")

        for c in ("STOP", "MODE CLOSED", "SOFT ON", f"SOFT RATE UP {SOFT_UP}",
                  f"SOFT RATE DOWN {SOFT_DOWN}", "FREQ 400", f"RPMMAX {RPMMAX}", "START"):
            send(ser, c)
            time.sleep(0.2)
            acks = [x for x in drain(ser, 0.3) if x.startswith("#")]
            log(f"CMD {c} -> {acks[:3]}")

        # 门闩：先到 2000 并 spin_detect；FAIL 禁止升高速档
        log("\n=== spin_detect 门闩 @2000 ===")
        send(ser, "RPM 2000")
        t_gate = time.time()
        while time.time() - t_gate < 2.5:
            send(ser, "RPM 2000")
            drain(ser, 0.2)
            time.sleep(0.3)
        gate = detect_spin(ser, seconds=3.0, keepalive="RPM 2000")
        log(gate.summary_line())
        if not gate.spinning:
            log("SPIN_DETECT FAIL — ESTOP，禁止升 10000")
            estop(ser)
            safe_stop(ser)
            try:
                require_spinning(gate, label="hi_step_10000_gate")
            except SystemExit as e:
                log(str(e))
                summary = {
                    "stamp": STAMP,
                    "target": TARGET,
                    "gate_ok": False,
                    "aborted": "spin_detect_fail",
                    "gate": gate.summary_line(),
                }
                JSONP.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
                LOG.write_text("\n".join(_out), encoding="utf-8")
                raise SystemExit(3) from e
        gate_ok = True
        load_pre = load_query(ser, "[LOAD @2000 pre-ramp]")

        log(f"\n=== 平滑升到 {TARGET:.0f} 并稳定保持 ===")
        send(ser, "LOAD RESET")
        time.sleep(0.1)
        drain(ser, 0.2)
        send(ser, f"RPM {TARGET:.0f}")
        t_seg = time.time()
        last_ka = 0.0
        last_poll = 0.0
        rpm_win: list[float] = []
        pulse_win: list[int] = []
        enchz_win: list[float] = []
        outhz_win: list[float] = []
        pulse_all: list[int] = []
        peak_rpm = 0.0
        outhz_last = -1.0
        spot_done = False
        load_mid_done = False
        arpm = 0.0

        while time.time() - t_seg < HOLD_S:
            noww = time.time()
            tseg = noww - t_seg
            if noww - last_ka >= KEEPALIVE_S:
                last_ka = noww
                send(ser, f"RPM {TARGET:.0f}")
            for ln in drain(ser, 0.12):
                if ln.startswith("#"):
                    if "ESTOP" in ln and "cleared" not in ln and "ACK ESTOP" not in ln:
                        aborted = ln[:120]
                    continue
                te = parse_telem(ln)
                if not te:
                    continue
                wall = noww - t_start
                cw.writerow([f"{wall:.2f}", f"{tseg:.2f}", int(te['t_ms']),
                             f"{te['rpm']:.1f}", te['pulse'], f"{te['target']:.0f}",
                             te['run'], f"{te['out_hz']:.5f}", f"{te['enc_hz']:.1f}"])
                arpm = abs(te['rpm'])
                pulse_all.append(te['pulse'])
                if arpm > peak_rpm:
                    peak_rpm = arpm
                if te['out_hz'] > 0:
                    outhz_last = te['out_hz']
                if tseg >= SETTLE_S:
                    rpm_win.append(arpm)
                    pulse_win.append(te['pulse'])
                    enchz_win.append(te['enc_hz'])
                    if te['out_hz'] > 0:
                        outhz_win.append(te['out_hz'])
            if aborted:
                break

            if (not spot_done) and tseg >= SPIN_SPOT_AT:
                spot_done = True
                log(f"\n--- 稳态抽检 spin_detect @t={tseg:.1f}s ---")
                spot_spin = detect_spin(ser, seconds=2.5, keepalive=f"RPM {TARGET:.0f}")
                log(spot_spin.summary_line())
                if not spot_spin.spinning:
                    aborted = "spot_spin_detect_fail"
                    log("稳态抽检 FAIL — 准备停车")
                    break
                if not load_mid_done:
                    load_mid_done = True
                    load_run = load_query(ser, f"[LOAD steady t={tseg:.0f}s]")

            if noww - last_poll >= POLL_S:
                last_poll = noww
                hz, cpu, loop = enc_query(ser)
                cur = rpm_win[-1] if rpm_win else arpm
                log(f"[t={tseg:4.1f}s] enc_sample_hz={hz} cpu%={cpu} "
                    f"pulse={pulse_all[-1] if pulse_all else '-'} enc_rpm~{cur:.0f}")
            if noww - last_flush >= 1.0:
                csvf.flush()
                last_flush = noww
            time.sleep(0.02)

        csvf.flush()
        if load_run is None and not aborted:
            load_run = load_query(ser, "[LOAD steady end]")
        hall = hall_meas_query(ser)

        n = len(rpm_win)
        mean_rpm = st.mean(rpm_win) if n else float("nan")
        std_rpm = st.pstdev(rpm_win) if n > 1 else 0.0
        mn = min(rpm_win) if n else float("nan")
        mx = max(rpm_win) if n else float("nan")
        spread = (mx - mn) if n else float("nan")
        mean_pulse = st.mean(pulse_win) if pulse_win else float("nan")
        max_pulse = max(pulse_all) if pulse_all else 0
        mean_enchz = st.mean(enchz_win) if enchz_win else float("nan")
        mean_outhz = st.mean(outhz_win) if outhz_win else (outhz_last if outhz_last > 0 else float("nan"))
        hall_rpm = (mean_outhz * GEAR * 60.0) if mean_outhz == mean_outhz and mean_outhz > 0 else float("nan")
        u = (mean_pulse - ff_pulse) if (ff_pulse and mean_pulse == mean_pulse) else None
        saturated = (mean_pulse == mean_pulse and mean_pulse >= SAT_PULSE) or (max_pulse >= 1995)
        overshoot = max(0.0, (peak_rpm - TARGET) / TARGET * 100.0)
        static_err = (mean_rpm - TARGET) if mean_rpm == mean_rpm else float("nan")
        hall_err_pct = (abs(hall_rpm - mean_rpm) / mean_rpm * 100.0) if (
            hall_rpm == hall_rpm and mean_rpm == mean_rpm and mean_rpm > 0) else float("nan")
        steady_s = (HOLD_S - SETTLE_S)
        reached = (mean_rpm == mean_rpm and abs(static_err) <= 0.05 * TARGET)

        log("\n--- 平滑降速 & 停车 ---")
        if aborted:
            estop(ser)
            safe_stop(ser)
        else:
            ramp_down(ser)
        load_post = load_query(ser, "[LOAD post-stop]")

        def _ld(d, k, default=None):
            return d.get(k, default) if isinstance(d, dict) else default

        overrun_run = _ld(load_run, "ctrl_overrun")
        enc_busy_run = _ld(load_run, "enc_busy_avg_us")
        enc_busy_max_run = _ld(load_run, "enc_busy_max_us")
        ring_ov_run = _ld(load_run, "enc_ring_overrun")
        load_ok = (
            load_run is not None
            and (overrun_run is None or overrun_run == 0)
            and (ring_ov_run is None or ring_ov_run == 0)
        )

        log("\n===== SUMMARY (10000RPM) =====")
        log(f"gate_ok={gate_ok} aborted={aborted}")
        log(f"idle enc_sample_hz={hz0} cpu%={cpu0} loop_hz={loop0}")
        log(f"稳态样本 n={n} 窗口=[{SETTLE_S:.0f},{HOLD_S:.0f}]s (~{steady_s:.0f}s)")
        log(f"编码器 enc_rpm: mean={mean_rpm:.1f} std={std_rpm:.1f} min={mn:.0f} max={mx:.0f} "
            f"spread={spread:.0f} peak={peak_rpm:.0f} overshoot={overshoot:.2f}%")
        if mean_rpm == mean_rpm:
            log(f"静差 static_err={static_err:.1f} ({static_err/TARGET*100:.2f}%)  "
                f"达到10000并稳定={'是' if reached else '否'}")
        log(f"pulse: mean={mean_pulse:.1f} max={max_pulse} 饱和(->2000)={'是' if saturated else '否'} "
            f"距2000余量≈{(2000 - mean_pulse) if mean_pulse == mean_pulse else float('nan'):.0f}us")
        log(f"PID 输出 u=pulse-ff: ff_pulse@10000={ff_pulse} -> u={u}"
            f"  (前馈末点={ff_max_rpm}RPM@{ff_max_pulse}us)")
        log(f"enc_sample_hz(运行中)≈{mean_enchz:.0f}Hz")
        log(f"霍尔交叉: out_hz≈{mean_outhz:.4f} -> hall_rpm=out_hz*27.744*60={hall_rpm:.0f} "
            f"vs enc_mean={mean_rpm:.0f} 误差={hall_err_pct:.2f}%")
        if hall:
            log(f"HALL MEAS: {hall[0]}  (out_rpm={hall[2]:.2f})")
        if spot_spin:
            log(f"稳态抽检: {spot_spin.summary_line()}")
        log(f"LOAD 运行中: enc_busy_avg={enc_busy_run}us max={enc_busy_max_run}us "
            f"ctrl_overrun={overrun_run} enc_ring_overrun={ring_ov_run} load_ok={load_ok}")

        margin_2000 = 2000 - mean_pulse if mean_pulse == mean_pulse else float("nan")
        cover_target = (ff_max_rpm is not None and ff_max_rpm >= TARGET)
        pass_ok = bool(
            reached and not aborted and not saturated and load_ok
            and (margin_2000 == margin_2000 and margin_2000 >= 150)
            and (hall_err_pct != hall_err_pct or hall_err_pct < 5.0)
        )
        log(f"\n顶档判定(10000，不再自动升档): pulse 距 2000 约 {margin_2000:.0f}us; "
            f"前馈图覆盖10000={'是' if cover_target else '否(末端%s RPM)' % (int(ff_max_rpm) if ff_max_rpm else '?')}; "
            f"本档合格={'是' if pass_ok else '否/谨慎'}")
        log("用户决策：保持浮点主路径；禁止自动超过 10000。")

        summary = {
            "stamp": STAMP,
            "target": TARGET,
            "gate_ok": gate_ok,
            "aborted": aborted,
            "reached_and_stable": bool(reached),
            "steady_window_s": [SETTLE_S, HOLD_S],
            "n_samples": n,
            "enc_rpm_mean": round(mean_rpm, 1) if mean_rpm == mean_rpm else None,
            "enc_rpm_std": round(std_rpm, 1),
            "enc_rpm_min": round(mn, 0) if n else None,
            "enc_rpm_max": round(mx, 0) if n else None,
            "spread": round(spread, 0) if n else None,
            "peak_rpm": round(peak_rpm, 0),
            "overshoot_pct": round(overshoot, 2),
            "static_err": round(static_err, 1) if static_err == static_err else None,
            "static_err_pct": round(static_err / TARGET * 100, 2) if static_err == static_err else None,
            "pulse_mean": round(mean_pulse, 1) if mean_pulse == mean_pulse else None,
            "pulse_max": max_pulse,
            "saturated": bool(saturated),
            "pulse_margin_to_2000": round(margin_2000, 0) if margin_2000 == margin_2000 else None,
            "ff_pulse_at_10000": ff_pulse,
            "ff_max_rpm": ff_max_rpm,
            "ff_max_pulse": ff_max_pulse,
            "ff_covers_10000": bool(cover_target),
            "pid_u": round(u, 1) if u is not None else None,
            "enc_sample_hz_run": round(mean_enchz, 0) if mean_enchz == mean_enchz else None,
            "enc_sample_hz_idle": hz0,
            "out_hz_mean": round(mean_outhz, 5) if mean_outhz == mean_outhz else None,
            "hall_rpm": round(hall_rpm, 0) if hall_rpm == hall_rpm else None,
            "hall_err_pct": round(hall_err_pct, 2) if hall_err_pct == hall_err_pct else None,
            "hall_meas_raw": hall[0] if hall else None,
            "spot_spin": spot_spin.summary_line() if spot_spin else None,
            "load_idle": load_idle,
            "load_pre_2000": load_pre,
            "load_run": load_run,
            "load_post": load_post,
            "load_ok": load_ok,
            "pass_ok": pass_ok,
            "ceiling_rpm": 10000,
            "csv": str(CSVP),
        }
        JSONP.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        LOG.write_text("\n".join(_out), encoding="utf-8")
        log(f"\nlog={LOG}")
        log(f"csv={CSVP}")
        log(f"json={JSONP}")
        return 0 if (reached and not aborted and gate_ok) else 2
    except SystemExit:
        raise
    except Exception as exc:
        try:
            estop(ser)
            safe_stop(ser)
        except Exception:
            pass
        log(f"ERROR: {exc}")
        LOG.write_text("\n".join(_out), encoding="utf-8")
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


if __name__ == "__main__":
    sys.exit(main())
