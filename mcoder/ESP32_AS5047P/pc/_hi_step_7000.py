# -*- coding: utf-8 -*-
"""高速阶梯测试（第一档）：闭环平滑升到 7000RPM，稳定保持 ~20s 后安全停车。

- MODE CLOSED + SOFT ON + RPMMAX 12000（≥8000）；目标 7000RPM。
- 后台独立进程运行；每 0.5s 重发 RPM 保活（<HOST_TIMEOUT 1.5s）防 ESTOP。
- CSV 每秒 flush。采集：编码器转速均值/spread、pulse、PID 输出 u(=pulse-ff)、
  是否饱和(pulse→2000)、霍尔盘频交叉核对(out_hz*27.744*60)、enc_sample_hz。
- 到时或异常 → 安全停车：平滑降速 → RPM 0 / STOP / PWM 1000；释放 COM 锁。
- 只做第一档（7000），做完写报告即停，不自动进入下一档。

用法: PYTHONIOENCODING=utf-8 python _hi_step_7000.py
"""
from __future__ import annotations
import csv
import json
import re
import statistics as st
import sys
import time
from datetime import datetime
from pathlib import Path

from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire
from analyze_learn_log import parse_learn_points, interp_pulse_for_rpm

PORT = "COM10"
GEAR = 27.744
TARGET = 7000.0
RPMMAX = 12000
SOFT_UP = 900          # rpm/s 平滑升
SOFT_DOWN = 700        # rpm/s 平滑降
SETTLE_S = 11.0        # 到该秒后计入稳态窗口（0→7000 @900rpm/s 约 7.8s + 余量）
HOLD_S = 32.0          # 段总时长；稳态窗口 [SETTLE_S, HOLD_S] ≈ 21s ≥ 20s
KEEPALIVE_S = 0.5      # < HOST_TIMEOUT_MS(1500)
POLL_S = 3.0           # ENC?/HALL? 轮询
SAT_PULSE = 1985       # 饱和判据（接近 2000）

LOGDIR = Path(__file__).resolve().parent / "logs"
STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG = LOGDIR / f"hi_step_7000_{STAMP}.txt"
CSVP = LOGDIR / f"hi_step_7000_{STAMP}.csv"
JSONP = LOGDIR / f"hi_step_7000_{STAMP}.json"

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
    for r in (5000, 3500, 2000, 1000):
        for _ in range(5):
            send(ser, f"RPM {r}")
            drain(ser, 0.15)
            time.sleep(0.3)
    safe_stop(ser)


def load_ff_points():
    cands = sorted(LOGDIR.glob("learn_relearn_*.csv"),
                   key=lambda p: p.stat().st_mtime, reverse=True)
    if not cands:
        return [], None
    rows = list(csv.DictReader(cands[0].open(encoding="utf-8")))
    return parse_learn_points(rows), cands[0]


def main():
    acquire("hi_step_7000")
    LOGDIR.mkdir(parents=True, exist_ok=True)

    ff_points, ff_csv = load_ff_points()
    ff_pulse = interp_pulse_for_rpm(ff_points, TARGET) if ff_points else None
    ff_max_rpm = max((r for _, r in ff_points), default=None)
    ff_max_pulse = None
    if ff_points:
        ff_max_pulse = max(ff_points, key=lambda x: x[1])[0]
    ff_7500 = interp_pulse_for_rpm(ff_points, 7500.0) if ff_points else None

    log(f"=== 高速阶梯 第一档 7000RPM  {datetime.now().isoformat(timespec='seconds')} ===")
    log(f"port={PORT} target={TARGET:.0f} rpmmax={RPMMAX} soft_up={SOFT_UP} soft_down={SOFT_DOWN}")
    log(f"hold={HOLD_S:.0f}s settle>={SETTLE_S:.0f}s keepalive={KEEPALIVE_S}s gear={GEAR}")
    log(f"FF csv={ff_csv} points={len(ff_points)} ff_max={ff_max_rpm}RPM@{ff_max_pulse}us "
        f"ff_pulse@7000={ff_pulse} ff_pulse@7500(clamped)={ff_7500}")
    if ff_max_rpm is not None and ff_max_rpm < 7000:
        log(f"注意: 前馈图最高仅 {ff_max_rpm:.0f}RPM，7000/7500 超出学习范围（PID 积分补偿），"
            f"ff_pulse 为末点钳位值")

    ser = open_port(PORT)
    csvf = open(CSVP, "w", newline="", encoding="utf-8")
    cw = csv.writer(csvf)
    cw.writerow(["wall_s", "t_seg_s", "t_ms", "enc_rpm", "pulse", "target_ramped",
                 "run", "out_hz", "enc_sample_hz"])
    t_start = time.time()
    last_flush = t_start

    aborted = None
    try:
        drain(ser, 0.6)
        safe_stop(ser)
        time.sleep(0.3)

        log("--- idle ENC? (电机停) ---")
        hz0, cpu0, loop0 = enc_query(ser)
        log(f"[IDLE] enc_sample_hz={hz0} cpu%={cpu0} loop_hz={loop0}")

        for c in ("STOP", "MODE CLOSED", "SOFT ON", f"SOFT RATE UP {SOFT_UP}",
                  f"SOFT RATE DOWN {SOFT_DOWN}", f"RPMMAX {RPMMAX}", "START"):
            send(ser, c)
            time.sleep(0.2)
            acks = [x for x in drain(ser, 0.3) if x.startswith("#")]
            log(f"CMD {c} -> {acks[:3]}")

        log(f"\n=== 平滑升到 {TARGET:.0f} 并稳定保持 ===")
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

        while time.time() - t_seg < HOLD_S:
            noww = time.time()
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
                tseg = noww - t_seg
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
            if noww - last_poll >= POLL_S:
                last_poll = noww
                hz, cpu, loop = enc_query(ser)
                cur = rpm_win[-1] if rpm_win else (arpm if 'arpm' in dir() else 0)
                log(f"[t={noww - t_seg:4.1f}s] enc_sample_hz={hz} cpu%={cpu} "
                    f"pulse={pulse_all[-1] if pulse_all else '-'} enc_rpm~{cur:.0f}")
            if noww - last_flush >= 1.0:
                csvf.flush()
                last_flush = noww
            time.sleep(0.02)

        csvf.flush()
        hall = hall_meas_query(ser)

        # ---- 统计 ----
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
        # 稳态窗口时长
        steady_s = (HOLD_S - SETTLE_S)
        reached = (mean_rpm == mean_rpm and abs(static_err) <= 0.05 * TARGET)

        # ---- 平滑降速停车 ----
        log("\n--- 平滑降速 & 停车 ---")
        if aborted:
            estop(ser)
        else:
            ramp_down(ser)

        log("\n===== SUMMARY (第一档 7000RPM) =====")
        log(f"aborted={aborted}")
        log(f"idle enc_sample_hz={hz0} cpu%={cpu0} loop_hz={loop0}")
        log(f"稳态样本 n={n} 窗口=[{SETTLE_S:.0f},{HOLD_S:.0f}]s (~{steady_s:.0f}s)")
        log(f"编码器 enc_rpm: mean={mean_rpm:.1f} std={std_rpm:.1f} min={mn:.0f} max={mx:.0f} "
            f"spread={spread:.0f} peak={peak_rpm:.0f} overshoot={overshoot:.2f}%")
        log(f"静差 static_err={static_err:.1f} ({static_err/TARGET*100:.2f}%)  达到7000并稳定={'是' if reached else '否'}")
        log(f"pulse: mean={mean_pulse:.1f} max={max_pulse} 饱和(->2000)={'是' if saturated else '否'} "
            f"距2000余量≈{2000 - mean_pulse:.0f}us")
        log(f"PID 输出 u=pulse-ff: ff_pulse@7000={ff_pulse} -> u={u}"
            f"  (前馈末点={ff_max_rpm}RPM@{ff_max_pulse}us, 7000 超出前馈图)")
        log(f"enc_sample_hz(运行中)≈{mean_enchz:.0f}Hz")
        log(f"霍尔交叉: out_hz≈{mean_outhz:.4f} -> hall_rpm=out_hz*27.744*60={hall_rpm:.0f} "
            f"vs enc_mean={mean_rpm:.0f} 误差={hall_err_pct:.2f}%")
        if hall:
            log(f"HALL MEAS: {hall[0]}  (out_rpm={hall[2]:.2f})")
        # 余量判断
        margin_2000 = 2000 - mean_pulse if mean_pulse == mean_pulse else float("nan")
        cover_7500 = (ff_max_rpm is not None and ff_max_rpm >= 7500)
        log(f"\n余量判断(是否可继续 7500): pulse 距 2000 约 {margin_2000:.0f}us; "
            f"前馈图覆盖7500={'是' if cover_7500 else '否(末点%s RPM)' % (int(ff_max_rpm) if ff_max_rpm else '?')}")

        summary = {
            "stamp": STAMP,
            "target": TARGET,
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
            "ff_pulse_at_7000": ff_pulse,
            "ff_max_rpm": ff_max_rpm,
            "ff_max_pulse": ff_max_pulse,
            "ff_covers_7500": bool(cover_7500),
            "pid_u": round(u, 1) if u is not None else None,
            "enc_sample_hz_run": round(mean_enchz, 0) if mean_enchz == mean_enchz else None,
            "enc_sample_hz_idle": hz0,
            "out_hz_mean": round(mean_outhz, 5) if mean_outhz == mean_outhz else None,
            "hall_rpm": round(hall_rpm, 0) if hall_rpm == hall_rpm else None,
            "hall_err_pct": round(hall_err_pct, 2) if hall_err_pct == hall_err_pct else None,
            "hall_meas_raw": hall[0] if hall else None,
            "csv": str(CSVP),
        }
        JSONP.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        LOG.write_text("\n".join(_out), encoding="utf-8")
        log(f"\nlog={LOG}")
        log(f"csv={CSVP}")
        log(f"json={JSONP}")
        return 0 if (reached and not aborted) else 2
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
            csvf.flush(); csvf.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
