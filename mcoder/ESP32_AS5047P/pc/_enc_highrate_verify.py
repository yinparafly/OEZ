# -*- coding: utf-8 -*-
"""高频编码器采样验证：enc_sample_hz≥1000 + 分段升速(2000→4000→6000) 不折叠 + 霍尔交叉核对。

安全：目标≥2000RPM，平滑升降不频繁低速启停；每 0.5s 重发 RPM 保活(<HOST_TIMEOUT 1.5s)；
CSV 每秒 flush；异常/结束 ESTOP→STOP→PWM 1000。日志/CSV 写 pc/logs/。

用法: PYTHONIOENCODING=utf-8 python _enc_highrate_verify.py
"""
from __future__ import annotations
import csv
import re
import sys
import time
from datetime import datetime
from pathlib import Path

from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire

PORT = "COM10"
GEAR = 27.744
STAGES = [2000, 4000, 6000]     # 分段目标转速
HOLD_S = 14.0                    # 每段稳态保持
KEEPALIVE_S = 0.5               # < HOST_TIMEOUT_MS(1500)
POLL_S = 3.0                     # ENC?/HALL? 轮询
LOGDIR = Path(__file__).resolve().parent / "logs"
STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG = LOGDIR / f"enc_highrate_{STAMP}.txt"
CSVP = LOGDIR / f"enc_highrate_{STAMP}.csv"

_out = []
def log(s: str) -> None:
    print(s, flush=True)
    _out.append(s)


def drain(ser, budget=0.2):
    return read_lines(ser, budget)


def parse_telem(line: str):
    if not line or line.startswith("#"):
        return None
    p = line.split(",")
    if len(p) < 25:  # 新增 enc_sample_hz 后共 25 列
        return None
    try:
        return {
            "t_ms": float(p[0]),
            "rpm": float(p[4]),
            "pulse": int(float(p[9])),
            "run": int(float(p[16])),
            "out_hz": float(p[22]),
            "enc_hz": float(p[24]),
        }
    except (ValueError, IndexError):
        return None


def query(ser, cmd, tag, timeout=1.2):
    drain(ser, 0.05)
    send(ser, cmd)
    end = time.time() + timeout
    hit = []
    while time.time() < end:
        for ln in drain(ser, 0.15):
            if ln.startswith(tag):
                hit.append(ln)
    for h in hit:
        log(f"   {h}")
    return hit


def enc_query(ser):
    hits = query(ser, "ENC?", "# ENC")
    for h in hits:
        m = re.search(r"meas_hz=([\d.]+).*?cpu%=([\d.]+).*?loop_hz=([\d.]+)", h)
        if m:
            return float(m.group(1)), float(m.group(2)), float(m.group(3))
    return None, None, None


def hall_query(ser):
    hits = query(ser, "# HALL MEAS", "# HALL", timeout=1.4)
    for h in hits:
        if h.startswith("# HALL MEAS"):
            m = re.search(r"out_rpm=([\-\d.]+)", h)
            if m:
                return float(m.group(1))
    return None


def safe_stop(ser):
    for c in ("RPM 0", "STOP", "PWM 1000"):
        send(ser, c)
        time.sleep(0.2)
    drain(ser, 0.6)


def main():
    acquire("enc_highrate")
    LOGDIR.mkdir(parents=True, exist_ok=True)
    log(f"=== ENC high-rate verify {datetime.now().isoformat(timespec='seconds')} ===")
    log(f"port={PORT} stages={STAGES} hold={HOLD_S}s keepalive={KEEPALIVE_S}s gear={GEAR}")

    ser = open_port(PORT)
    csvf = open(CSVP, "w", newline="", encoding="utf-8")
    cw = csv.writer(csvf)
    cw.writerow(["wall_s", "stage_rpm", "t_ms", "enc_rpm", "pulse", "run", "out_hz", "enc_sample_hz"])
    t_start = time.time()
    last_flush = t_start

    try:
        drain(ser, 0.6)
        # banner
        for ln in drain(ser, 0.4):
            if ln.startswith("#") and ("enc_sample" in ln or "ENC" in ln):
                log(f"BANNER {ln}")
        safe_stop(ser)
        time.sleep(0.3)

        log("--- idle ENC? (电机停) ---")
        hz0, cpu0, loop0 = enc_query(ser)
        log(f"[IDLE] enc_sample_hz={hz0} cpu%={cpu0} loop_hz={loop0}")

        for c in ("STOP", "MODE CLOSED", "SOFT ON", "SOFT RATE UP 800",
                  "SOFT RATE DOWN 600", "RPMMAX 6000", "START"):
            send(ser, c)
            time.sleep(0.2)
            acks = [x for x in drain(ser, 0.3) if x.startswith("#")]
            log(f"CMD {c} -> {acks[:3]}")

        results = []
        for stage in STAGES:
            log(f"\n=== STAGE RPM {stage} ===")
            send(ser, f"RPM {stage}")
            t_stage = time.time()
            last_ka = 0.0
            last_poll = 0.0
            rpm_win = []
            enchz_win = []
            outhz_last = -1.0
            while time.time() - t_stage < HOLD_S:
                noww = time.time()
                if noww - last_ka >= KEEPALIVE_S:
                    last_ka = noww
                    send(ser, f"RPM {stage}")
                for ln in drain(ser, 0.15):
                    te = parse_telem(ln)
                    if te:
                        wall = noww - t_start
                        cw.writerow([f"{wall:.2f}", stage, int(te['t_ms']),
                                     f"{te['rpm']:.1f}", te['pulse'], te['run'],
                                     f"{te['out_hz']:.3f}", f"{te['enc_hz']:.1f}"])
                        if noww - t_stage >= 5.0:  # 稳态窗口
                            rpm_win.append(te['rpm'])
                            enchz_win.append(te['enc_hz'])
                        if te['out_hz'] > 0:
                            outhz_last = te['out_hz']
                if noww - last_poll >= POLL_S:
                    last_poll = noww
                    hz, cpu, loop = enc_query(ser)
                    log(f"[t={noww - t_stage:4.1f}s] enc_hz={hz} cpu%={cpu} loop_hz={loop}")
                if noww - last_flush >= 1.0:
                    csvf.flush()
                    last_flush = noww
                time.sleep(0.02)
            # 段末霍尔核对
            out_rpm = hall_query(ser)
            n = len(rpm_win)
            mean_rpm = sum(rpm_win) / n if n else float("nan")
            mn = min(rpm_win) if n else float("nan")
            mx = max(rpm_win) if n else float("nan")
            mean_enchz = sum(enchz_win) / len(enchz_win) if enchz_win else float("nan")
            hall_rpm = (outhz_last * GEAR * 60.0) if outhz_last > 0 else float("nan")
            spread = (mx - mn) if n else float("nan")
            results.append((stage, mean_rpm, mn, mx, spread, mean_enchz, outhz_last, hall_rpm, out_rpm))
            log(f"[STAGE {stage}] enc_rpm mean={mean_rpm:.0f} min={mn:.0f} max={mx:.0f} spread={spread:.0f} "
                f"enc_sample_hz~{mean_enchz:.0f} out_hz={outhz_last:.3f} hall_rpm(out*gear*60)={hall_rpm:.0f} "
                f"HALL_MEAS out_rpm={out_rpm}")
            csvf.flush()

        # 平滑降速后停
        log("\n--- ramp down & stop ---")
        for r in (3000, 1500):
            send(ser, f"RPM {r}")
            for _ in range(6):
                send(ser, f"RPM {r}")
                drain(ser, 0.15)
                time.sleep(0.35)
        safe_stop(ser)

        log("\n===== SUMMARY =====")
        log(f"idle enc_sample_hz={hz0} cpu%={cpu0} loop_hz={loop0}")
        folding = False
        for (stg, mean, mn, mx, spread, ehz, ohz, hrpm, omeas) in results:
            # 折叠判据：稳态 spread 过大(>目标30%)或均值远低于目标而油门在升
            fold_flag = spread > max(400.0, 0.30 * stg)
            folding = folding or fold_flag
            match = ""
            if hrpm == hrpm and mean == mean and mean > 0:
                err = abs(hrpm - mean) / mean * 100.0
                match = f"hall_err={err:.1f}%"
            log(f"  RPM{stg}: enc_mean={mean:.0f} spread={spread:.0f} enc_hz~{ehz:.0f} "
                f"hall_rpm={hrpm:.0f} {match} {'<<FOLD?' if fold_flag else 'stable'}")
        log(f"结论: enc_sample_hz≈{hz0 or ehz:.0f}Hz(target 2000); 折叠={'是' if folding else '否'}")
        LOG.write_text("\n".join(_out), encoding="utf-8")
        log(f"log={LOG}")
        log(f"csv={CSVP}")
        return 0
    except Exception as exc:
        try:
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
