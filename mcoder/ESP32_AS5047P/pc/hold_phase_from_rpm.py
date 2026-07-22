#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从稳定闭环转速做 HOLD PHASE 软降停机（COM10）。

流程：HOST OFF → CLOSED → RPM 5000 稳态 → HOLD PHASE 0 <crawl> <stop_ms>
优先 stop_ms=2000；若冲偏/霍尔先到则再试 3000。

用法:
  set PYTHONIOENCODING=utf-8
  python hold_phase_from_rpm.py --port COM10
  python hold_phase_from_rpm.py --port COM10 --rpm 5000 --stop-ms 2000
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
TARGET_RPM = 5000.0
CRAWL_RPM = 80.0
SOFT_UP = 900.0
SOFT_DOWN = 2000.0
RPMMAX = 12000
SETTLE_S = 4.0
LOGDIR = Path(__file__).resolve().parent / "logs"

_out: list[str] = []


def log(s: str) -> None:
    print(s, flush=True)
    _out.append(s)


def drain(ser, budget: float = 0.15) -> list[str]:
    lines = read_lines(ser, budget)
    for ln in lines:
        if ln.startswith("#") or not ln[0:1].isdigit():
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
        drain(ser, 0.15)


def hold_until_rpm(ser, target: float, settle_s: float, band: float = 250.0) -> dict:
    """Ramp/hold target; return last telem stats when within band for settle_s."""
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
                    mean = sum(win) / len(win)
                    return {
                        "ok": True,
                        "mean_rpm": mean,
                        "last_rpm": rpm,
                        "pulse": t["pulse"],
                        "elapsed": now - t0,
                    }
            else:
                ok_since = None
        time.sleep(0.02)
    mean = sum(win) / len(win) if win else float("nan")
    return {"ok": False, "mean_rpm": mean, "last_rpm": win[-1] if win else 0.0, "elapsed": 45.0}


def run_hold_phase(ser, target_deg: float, crawl: float, stop_ms: int, timeout: float) -> dict:
    cmd = f"HOLD PHASE {target_deg:g} {crawl:g} {stop_ms}"
    log(f">> {cmd}")
    t_cmd = time.time()
    send(ser, cmd)
    lines: list[str] = []
    done = ""
    brake_line = ""
    soft_line = ""
    overshoot = False
    end = time.time() + timeout
    while time.time() < end:
        for ln in drain(ser, 0.12):
            lines.append(ln)
            if "SOFTDONE" in ln:
                soft_line = ln
            if "BRAKE" in ln and "HOLD PHASE" in ln:
                brake_line = ln
                if "before_lead" in ln:
                    overshoot = True
            if "HOLD PHASE DONE" in ln or ln.startswith("# ESTOP") or "ERR HOLD PHASE" in ln:
                done = ln
                wall = time.time() - t_cmd
                return {
                    "ok": "DONE" in ln,
                    "done": done,
                    "brake": brake_line,
                    "soft": soft_line,
                    "overshoot": overshoot,
                    "wall_s": wall,
                    "lines": lines,
                }
        time.sleep(0.02)
    return {
        "ok": False,
        "done": "TIMEOUT",
        "brake": brake_line,
        "soft": soft_line,
        "overshoot": overshoot,
        "wall_s": time.time() - t_cmd,
        "lines": lines,
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
        m = re.search(rf"{key}=([\-\d.]+)", line)
        if m:
            out[key] = float(m.group(1))
    m = re.search(r"reason=(\S+)", line)
    if m:
        out["reason"] = m.group(1)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="5000RPM → HOLD PHASE 0 soft-down test")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--rpm", type=float, default=TARGET_RPM)
    ap.add_argument("--crawl", type=float, default=CRAWL_RPM)
    ap.add_argument("--stop-ms", type=int, default=2000, help="preferred profile ms (~2s)")
    ap.add_argument("--fallback-ms", type=int, default=3000)
    ap.add_argument("--no-fallback", action="store_true")
    ap.add_argument("--hall-cal", action="store_true", help="force HALL CAL at current pose")
    args = ap.parse_args()

    LOGDIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOGDIR / f"hold_phase_from_rpm_{stamp}.txt"

    acquire("hold_phase_from_rpm")
    rc = 1
    try:
        log(f"=== HOLD PHASE from {args.rpm:.0f}RPM  {datetime.now().isoformat(timespec='seconds')} ===")
        log(f"port={args.port} stop_ms={args.stop_ms} fallback={args.fallback_ms}")
        ser = open_port(args.port)
        time.sleep(0.5)
        ser.reset_input_buffer()

        send(ser, "TELEM OFF")
        drain(ser, 0.3)
        send(ser, "HOST OFF")
        wait_ack(ser, lambda s: "HOST OFF" in s, 1.5)
        send(ser, "HALL?")
        hall_lines = wait_ack(ser, lambda s: "cal=" in s, 1.5)
        send(ser, "PHASE?")
        wait_ack(ser, lambda s: "PHASE" in s, 1.0)
        send(ser, "PHASE LEAD?")
        wait_ack(ser, lambda s: "PHASE LEAD" in s, 1.0)

        cal_ok = any("cal=1" in s for s in hall_lines)
        if args.hall_cal:
            log("--- HALL CAL (forced) ---")
            send(ser, "HALL CAL")
            wait_ack(ser, lambda s: "HALL CAL" in s, 1.0)
            cal_ok = True
        elif not cal_ok:
            log("--- cal=0：先低速过 GPIO4 自动标定，不强制 HALL CAL ---")

        safe_stop(ser)
        time.sleep(0.3)
        send(ser, "STOP")
        drain(ser, 0.3)

        for c in (
            "MODE CLOSED",
            "SOFT ON",
            f"SOFT RATE UP {SOFT_UP:.0f}",
            f"SOFT RATE DOWN {SOFT_DOWN:.0f}",
            f"RPMMAX {RPMMAX}",
            "FREQ 400",
            "TELEM ON",
        ):
            send(ser, c)
            drain(ser, 0.15)

        # Gate at 2000（同时让 GPIO4 沿标定盘零）
        log("--- gate RPM 2000 ---")
        send(ser, "RPM 2000")
        gate = detect_spin(ser, seconds=3.0, keepalive="RPM 2000")
        log(f"gate {gate.summary_line()}")
        if not gate.spinning:
            log("FAIL: gate not spinning — ESTOP")
            safe_stop(ser)
            return 4

        if not cal_ok and not args.hall_cal:
            send(ser, "HALL?")
            hl = wait_ack(ser, lambda s: "cal=" in s, 1.5)
            cal_ok = any("cal=1" in s for s in hl)
            if not cal_ok:
                log("WARN: 过圈后仍 cal=0，改用 HALL CAL（盘零=当前姿，非必然 GPIO4）")
                send(ser, "HALL CAL")
                wait_ack(ser, lambda s: "HALL CAL" in s, 1.0)
                cal_ok = True

        if not cal_ok:
            log("FAIL: flap cal=0")
            safe_stop(ser)
            return 3

        log(f"--- ramp to {args.rpm:.0f} ---")
        send(ser, f"RPM {args.rpm:.0f}")
        hold = hold_until_rpm(ser, args.rpm, SETTLE_S, band=300.0)
        log(
            f"stable ok={hold['ok']} mean={hold['mean_rpm']:.1f} "
            f"last={hold['last_rpm']:.1f} pulse={hold.get('pulse')} t={hold['elapsed']:.1f}s"
        )
        if not hold["ok"]:
            log("FAIL: did not stabilize near target — ESTOP")
            safe_stop(ser)
            return 5

        send(ser, "PHASE?")
        wait_ack(ser, lambda s: "disc_deg=" in s, 1.0)
        send(ser, "HALL?")
        wait_ack(ser, lambda s: "HALL" in s, 1.2)

        profiles = [args.stop_ms]
        if not args.no_fallback and args.fallback_ms != args.stop_ms:
            profiles.append(args.fallback_ms)

        final = None
        for i, ms in enumerate(profiles):
            if i > 0:
                log(f"--- fallback: re-accelerate then profile {ms}ms ---")
                safe_stop(ser)
                time.sleep(0.5)
                send(ser, "STOP")
                drain(ser, 0.2)
                send(ser, "MODE CLOSED")
                send(ser, "SOFT ON")
                send(ser, "TELEM ON")
                send(ser, "RPM 2000")
                detect_spin(ser, seconds=2.5, keepalive="RPM 2000")
                send(ser, f"RPM {args.rpm:.0f}")
                hold2 = hold_until_rpm(ser, args.rpm, 3.0, band=350.0)
                log(f"re-stable ok={hold2['ok']} mean={hold2['mean_rpm']:.1f}")
                if not hold2["ok"]:
                    break

            log(f"--- HOLD PHASE 0 crawl={args.crawl:g} stop_ms={ms} ---")
            send(ser, "TELEM OFF")
            drain(ser, 0.2)
            r = run_hold_phase(ser, 0.0, args.crawl, ms, timeout=25.0)
            final = r
            log(f"wall_s={r['wall_s']:.3f} ok={r['ok']} overshoot={r['overshoot']}")
            log(f"soft: {r['soft'] or '(none)'}")
            log(f"brake: {r['brake'] or '(none)'}")
            log(f"done: {r['done']}")

            send(ser, "PHASE?")
            wait_ack(ser, lambda s: "disc_deg=" in s, 1.0)
            send(ser, "HALL?")
            wait_ack(ser, lambda s: "HALL" in s, 1.2)
            send(ser, "PWM 1000")
            drain(ser, 0.2)

            if r["ok"] and not r["overshoot"]:
                break
            if (not r["ok"] or r["overshoot"]) and i + 1 < len(profiles):
                log("WARN: miss/overshoot — try longer soft-down fallback")
                continue
            break

        safe_stop(ser)

        # Chinese summary block
        log("")
        log("======== 中文结果 ========")
        if final is None:
            log("结果: 失败（未执行 HOLD PHASE）")
            rc = 1
        else:
            parsed = parse_done(final["done"]) if "DONE" in final["done"] else {}
            disc = parsed.get("disc")
            phase_err = abs(disc) if disc is not None else None
            if disc is not None and disc > 180:
                phase_err = min(abs(disc), abs(360 - disc))
            elif disc is not None:
                phase_err = min(abs(disc), abs(360 - abs(disc))) if disc else abs(disc)

            log(f"结果: {'成功' if final['ok'] else '失败'}")
            log(f"壁钟停机时间: {final['wall_s']:.3f} s（指令→DONE/ESTOP）")
            if "elapsed_ms" in parsed:
                log(f"固件 elapsed_ms: {parsed['elapsed_ms']:.0f} ms")
            if "brake_to_stop_ms" in parsed:
                log(f"制动→停稳 brake_to_stop_ms: {parsed['brake_to_stop_ms']:.0f} ms")
            if "soft_ms" in parsed:
                log(f"软降设定 soft_ms: {parsed['soft_ms']:.0f} / profile {parsed.get('profile_ms', float('nan')):.0f} ms")
            if disc is not None:
                log(f"最终盘相位 disc: {disc:.2f}°  相位误差≈{phase_err:.2f}°")
            else:
                log(f"最终盘相位: 见 PHASE?/DONE 行")
            hc = parsed.get("hall_confirm")
            log(f"霍尔确认 hall_confirm: {int(hc) if hc is not None else '未知'}")
            log(f"冲偏/霍尔先到 overshoot: {'是' if final['overshoot'] else '否'}")
            if final["brake"]:
                log(f"制动原因: {final['brake']}")
            log(f"DONE/ERR: {final['done']}")
            rc = 0 if final["ok"] else 1

        log(f"日志: {log_path}")
        log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
        return rc
    except Exception as e:
        log(f"EXCEPTION: {e}")
        try:
            safe_stop(ser)  # type: ignore[name-defined]
        except Exception:
            pass
        log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
        return 99
    finally:
        try:
            release()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
