#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HOLD PHASE 自学习：迭代测着陆误差 → 调 PHASE LEAD（必要时 PHASE OFFSET 180）。

传感前提：至少一次霍尔 latch（GPIO4=0 / GPIO5=180）后，用磁编推算 disc。
若 GPIO4 全程无沿：只能信编码器相对最近一次 GPIO5 latch，物理 0° 无法霍尔确认。

用法:
  set PYTHONIOENCODING=utf-8
  python hold_phase_learn.py --port COM10 --iters 4 --rpm 5000 --stop-ms 2000
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


def wrap180(d: float) -> float:
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def wrap360(d: float) -> float:
    while d < 0.0:
        d += 360.0
    while d >= 360.0:
        d -= 360.0
    return d


def parse_kv(line: str, key: str):
    m = re.search(rf"{re.escape(key)}=([\-\d.]+)", line)
    return float(m.group(1)) if m else None


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


def hold_until_rpm(ser, target: float, settle_s: float, band: float = 300.0) -> dict:
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
                        "elapsed": now - t0,
                    }
            else:
                ok_since = None
        time.sleep(0.02)
    return {"ok": False, "mean": sum(win) / len(win) if win else 0.0, "elapsed": 45.0}


def run_hold(ser, target: float, crawl: float, stop_ms: int, timeout: float = 28.0) -> dict:
    cmd = f"HOLD PHASE {target:g} {crawl:g} {stop_ms}"
    log(f">> {cmd}")
    t0 = time.time()
    send(ser, cmd)
    done = ""
    brake = ""
    while time.time() - t0 < timeout:
        for ln in drain(ser, 0.12):
            if "BRAKE" in ln and "HOLD PHASE" in ln:
                brake = ln
            if "HOLD PHASE DONE" in ln or ln.startswith("# ESTOP") or "ERR HOLD PHASE" in ln:
                done = ln
                return {
                    "ok": "DONE" in ln,
                    "done": done,
                    "brake": brake,
                    "wall_s": time.time() - t0,
                }
        time.sleep(0.02)
    return {"ok": False, "done": "TIMEOUT", "brake": brake, "wall_s": time.time() - t0}


def read_phase(ser) -> dict:
    send(ser, "PHASE?")
    lines = wait_ack(ser, lambda s: "disc_deg=" in s, 1.5)
    out = {"raw": lines[-1] if lines else ""}
    for ln in lines:
        if "disc_deg=" in ln:
            for k in ("disc_deg", "disc_ref", "offset", "cal", "lead", "esc_sense"):
                v = parse_kv(ln, k if k != "lead" else "lead")
                if v is not None:
                    out[k] = v
            # lead printed as lead=10.0/100ms — parse first number
            m = re.search(r"lead=([\-\d.]+)", ln)
            if m:
                out["lead"] = float(m.group(1))
            m = re.search(r"offset=([\-\d.]+)", ln)
            if m:
                out["offset"] = float(m.group(1))
    return out


def read_hall(ser) -> dict:
    send(ser, "HALL?")
    lines = wait_ack(ser, lambda s: "irq_dn=" in s or "cal=" in s, 1.8)
    out = {"lines": lines, "irq_dn": 0, "irq_up": 0, "cal": 0, "dn_cnt": 0, "up_cnt": 0}
    for ln in lines:
        for k in ("irq_dn", "irq_up", "cal", "dn_cnt", "up_cnt"):
            v = parse_kv(ln, k)
            if v is not None:
                out[k] = int(v)
    return out


def geometry_probe(ser, crawl: float = 80.0, max_s: float = 12.0) -> dict:
    """停稳后低速爬行，看下一个霍尔是 GPIO4 还是 GPIO5，估计物理落点。"""
    h0 = read_hall(ser)
    dn0, up0 = h0["irq_dn"], h0["irq_up"]
    ph0 = read_phase(ser)
    disc0 = ph0.get("disc_deg")
    log(f"--- geometry probe from disc={disc0} irq_dn={dn0} irq_up={up0} ---")
    send(ser, "MODE OPEN")
    send(ser, "TELEM ON")
    # 开环爬行：用低 RPM 闭环更稳
    send(ser, "MODE CLOSED")
    send(ser, "SOFT ON")
    send(ser, f"RPM {crawl:.0f}")
    t0 = time.time()
    hit = None
    while time.time() - t0 < max_s:
        for ln in drain(ser, 0.1):
            if "HALL DOWN" in ln:
                hit = "GPIO4/0"
            elif "HALL UP" in ln:
                hit = "GPIO5/180"
            if hit:
                break
        if hit:
            break
        h = read_hall(ser)
        if h["irq_dn"] > dn0:
            hit = "GPIO4/0"
            break
        if h["irq_up"] > up0:
            hit = "GPIO5/180"
            break
    safe_stop(ser)
    ph1 = read_phase(ser)
    disc1 = ph1.get("disc_deg")
    travel = None
    if disc0 is not None and disc1 is not None:
        travel = wrap360(disc1 - disc0)
    result = {
        "hit": hit,
        "disc0": disc0,
        "disc1": disc1,
        "travel_disc": travel,
        "dt": time.time() - t0,
        "irq_dn0": dn0,
        "irq_up0": up0,
    }
    log(
        f"geometry hit={hit} travel_disc={travel} dt={result['dt']:.2f}s "
        f"(若目标0°后很快碰到 GPIO5 → 可能停在物理~180°)"
    )
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description="HOLD PHASE lead/offset learn loop")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--target", type=float, default=0.0)
    ap.add_argument("--rpm", type=float, default=5000.0)
    ap.add_argument("--crawl", type=float, default=80.0)
    ap.add_argument("--stop-ms", type=int, default=2000)
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--tol", type=float, default=10.0, help="|error| success threshold deg")
    ap.add_argument("--lead0", type=float, default=10.0)
    ap.add_argument("--no-geom", action="store_true", help="skip post-stop hall geometry probe")
    ap.add_argument("--no-save", action="store_true")
    args = ap.parse_args()

    LOGDIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOGDIR / f"hold_phase_learn_{stamp}.txt"

    acquire("hold_phase_learn")
    rc = 1
    trend: list[dict] = []
    try:
        log(f"=== HOLD PHASE LEARN {datetime.now().isoformat(timespec='seconds')} ===")
        log(
            f"port={args.port} target={args.target} rpm={args.rpm} "
            f"stop_ms={args.stop_ms} iters={args.iters} tol={args.tol}"
        )
        ser = open_port(args.port)
        time.sleep(0.8)
        ser.reset_input_buffer()

        send(ser, "TELEM OFF")
        drain(ser, 0.3)
        send(ser, "HOST OFF")
        wait_ack(ser, lambda s: "HOST OFF" in s, 1.5)

        lead = args.lead0
        offset = 0.0
        send(ser, f"PHASE LEAD {lead:.2f} 100 0.03")
        wait_ack(ser, lambda s: "PHASE LEAD" in s, 1.0)
        send(ser, "PHASE OFFSET 0")
        wait_ack(ser, lambda s: "PHASE OFFSET" in s, 1.0)

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

        # Gate + Hall latch
        log("--- gate RPM 2000 (need Hall latch) ---")
        send(ser, "RPM 2000")
        gate = detect_spin(ser, seconds=3.5, keepalive="RPM 2000")
        log(f"gate {gate.summary_line()}")
        h = read_hall(ser)
        log(f"after gate cal={h['cal']} irq_dn={h['irq_dn']} irq_up={h['irq_up']}")
        if h["irq_dn"] == 0 and h["irq_up"] == 0:
            log("BLOCK: 无任何霍尔沿 — 请先修 GPIO4/5 接线/电平/磁钢，再谈相位学习")
            safe_stop(ser)
            log_path.write_text("\n".join(_out) + "\n", encoding="utf-8")
            return 2
        if h["irq_dn"] == 0:
            log(
                "WARN: GPIO4(irq_dn)=0 全程无下扑沿 — 只能靠 GPIO5=180° latch；"
                "物理 0° 无法霍尔确认；若装反将表现为指令0°停在物理~180°"
            )

        for it in range(1, args.iters + 1):
            log(f"\n======== ITER {it}/{args.iters} lead={lead:.2f} offset={offset:.1f} ========")
            safe_stop(ser)
            time.sleep(0.4)
            send(ser, "STOP")
            drain(ser, 0.2)
            send(ser, "MODE CLOSED")
            send(ser, "SOFT ON")
            send(ser, "TELEM ON")
            send(ser, f"PHASE LEAD {lead:.2f} 100 0.03")
            drain(ser, 0.15)
            send(ser, f"PHASE OFFSET {offset:.1f}")
            drain(ser, 0.15)

            send(ser, "RPM 2000")
            detect_spin(ser, seconds=2.0, keepalive="RPM 2000")
            send(ser, f"RPM {args.rpm:.0f}")
            st = hold_until_rpm(ser, args.rpm, 3.0, band=350.0)
            log(f"stable ok={st['ok']} mean={st.get('mean', 0):.1f}")
            if not st["ok"]:
                log("FAIL stabilize")
                break

            send(ser, "TELEM OFF")
            drain(ser, 0.15)
            h_before = read_hall(ser)
            r = run_hold(ser, args.target, args.crawl, args.stop_ms)
            log(f"done: {r['done']}")
            ph = read_phase(ser)
            h_after = read_hall(ser)
            disc = ph.get("disc_deg")
            if disc is None and "DONE" in r["done"]:
                disc = parse_kv(r["done"], "disc")
            if disc is None:
                log("FAIL: no disc reading")
                break
            err = wrap180(disc - args.target)
            abs_err = abs(err)
            hall_confirm = parse_kv(r["done"], "hall_confirm") if "DONE" in r["done"] else None
            row = {
                "iter": it,
                "disc": disc,
                "err": err,
                "abs_err": abs_err,
                "lead": lead,
                "offset": offset,
                "ok": r["ok"],
                "hall_confirm": hall_confirm,
                "irq_dn": h_after["irq_dn"],
                "irq_up": h_after["irq_up"],
                "delta_dn": h_after["irq_dn"] - h_before["irq_dn"],
                "delta_up": h_after["irq_up"] - h_before["irq_up"],
            }
            trend.append(row)
            log(
                f"ITER{it}: disc={disc:.2f} err={err:+.2f} |err|={abs_err:.2f} "
                f"lead={lead:.2f} offset={offset:.1f} hall_confirm={hall_confirm} "
                f"Δirq_dn={row['delta_dn']} Δirq_up={row['delta_up']}"
            )

            # 几何探针：目标 0° 后很快碰到 GPIO5 → 怀疑停在物理 180°
            if not args.no_geom and abs_err <= args.tol + 5.0:
                geo = geometry_probe(ser, crawl=70.0, max_s=10.0)
                row["geom_hit"] = geo["hit"]
                row["geom_travel"] = geo["travel_disc"]
                if (
                    args.target == 0.0
                    and geo["hit"] == "GPIO5/180"
                    and geo["travel_disc"] is not None
                    and geo["travel_disc"] < 50.0
                    and offset < 90.0
                ):
                    log(
                        "DETECT: 停在『固件0°』后盘行程<50°即遇 GPIO5 → "
                        "判定 0↔180 标架反了，PHASE OFFSET += 180"
                    )
                    offset = wrap360(offset + 180.0)
                    send(ser, f"PHASE OFFSET {offset:.1f}")
                    drain(ser, 0.2)
                    # 本轮误差在错误标架下测的，不调 lead，直接下一轮
                    continue

            if abs_err <= args.tol:
                log(f"PASS |err|={abs_err:.2f} <= tol={args.tol}")
                rc = 0
                break

            # lead 学习：欠冲(err<0，停在目标前)→减小 lead；过冲(err>0)→加大 lead
            # 单次增益 0.55，夹在 [2, 45]
            lead_new = lead + 0.55 * err
            if lead_new < 2.0:
                lead_new = 2.0
            if lead_new > 45.0:
                lead_new = 45.0
            log(f"lead update {lead:.2f} → {lead_new:.2f} (gain*err)")
            lead = lead_new

        send(ser, f"PHASE LEAD {lead:.2f} 100 0.03")
        drain(ser, 0.15)
        send(ser, f"PHASE OFFSET {offset:.1f}")
        drain(ser, 0.15)
        if not args.no_save and trend:
            send(ser, "PHASE LEAD SAVE")
            wait_ack(ser, lambda s: "SAVE" in s, 1.0)
            log("NVS: PHASE LEAD SAVE (含 hphoff offset)")

        safe_stop(ser)
        log("\n======== 中文学习结果 ========")
        log(f"迭代次数: {len(trend)} / {args.iters}")
        for row in trend:
            log(
                f"  #{row['iter']}: disc={row['disc']:.1f}° err={row['err']:+.1f}° "
                f"|err|={row['abs_err']:.1f} lead={row['lead']:.1f} offset={row['offset']:.0f} "
                f"Δdn={row['delta_dn']} Δup={row['delta_up']} "
                f"geom={row.get('geom_hit', '-')}"
            )
        if trend:
            log(
                f"误差趋势 |err|: "
                + " → ".join(f"{r['abs_err']:.1f}" for r in trend)
            )
            log(f"最终 lead={lead:.2f} offset={offset:.1f}")
            if any(r["irq_dn"] == 0 and r == trend[0] for r in trend[:1]) or (
                trend and trend[-1]["irq_dn"] == 0
            ):
                log("物理注意: GPIO4 无沿时，编码器『0°』未必等于物理下扑 0°；请修霍尔后再信绝对相位")
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
