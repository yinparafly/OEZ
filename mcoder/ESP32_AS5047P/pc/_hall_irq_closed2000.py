# -*- coding: utf-8 -*-
"""Closed-loop 2000RPM x ~35s + Hall IRQ poll."""
from __future__ import annotations
import re, sys, time
from datetime import datetime
from pathlib import Path

from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire

PORT = "COM10"
TARGET_RPM = 2000.0
HOLD_S = 30.0
POLL_S = 5.0
LOG = Path("logs") / f"hall_irq_closed2000_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

def drain(ser, budget=0.4):
    return read_lines(ser, budget)

def parse_telem(line: str):
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) < 12:
        return None
    try:
        return {
            "rpm": float(parts[4]),
            "pulse": int(float(parts[9])),
            "target": float(parts[10]),
            "mode": int(float(parts[11])),
            "run": int(float(parts[16])) if len(parts) > 16 else -1,
        }
    except ValueError:
        return None

def hall_query(ser):
    # clear some telem noise then query
    drain(ser, 0.15)
    send(ser, "HALL?")
    lines = []
    # HALL? prints 4 lines now (status/samp/last/meas); telem floods — collect longer
    t_end = time.time() + 1.4
    info = {}
    events = []
    while time.time() < t_end:
        chunk = drain(ser, 0.2)
        for ln in chunk:
            lines.append(ln)
            if "# HALL DOWN" in ln or "# HALL UP" in ln:
                events.append(ln)
            if "irq_dn=" in ln:
                m = re.search(r"irq_dn=(\d+).*?irq_up=(\d+)", ln)
                if m:
                    info["irq_dn"] = int(m.group(1))
                    info["irq_up"] = int(m.group(2))
                m2 = re.search(r"\birq=(\S+)", ln)
                if m2:
                    info["irq"] = m2.group(1)
                m3 = re.search(r"active=(\S+)", ln)
                if m3:
                    info["active"] = m3.group(1)
                m4 = re.search(r"dn_cnt=(\d+).*?up_cnt=(\d+)", ln)
                if m4:
                    info["dn_cnt"] = int(m4.group(1))
                    info["up_cnt"] = int(m4.group(2))
                m5 = re.search(r"isr_ok=(\d+)", ln)
                if m5:
                    info["isr_ok"] = int(m5.group(1))
                info["status"] = ln
            if "dn_low%" in ln or "poll_dn=" in ln or "# HALL SAMP" in ln:
                info["samp"] = ln
                m = re.search(r"dn_low%=([0-9.]+).*?up_low%=([0-9.]+).*?poll_dn=(\d+).*?poll_up=(\d+)", ln)
                if m:
                    info["dn_low_pct"] = float(m.group(1))
                    info["up_low_pct"] = float(m.group(2))
                    info["poll_dn"] = int(m.group(3))
                    info["poll_up"] = int(m.group(4))
            if ln.startswith("# HALL MEAS"):
                info["meas"] = ln
        if "irq_dn" in info and "meas" in info:
            break
    return info, events, lines

def safe_stop(ser):
    for c in ("RPM 0", "STOP", "PWM 1000"):
        send(ser, c)
        time.sleep(0.2)
    return drain(ser, 0.8)

def main():
    acquire("hall_irq_closed2000")
    LOG.parent.mkdir(parents=True, exist_ok=True)
    out = []
    def log(s):
        print(s, flush=True)
        out.append(s)

    log(f"=== CLOSED 2000RPM + Hall IRQ {datetime.now().isoformat(timespec='seconds')} ===")
    log(f"port={PORT} target={TARGET_RPM} hold={HOLD_S}s poll={POLL_S}s gear=27.744")
    expect_hz = TARGET_RPM / 27.744 / 60.0
    log(f"expect out_Hz~{expect_hz:.3f} => ~{expect_hz*30:.1f} edges/ch in 30s")

    ser = open_port(PORT)
    try:
        drain(ser, 0.5)
        # ensure idle
        safe_stop(ser)
        time.sleep(0.3)
        drain(ser, 0.3)

        send(ser, "PING")
        for ln in drain(ser, 0.5):
            if ln.startswith("#"):
                log(f"  {ln}")

        base, ev, raw_lines = hall_query(ser)
        log(f"[BASE] status={base.get('status')}")
        log(f"[BASE] samp={base.get('samp')}")
        log(f"[BASE] meas={base.get('meas')}")
        irq0_dn = base.get("irq_dn")
        irq0_up = base.get("irq_up")
        log(f"[BASE] irq={base.get('irq')} active={base.get('active')} isr_ok={base.get('isr_ok')} irq_dn={irq0_dn} irq_up={irq0_up}")
        if base.get("irq") and ("ANYEDGE" not in str(base.get("irq")) and base.get("irq") != "FALLING"):
            log(f"WARN: irq={base.get('irq')} — expect ANYEDGE+SW after flash_com10.bat")
        if irq0_dn is None:
            log("WARN: failed to parse irq baseline from HALL?")

        for cmd in ("STOP", "MODE CLOSED", "SOFT ON", "RPMMAX 6000", "START", f"RPM {int(TARGET_RPM)}"):
            send(ser, cmd)
            time.sleep(0.15)
            acks = [ln for ln in drain(ser, 0.35) if ln.startswith("#")]
            log(f"CMD {cmd} -> {acks[:4]}")

        t0 = time.time()
        last_poll = -999.0
        rpm_win = []  # (t, rpm) after settle
        hall_events = []
        samples = []

        while True:
            elapsed = time.time() - t0
            if elapsed >= HOLD_S:
                break
            for ln in drain(ser, 0.2):
                if "# HALL DOWN" in ln or "# HALL UP" in ln:
                    hall_events.append((elapsed, ln))
                    log(f"[t={elapsed:5.1f}s] EVENT {ln}")
                te = parse_telem(ln)
                if te and elapsed >= 5.0:  # after soft ramp settle window
                    rpm_win.append((elapsed, te["rpm"]))
            if elapsed - last_poll >= POLL_S:
                last_poll = elapsed
                # grab recent rpm from brief telem
                recent = []
                for ln in drain(ser, 0.25):
                    if "# HALL DOWN" in ln or "# HALL UP" in ln:
                        hall_events.append((elapsed, ln))
                        log(f"[t={elapsed:5.1f}s] EVENT {ln}")
                    te = parse_telem(ln)
                    if te:
                        recent.append(te["rpm"])
                        if elapsed >= 5.0:
                            rpm_win.append((elapsed, te["rpm"]))
                info, evs, _ = hall_query(ser)
                for e in evs:
                    hall_events.append((elapsed, e))
                    log(f"[t={elapsed:5.1f}s] EVENT {e}")
                dn = info.get("irq_dn")
                up = info.get("irq_up")
                ddn = (dn - irq0_dn) if (dn is not None and irq0_dn is not None) else None
                dup = (up - irq0_up) if (up is not None and irq0_up is not None) else None
                rpm_now = sum(recent)/len(recent) if recent else float("nan")
                samples.append((elapsed, dn, up, rpm_now))
                log(f"[t={elapsed:5.1f}s] rpm~{rpm_now:.1f} irq_dn={dn}(+{ddn}) irq_up={up}(+{dup})")
                log(f"         {info.get('status')}")
                if info.get("samp"):
                    log(f"         {info.get('samp')}")
            time.sleep(0.05)

        # final
        elapsed = time.time() - t0
        recent = []
        for ln in drain(ser, 0.3):
            if "# HALL DOWN" in ln or "# HALL UP" in ln:
                hall_events.append((elapsed, ln))
                log(f"[t={elapsed:5.1f}s] EVENT {ln}")
            te = parse_telem(ln)
            if te:
                recent.append(te["rpm"])
                rpm_win.append((elapsed, te["rpm"]))
        info, evs, _ = hall_query(ser)
        for e in evs:
            hall_events.append((elapsed, e))
            log(f"[t={elapsed:5.1f}s] EVENT {e}")
        dn = info.get("irq_dn")
        up = info.get("irq_up")
        ddn = (dn - irq0_dn) if (dn is not None and irq0_dn is not None) else None
        dup = (up - irq0_up) if (up is not None and irq0_up is not None) else None
        rpm_now = sum(recent)/len(recent) if recent else float("nan")
        log(f"[FINAL t={elapsed:.1f}s] rpm~{rpm_now:.1f} irq_dn={dn}(+{ddn}) irq_up={up}(+{dup})")
        log(f"  status={info.get('status')}")
        log(f"  samp={info.get('samp')}")
        log(f"  meas={info.get('meas')}")

        stop_lines = safe_stop(ser)
        log(f"SAFE STOP -> {[x for x in stop_lines if x.startswith('#')][:8]}")

        # RPM stats: use samples after t>=8s (soft ~600rpm/s -> 2000 in ~3.3s)
        settled = [r for t, r in rpm_win if t >= 8.0]
        if settled:
            mean_rpm = sum(settled) / len(settled)
            mn = min(settled)
            mx = max(settled)
        else:
            mean_rpm = mn = mx = float("nan")

        log("--- SUMMARY ---")
        log(f"hold_s={HOLD_S:.0f} target={TARGET_RPM:.0f}")
        log(f"encoder RPM mean={mean_rpm:.1f} min={mn:.1f} max={mx:.1f} (n={len(settled)}, t>=8s)")
        log(f"irq baseline dn={irq0_dn} up={irq0_up}")
        log(f"irq final    dn={dn} up={up}  delta dn={ddn} up={dup}")
        log(f"# HALL events: {len(hall_events)}")
        for t, e in hall_events[:30]:
            log(f"  @{t:.1f}s {e}")

        spun = (not (mean_rpm != mean_rpm)) and mean_rpm > 500  # not nan and spinning
        irq_ok = (ddn is not None and dup is not None and (ddn > 0 or dup > 0))
        low_seen = (info.get("dn_low_pct") or 0) > 0.01 or (info.get("up_low_pct") or 0) > 0.01
        poll_seen = (info.get("poll_dn") or 0) > 0 or (info.get("poll_up") or 0) > 0
        if irq_ok and spun:
            log(f"结论: 闭环约{mean_rpm:.0f}RPM 保持{HOLD_S:.0f}s 后 irq 增加(dn+{ddn}/up+{dup}) —— 中断已通")
        elif spun and not irq_ok and not low_seen and not poll_seen:
            log(
                f"结论: 编码器约{mean_rpm:.0f}RPM 在转，但 dn/up 低电平占比=0、poll_edge=0、irq=0。"
                "根因不是 ISR 注册失败，而是 GPIO4/5 焊盘侧从未看到低电平（示波器方波未到达 MCU 数字脚，"
                "或低电平未低于 VIH/VIL 阈值）。请在 ESP32 排针 IO4/IO5 对地测幅值，并核对分压/接线。"
            )
        elif spun and not irq_ok and (low_seen or poll_seen):
            log(
                f"结论: 编码器约{mean_rpm:.0f}RPM，digitalRead/poll 已见边沿但 irq 未增 —— 属 ISR 路径问题"
            )
        else:
            log("结论: 编码器未稳定到目标转速，不作静止「读不到」结论；需查闭环/ESC")

        LOG.write_text("\n".join(out), encoding="utf-8")
        log(f"log: {LOG.resolve()}")
        return 0 if irq_ok else 2
    except Exception as exc:
        try:
            safe_stop(ser)
        except Exception:
            pass
        log(f"ERROR: {exc}")
        LOG.write_text("\n".join(out), encoding="utf-8")
        raise
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    sys.exit(main())
