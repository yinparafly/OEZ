# -*- coding: utf-8 -*-
"""Closed-loop 2000RPM + Hall IRQ poll, with <1.5s keepalive to avoid host_timeout ESTOP.

去内部上拉复测：MODE CLOSED + SOFT ON + RPMMAX 6000 + START + RPM 2000，保持~32s。
每 ~0.8s 重发 RPM 2000 保活（HOST_TIMEOUT_MS=1500），每 5s 查 HALL? 记 irq/low%。
结束 RPM 0 / STOP / PWM 1000。日志写 logs/。
"""
from __future__ import annotations
import re, sys, time
from datetime import datetime
from pathlib import Path

from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire

PORT = "COM10"
TARGET_RPM = 2000.0
HOLD_S = 32.0
POLL_S = 5.0
KEEPALIVE_S = 0.8  # < HOST_TIMEOUT_MS(1500ms)
LOG = Path(__file__).resolve().parent / "logs" / f"hall_irq_ka_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"


def drain(ser, budget=0.3):
    return read_lines(ser, budget)


def parse_telem(line: str):
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) < 12:
        return None
    try:
        return {"rpm": float(parts[4]), "run": int(float(parts[16])) if len(parts) > 16 else -1}
    except ValueError:
        return None


def hall_query(ser):
    drain(ser, 0.1)
    send(ser, "HALL?")
    t_end = time.time() + 1.4
    info, events, lines = {}, [], []
    while time.time() < t_end:
        for ln in drain(ser, 0.2):
            lines.append(ln)
            if "# HALL DOWN" in ln or "# HALL UP" in ln:
                events.append(ln)
            if "irq_dn=" in ln:
                m = re.search(r"irq_dn=(\d+).*?irq_up=(\d+)", ln)
                if m:
                    info["irq_dn"] = int(m.group(1)); info["irq_up"] = int(m.group(2))
                m2 = re.search(r"\birq=(\S+)", ln)
                if m2: info["irq"] = m2.group(1)
                m3 = re.search(r"active=(\S+)", ln)
                if m3: info["active"] = m3.group(1)
                m4 = re.search(r"dn_cnt=(\d+).*?up_cnt=(\d+)", ln)
                if m4:
                    info["dn_cnt"] = int(m4.group(1)); info["up_cnt"] = int(m4.group(2))
                m5 = re.search(r"isr_ok=(\d+)", ln)
                if m5: info["isr_ok"] = int(m5.group(1))
                info["status"] = ln
            if "# HALL SAMP" in ln:
                info["samp"] = ln
                m = re.search(r"dn_low%=([0-9.]+).*?up_low%=([0-9.]+).*?poll_dn=(\d+).*?poll_up=(\d+)", ln)
                if m:
                    info["dn_low_pct"] = float(m.group(1)); info["up_low_pct"] = float(m.group(2))
                    info["poll_dn"] = int(m.group(3)); info["poll_up"] = int(m.group(4))
            if ln.startswith("# HALL MEAS"):
                info["meas"] = ln
        if "irq_dn" in info and "meas" in info:
            break
    return info, events, lines


def safe_stop(ser):
    for c in ("RPM 0", "STOP", "PWM 1000"):
        send(ser, c); time.sleep(0.2)
    return drain(ser, 0.8)


def main():
    acquire("hall_irq_ka")
    LOG.parent.mkdir(parents=True, exist_ok=True)
    out = []
    def log(s):
        print(s, flush=True); out.append(s)

    log(f"=== CLOSED 2000RPM + Hall IRQ (keepalive) {datetime.now().isoformat(timespec='seconds')} ===")
    log(f"port={PORT} target={TARGET_RPM} hold={HOLD_S}s poll={POLL_S}s keepalive={KEEPALIVE_S}s gear=27.744")
    expect_hz = TARGET_RPM / 27.744 / 60.0
    log(f"expect out_Hz~{expect_hz:.3f} => ~{expect_hz*HOLD_S:.1f} edges/ch in {HOLD_S:.0f}s")

    ser = open_port(PORT)
    try:
        drain(ser, 0.5)
        safe_stop(ser)
        time.sleep(0.3)
        drain(ser, 0.3)

        base, ev, _ = hall_query(ser)
        log(f"[BASE] status={base.get('status')}")
        log(f"[BASE] samp={base.get('samp')}")
        log(f"[BASE] meas={base.get('meas')}")
        irq0_dn = base.get("irq_dn"); irq0_up = base.get("irq_up")
        log(f"[BASE] irq={base.get('irq')} active={base.get('active')} isr_ok={base.get('isr_ok')} irq_dn={irq0_dn} irq_up={irq0_up}")

        for cmd in ("STOP", "MODE CLOSED", "SOFT ON", "RPMMAX 6000", "START", f"RPM {int(TARGET_RPM)}"):
            send(ser, cmd); time.sleep(0.15)
            acks = [ln for ln in drain(ser, 0.35) if ln.startswith("#")]
            log(f"CMD {cmd} -> {acks[:4]}")

        t0 = time.time()
        last_poll = -999.0
        last_ka = time.time()
        rpm_win = []
        hall_events = []

        while True:
            elapsed = time.time() - t0
            if elapsed >= HOLD_S:
                break
            # keepalive: resend RPM setpoint to refresh last_host_ms (<1.5s)
            if time.time() - last_ka >= KEEPALIVE_S:
                last_ka = time.time()
                send(ser, f"RPM {int(TARGET_RPM)}")
            for ln in drain(ser, 0.2):
                if "# HALL DOWN" in ln or "# HALL UP" in ln:
                    hall_events.append((elapsed, ln))
                    log(f"[t={elapsed:5.1f}s] EVENT {ln}")
                te = parse_telem(ln)
                if te and elapsed >= 6.0:
                    rpm_win.append((elapsed, te["rpm"]))
            if elapsed - last_poll >= POLL_S:
                last_poll = elapsed
                info, evs, _ = hall_query(ser)
                for e in evs:
                    hall_events.append((elapsed, e))
                    log(f"[t={elapsed:5.1f}s] EVENT {e}")
                dn = info.get("irq_dn"); up = info.get("irq_up")
                ddn = (dn - irq0_dn) if (dn is not None and irq0_dn is not None) else None
                dup = (up - irq0_up) if (up is not None and irq0_up is not None) else None
                log(f"[t={elapsed:5.1f}s] irq_dn={dn}(+{ddn}) irq_up={up}(+{dup})")
                log(f"         {info.get('status')}")
                if info.get("samp"):
                    log(f"         {info.get('samp')}")
            time.sleep(0.03)

        elapsed = time.time() - t0
        info, evs, _ = hall_query(ser)
        for e in evs:
            hall_events.append((elapsed, e))
            log(f"[t={elapsed:5.1f}s] EVENT {e}")
        dn = info.get("irq_dn"); up = info.get("irq_up")
        ddn = (dn - irq0_dn) if (dn is not None and irq0_dn is not None) else None
        dup = (up - irq0_up) if (up is not None and irq0_up is not None) else None
        log(f"[FINAL t={elapsed:.1f}s] irq_dn={dn}(+{ddn}) irq_up={up}(+{dup})")
        log(f"  status={info.get('status')}")
        log(f"  samp={info.get('samp')}")
        log(f"  meas={info.get('meas')}")

        stop_lines = safe_stop(ser)
        log(f"SAFE STOP -> {[x for x in stop_lines if x.startswith('#')][:8]}")

        settled = [r for t, r in rpm_win if t >= 8.0]
        mean_rpm = (sum(settled) / len(settled)) if settled else float("nan")
        mn = min(settled) if settled else float("nan")
        mx = max(settled) if settled else float("nan")

        log("--- SUMMARY ---")
        log(f"hold_s={HOLD_S:.0f} target={TARGET_RPM:.0f}")
        log(f"encoder RPM mean={mean_rpm:.1f} min={mn:.1f} max={mx:.1f} (n={len(settled)}, t>=8s)")
        log(f"irq baseline dn={irq0_dn} up={irq0_up}")
        log(f"irq final    dn={dn} up={up}  delta dn={ddn} up={dup}")
        log(f"low%: dn={info.get('dn_low_pct')} up={info.get('up_low_pct')} poll_dn={info.get('poll_dn')} poll_up={info.get('poll_up')}")
        log(f"# HALL events: {len(hall_events)}")
        for t, e in hall_events[:40]:
            log(f"  @{t:.1f}s {e}")

        spun = (mean_rpm == mean_rpm) and mean_rpm > 500
        irq_ok = (ddn is not None and dup is not None and (ddn > 0 or dup > 0))
        low_seen = (info.get("dn_low_pct") or 0) > 0.01 or (info.get("up_low_pct") or 0) > 0.01
        if irq_ok and spun:
            log(f"结论: 去内部上拉后，闭环约{mean_rpm:.0f}RPM 保持后 irq 增加(dn+{ddn}/up+{dup})、low%>0 —— 霍尔读到低电平，内部上拉为主因")
        elif spun and low_seen and not irq_ok:
            log("结论: low%>0 但 irq 未增 —— 查软件判沿方向(active LOW)")
        elif spun and not low_seen:
            log("结论: 去上拉后 low% 仍=0 —— 外部分压未把脚拉低，需硬件换 10k/20k 或查接线")
        else:
            log("结论: 电机未稳定到目标转速，需查闭环/ESC/保活")

        LOG.write_text("\n".join(out), encoding="utf-8")
        log(f"log: {LOG}")
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
