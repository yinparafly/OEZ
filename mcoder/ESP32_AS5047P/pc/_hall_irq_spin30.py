# -*- coding: utf-8 -*-
"""One-shot: Hall IRQ while motor spinning 25-30s (large gear ratio)."""
from __future__ import annotations
import re, sys, time
from pathlib import Path
from datetime import datetime

from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire

PORT = "COM10"
PWM = 1350
HOLD_S = 28.0
POLL_S = 4.0
LOG = Path(__file__).resolve().parent / "logs" / f"hall_irq_spin_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

def drain(ser, budget=0.35):
    return read_lines(ser, budget)

def hall_query(ser, label=""):
    send(ser, "HALL?")
    lines = drain(ser, 0.6)
    # also catch delayed lines
    lines += drain(ser, 0.25)
    info = {}
    hall_events = []
    for ln in lines:
        if "# HALL DOWN" in ln or "# HALL UP" in ln:
            hall_events.append(ln)
        if "irq_dn=" in ln or "irq_mode=" in ln or ln.startswith("# HALL"):
            m = re.search(r"irq_dn=(\d+).*?irq_up=(\d+)", ln)
            if m:
                info["irq_dn"] = int(m.group(1))
                info["irq_up"] = int(m.group(2))
            m2 = re.search(r"irq_mode=(\S+)", ln)
            if m2:
                info["irq_mode"] = m2.group(1)
            m3 = re.search(r"active=(\S+)", ln)
            if m3:
                info["active"] = m3.group(1)
            m4 = re.search(r"dn_lvl=(\d+).*?up_lvl=(\d+)", ln)
            if m4:
                info["dn_lvl"] = int(m4.group(1))
                info["up_lvl"] = int(m4.group(2))
            info["raw"] = ln
    return info, hall_events, lines

def safe_stop(ser):
    send(ser, "STOP")
    time.sleep(0.15)
    send(ser, "PWM 1000")
    time.sleep(0.2)
    return drain(ser, 0.5)

def main():
    acquire("hall_irq_spin_30s")
    LOG.parent.mkdir(parents=True, exist_ok=True)
    out = []
    def log(s):
        print(s, flush=True)
        out.append(s)

    log(f"=== Hall IRQ spin test {datetime.now().isoformat(timespec='seconds')} ===")
    log(f"port={PORT} pwm={PWM} hold={HOLD_S}s poll={POLL_S}s gear~27.744")
    ser = open_port(PORT)
    try:
        drain(ser, 0.4)
        send(ser, "PING")
        ping = drain(ser, 0.5)
        for ln in ping:
            if "ACK" in ln or "ctrl_hz" in ln or "ESP32" in ln:
                log(f"  {ln}")

        # baseline
        base, ev, raw = hall_query(ser, "baseline")
        log(f"[t=0 BASE] HALL? -> {base.get('raw', raw)}")
        for e in ev:
            log(f"  EVENT {e}")
        irq0_dn = base.get("irq_dn", None)
        irq0_up = base.get("irq_up", None)
        log(f"  irq_mode={base.get('irq_mode')} active={base.get('active')} irq_dn={irq0_dn} irq_up={irq0_up}")
        if base.get("irq_mode") and base.get("irq_mode") != "FALLING":
            log("WARN: irq_mode is not FALLING — firmware may need flash_com10.bat")

        # start spin
        for cmd in ("MODE OPEN", "START", f"PWM {PWM}"):
            send(ser, cmd)
            time.sleep(0.12)
            ack = drain(ser, 0.25)
            log(f"CMD {cmd} -> {ack[:6]}")

        t0 = time.time()
        samples = []
        hall_evt_all = []
        last_poll = -999.0
        rpm_samples = []

        while True:
            elapsed = time.time() - t0
            if elapsed >= HOLD_S:
                break
            lines = drain(ser, 0.2)
            for ln in lines:
                if "# HALL DOWN" in ln or "# HALL UP" in ln:
                    hall_evt_all.append((elapsed, ln))
                    log(f"[t={elapsed:5.1f}s] EVENT {ln}")
                elif not ln.startswith("#"):
                    parts = ln.split(",")
                    if len(parts) >= 5:
                        try:
                            rpm = float(parts[4])
                            rpm_samples.append((elapsed, rpm))
                        except ValueError:
                            pass
            if elapsed - last_poll >= POLL_S:
                last_poll = elapsed
                info, ev, raw = hall_query(ser, f"t={elapsed:.1f}")
                # hall_query drains; events may be in raw
                for e in ev:
                    hall_evt_all.append((elapsed, e))
                    log(f"[t={elapsed:5.1f}s] EVENT {e}")
                for ln in raw:
                    if "# HALL DOWN" in ln or "# HALL UP" in ln:
                        if (elapsed, ln) not in hall_evt_all:
                            hall_evt_all.append((elapsed, ln))
                            log(f"[t={elapsed:5.1f}s] EVENT {ln}")
                dn = info.get("irq_dn")
                up = info.get("irq_up")
                samples.append((elapsed, dn, up, info.get("raw")))
                ddn = (dn - irq0_dn) if (dn is not None and irq0_dn is not None) else None
                dup = (up - irq0_up) if (up is not None and irq0_up is not None) else None
                log(f"[t={elapsed:5.1f}s] HALL? irq_dn={dn}(+{ddn}) irq_up={up}(+{dup}) raw={info.get('raw')}")
            time.sleep(0.05)

        # final HALL?
        info, ev, raw = hall_query(ser, "final")
        elapsed = time.time() - t0
        for e in ev:
            hall_evt_all.append((elapsed, e))
            log(f"[t={elapsed:5.1f}s] EVENT {e}")
        dn = info.get("irq_dn")
        up = info.get("irq_up")
        samples.append((elapsed, dn, up, info.get("raw")))
        ddn = (dn - irq0_dn) if (dn is not None and irq0_dn is not None) else None
        dup = (up - irq0_up) if (up is not None and irq0_up is not None) else None
        log(f"[t={elapsed:5.1f}s FINAL] irq_dn={dn}(+{ddn}) irq_up={up}(+{dup})")
        log(f"  raw={info.get('raw')}")

        # stop
        stop_lines = safe_stop(ser)
        log(f"SAFE STOP -> {stop_lines[:8]}")

        # rpm summary
        if rpm_samples:
            rpms = [r for _, r in rpm_samples if r > 50]
            if rpms:
                mean_rpm = sum(rpms) / len(rpms)
                out_hz = mean_rpm / 27.744 / 60.0
                expect_edges = out_hz * HOLD_S  # per channel ~1 per rev
                log(f"RPM mean~{mean_rpm:.0f} (n={len(rpms)}) => out_Hz~{out_hz:.2f} => ~{expect_edges:.1f} edges/ch in {HOLD_S:.0f}s")
            else:
                log("WARN: no meaningful RPM samples (motor may not have spun)")
        else:
            log("WARN: no telem RPM parsed")

        log("--- SUMMARY ---")
        log(f"hold_s={HOLD_S:.0f} pwm={PWM}")
        log(f"irq baseline dn={irq0_dn} up={irq0_up}")
        log(f"irq final    dn={dn} up={up}  delta dn={ddn} up={dup}")
        log(f"# HALL events captured: {len(hall_evt_all)}")
        for t, e in hall_evt_all[:20]:
            log(f"  @{t:.1f}s {e}")
        ok = (ddn is not None and dup is not None and (ddn > 0 or dup > 0))
        if ok:
            log("结论: 电机持续转动后 irq 增加 —— 中断路径已通")
        elif rpm_samples and any(r > 200 for _, r in rpm_samples):
            log("结论: 电机有转速但 irq 未增加 —— 中断可能未通或沿极性/线序问题")
        else:
            log("结论: 未能确认电机有效转动，不作「读不到」静止结论；需检查 ESC/START")
        LOG.write_text("\n".join(out), encoding="utf-8")
        log(f"log saved: {LOG}")
        return 0 if ok else 2
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
