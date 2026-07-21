# -*- coding: utf-8 -*-
"""CLOSED 2000RPM >=30s + Hall IRQ; keepalive <1.5s host timeout."""
from __future__ import annotations
import re, sys, time
from datetime import datetime
from pathlib import Path
from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire

PORT="COM10"; TARGET=2000.0; HOLD=35.0; POLL=5.0
LOG=Path("logs")/f"hall_irq_closed2000_ka_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

def drain(ser,b=0.25):
    return read_lines(ser,b)

def parse_telem(line):
    if not line or line.startswith("#"): return None
    p=line.split(",")
    if len(p)<17: return None
    try:
        return dict(rpm=float(p[4]), pulse=int(float(p[9])), target=float(p[10]),
                    mode=int(float(p[11])), run=int(float(p[16])))
    except ValueError:
        return None

def hall_query(ser):
    send(ser,"HALL?")
    info={}; events=[]; t_end=time.time()+1.0
    while time.time()<t_end:
        send(ser,"PING")  # keepalive during drain
        for ln in drain(ser,0.2):
            if "# HALL DOWN" in ln or "# HALL UP" in ln: events.append(ln)
            if "irq_dn=" in ln:
                m=re.search(r"irq_dn=(\d+).*?irq_up=(\d+)",ln)
                if m: info["irq_dn"]=int(m.group(1)); info["irq_up"]=int(m.group(2))
                m2=re.search(r"\birq=(\S+)",ln)
                if m2: info["irq"]=m2.group(1)
                info["status"]=ln
            if ln.startswith("# HALL MEAS"): info["meas"]=ln
            if "ACK PING" in ln: pass
        if "irq_dn" in info: break
    return info, events

def safe_stop(ser):
    for c in ("RPM 0","STOP","STOP","PWM 1000"):
        send(ser,c); time.sleep(0.2)
    return [x for x in drain(ser,0.8) if x.startswith("#")]

def main():
    acquire("hall_irq_closed2000_ka")
    LOG.parent.mkdir(parents=True, exist_ok=True)
    out=[]
    def log(s):
        print(s,flush=True); out.append(s)

    log(f"=== CLOSED 2000 + keepalive + Hall {datetime.now().isoformat(timespec='seconds')} ===")
    ser=open_port(PORT)
    try:
        drain(ser,0.4)
        log(f"pre-stop: {safe_stop(ser)}")
        time.sleep(0.4); drain(ser,0.3)

        base,_=hall_query(ser)
        irq0_dn=base.get("irq_dn"); irq0_up=base.get("irq_up")
        log(f"[BASE] {base.get('status')}")
        log(f"[BASE] irq={base.get('irq')} dn={irq0_dn} up={irq0_up}")

        # clear ESTOP again right before arming
        send(ser,"STOP"); time.sleep(0.15)
        for cmd in ("MODE CLOSED","SOFT ON","RPMMAX 6000","START",f"RPM {int(TARGET)}"):
            send(ser,cmd); time.sleep(0.12)
            acks=[ln for ln in drain(ser,0.3) if ln.startswith("#")]
            log(f"CMD {cmd} -> {acks[:5]}")
            if cmd=="START" and any("ERR" in a for a in acks):
                send(ser,"STOP"); time.sleep(0.2); drain(ser,0.3)
                send(ser,"START"); time.sleep(0.15)
                acks=[ln for ln in drain(ser,0.3) if ln.startswith("#")]
                log(f"CMD START(retry) -> {acks[:5]}")

        t0=time.time(); last_poll=-999.0; last_ka=0.0
        rpm_settled=[]; hall_events=[]; pulses=[]

        while True:
            elapsed=time.time()-t0
            if elapsed>=HOLD: break
            # keepalive every 0.8s
            if elapsed-last_ka>=0.8:
                send(ser,"PING"); last_ka=elapsed
            for ln in drain(ser,0.15):
                if "# HALL DOWN" in ln or "# HALL UP" in ln:
                    hall_events.append((elapsed,ln)); log(f"[t={elapsed:5.1f}] EVENT {ln}")
                te=parse_telem(ln)
                if te:
                    if elapsed>=8.0: rpm_settled.append(te["rpm"])
                    if elapsed-last_ka<0.85: pulses.append((elapsed,te["pulse"],te["target"],te["run"],te["rpm"]))
            if elapsed-last_poll>=POLL:
                last_poll=elapsed
                # snapshot telem
                snap=[]
                for ln in drain(ser,0.2):
                    te=parse_telem(ln)
                    if te: snap.append(te)
                    if "# HALL DOWN" in ln or "# HALL UP" in ln:
                        hall_events.append((elapsed,ln)); log(f"[t={elapsed:5.1f}] EVENT {ln}")
                info,evs=hall_query(ser); last_ka=time.time()-t0
                for e in evs:
                    hall_events.append((elapsed,e)); log(f"[t={elapsed:5.1f}] EVENT {e}")
                dn=info.get("irq_dn"); up=info.get("irq_up")
                ddn=(dn-irq0_dn) if dn is not None and irq0_dn is not None else None
                dup=(up-irq0_up) if up is not None and irq0_up is not None else None
                if snap:
                    r=sum(x["rpm"] for x in snap)/len(snap)
                    pu=snap[-1]["pulse"]; tg=snap[-1]["target"]; run=snap[-1]["run"]
                else:
                    r=pu=tg=run=float("nan")
                log(f"[t={elapsed:5.1f}] rpm~{r:.1f} pulse={pu} tgt={tg} run={run} irq_dn={dn}(+{ddn}) irq_up={up}(+{dup})")
            time.sleep(0.03)

        elapsed=time.time()-t0
        info,evs=hall_query(ser)
        for e in evs:
            hall_events.append((elapsed,e)); log(f"[t={elapsed:5.1f}] EVENT {e}")
        dn=info.get("irq_dn"); up=info.get("irq_up")
        ddn=(dn-irq0_dn) if dn is not None and irq0_dn is not None else None
        dup=(up-irq0_up) if up is not None and irq0_up is not None else None
        mean_rpm=sum(rpm_settled)/len(rpm_settled) if rpm_settled else float("nan")
        mn=min(rpm_settled) if rpm_settled else float("nan")
        mx=max(rpm_settled) if rpm_settled else float("nan")
        log(f"[FINAL t={elapsed:.1f}] irq_dn={dn}(+{ddn}) irq_up={up}(+{dup})")
        log(f"  {info.get('status')}")
        log(f"  {info.get('meas')}")
        log(f"SAFE STOP: {safe_stop(ser)}")

        log("--- SUMMARY ---")
        log(f"hold={HOLD:.0f}s target={TARGET:.0f}")
        log(f"encoder RPM mean={mean_rpm:.1f} min={mn:.1f} max={mx:.1f} n={len(rpm_settled)} (t>=8s)")
        log(f"irq base dn={irq0_dn} up={irq0_up} -> final dn={dn} up={up}  delta +{ddn}/+{dup}")
        log(f"# HALL events={len(hall_events)}")
        for t,e in hall_events[:20]: log(f"  @{t:.1f} {e}")
        spun = rpm_settled and mean_rpm>500
        irq_ok = ddn is not None and dup is not None and (ddn>0 or dup>0)
        if irq_ok and spun:
            log(f"结论: 闭环约{mean_rpm:.0f}RPM 保持{HOLD:.0f}s，irq增加 —— 中断已通")
        elif spun and not irq_ok:
            log(f"结论: 编码器约{mean_rpm:.0f}RPM 持续转动{HOLD:.0f}s，irq未增加、无#HALL —— 中断未通（非静止误判）")
        else:
            log("结论: 未稳定转起来，不作静止读不到结论")
        LOG.write_text("\n".join(out),encoding="utf-8")
        log(f"log: {LOG.resolve()}")
        return 0 if irq_ok else 2
    except Exception as exc:
        try: safe_stop(ser)
        except Exception: pass
        log(f"ERROR:{exc}"); LOG.write_text("\n".join(out),encoding="utf-8"); raise
    finally:
        try: ser.close()
        except Exception: pass

if __name__=="__main__":
    sys.exit(main())
