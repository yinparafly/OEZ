# -*- coding: utf-8 -*-
"""越过旧 7500RPM 折叠界：分段 6000→8000，确认编码器测速仍不折叠、与霍尔吻合。
安全：keepalive 0.5s，短保持，异常/结束 ESTOP→STOP→PWM1000。"""
from __future__ import annotations
import re, sys, time
from datetime import datetime
from pathlib import Path
from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire

PORT = "COM10"; GEAR = 27.744
STAGES = [6000, 8000]
HOLD_S = 12.0; KA_S = 0.5
LOGDIR = Path(__file__).resolve().parent / "logs"
LOG = LOGDIR / f"enc_cross7500_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
_o = []
def log(s): print(s, flush=True); _o.append(s)
def drain(ser, b=0.15): return read_lines(ser, b)

def parse(line):
    if not line or line.startswith("#"): return None
    p = line.split(",")
    if len(p) < 25: return None
    try: return {"rpm": float(p[4]), "out_hz": float(p[22]), "enc_hz": float(p[24])}
    except Exception: return None

def enc_q(ser):
    drain(ser, 0.05); send(ser, "ENC?"); end = time.time()+1.0
    while time.time() < end:
        for ln in drain(ser, 0.15):
            if ln.startswith("# ENC"):
                m = re.search(r"meas_hz=([\d.]+).*?cpu%=([\d.]+).*?loop_hz=([\d.]+).*?rpm=([\-\d.]+)", ln)
                if m: return ln, float(m.group(1)), float(m.group(2)), float(m.group(4))
    return None, None, None, None

def safe_stop(ser):
    for c in ("RPM 0","STOP","PWM 1000"): send(ser,c); time.sleep(0.2)
    drain(ser,0.5)

def main():
    acquire("enc_cross7500"); LOGDIR.mkdir(parents=True, exist_ok=True)
    log(f"=== cross-7500 verify {datetime.now().isoformat(timespec='seconds')} stages={STAGES} ===")
    ser = open_port(PORT)
    try:
        drain(ser,0.5); safe_stop(ser); time.sleep(0.3)
        for c in ("STOP","MODE CLOSED","SOFT ON","SOFT RATE UP 900","SOFT RATE DOWN 700","RPMMAX 9000","START"):
            send(ser,c); time.sleep(0.2)
            log(f"CMD {c} -> {[x for x in drain(ser,0.3) if x.startswith('#')][:2]}")
        res = []
        for stg in STAGES:
            log(f"\n=== STAGE {stg} ===")
            send(ser, f"RPM {stg}"); t0=time.time(); lka=0; lp=0; win=[]; oh=-1
            while time.time()-t0 < HOLD_S:
                n=time.time()
                if n-lka>=KA_S: lka=n; send(ser,f"RPM {stg}")
                for ln in drain(ser,0.15):
                    te=parse(ln)
                    if te:
                        if n-t0>=5.0: win.append(te["rpm"])
                        if te["out_hz"]>0: oh=te["out_hz"]
                if n-lp>=3.0:
                    lp=n; ln2,hz,cpu,r=enc_q(ser)
                    log(f"[t={n-t0:4.1f}s] enc_hz={hz} cpu%={cpu} enc_rpm={r}")
                time.sleep(0.02)
            mean=sum(win)/len(win) if win else float('nan')
            mn=min(win) if win else float('nan'); mx=max(win) if win else float('nan')
            hall=oh*GEAR*60 if oh>0 else float('nan')
            res.append((stg,mean,mn,mx,mx-mn if win else float('nan'),oh,hall))
            log(f"[STAGE {stg}] enc_mean={mean:.0f} min={mn:.0f} max={mx:.0f} spread={(mx-mn) if win else 0:.0f} out_hz={oh:.3f} hall_rpm={hall:.0f}")
        for r in (5000,2500):
            for _ in range(5): send(ser,f"RPM {r}"); drain(ser,0.15); time.sleep(0.35)
        safe_stop(ser)
        log("\n===== SUMMARY (cross 7500) =====")
        for (stg,mean,mn,mx,sp,oh,hall) in res:
            err = abs(hall-mean)/mean*100 if (mean==mean and mean>0 and hall==hall) else float('nan')
            over = "OVER-7500✓" if mean>7500 else ""
            log(f"  RPM{stg}: enc_mean={mean:.0f} spread={sp:.0f} hall_rpm={hall:.0f} err={err:.1f}% {over}")
        LOG.write_text("\n".join(_o),encoding="utf-8"); log(f"log={LOG}")
        return 0
    except Exception as e:
        try: safe_stop(ser)
        except Exception: pass
        log(f"ERROR {e}"); LOG.write_text("\n".join(_o),encoding="utf-8"); raise
    finally:
        try: ser.close()
        except Exception: pass

if __name__=="__main__": sys.exit(main())
