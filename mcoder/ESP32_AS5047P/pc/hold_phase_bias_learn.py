#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Post-stop est_disc vs Hall truth bias diagnostic + self-learn (COM10)."""
from __future__ import annotations
import argparse, os, re, sys, time
from datetime import datetime
from pathlib import Path
os.environ.setdefault("PYTHONIOENCODING", "utf-8")
from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire, release
from motor_spin_detect import detect_spin

PORT, LOGDIR = "COM10", Path(__file__).resolve().parent / "logs"
_out: list[str] = []

def log(s: str) -> None:
    print(s, flush=True); _out.append(s)

def drain(ser, budget=0.15):
    lines = read_lines(ser, budget)
    for ln in lines:
        if ln.startswith("#") or (ln and not ln[0:1].isdigit()):
            log(ln)
    return lines

def wait_ack(ser, pred, timeout=2.0):
    end = time.time() + timeout; got = []
    while time.time() < end:
        for ln in drain(ser, 0.12):
            got.append(ln)
            if pred(ln): return got
        time.sleep(0.02)
    return got

def parse_kv(line, key):
    m = re.search(rf"{re.escape(key)}=([\-\d.]+(?:[eE][\-+]?\d+)?)", line)
    return float(m.group(1)) if m else None

def parse_kv_str(line, key):
    m = re.search(rf"{re.escape(key)}=([^\s]+)", line)
    return m.group(1) if m else None

def wrap180(d):
    while d > 180.0: d -= 360.0
    while d < -180.0: d += 360.0
    return d

def wrap360(d):
    while d < 0.0: d += 360.0
    while d >= 360.0: d -= 360.0
    return d

def in_band(disc, target, tol=5.0):
    return abs(wrap180(disc - target)) <= tol

def safe_stop(ser):
    for c in ("ESTOP", "STOP", "RPM 0", "PWM 1000"):
        send(ser, c); drain(ser, 0.12)

def parse_telem(line):
    if not line or line.startswith("#"): return None
    p = line.split(",")
    if len(p) < 17: return None
    try: return {"rpm": float(p[4])}
    except (ValueError, IndexError): return None

def read_hall(ser):
    send(ser, "HALL?")
    lines = wait_ack(ser, lambda s: "irq_dn=" in s or ("cal=" in s and "gear=" in s), 2.0)
    lines = list(lines) + list(drain(ser, 0.2))
    out = {"cal": 0, "irq_dn": 0, "irq_up": 0}
    for ln in lines:
        for k in ("irq_dn", "irq_up", "cal"):
            v = parse_kv(ln, k)
            if v is not None: out[k] = int(v)
    return out

def read_phase(ser):
    send(ser, "PHASE?")
    lines = wait_ack(ser, lambda s: "disc=" in s or "disc_deg=" in s, 1.5)
    blob = " ".join(lines)
    return {
        "disc": parse_kv(blob, "disc_deg") or parse_kv(blob, "disc"),
        "disc_ref": parse_kv(blob, "disc_ref"),
        "offset": parse_kv(blob, "offset"),
        "brake_k": parse_kv(blob, "brake_k"),
    }

def read_brakek(ser):
    send(ser, "PHASE BRAKEK?")
    lines = wait_ack(ser, lambda s: "BRAKEK" in s or "k=" in s, 1.2)
    blob = " ".join(lines)
    return {"k": parse_kv(blob, "k") or parse_kv(blob, "brake_k"), "learn_n": parse_kv(blob, "learn_n")}

def hold_until_rpm(ser, target, settle_s, band=350.0):
    t0 = time.time(); last_ka = 0.0; win = []; ok_since = None
    while time.time() - t0 < 45.0:
        now = time.time()
        if now - last_ka >= 0.45:
            send(ser, f"RPM {target:.0f}"); last_ka = now
        for ln in drain(ser, 0.08):
            t = parse_telem(ln)
            if not t: continue
            rpm = abs(t["rpm"]); win.append(rpm)
            if len(win) > 40: win.pop(0)
            if abs(rpm - target) <= band:
                if ok_since is None: ok_since = now
                elif now - ok_since >= settle_s and len(win) >= 8:
                    return {"ok": True, "mean": sum(win)/len(win)}
            else:
                ok_since = None
        time.sleep(0.01)
    return {"ok": False, "mean": sum(win)/len(win) if win else 0.0}

def run_hold(ser, target, crawl, stop_ms, timeout=90.0):
    send(ser, f"HOLD PHASE {target:g} {crawl:g} {stop_ms}")
    log(f">> HOLD PHASE {target:g} {crawl:g} {stop_ms}")
    t0 = time.time(); brake = done = softdone = ""; latches = []
    rpm_at_hold_end = None; remain_soft = None
    while time.time() - t0 < timeout:
        for ln in drain(ser, 0.12):
            if ("HALL DOWN" in ln or "HALL UP" in ln) and "motor_cnt=" in ln:
                latches.append(ln); log(f"latch: {ln}")
            if "SOFTDONE" in ln:
                softdone = ln
                remain_soft = parse_kv(ln, "remain")
                rpm_at_hold_end = parse_kv(ln, "rpm")
                log(f"softdone: {ln}")
            if "BRAKE" in ln and "HOLD PHASE" in ln and "BRAKEK" not in ln:
                brake = ln
            if "BRAKEK learn" in ln:
                log(f"learn: {ln}")
            if "HOLD PHASE DONE" in ln or ln.startswith("# ESTOP") or "ERR HOLD PHASE" in ln:
                return {"ok": "DONE" in ln, "done": ln, "brake": brake, "latches": latches,
                        "softdone": softdone, "rpm_at_hold_end": rpm_at_hold_end,
                        "remain_soft": remain_soft}
        time.sleep(0.02)
    return {"ok": False, "done": "TIMEOUT", "brake": brake, "latches": latches,
            "softdone": softdone, "rpm_at_hold_end": rpm_at_hold_end, "remain_soft": remain_soft}

def crawl_to_hall(ser, target_deg, crawl_rpm=90.0, max_s=25.0):
    want0 = abs(wrap180(target_deg - 0.0)) < 1.0
    want180 = abs(wrap180(target_deg - 180.0)) < 1.0
    h0 = read_hall(ser); dn0, up0 = h0["irq_dn"], h0["irq_up"]
    ph0 = read_phase(ser); est_at_start = ph0.get("disc")
    log(f"--- crawl_to_hall target={target_deg} est={est_at_start} dn={dn0} up={up0} ---")
    send(ser, "MODE CLOSED"); send(ser, "SOFT ON"); send(ser, "TELEM ON")
    send(ser, f"RPM {crawl_rpm:.0f}")
    t0 = time.time(); hit = None; hall_disc = None; motor_cnt = None; last_ka = 0.0
    while time.time() - t0 < max_s:
        now = time.time()
        if now - last_ka >= 0.4:
            send(ser, f"RPM {crawl_rpm:.0f}"); last_ka = now
        for ln in drain(ser, 0.1):
            if "HALL DOWN" in ln and "motor_cnt=" in ln and want0:
                hit, hall_disc, motor_cnt = "GPIO4/0", 0.0, parse_kv(ln, "motor_cnt")
                log(f"hall_hit: {ln}"); break
            if "HALL UP" in ln and "motor_cnt=" in ln and want180:
                hit, hall_disc, motor_cnt = "GPIO5/180", 180.0, parse_kv(ln, "motor_cnt")
                log(f"hall_hit: {ln}"); break
            if "HALL UP" in ln and want0:
                log(f"opp_hall: {ln}")
            if "HALL DOWN" in ln and want180:
                log(f"opp_hall: {ln}")
        if hit: break
        if time.time() - t0 > 1.0:
            h = read_hall(ser)
            if want0 and h["irq_dn"] > dn0:
                hit, hall_disc = "GPIO4/0", 0.0
                log(f"hall_hit irq_dn {dn0}->{h['irq_dn']}"); break
            if want180 and h["irq_up"] > up0:
                hit, hall_disc = "GPIO5/180", 180.0
                log(f"hall_hit irq_up {up0}->{h['irq_up']}"); break
    safe_stop(ser); time.sleep(0.25)
    ph1 = read_phase(ser)
    # Never adopt opposite Hall disc_ref as truth (e.g. target=0 but crawl passed GPIO5 first)
    if hall_disc is None and ph1.get("disc_ref") is not None:
        ref = float(ph1["disc_ref"])
        if want0 and abs(wrap180(ref - 0.0)) <= 1.0:
            hall_disc = 0.0; hit = hit or "ref=0"
        elif want180 and abs(wrap180(ref - 180.0)) <= 1.0:
            hall_disc = 180.0; hit = hit or "ref=180"
        else:
            log(f"ignore opp disc_ref={ref} (want0={want0} want180={want180})")
    # If still no hall: use target angle as diagnostic truth when DONE had hall_confirm
    return {"hit": hit, "hall_disc": hall_disc, "est_at_crawl_start": est_at_start,
            "disc_after": ph1.get("disc"), "disc_ref": ph1.get("disc_ref"),
            "motor_cnt": motor_cnt, "ok": hall_disc is not None, "dt": time.time()-t0}

def classify(est_disc, hall_disc, stop_err, bias):
    if est_disc is None and stop_err is None: return "UNKNOWN"
    ae = abs(stop_err) if stop_err is not None else 999.0
    # Prefer stop error vs target when Hall truth missing/suspect
    if bias is None:
        if ae <= 5.0: return "OK"
        if ae <= 15.0: return "STRATEGY"
        return "UNKNOWN"
    ab = abs(bias)
    # Good/near landing: strategy/OK — do NOT call FRAME from a bad crawl hall
    if ae <= 5.0 and ab <= 20.0: return "OK"
    if ae <= 15.0: return "STRATEGY"
    if ab <= 15.0 and ae > 15.0: return "STRATEGY"
    if ab >= 140.0 and ae >= 90.0: return "ESTIMATION_FRAME"
    if ab >= 140.0: return "ESTIMATION"  # large crawl bias but stop was not opposite
    if ab >= 30.0: return "ESTIMATION"
    return "MIXED"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--target", type=float, default=0.0)
    ap.add_argument("--rpm", type=float, default=2500.0)
    ap.add_argument("--iters", type=int, default=5)
    ap.add_argument("--crawl", type=float, default=150.0)
    ap.add_argument("--hall-crawl", type=float, default=90.0)
    ap.add_argument("--stop-ms", type=int, default=2500)
    ap.add_argument("--tol", type=float, default=5.0)
    ap.add_argument("--settle", type=float, default=2.5)
    args = ap.parse_args()
    LOGDIR.mkdir(parents=True, exist_ok=True)
    log_path = LOGDIR / f"hold_phase_bias_learn_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    acquire("hold_phase_bias_learn")
    ser = None; rc = 1; rows = []; offset = 0.0; flipped = False
    try:
        log(f"=== BIAS LEARN {datetime.now().isoformat(timespec='seconds')} ===")
        log(f"target={args.target} rpm={args.rpm} iters={args.iters} crawl={args.crawl} stop_ms={args.stop_ms} soft_down=500 hold_to=90s")
        ser = open_port(args.port); time.sleep(0.7); ser.reset_input_buffer()
        send(ser, "TELEM OFF"); drain(ser, 0.25)
        send(ser, "HOST OFF"); wait_ack(ser, lambda s: "HOST OFF" in s, 1.5)
        send(ser, "FIXED?"); wait_ack(ser, lambda s: "FIXED" in s, 1.0)
        send(ser, "ENC?"); wait_ack(ser, lambda s: "ENC" in s or "collect" in s or "Hz" in s, 1.2)
        # float path; enc collect 2k / ctrl 400; gentler soft-down (keep crawl via FW OPEN pulse)
        for c in ("MODE CLOSED","SOFT ON","SOFT RATE UP 700","SOFT RATE DOWN 500","RPMMAX 12000","ENC 2000","FREQ 400","TELEM ON"):
            send(ser, c); drain(ser, 0.08)
        send(ser, "PHASE PLAN?"); wait_ack(ser, lambda s: "PLAN" in s or "brake_k=" in s, 1.2)
        send(ser, "PHASE REFINE ON"); wait_ack(ser, lambda s: "REFINE" in s, 1.0)
        send(ser, f"PHASE TOL {args.tol:g}"); wait_ack(ser, lambda s: "TOL" in s, 1.0)
        send(ser, "PHASE OFFSET 0"); wait_ack(ser, lambda s: "OFFSET" in s, 1.0)
        send(ser, "RPM 2000")
        gate = detect_spin(ser, seconds=3.0, keepalive="RPM 2000")
        log(f"gate {gate.summary_line()}")
        h = read_hall(ser)
        log(f"Hall cal={h['cal']} irq_dn={h['irq_dn']} irq_up={h['irq_up']}")
        if h["irq_dn"] == 0 and h["irq_up"] == 0:
            time.sleep(0.2); h = read_hall(ser)
        if h["irq_dn"] == 0 and h["irq_up"] == 0:
            log("BLOCK: Hall dead"); safe_stop(ser)
            log_path.write_text("\n".join(_out)+"\n", encoding="utf-8"); return 2
        for it in range(1, args.iters+1):
            log(f"\n======== ITER {it}/{args.iters} offset={offset:.0f} ========")
            safe_stop(ser); time.sleep(0.3)
            send(ser, "STOP"); drain(ser, 0.15)
            send(ser, "MODE CLOSED"); send(ser, "SOFT ON")
            send(ser, "SOFT RATE UP 700"); send(ser, "SOFT RATE DOWN 500")
            send(ser, f"PHASE OFFSET {offset:.1f}")
            send(ser, "PHASE REFINE ON"); send(ser, f"PHASE TOL {args.tol:g}")
            send(ser, "TELEM ON"); drain(ser, 0.15)
            send(ser, "RPM 2000"); detect_spin(ser, seconds=1.4, keepalive="RPM 2000")
            send(ser, f"RPM {args.rpm:.0f}")
            st = hold_until_rpm(ser, args.rpm, args.settle)
            log(f"stable ok={st['ok']} mean={st.get('mean',0):.1f}")
            if not st["ok"]:
                log("FAIL stabilize"); break
            bk_b = read_brakek(ser)
            send(ser, "TELEM OFF"); drain(ser, 0.1)
            r = run_hold(ser, args.target, args.crawl, args.stop_ms)
            log(f"done: {r['done']}")
            if r["brake"]: log(f"brake: {r['brake']}")
            safe_stop(ser); time.sleep(0.2)
            ph = read_phase(ser)
            est_disc = parse_kv(r["done"], "disc")
            if est_disc is None: est_disc = ph.get("disc")
            stop_err = parse_kv(r["done"], "err")
            if stop_err is None and est_disc is not None:
                stop_err = wrap180(est_disc - args.target)
            reason = parse_kv_str(r["done"], "reason")
            hall_c = parse_kv(r["done"], "hall_confirm")
            refine = parse_kv(r["done"], "refine")
            log(f"est_disc={est_disc} stop_err={stop_err} reason={reason} refine={refine} hall_c={hall_c}")
            crawl = crawl_to_hall(ser, args.target, crawl_rpm=args.hall_crawl)
            hall_disc = crawl.get("hall_disc")
            if hall_disc is None and hall_c:
                hall_disc = args.target  # DONE already hall-confirmed at target marker
                log(f"hall_disc fallback to target={args.target} (hall_confirm)")
            bias = wrap180(est_disc - hall_disc) if (est_disc is not None and hall_disc is not None) else (
                wrap180(est_disc - args.target) if est_disc is not None else None)
            klass = classify(est_disc, hall_disc, stop_err, bias)
            bk_a = read_brakek(ser)
            row = {"iter":it,"est_disc":est_disc,"hall_disc":hall_disc,"bias":bias,"stop_err":stop_err,
                   "class":klass,"reason":reason,"refine":refine,"hall_confirm":hall_c,
                   "k_before":bk_b["k"],"k_after":bk_a["k"],"offset":offset,"hit":crawl.get("hit"),
                   "motor_cnt":crawl.get("motor_cnt"),"band": in_band(est_disc,args.target,args.tol) if est_disc is not None else False}
            log(f"BIAS est={est_disc} hall={hall_disc} bias={bias} stop_err={stop_err} class={klass} k {bk_b['k']}->{bk_a['k']} offset={offset}")
            remain_crawl = r.get("remain_soft")
            if remain_crawl is None and stop_err is not None:
                remain_crawl = abs(stop_err)
            rpm_end = r.get("rpm_at_hold_end")
            latch_age = None
            if r.get("latches"):
                last = r["latches"][-1]
                latch_age = parse_kv(last, "t_ms")
            log(f"diag remain_soft={remain_crawl} est_disc={est_disc} bias={bias} rpm_at_softdone={rpm_end} hall_latch_t_ms={latch_age} n_latch={len(r.get('latches') or [])}")
            red = False; red_lvl = ""; red_branch = ""
            ae = abs(stop_err) if stop_err is not None else 0.0
            rcrawl = abs(remain_crawl) if remain_crawl is not None else 0.0
            rpm_hi = rpm_end is not None and rpm_end > 1000
            rpm_lo = rpm_end is not None and rpm_end < 50
            if ae > 90 or rcrawl > 90:
                red = True
                if ae > 200 or rcrawl > 200:
                    red_lvl = ">200"
                elif ae > 120 or rcrawl > 120:
                    red_lvl = ">120"
                else:
                    red_lvl = ">90"
                if rpm_lo and rcrawl > 90:
                    red_branch = "B"; log(f"*** 红旗 RED FLAG {red_lvl} branch=B *** 软降→RPM≈0 且 remain≫90")
                elif rpm_hi and rcrawl > 90:
                    red_branch = "B"; log(f"*** 红旗 RED FLAG {red_lvl} branch=B *** 软降未入爬行带(仍高速) softdone.remain={rcrawl:.1f} rpm={rpm_end}")
                elif ae >= 140:
                    red_branch = "A"; log(f"*** 红旗 RED FLAG {red_lvl} branch=A *** 估计坐标系/近180°帧偏差")
                else:
                    red_branch = "C"; log(f"*** 红旗 RED FLAG {red_lvl} branch=C *** 策略/监督/切油时机")
            row["red"] = red; row["red_lvl"] = red_lvl; row["red_branch"] = red_branch
            row["remain_crawl"] = remain_crawl
            row["rpm_at_hold_end"] = rpm_end; row["hall_latch_t_ms"] = latch_age
            action = "none"
            # One-shot ~180 OFFSET flip is dangerous (iter4 false FRAME); need prior FRAME too
            frame_hits = sum(1 for x in rows if x.get("class") == "ESTIMATION_FRAME")
            if klass == "OK" or (row["band"] and abs(bias or 99) <= 15):
                rc = 0; action = "pass"; log("PASS")
            elif (klass == "ESTIMATION_FRAME" and abs(bias or 0) >= 140
                  and (stop_err is None or abs(stop_err) >= 90) and not flipped
                  and frame_hits >= 1):
                offset = wrap360(offset + 180.0); flipped = True
                send(ser, f"PHASE OFFSET {offset:.1f}"); wait_ack(ser, lambda s: "OFFSET" in s, 1.0)
                action = f"OFFSET->{offset:.0f}"
                log(f"meta-learn: PHASE OFFSET={offset:.0f} (2× FRAME)")
            elif klass == "ESTIMATION_FRAME":
                action = "frame_watch"; log("红旗观察: 单次 FRAME 不自动 OFFSET（防误翻半圈）")
            elif klass == "STRATEGY":
                action = "trust_k_EMA"; log("自学习: 策略问题→依赖 brake_k EMA")
            row["action"] = action
            rows.append(row)
            safe_stop(ser); time.sleep(0.4)
        send(ser, "HOST OFF"); drain(ser, 0.2); safe_stop(ser)
        log("\n======== 中文偏差表 ========")
        log("| iter | est_disc | hall_disc | bias | stop_err | remain_crawl | rpm_soft | red | class | k | offset | action |")
        for r in rows:
            log(f"| {r['iter']} | {r['est_disc']} | {r['hall_disc']} | {r['bias']} | {r['stop_err']} | {r.get('remain_crawl')} | {r.get('rpm_at_hold_end')} | {r.get('red_lvl') or ('RED' if r.get('red') else 'ok')} | {r['class']} | {r['k_before']}→{r['k_after']} | {r['offset']} | {r.get('action')} |")
        if rows:
            biases = [r["bias"] for r in rows if r["bias"] is not None]
            if biases:
                log("bias序列: " + " → ".join(f"{b:+.1f}" for b in biases))
            log(f"最终 offset={offset} flipped={flipped} brake_k={rows[-1]['k_after']}")
        log(f"日志: {log_path}")
        log_path.write_text("\n".join(_out)+"\n", encoding="utf-8")
        return rc if rows else 1
    except Exception as e:
        log(f"EXCEPTION: {e}")
        try:
            if ser: safe_stop(ser)
        except Exception: pass
        log_path.write_text("\n".join(_out)+"\n", encoding="utf-8"); return 99
    finally:
        try:
            if ser is not None:
                try:
                    send(ser, "HOST OFF"); drain(ser, 0.15); safe_stop(ser)
                except Exception: pass
                try: ser.close()
                except Exception: pass
        finally:
            release()

if __name__ == "__main__":
    sys.exit(main())
