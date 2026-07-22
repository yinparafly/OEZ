# -*- coding: utf-8 -*-
"""不转排查：安全停车 → 状态查询 → 开环短试 → 闭环2000；均用 motor_spin_detect 自判。"""
from __future__ import annotations

import os
import sys
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("PYTHONIOENCODING", "utf-8")

from auto_learn_test import open_port, send, read_lines
from com_port_guard import acquire, release
from motor_spin_detect import (
    arm_closed,
    arm_open_pwm,
    detect_spin,
    drain,
    safe_stop,
)

PORT = "COM10"
LOGDIR = Path(__file__).resolve().parent / "logs"
STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG = LOGDIR / f"diag_nospin_{STAMP}.txt"
_out: list[str] = []


def log(s: str) -> None:
    print(s, flush=True)
    _out.append(s)


def cmd(ser, c: str, wait=0.35):
    send(ser, c)
    time.sleep(wait)
    acks = [x for x in drain(ser, wait) if x.startswith("#")]
    for a in acks[:5]:
        log(f"  [{c}] {a}")
    return acks


def main() -> int:
    acquire("diag_nospin")
    LOGDIR.mkdir(parents=True, exist_ok=True)
    log(f"=== DIAG no-spin + spin_detect {datetime.now().isoformat(timespec='seconds')} ===")
    ser = open_port(PORT)
    results = {}
    try:
        drain(ser, 0.5)
        log("--- 安全停车 ---")
        safe_stop(ser)
        time.sleep(0.3)

        log("--- 静态查询 ---")
        for c in ("PING", "CTRL?", "ENC?", "LOAD?", "TELEM?", "FREQ?", "PROTO?"):
            cmd(ser, c, 0.35)

        log("\n--- IDLE 3s ---")
        r = detect_spin(ser, seconds=3.0, keepalive="PING")
        log(r.summary_line())
        results["idle"] = r

        log("\n--- OPEN PWM1200 ~3.5s ---")
        safe_stop(ser)
        arm_open_pwm(ser, 1200)
        r = detect_spin(ser, seconds=3.5, keepalive="PWM 1200")
        log(r.summary_line())
        results["openloop"] = r
        send(ser, "PWM 1000")
        time.sleep(0.15)
        safe_stop(ser)
        time.sleep(0.4)

        log("\n--- CLOSED RPM2000（斜坡等待+检测窗 5s）---")
        arm_closed(ser, 2000.0, soft_up=1000.0)
        # 斜坡预热，保活
        t0 = time.time()
        while time.time() - t0 < 2.5:
            send(ser, "RPM 2000")
            drain(ser, 0.2)
            time.sleep(0.3)
        r = detect_spin(ser, seconds=5.0, keepalive="RPM 2000")
        log(r.summary_line())
        results["closed2000"] = r

        log("--- 停车 + LOAD? ---")
        safe_stop(ser)
        cmd(ser, "LOAD?", 0.45)
        cmd(ser, "ENC?", 0.35)

        log("\n========== FINAL ==========")
        for k, rr in results.items():
            log(
                f"{k}: enc_rpm={rr.enc_rpm_mean:.1f} hall_irqΔ={rr.hall_irq_delta} "
                f"conf={rr.confidence} → {'在转' if rr.spinning else '不转'} | {rr.reason}"
            )

        cl = results["closed2000"]
        ol = results["openloop"]
        if cl.spinning:
            log("门闩: PASS — 可继续问用户是否做 7500")
            rc = 0
        elif ol.spinning and not cl.spinning:
            log("门闩: FAIL — 开环能转闭环不能；查前馈/ESTOP/保活/斜坡")
            rc = 2
        else:
            log("门闩: FAIL — 开环也不转；查 ESC 解锁/PROTO/电源/信号")
            rc = 3

        LOG.write_text("\n".join(_out), encoding="utf-8")
        log(f"log={LOG}")
        return rc
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
            safe_stop(ser)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        release()


if __name__ == "__main__":
    sys.exit(main())
