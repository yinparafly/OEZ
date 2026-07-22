# -*- coding: utf-8 -*-
"""电机轴是否在转：磁编码器主判 + 霍尔辅判（可复用模块 + CLI）。

判定（观察窗 T 秒，建议 ≥2–3s）：
  1) 编码器主判：|rpm| 中位数（或均值）> RPM_SPIN_MIN，且 |rpm|>阈值的样本占比 > FRAC_MIN
  2) 霍尔辅判：irq_dn+irq_up 增量 ≥1，或 telem/HALL out_hz>0
     - 编码器过 → 「电机轴在转」（大减速比时霍尔可为 0）
     - 霍尔过   → 「输出端有沿」
  3) spinning=True 当且仅当编码器主判通过（霍尔只影响 confidence/reason）

用法:
  python motor_spin_detect.py --port COM10 --seconds 5
  python motor_spin_detect.py --port COM10 --seconds 5 --rpm 2000   # 闭环给目标再检测
  python motor_spin_detect.py --port COM10 --seconds 3 --open-pwm 1200

代码接入:
  from motor_spin_detect import detect_spin, require_spinning, SpinResult
  r = detect_spin(ser, seconds=3.0, keepalive="RPM 2000")
  if not r.spinning: ...
"""
from __future__ import annotations

import argparse
import os
import re
import statistics as st
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

os.environ.setdefault("PYTHONIOENCODING", "utf-8")

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from auto_learn_test import open_port, read_lines, send  # noqa: E402
from com_port_guard import acquire, release  # noqa: E402

# ---- 判定默认参数 ----
RPM_SPIN_MIN = 90.0       # |rpm| 中位/均值阈值
FRAC_MIN = 0.70           # 样本中 |rpm|>阈值 占比
KEEPALIVE_S = 0.4         # < HOST_TIMEOUT 1.5s
GEAR = 27.744
DEFAULT_PORT = "COM10"

RUN_NAME = {0: "IDLE", 1: "RUN", 2: "ESTOP"}
MODE_NAME = {0: "OPEN", 1: "CLOSED", 2: "LEARN"}


@dataclass
class SpinResult:
    spinning: bool                 # 电机轴在转（编码器主判）
    enc_rpm_mean: float
    enc_rpm_median: float
    enc_rpm_max: float
    spin_frac: float               # |rpm|>阈值 占比
    hall_irq_delta: int            # Δ(irq_dn+irq_up)，无数据时 -1
    hall_irq_dn_delta: int
    hall_irq_up_delta: int
    hall_out_hz: float             # 窗内/查询到的 out_hz，无则 nan
    hall_output_moving: bool       # 输出端有沿（霍尔辅判）
    reason: str
    confidence: str                # high / medium / low
    n_samples: int
    window_s: float
    run: int
    mode: int
    pulse_mean: float
    aborted: str | None = None

    def summary_line(self) -> str:
        return (
            f"spinning={self.spinning} enc_rpm={self.enc_rpm_mean:.1f} "
            f"(med={self.enc_rpm_median:.1f} max={self.enc_rpm_max:.1f} "
            f"frac={self.spin_frac:.2f}) hall_irq Δ={self.hall_irq_delta} "
            f"out_hz={self.hall_out_hz:.4f} conf={self.confidence} | {self.reason}"
        )


def drain(ser, budget: float = 0.2) -> list[str]:
    return read_lines(ser, budget)


def parse_telem(line: str) -> dict[str, float | int] | None:
    if not line or line.startswith("#"):
        return None
    p = line.split(",")
    if len(p) < 17:
        return None
    try:
        out: dict[str, float | int] = {
            "t_ms": float(p[0]),
            "rpm": float(p[4]),
            "pulse": int(float(p[9])),
            "target": float(p[10]),
            "mode": int(float(p[11])),
            "run": int(float(p[16])),
        }
        if len(p) > 22:
            out["out_hz"] = float(p[22])
        if len(p) > 24:
            out["enc_hz"] = float(p[24])
        return out
    except (ValueError, IndexError):
        return None


def hall_irq_snap(ser, dwell: float = 0.9, keepalive: str | None = "PING") -> dict[str, Any]:
    """发 HALL?，返回 irq_dn/up、out_hz 等。"""
    drain(ser, 0.05)
    send(ser, "HALL?")
    info: dict[str, Any] = {}
    end = time.time() + dwell
    last_ka = 0.0
    while time.time() < end:
        now = time.time()
        if keepalive and (now - last_ka) >= KEEPALIVE_S:
            send(ser, keepalive)
            last_ka = now
        for ln in drain(ser, 0.12):
            if "irq_dn=" in ln:
                m = re.search(r"irq_dn=(\d+).*?irq_up=(\d+)", ln)
                if m:
                    info["irq_dn"] = int(m.group(1))
                    info["irq_up"] = int(m.group(2))
                info["status"] = ln
            if ln.startswith("# HALL MEAS") or "out_hz=" in ln:
                m = re.search(r"out_hz=([\-\d.]+)", ln)
                if m:
                    info["out_hz"] = float(m.group(1))
                m2 = re.search(r"out_rpm=([\-\d.]+)", ln)
                if m2:
                    info["out_rpm"] = float(m2.group(1))
                info.setdefault("meas", ln)
            if "# HALL DOWN" in ln or "# HALL UP" in ln:
                info.setdefault("events", []).append(ln)
        if "irq_dn" in info and ("out_hz" in info or time.time() > end - 0.15):
            # 有 irq 即可提前结束；meas 尽量拿到
            if "out_hz" in info or "irq_dn" in info:
                if "irq_dn" in info and (time.time() - (end - dwell)) > 0.45:
                    break
    return info


def evaluate_samples(
    samples: list[dict],
    hall0: dict,
    hall1: dict,
    *,
    window_s: float,
    rpm_min: float = RPM_SPIN_MIN,
    frac_min: float = FRAC_MIN,
    aborted: str | None = None,
) -> SpinResult:
    rpms = [abs(float(s["rpm"])) for s in samples]
    pulses = [float(s["pulse"]) for s in samples]
    runs = [int(s["run"]) for s in samples]
    modes = [int(s["mode"]) for s in samples]
    outhz_list = [float(s["out_hz"]) for s in samples if "out_hz" in s and float(s["out_hz"]) > 0]

    n = len(rpms)
    mean_rpm = float(st.mean(rpms)) if n else float("nan")
    med_rpm = float(st.median(rpms)) if n else float("nan")
    max_rpm = float(max(rpms)) if n else 0.0
    spin_frac = (sum(1 for r in rpms if r > rpm_min) / n) if n else 0.0
    # 主判：中位数优先，均值兜底
    level = med_rpm if med_rpm == med_rpm else mean_rpm
    enc_ok = (n >= 5) and (level == level) and (level > rpm_min) and (spin_frac >= frac_min)

    ddn = dup = -1
    if hall0.get("irq_dn") is not None and hall1.get("irq_dn") is not None:
        ddn = int(hall1["irq_dn"]) - int(hall0["irq_dn"])
    if hall0.get("irq_up") is not None and hall1.get("irq_up") is not None:
        dup = int(hall1["irq_up"]) - int(hall0["irq_up"])
    irq_delta = -1
    if ddn >= 0 and dup >= 0:
        irq_delta = ddn + dup
    elif ddn >= 0:
        irq_delta = ddn
    elif dup >= 0:
        irq_delta = dup

    hall_out = float("nan")
    if hall1.get("out_hz") is not None:
        hall_out = float(hall1["out_hz"])
    elif outhz_list:
        hall_out = float(st.mean(outhz_list))
    hall_ok = (irq_delta >= 1) or (hall_out == hall_out and hall_out > 0.0)

    if enc_ok and hall_ok:
        reason = "电机轴在转 + 输出端有沿（编码器+霍尔）"
        confidence = "high"
    elif enc_ok and not hall_ok:
        reason = "电机轴在转（编码器过；霍尔未增——大减速比/未接线/窗短）"
        confidence = "medium"
    elif (not enc_ok) and hall_ok:
        reason = "输出端有沿但编码器未过阈——查编码/轴联/转速过低"
        confidence = "low"
    else:
        reason = "不转（编码器未过阈且霍尔 irq 不增）"
        confidence = "high" if n >= 10 else "medium"

    if aborted:
        reason = f"中止({aborted[:60]})；" + reason
        confidence = "low"

    return SpinResult(
        spinning=bool(enc_ok),
        enc_rpm_mean=mean_rpm if mean_rpm == mean_rpm else 0.0,
        enc_rpm_median=med_rpm if med_rpm == med_rpm else 0.0,
        enc_rpm_max=max_rpm,
        spin_frac=spin_frac,
        hall_irq_delta=irq_delta,
        hall_irq_dn_delta=ddn,
        hall_irq_up_delta=dup,
        hall_out_hz=hall_out if hall_out == hall_out else 0.0,
        hall_output_moving=bool(hall_ok),
        reason=reason,
        confidence=confidence,
        n_samples=n,
        window_s=window_s,
        run=runs[-1] if runs else -1,
        mode=modes[-1] if modes else -1,
        pulse_mean=float(st.mean(pulses)) if pulses else float("nan"),
        aborted=aborted,
    )


def detect_spin(
    ser,
    *,
    seconds: float = 3.0,
    keepalive: str | None = "PING",
    rpm_min: float = RPM_SPIN_MIN,
    frac_min: float = FRAC_MIN,
    hall_dwell: float = 0.85,
) -> SpinResult:
    """在已打开的串口上采集窗口并判定。调用方负责保活命令内容（如 RPM 2000）。"""
    hall0 = hall_irq_snap(ser, dwell=hall_dwell, keepalive=keepalive)
    samples: list[dict] = []
    aborted = None
    t0 = time.time()
    last_ka = 0.0
    while time.time() - t0 < seconds:
        now = time.time()
        if keepalive and (now - last_ka) >= KEEPALIVE_S:
            send(ser, keepalive)
            last_ka = now
        for ln in drain(ser, 0.12):
            if ln.startswith("#"):
                if "ESTOP" in ln and "cleared" not in ln and "ACK ESTOP" not in ln:
                    aborted = ln[:120]
                continue
            te = parse_telem(ln)
            if te:
                samples.append(te)
        if aborted:
            break
        time.sleep(0.02)
    # 窗内保活继续
    ka = keepalive
    hall1 = hall_irq_snap(ser, dwell=hall_dwell, keepalive=ka)
    return evaluate_samples(
        samples, hall0, hall1,
        window_s=seconds, rpm_min=rpm_min, frac_min=frac_min, aborted=aborted,
    )


def safe_stop(ser) -> None:
    for c in ("ESTOP", "STOP", "RPM 0", "PWM 1000"):
        send(ser, c)
        time.sleep(0.12)
    drain(ser, 0.4)


def require_spinning(result: SpinResult, *, label: str = "spin_detect") -> SpinResult:
    """实验门闩：FAIL 则抛 SystemExit（调用方应先 ESTOP）。"""
    if result.spinning:
        return result
    raise SystemExit(
        f"[{label}] SPIN_DETECT FAIL — 禁止继续高速档\n"
        f"  {result.summary_line()}\n"
        f"  run={result.run}({RUN_NAME.get(result.run,'?')}) "
        f"mode={result.mode}({MODE_NAME.get(result.mode,'?')}) "
        f"pulse≈{result.pulse_mean}"
    )


def arm_closed(ser, rpm: float, *, soft_up: float = 1000.0, rpmmax: int = 12000) -> None:
    for c in (
        "STOP",
        "MODE CLOSED",
        "SOFT ON",
        f"SOFT RATE UP {soft_up:.0f}",
        "SOFT RATE DOWN 800",
        "FREQ 400",
        f"RPMMAX {rpmmax}",
        "START",
        f"RPM {rpm:.0f}",
    ):
        send(ser, c)
        time.sleep(0.18)
        drain(ser, 0.2)


def arm_open_pwm(ser, pwm: int) -> None:
    for c in ("STOP", "MODE OPEN", "FREQ 400", "SOFT OFF", "START"):
        send(ser, c)
        time.sleep(0.15)
        drain(ser, 0.15)
    for pw in (1000, 1050, min(1100, pwm), pwm):
        send(ser, f"PWM {pw}")
        time.sleep(0.2)
        drain(ser, 0.1)


def main() -> int:
    ap = argparse.ArgumentParser(description="电机在转自判（编码器+霍尔）")
    ap.add_argument("--port", default=os.environ.get("OEZ_COM", DEFAULT_PORT))
    ap.add_argument("--seconds", type=float, default=5.0, help="观察窗秒数")
    ap.add_argument("--rpm", type=float, default=0.0, help=">0 则闭环设目标后检测")
    ap.add_argument("--open-pwm", type=int, default=0, help=">1000 则开环 PWM 短试")
    ap.add_argument("--rpm-min", type=float, default=RPM_SPIN_MIN)
    ap.add_argument("--frac-min", type=float, default=FRAC_MIN)
    ap.add_argument("--no-arm", action="store_true", help="不发 START/RPM，只听现有遥测")
    ap.add_argument("--json", action="store_true", help="额外打印 JSON")
    args = ap.parse_args()

    acquire("motor_spin_detect")
    ser = open_port(args.port)
    log_lines: list[str] = []

    def log(s: str) -> None:
        print(s, flush=True)
        log_lines.append(s)

    try:
        log(f"=== motor_spin_detect {datetime.now().isoformat(timespec='seconds')} ===")
        log(f"port={args.port} window={args.seconds}s rpm_min={args.rpm_min} frac_min={args.frac_min}")
        drain(ser, 0.5)
        safe_stop(ser)
        time.sleep(0.25)

        keepalive = "PING"
        if not args.no_arm:
            if args.open_pwm > 1000:
                log(f"arm OPEN PWM {args.open_pwm}")
                arm_open_pwm(ser, args.open_pwm)
                keepalive = f"PWM {args.open_pwm}"
            elif args.rpm > 0:
                log(f"arm CLOSED RPM {args.rpm:.0f}")
                arm_closed(ser, args.rpm)
                keepalive = f"RPM {args.rpm:.0f}"
            else:
                log("no setpoint — listen only (PING keepalive)")

        # 闭环斜坡：多等一点再采窗
        if args.rpm > 0 and not args.no_arm:
            ramp_wait = min(4.0, max(1.5, args.rpm / 1000.0))
            log(f"ramp wait {ramp_wait:.1f}s …")
            t0 = time.time()
            while time.time() - t0 < ramp_wait:
                send(ser, keepalive)
                drain(ser, 0.2)
                time.sleep(0.25)

        result = detect_spin(
            ser,
            seconds=args.seconds,
            keepalive=keepalive,
            rpm_min=args.rpm_min,
            frac_min=args.frac_min,
        )
        log(result.summary_line())
        log(
            f"细节: n={result.n_samples} run={result.run}({RUN_NAME.get(result.run,'?')}) "
            f"mode={result.mode}({MODE_NAME.get(result.mode,'?')}) "
            f"pulse_mean={result.pulse_mean:.1f} "
            f"hall_dnΔ={result.hall_irq_dn_delta} hall_upΔ={result.hall_irq_up_delta}"
        )
        log(f"结论: {'在转' if result.spinning else '不转'} — {result.reason}")
        if args.json:
            import json
            d = asdict(result)
            print(json.dumps(d, ensure_ascii=False, indent=2), flush=True)

        log("--- safe stop ---")
        safe_stop(ser)
        return 0 if result.spinning else 2
    except Exception as exc:
        try:
            safe_stop(ser)
        except Exception:
            pass
        print(f"ERROR: {exc}", flush=True)
        return 1
    finally:
        try:
            ser.close()
        except Exception:
            pass
        release()


if __name__ == "__main__":
    sys.exit(main())
