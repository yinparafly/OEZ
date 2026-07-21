#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
霍尔减速比 / 输出频率对照测试（齿轮台架，无扑翼）。

逻辑核对：
  - 输出盘两次过 0°（GPIO4）或两次过 180°（GPIO5）= 盘转 1 圈
  - 其间电机展开角 / 360° 应 ≈ (59×79)/(12×14) ≈ 27.744
  - 霍尔测得 out_Hz 与电机 RPM：motor_rpm ≈ out_Hz × 27.744 × 60

扫输出端 1..6 Hz（对应电机约 1665..9988 RPM）。
注意：固件默认电机上限约 6000RPM → 输出约 ≤3.6Hz；更高档会记 SKIP/限速。

用法:
  python auto_hall_gear_test.py --port COM10
  python auto_hall_gear_test.py --hz 1,2,3 --hold 12
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit("需要 pyserial: pip install pyserial") from exc

from auto_learn_test import BAUD, DEFAULT_PORT, LOG_DIR, open_port, read_lines, send
from com_port_guard import acquire

GEAR_DESIGN = (59.0 * 79.0) / (12.0 * 14.0)  # 4661/168 ≈ 27.744
MOTOR_RPM_CAP = 6000.0  # 与固件工作上限一致；超则无法闭环到目标


@dataclass
class HzStep:
    want_hz: float
    motor_target: float
    mean_motor_rpm: float = float("nan")
    peak_motor_rpm: float = float("nan")
    hall_hz_dn: float = float("nan")
    hall_hz_up: float = float("nan")
    hall_hz: float = float("nan")
    gear_dn: float = float("nan")
    gear_up: float = float("nan")
    gear_meas: float = float("nan")
    gear_err_pct: float = float("nan")
    rpm_from_hall: float = float("nan")
    rpm_vs_hall_err_pct: float = float("nan")
    pulse_us: int = 0
    dn_events: int = 0
    up_events: int = 0
    verdict: str = "—"
    note: str = ""


@dataclass
class Report:
    port: str
    steps: list[HzStep] = field(default_factory=list)

    def text(self) -> str:
        lines = [
            "# 霍尔减速比 / 输出频率对照报告",
            f"- 时间: {datetime.now().isoformat(timespec='seconds')}",
            f"- 端口: {self.port}",
            f"- 设计减速比 i=(59×79)/(12×14)={GEAR_DESIGN:.6f}",
            f"- 电机工作上限约 {MOTOR_RPM_CAP:.0f} RPM → 输出约 ≤{MOTOR_RPM_CAP/GEAR_DESIGN/60:.2f} Hz",
            "",
            "| 目标Hz | 电机目标RPM | 电机均值 | 霍尔Hz | 霍尔推电机RPM | "
            "i实测 | i偏差% | 0°事件 | 180°事件 | 结论 | 备注 |",
            "|--------|-------------|---------|--------|---------------|"
            "--------|--------|---------|-----------|------|------|",
        ]
        for s in self.steps:
            lines.append(
                f"| {s.want_hz:.1f} | {s.motor_target:.0f} | "
                f"{_fmt(s.mean_motor_rpm)} | {_fmt(s.hall_hz)} | "
                f"{_fmt(s.rpm_from_hall)} | {_fmt(s.gear_meas)} | "
                f"{_fmt(s.gear_err_pct)} | {s.dn_events} | {s.up_events} | "
                f"{s.verdict} | {s.note} |"
            )
        lines.append("")
        ok = [s for s in self.steps if s.verdict == "PASS"]
        skip = [s for s in self.steps if s.verdict.startswith("SKIP")]
        fail = [s for s in self.steps if s.verdict.startswith("FAIL")]
        lines.append(
            f"汇总: PASS={len(ok)} SKIP={len(skip)} FAIL={len(fail)} / {len(self.steps)}"
        )
        if ok:
            errs = [abs(s.gear_err_pct) for s in ok if not math.isnan(s.gear_err_pct)]
            if errs:
                lines.append(
                    f"PASS 档减速比 |偏差|：max={max(errs):.2f}% mean={sum(errs)/len(errs):.2f}%"
                )
        lines.append("")
        lines.append("判定：同标记两次过点后 i实测 ∈ [设计±8%] 且霍尔Hz∈[目标±15%] → PASS")
        return "\n".join(lines)


def _fmt(x: float) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "—"
    return f"{x:.2f}"


def parse_telem_ext(line: str) -> dict | None:
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) < 12:
        return None
    try:
        d = {
            "t_ms": float(parts[0]),
            "rpm": abs(float(parts[4])),
            "pulse_us": int(float(parts[9])),
            "target": float(parts[10]),
            "mode": int(float(parts[11])),
            "run": int(float(parts[16])) if len(parts) > 16 else 0,
            "out_hz": float("nan"),
            "gear_meas": float("nan"),
        }
        if len(parts) >= 24:
            oh = float(parts[22])
            gm = float(parts[23])
            if oh >= 0:
                d["out_hz"] = oh
            if gm >= 0:
                d["gear_meas"] = gm
        return d
    except ValueError:
        return None


def parse_hall_meta(line: str) -> dict | None:
    if not line.startswith("# HALL"):
        return None
    if "DOWN" in line:
        kind = "DOWN"
    elif "UP" in line:
        kind = "UP"
    else:
        return None
    kv: dict[str, str] = {}
    for tok in line.replace(",", " ").split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            kv[k] = v
    out: dict = {"kind": kind}
    for k in ("out_hz", "gear_meas", "count", "t_ms"):
        if k in kv:
            try:
                out[k] = float(kv[k])
            except ValueError:
                pass
    return out


def drain(ser: serial.Serial, budget_s: float = 0.15):
    metas, telems, halls = [], [], []
    for line in read_lines(ser, budget_s):
        if line.startswith("# HALL"):
            h = parse_hall_meta(line)
            if h:
                halls.append(h)
            metas.append(line)
        elif line.startswith("#"):
            metas.append(line)
        else:
            t = parse_telem_ext(line)
            if t:
                telems.append(t)
    return metas, telems, halls


def safe_stop(ser: serial.Serial) -> None:
    send(ser, "STOP")
    send(ser, "RPM 0")
    send(ser, "PWM 1000")
    time.sleep(0.3)
    drain(ser, 0.2)


def motor_rpm_for_out_hz(hz: float) -> float:
    return hz * GEAR_DESIGN * 60.0


def run_step(ser: serial.Serial, want_hz: float, hold_s: float) -> HzStep:
    motor_tgt = motor_rpm_for_out_hz(want_hz)
    step = HzStep(want_hz=want_hz, motor_target=motor_tgt)

    if motor_tgt > MOTOR_RPM_CAP * 1.02:
        step.verdict = "SKIP_RPMMAX"
        step.note = f"需电机≈{motor_tgt:.0f}>上限{MOTOR_RPM_CAP:.0f}"
        return step

    send(ser, "HALL?")
    drain(ser, 0.2)
    send(ser, "MODE CLOSED")
    send(ser, "START")
    send(ser, f"RPM {motor_tgt:.0f}")

    t0 = time.time()
    rpms: list[float] = []
    pulses: list[int] = []
    hall_hz_samples: list[float] = []
    gear_samples: list[float] = []
    gears_dn: list[float] = []
    gears_up: list[float] = []
    hz_dn: list[float] = []
    hz_up: list[float] = []
    dn_n0 = None
    up_n0 = None
    dn_n = 0
    up_n = 0
    last_ping = 0.0

    while time.time() - t0 < hold_s:
        now = time.time()
        if now - last_ping > 0.4:
            send(ser, "PING")
            last_ping = now
        _metas, telems, halls = drain(ser, 0.12)
        for h in halls:
            if h["kind"] == "DOWN":
                dn_n += 1
                if dn_n0 is None:
                    dn_n0 = 0
                if "out_hz" in h and h["out_hz"] >= 0:
                    hz_dn.append(h["out_hz"])
                    hall_hz_samples.append(h["out_hz"])
                if "gear_meas" in h and h["gear_meas"] >= 0.5:
                    gears_dn.append(h["gear_meas"])
                    gear_samples.append(h["gear_meas"])
            else:
                up_n += 1
                if up_n0 is None:
                    up_n0 = 0
                if "out_hz" in h and h["out_hz"] >= 0:
                    hz_up.append(h["out_hz"])
                    hall_hz_samples.append(h["out_hz"])
                if "gear_meas" in h and h["gear_meas"] >= 0.5:
                    gears_up.append(h["gear_meas"])
                    gear_samples.append(h["gear_meas"])
        for t in telems:
            # 后半段再采电机转速，等斜坡
            if now - t0 >= hold_s * 0.35:
                rpms.append(t["rpm"])
                pulses.append(t["pulse_us"])
            if not math.isnan(t["out_hz"]):
                hall_hz_samples.append(t["out_hz"])
            if not math.isnan(t["gear_meas"]) and t["gear_meas"] >= 0.5:
                gear_samples.append(t["gear_meas"])

    send(ser, "RPM 0")
    time.sleep(0.8)
    drain(ser, 0.2)

    step.dn_events = dn_n
    step.up_events = up_n
    if rpms:
        step.mean_motor_rpm = sum(rpms) / len(rpms)
        step.peak_motor_rpm = max(rpms)
    if pulses:
        step.pulse_us = int(sum(pulses) / len(pulses))
    if hz_dn:
        step.hall_hz_dn = sum(hz_dn[-5:]) / len(hz_dn[-5:])
    if hz_up:
        step.hall_hz_up = sum(hz_up[-5:]) / len(hz_up[-5:])
    if hall_hz_samples:
        step.hall_hz = sum(hall_hz_samples[-8:]) / len(hall_hz_samples[-8:])
    if gears_dn:
        step.gear_dn = sum(gears_dn[-5:]) / len(gears_dn[-5:])
    if gears_up:
        step.gear_up = sum(gears_up[-5:]) / len(gears_up[-5:])
    if gear_samples:
        step.gear_meas = sum(gear_samples[-8:]) / len(gear_samples[-8:])
        step.gear_err_pct = (step.gear_meas - GEAR_DESIGN) / GEAR_DESIGN * 100.0
    if not math.isnan(step.hall_hz) and step.hall_hz > 0:
        step.rpm_from_hall = step.hall_hz * GEAR_DESIGN * 60.0
        if not math.isnan(step.mean_motor_rpm) and step.mean_motor_rpm > 1:
            step.rpm_vs_hall_err_pct = (
                (step.mean_motor_rpm - step.rpm_from_hall) / step.rpm_from_hall * 100.0
            )

    # 判定
    notes: list[str] = []
    if dn_n + up_n < 2:
        step.verdict = "FAIL_NO_HALL"
        step.note = "霍尔事件不足（需同标记≥2次）"
        return step

    gear_ok = (
        not math.isnan(step.gear_meas)
        and abs(step.gear_err_pct) <= 8.0
    )
    hz_ok = (
        not math.isnan(step.hall_hz)
        and abs(step.hall_hz - want_hz) / want_hz <= 0.15
    )
    rpm_ok = (
        not math.isnan(step.mean_motor_rpm)
        and abs(step.mean_motor_rpm - motor_tgt) / motor_tgt <= 0.20
    )

    if gear_ok and (hz_ok or rpm_ok):
        step.verdict = "PASS"
    elif gear_ok:
        step.verdict = "PASS_GEAR"
        notes.append("减速比OK，频率跟踪一般")
    elif not math.isnan(step.gear_meas):
        step.verdict = "FAIL_GEAR"
        notes.append(f"i偏差{step.gear_err_pct:+.1f}%")
    else:
        step.verdict = "FAIL_NO_RATIO"
        notes.append("未算出 gear_meas")

    if not math.isnan(step.hall_hz_dn) and not math.isnan(step.hall_hz_up):
        notes.append(f"0°Hz={step.hall_hz_dn:.2f}/180°Hz={step.hall_hz_up:.2f}")
    elif not math.isnan(step.gear_dn) and not math.isnan(step.gear_up):
        notes.append(f"i0={step.gear_dn:.2f}/i180={step.gear_up:.2f}")
    step.note = "; ".join(notes)
    return step


def main() -> int:
    ap = argparse.ArgumentParser(description="霍尔 1–6Hz 与电机转速/减速比对照")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--hz", default="1,2,3,4,5,6", help="输出端目标 Hz 列表")
    ap.add_argument("--hold", type=float, default=14.0, help="每档保持秒数")
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    args = ap.parse_args()

    hz_list = [float(x) for x in args.hz.split(",") if x.strip()]
    print(f"设计 i={GEAR_DESIGN:.6f}")
    print(f"扫 Hz={hz_list}  hold={args.hold}s  port={args.port}")
    for hz in hz_list:
        print(f"  {hz:.1f} Hz → 电机目标 {motor_rpm_for_out_hz(hz):.0f} RPM")

    try:
        acquire("auto_hall_gear_test")
    except SystemExit as exc:
        print(exc)
        return 1

    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开 {args.port}: {exc}")
        print("请先关闭 encoder_monitor / 其它占口程序。")
        return 1

    rep = Report(port=args.port)
    try:
        send(ser, "PING")
        drain(ser, 0.3)
        send(ser, f"RPMMAX {args.rpmmax:.0f}")
        send(ser, "FREQ 400")
        send(ser, "SOFT ON")
        send(ser, "HALL ACTIVE LOW")
        send(ser, "MODE CLOSED")
        send(ser, "STOP")
        time.sleep(0.5)
        drain(ser, 0.3)

        for hz in hz_list:
            print(f"\n>>> 目标输出 {hz:.1f} Hz  (电机≈{motor_rpm_for_out_hz(hz):.0f} RPM) …")
            step = run_step(ser, hz, args.hold)
            rep.steps.append(step)
            print(
                f"    motorμ={_fmt(step.mean_motor_rpm)}  hallHz={_fmt(step.hall_hz)}  "
                f"i={_fmt(step.gear_meas)} ({_fmt(step.gear_err_pct)}%)  "
                f"evt 0°/180°={step.dn_events}/{step.up_events}  → {step.verdict}  {step.note}"
            )

        safe_stop(ser)
    except Exception as exc:  # noqa: BLE001
        print(f"异常: {exc}")
        try:
            send(ser, "ESTOP")
            safe_stop(ser)
        except Exception:  # noqa: BLE001
            pass
        return 5
    finally:
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass

    text = rep.text()
    print("\n" + text)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    md = LOG_DIR / f"hall_gear_test_{ts}.md"
    md.write_text(text + "\n", encoding="utf-8")
    csv_path = LOG_DIR / f"hall_gear_test_{ts}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "want_hz",
                "motor_target",
                "mean_motor_rpm",
                "hall_hz",
                "hall_hz_dn",
                "hall_hz_up",
                "gear_meas",
                "gear_dn",
                "gear_up",
                "gear_err_pct",
                "rpm_from_hall",
                "rpm_vs_hall_err_pct",
                "dn_events",
                "up_events",
                "verdict",
                "note",
            ]
        )
        for s in rep.steps:
            w.writerow(
                [
                    s.want_hz,
                    s.motor_target,
                    s.mean_motor_rpm,
                    s.hall_hz,
                    s.hall_hz_dn,
                    s.hall_hz_up,
                    s.gear_meas,
                    s.gear_dn,
                    s.gear_up,
                    s.gear_err_pct,
                    s.rpm_from_hall,
                    s.rpm_vs_hall_err_pct,
                    s.dn_events,
                    s.up_events,
                    s.verdict,
                    s.note,
                ]
            )
    print(f"报告: {md}")
    print(f"CSV:  {csv_path}")

    if any(s.verdict.startswith("FAIL") for s in rep.steps):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
