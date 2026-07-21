#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
负载变更后 · 第一步：转速计算正确性检查（不调 PID）。

独立参照：霍尔同标记两次过点 → out_Hz，则
  rpm_hall = out_Hz × (59×79)/(12×14) × 60
应与编码器滤波转速 |rpm_enc| 一致。

另用展开角窗口：rpm_unwrap = (Δunwrap_deg/360)/Δt×60
交叉核对滤波是否拖尾/乱跳。

用法:
  python auto_rpm_check.py --port COM10
  python auto_rpm_check.py --targets 800,1500,2500 --hold 10
  python auto_rpm_check.py --open --pulses 1200,1300,1400
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

GEAR = (59.0 * 79.0) / (12.0 * 14.0)  # ≈27.744


@dataclass
class Step:
    label: str
    target_or_pulse: float
    mean_enc: float = float("nan")
    mean_unwrap: float = float("nan")
    mean_hall: float = float("nan")
    hall_hz: float = float("nan")
    gear_meas: float = float("nan")
    enc_vs_hall_pct: float = float("nan")
    unwrap_vs_enc_pct: float = float("nan")
    dn_n: int = 0
    up_n: int = 0
    pulse_us: int = 0
    verdict: str = "—"
    note: str = ""


@dataclass
class Report:
    port: str
    mode: str
    steps: list[Step] = field(default_factory=list)

    def text(self) -> str:
        lines = [
            "# 转速计算正确性检查（负载变更 · 第1步）",
            f"- 时间: {datetime.now().isoformat(timespec='seconds')}",
            f"- 端口: {self.port}  模式: {self.mode}",
            f"- 设计减速比 i={GEAR:.6f}",
            f"- 判据: 有霍尔时 |enc-hall|/hall<=8%; 无霍尔时 |unwrap-enc|/enc<=5%",
            "",
            "| 档 | 编码器RPM | 展开角RPM | 霍尔推RPM | 霍尔Hz | i实测 | "
            "enc-hall% | unwrap-enc% | 0/180 | 结论 |",
            "|----|-----------|-----------|-----------|--------|--------|"
            "----------|-------------|-------|------|",
        ]
        for s in self.steps:
            lines.append(
                f"| {s.label} | {_f(s.mean_enc)} | {_f(s.mean_unwrap)} | "
                f"{_f(s.mean_hall)} | {_f(s.hall_hz)} | {_f(s.gear_meas)} | "
                f"{_f(s.enc_vs_hall_pct)} | {_f(s.unwrap_vs_enc_pct)} | "
                f"{s.dn_n}/{s.up_n} | {s.verdict} |"
            )
        ok = sum(1 for s in self.steps if s.verdict.startswith("PASS"))
        fail = sum(1 for s in self.steps if s.verdict.startswith("FAIL"))
        skip = sum(1 for s in self.steps if s.verdict.startswith("SKIP"))
        lines += [
            "",
            f"汇总: PASS={ok} FAIL={fail} SKIP={skip} / {len(self.steps)}",
            "",
            "## 下一步建议",
        ]
        hall_ok = any(
            s.verdict == "PASS" and not math.isnan(s.mean_hall) for s in self.steps
        )
        internal_ok = any(s.verdict == "PASS_INTERNAL" for s in self.steps)
        if hall_ok:
            lines.append(
                "转速与霍尔一致 -> 不要改转速公式；"
                "第2步：对新负载 LEARN 重建前馈图，再 SUGGEST/闭环调 PID，视需要调 KA/ADG。"
            )
        elif internal_ok and fail == 0:
            lines.append(
                "无霍尔事件，但编码器滤波转速与展开角窗口一致 -> 转速公式大概率正确；"
                "建议接好霍尔后再做一次交叉验证。第2步仍应对新负载重做 LEARN+PID。"
            )
        elif fail > 0 and not internal_ok:
            lines.append(
                "编码器与参照不一致 -> 先查霍尔/磁环/SPI/打滑，暂缓改 PID。"
            )
        else:
            lines.append(
                "霍尔事件不足且内部一致性不足 -> 确认磁铁过 GPIO4/5 或降低脉宽重跑。"
            )
        return "\n".join(lines)


def _f(x: float) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "—"
    return f"{x:.1f}"


def parse_telem(line: str) -> dict | None:
    if not line or line.startswith("#"):
        return None
    p = line.split(",")
    if len(p) < 12:
        return None
    try:
        d = {
            "t_ms": float(p[0]),
            "raw": int(float(p[1])),
            "rpm": abs(float(p[4])),
            "pulse": int(float(p[9])),
            "target": float(p[10]),
            "out_hz": float("nan"),
            "gear": float("nan"),
        }
        if len(p) >= 24:
            oh, gm = float(p[22]), float(p[23])
            if oh >= 0:
                d["out_hz"] = oh
            if gm >= 0:
                d["gear"] = gm
        return d
    except ValueError:
        return None


def parse_hall(line: str) -> dict | None:
    if not line.startswith("# HALL"):
        return None
    if "DOWN" in line:
        kind = "DOWN"
    elif "UP" in line:
        kind = "UP"
    else:
        return None
    kv = {}
    for tok in line.replace(",", " ").split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            kv[k] = v
    out: dict = {"kind": kind}
    for k in ("out_hz", "gear_meas", "t_ms"):
        if k in kv:
            try:
                out[k] = float(kv[k])
            except ValueError:
                pass
    return out


def drain(ser: serial.Serial, budget: float = 0.12):
    telem, halls, meta = [], [], []
    for line in read_lines(ser, budget):
        if line.startswith("# HALL") and ("DOWN" in line or "UP" in line):
            h = parse_hall(line)
            if h:
                halls.append(h)
            meta.append(line)
        elif line.startswith("#"):
            meta.append(line)
        else:
            t = parse_telem(line)
            if t:
                telem.append(t)
    return telem, halls, meta


def safe_stop(ser: serial.Serial) -> None:
    send(ser, "STOP")
    send(ser, "RPM 0")
    send(ser, "PWM 1000")
    time.sleep(0.25)
    drain(ser, 0.15)


def run_step(
    ser: serial.Serial,
    *,
    open_loop: bool,
    value: float,
    hold_s: float,
) -> Step:
    if open_loop:
        label = f"PWM{int(value)}"
        step = Step(label=label, target_or_pulse=value)
        send(ser, "MODE OPEN")
        send(ser, "START")
        send(ser, f"PWM {int(value)}")
    else:
        label = f"RPM{int(value)}"
        step = Step(label=label, target_or_pulse=value)
        send(ser, "MODE CLOSED")
        send(ser, "START")
        send(ser, f"RPM {value:.0f}")

    t0 = time.time()
    encs: list[float] = []
    pulses: list[int] = []
    hall_hz: list[float] = []
    gears: list[float] = []
    rpm_hall_list: list[float] = []
    dn_n = up_n = 0
    # 展开角窗口：用 raw 差分累计（从遥测 deg 不够；用连续 raw unwrap）
    last_raw = None
    unwrap_acc = 0.0
    unwrap_t0 = None
    unwrap_samples: list[tuple[float, float]] = []  # (t, unwrap_deg)
    last_ping = 0.0

    while time.time() - t0 < hold_s:
        now = time.time()
        if now - last_ping > 0.4:
            send(ser, "PING")
            last_ping = now
        telem, halls, _ = drain(ser, 0.1)
        for h in halls:
            if h["kind"] == "DOWN":
                dn_n += 1
            else:
                up_n += 1
            if "out_hz" in h and h["out_hz"] > 0:
                hall_hz.append(h["out_hz"])
                rpm_hall_list.append(h["out_hz"] * GEAR * 60.0)
            if "gear_meas" in h and h["gear_meas"] > 0.5:
                gears.append(h["gear_meas"])
        for t in telem:
            if now - t0 >= hold_s * 0.3:
                encs.append(t["rpm"])
                pulses.append(t["pulse"])
            if not math.isnan(t["out_hz"]) and t["out_hz"] > 0:
                hall_hz.append(t["out_hz"])
                rpm_hall_list.append(t["out_hz"] * GEAR * 60.0)
            if not math.isnan(t["gear"]) and t["gear"] > 0.5:
                gears.append(t["gear"])
            # raw unwrap
            raw = t["raw"]
            if last_raw is not None:
                d = raw - last_raw
                if d > 8192:
                    d -= 16384
                if d < -8192:
                    d += 16384
                unwrap_acc += d * (360.0 / 16384.0)
            last_raw = raw
            if unwrap_t0 is None:
                unwrap_t0 = now
            unwrap_samples.append((now, unwrap_acc))

    if open_loop:
        send(ser, "PWM 1000")
    else:
        send(ser, "RPM 0")
    time.sleep(0.6)
    drain(ser, 0.15)

    step.dn_n, step.up_n = dn_n, up_n
    if encs:
        step.mean_enc = sum(encs) / len(encs)
    if pulses:
        step.pulse_us = int(sum(pulses) / len(pulses))
    if hall_hz:
        step.hall_hz = sum(hall_hz[-6:]) / len(hall_hz[-6:])
    if rpm_hall_list:
        step.mean_hall = sum(rpm_hall_list[-6:]) / len(rpm_hall_list[-6:])
    if gears:
        step.gear_meas = sum(gears[-6:]) / len(gears[-6:])

    # 后半窗口展开角转速
    if len(unwrap_samples) >= 20:
        mid = unwrap_samples[len(unwrap_samples) // 2]
        end = unwrap_samples[-1]
        dt = end[0] - mid[0]
        if dt > 0.2:
            step.mean_unwrap = abs(end[1] - mid[1]) / 360.0 / dt * 60.0

    notes: list[str] = []
    if not math.isnan(step.mean_unwrap) and not math.isnan(step.mean_enc) and step.mean_enc > 1:
        step.unwrap_vs_enc_pct = (
            (step.mean_unwrap - step.mean_enc) / step.mean_enc * 100.0
        )

    # 无霍尔：仅做编码器滤波 vs 展开角窗口
    if dn_n + up_n < 2 or math.isnan(step.mean_hall) or step.mean_hall < 50:
        if math.isnan(step.mean_enc) or step.mean_enc < 30:
            step.verdict = "FAIL_NO_ENC"
            step.note = "无霍尔且编码器转速过低"
            return step
        if not math.isnan(step.unwrap_vs_enc_pct) and abs(step.unwrap_vs_enc_pct) <= 5.0:
            step.verdict = "PASS_INTERNAL"
            step.note = "无霍尔事件; 滤波≈展开角"
        else:
            step.verdict = "FAIL_INTERNAL"
            step.note = (
                f"无霍尔; unwrap-enc={_f(step.unwrap_vs_enc_pct)}%"
            )
        return step

    if math.isnan(step.mean_enc) or step.mean_enc < 30:
        step.verdict = "FAIL_NO_ENC"
        step.note = "编码器转速过低"
        return step

    step.enc_vs_hall_pct = (step.mean_enc - step.mean_hall) / step.mean_hall * 100.0

    enc_ok = abs(step.enc_vs_hall_pct) <= 8.0
    unwrap_ok = math.isnan(step.unwrap_vs_enc_pct) or abs(step.unwrap_vs_enc_pct) <= 5.0
    gear_ok = math.isnan(step.gear_meas) or abs(step.gear_meas - GEAR) / GEAR <= 0.10

    if enc_ok and unwrap_ok:
        step.verdict = "PASS"
        if not gear_ok:
            notes.append(f"i={step.gear_meas:.2f}略偏")
    elif enc_ok:
        step.verdict = "PASS_ENC_HALL"
        notes.append("滤波与展开角略差，可接受")
    else:
        step.verdict = "FAIL_MISMATCH"
        notes.append(f"enc-hall {step.enc_vs_hall_pct:+.1f}%")

    step.note = "; ".join(notes)
    return step


def main() -> int:
    ap = argparse.ArgumentParser(description="负载变更后转速计算检查")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--targets", default="800,1500,2500", help="闭环目标 RPM")
    ap.add_argument("--open", action="store_true", help="开环按脉宽测（不依赖旧前馈）")
    ap.add_argument("--pulses", default="1200,1300,1400,1500", help="开环脉宽")
    ap.add_argument("--hold", type=float, default=12.0)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    args = ap.parse_args()

    try:
        acquire("auto_rpm_check")
    except SystemExit as exc:
        print(exc)
        return 1

    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开 {args.port}: {exc}")
        print("请先关闭 encoder_monitor / 其它占口程序。")
        return 1

    mode = "OPEN" if args.open else "CLOSED"
    rep = Report(port=args.port, mode=mode)
    print(f"第1步：转速校核  port={args.port} mode={mode} i={GEAR:.4f}")

    try:
        send(ser, "PING")
        drain(ser, 0.3)
        send(ser, f"RPMMAX {args.rpmmax:.0f}")
        send(ser, "FREQ 400")
        send(ser, "SOFT ON")
        send(ser, "HALL ACTIVE LOW")
        send(ser, "KA 0")
        send(ser, "ADG OFF")
        safe_stop(ser)
        time.sleep(0.4)

        if args.open:
            values = [float(x) for x in args.pulses.split(",") if x.strip()]
        else:
            values = [float(x) for x in args.targets.split(",") if x.strip()]

        for v in values:
            print(f"\n>>> {mode} {v:.0f} …")
            step = run_step(ser, open_loop=args.open, value=v, hold_s=args.hold)
            rep.steps.append(step)
            print(
                f"    enc={_f(step.mean_enc)}  unwrap={_f(step.mean_unwrap)}  "
                f"hall→rpm={_f(step.mean_hall)}  Δ%={_f(step.enc_vs_hall_pct)}  "
                f"evt={step.dn_n}/{step.up_n}  → {step.verdict}  {step.note}"
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
    md = LOG_DIR / f"rpm_check_{ts}.md"
    md.write_text(text + "\n", encoding="utf-8")
    csv_path = LOG_DIR / f"rpm_check_{ts}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "label",
                "mean_enc",
                "mean_unwrap",
                "mean_hall",
                "hall_hz",
                "gear_meas",
                "enc_vs_hall_pct",
                "unwrap_vs_enc_pct",
                "dn",
                "up",
                "verdict",
                "note",
            ]
        )
        for s in rep.steps:
            w.writerow(
                [
                    s.label,
                    s.mean_enc,
                    s.mean_unwrap,
                    s.mean_hall,
                    s.hall_hz,
                    s.gear_meas,
                    s.enc_vs_hall_pct,
                    s.unwrap_vs_enc_pct,
                    s.dn_n,
                    s.up_n,
                    s.verdict,
                    s.note,
                ]
            )
    print(f"报告: {md}")

    if any(s.verdict.startswith("FAIL") for s in rep.steps):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
