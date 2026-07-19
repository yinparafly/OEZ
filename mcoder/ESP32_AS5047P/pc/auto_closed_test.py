#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
闭环自动试转：应用学习建议 PID，按分析顺序试目标转速。

默认逻辑（与学习死区一致）:
  - 先读最新 learn CSV，PC 重算 PID 并下发
  - 试转顺序默认 300 → 1000（若分析判定 300 在死区，则直接从 1000 起）
  - 编码器 |rpm| 判转；某档 FAIL 则试下一档；全部 FAIL 则报告
  - 不超 RPMMAX；异常 ESTOP

用法:
  python auto_closed_test.py --port COM10
  python auto_closed_test.py --targets 300,1000 --hold 5
  python auto_closed_test.py --log pc/logs/learn_xxx.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    raise SystemExit("需要 pyserial: pip install pyserial")

from analyze_learn_log import (
    analyze_pid,
    latest_log,
    parse_firmware_suggest,
    parse_learn_points,
    recommend_closed_targets,
    suggest_pid_from_points,
)
from auto_learn_test import (
    BAUD,
    DEFAULT_PORT,
    LOG_DIR,
    open_port,
    parse_telem,
    read_lines,
    send,
    wait_banner,
)
from com_port_guard import acquire

MIN_SPIN = 40.0
ERR_REL = 0.20
ERR_ABS = 120.0


@dataclass
class ProbeResult:
    target: float
    mean_rpm: float
    peak_rpm: float
    pulse_us: int
    verdict: str
    note: str = ""


@dataclass
class ClosedReport:
    port: str
    log_path: Path | None
    kp: float
    ki: float
    steps: list[ProbeResult] = field(default_factory=list)

    def summary(self) -> str:
        lines = [
            f"port={self.port}",
            f"learn={self.log_path.name if self.log_path else '?'}",
            f"applied PID kp={self.kp:.4f} ki={self.ki:.4f} kd=0",
            "",
            f"{'target':>8} {'mean':>8} {'peak':>8} {'pulse':>6}  verdict  note",
        ]
        for s in self.steps:
            lines.append(
                f"{s.target:8.0f} {s.mean_rpm:8.1f} {s.peak_rpm:8.1f} "
                f"{s.pulse_us:6d}  {s.verdict:<10} {s.note}"
            )
        passed = [s for s in self.steps if s.verdict == "PASS"]
        lines.append("")
        if passed:
            lines.append(
                f"OVERALL: PASS（最低成功目标 {min(s.target for s in passed):.0f} RPM）"
            )
        elif any(s.verdict == "FAIL_START" for s in self.steps):
            lines.append(
                "OVERALL: FAIL_START — 闭环未转起来。"
                "开环学习若已转，多半是低目标落死区或 PID/前馈未下发；"
                "可加大目标或查 MODE CLOSED + map。"
            )
        else:
            lines.append("OVERALL: FAIL")
        return "\n".join(lines)


def drain(ser: serial.Serial, budget_s: float = 0.12):
    metas, telems = [], []
    for line in read_lines(ser, budget_s):
        if line.startswith("#"):
            metas.append(line)
        else:
            t = parse_telem(line)
            if t:
                telems.append(t)
    return metas, telems


def safe_stop(ser: serial.Serial) -> None:
    send(ser, "STOP")
    send(ser, "RPM 0")
    send(ser, "PWM 1000")
    time.sleep(0.2)
    drain(ser, 0.2)


def resolve_pid_and_targets(
    log_path: Path | None, targets_arg: list[float] | None
) -> tuple[float, float, list[float], str]:
    kp, ki = 0.08, 0.25
    note = "默认 PID"
    order = targets_arg or [300.0, 1000.0]

    if log_path and log_path.exists():
        rows = list(csv.DictReader(log_path.open(encoding="utf-8")))
        points = parse_learn_points(rows)
        fw = parse_firmware_suggest(rows)
        pc = suggest_pid_from_points(points)
        if pc:
            kp, ki = pc["kp"], pc["ki"]
            note = "PC 重算 PID"
        elif fw.get("fw_kp") is not None:
            kp, ki = fw["fw_kp"], fw["fw_ki"]
            note = "固件 SUGGEST PID"
        if targets_arg is None and points:
            rec = recommend_closed_targets(points)
            order = list(rec["try_order"])
            note += f"；起步 {rec['reason'][:80]}"
    return kp, ki, order, note


def run_probe(ser: serial.Serial, target: float, hold_s: float, rpmmax: float) -> ProbeResult:
    if target > rpmmax * 0.9:
        target = rpmmax * 0.9
    send(ser, "MODE CLOSED")
    send(ser, "SOFT ON")
    send(ser, f"RPM {target:.0f}")
    send(ser, "START")

    samples: list[dict] = []
    t0 = time.time()
    while time.time() - t0 < hold_s:
        metas, telems = drain(ser, 0.1)
        for m in metas:
            if "ESTOP" in m and "overspeed" in m.lower():
                safe_stop(ser)
                return ProbeResult(target, 0, 0, 1000, "ABORT", m[:100])
            if "ESTOP" in m and "cleared" not in m.lower() and "ACK ESTOP" not in m:
                if "ESTOP" in m:
                    safe_stop(ser)
                    return ProbeResult(target, 0, 0, 1000, "ABORT", m[:100])
        samples.extend(telems)

    safe_stop(ser)
    time.sleep(1.5)

    if not samples:
        return ProbeResult(target, 0, 0, 1000, "FAIL_START", "无遥测")

    tail = samples[-max(8, len(samples) // 3) :]
    mean = sum(abs(s["rpm"]) for s in tail) / len(tail)
    peak = max(abs(s["rpm"]) for s in samples)
    pulse = int(tail[-1]["pulse_us"])

    if peak < MIN_SPIN and mean < MIN_SPIN:
        return ProbeResult(
            target, mean, peak, pulse, "FAIL_START", f"{hold_s:.0f}s 编码器未见转"
        )

    tol = max(ERR_ABS, abs(target) * ERR_REL)
    if abs(mean - target) <= tol:
        return ProbeResult(target, mean, peak, pulse, "PASS", f"tol±{tol:.0f}")
    # 已转起来但跟踪差：仍算「能动」，标 TRACK_WEAK，便于 300 失败换 1000 的流程继续
    return ProbeResult(
        target,
        mean,
        peak,
        pulse,
        "TRACK_WEAK",
        f"已转 mean={mean:.0f} |err|={abs(mean - target):.0f}",
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="闭环试转 300→1000 + 应用学习 PID")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--log", default="", help="learn CSV；空则取最新")
    ap.add_argument("--targets", default="", help="逗号分隔；空则按日志死区分析")
    ap.add_argument("--hold", type=float, default=5.0)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    ap.add_argument("--analyze-only", action="store_true", help="只分析 PID，不给油")
    args = ap.parse_args()

    logs_dir = Path(__file__).resolve().parent / "logs"
    log_path = Path(args.log) if args.log.strip() else latest_log(logs_dir)

    if args.analyze_only:
        if not log_path or not log_path.exists():
            print("无学习日志")
            return 1
        print(analyze_pid(log_path))
        return 0

    targets_arg = None
    if args.targets.strip():
        targets_arg = [float(x) for x in args.targets.split(",") if x.strip()]

    kp, ki, order, note = resolve_pid_and_targets(log_path, targets_arg)
    print(f"PID: kp={kp:.4f} ki={ki:.4f}  ({note})")
    print(f"试转顺序: {order}")
    if log_path and log_path.exists():
        print(analyze_pid(log_path))
        print()

    print(f"打开 {args.port} @ {BAUD} …")
    try:
        acquire("auto_closed_test")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}\n请先关闭 encoder_monitor。")
        return 1

    report = ClosedReport(port=args.port, log_path=log_path, kp=kp, ki=ki)
    try:
        info = wait_banner(ser)
        print("板子:", info)
        send(ser, "STOP")
        send(ser, f"RPMMAX {args.rpmmax:.0f}")
        send(ser, f"PID {kp:.4f} {ki:.4f} 0")
        send(ser, "MODE CLOSED")
        send(ser, "SOFT ON")
        time.sleep(0.2)
        drain(ser, 0.3)

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = LOG_DIR / f"closed_probe_{ts}.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(["target", "mean", "peak", "pulse", "verdict", "note"])

            for tgt in order:
                print(f"\n>>> 闭环目标 {tgt:.0f} RPM …")
                res = run_probe(ser, tgt, args.hold, args.rpmmax)
                report.steps.append(res)
                w.writerow(
                    [
                        f"{res.target:.0f}",
                        f"{res.mean_rpm:.2f}",
                        f"{res.peak_rpm:.2f}",
                        res.pulse_us,
                        res.verdict,
                        res.note,
                    ]
                )
                fp.flush()
                print(
                    f"    mean={res.mean_rpm:.1f} peak={res.peak_rpm:.1f} "
                    f"pulse={res.pulse_us} → {res.verdict} {res.note}"
                )
                if res.verdict == "ABORT":
                    break
                # 300 FAIL_START → 继续试 1000；PASS/TRACK_WEAK 也继续下一档做验证
                if res.verdict == "PASS" and tgt >= 1000:
                    # 1000 已稳，可提前结束
                    break

        safe_stop(ser)
        text = report.summary() + f"\n\ncsv={csv_path}\n"
        rep = csv_path.with_suffix(".report.txt")
        rep.write_text(text, encoding="utf-8")
        print("\n======== 闭环试转报告 ========")
        print(text)
        print(f"报告: {rep}")
        if any(s.verdict == "PASS" for s in report.steps):
            return 0
        return 4
    except KeyboardInterrupt:
        print("\n中断")
        safe_stop(ser)
        return 130
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
            safe_stop(ser)
        except Exception:  # noqa: BLE001
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
