#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
T1 PWM 刷新率自动扫频：开环给固定脉宽，用 AS5047P 遥测转速判定是否转起来。

规则（分歧方案 T1′）:
  - 每档 FREQ：停转确认 → 设频 → 开环 PWM 探点 → 保持 hold 秒（默认 3）
  - 给油期间周期 PING（防 host_timeout≈1.5s 误急停）
  - 编码器 |rpm| 超过阈值 → PASS（可见转可提前结束本档）；否则跑满时限 FAIL
  - 任一档 FAIL → 不再测更高频率，恢复 50Hz + 1000μs，并写出冻结表
  - 50Hz 都转不起来 → 视为台架/油门问题，停止并诊断（不是「电调不支持高频」）

用法:
  python auto_freq_test.py --port COM10
  python auto_freq_test.py --hold 5 --pulse 1100
  python auto_freq_test.py --freqs 50,100,150,200
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

DEFAULT_FREQS = (50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600)
SAFE_HZ = 50
MIN_SPIN_RPM = 40.0  # 低于此视为「没动起来」（磁编码器可分辨）
STOP_CONFIRM_S = 1.5
GAP_S = 0.8  # 换频前停油间隙


@dataclass
class FreqResult:
    hz: int
    pulse_us: int
    hold_s: float
    mean_rpm: float
    peak_rpm: float
    moved: bool
    verdict: str  # PASS / FAIL / SKIP
    note: str = ""


@dataclass
class FreqReport:
    port: str
    pulse_us: int
    hold_s: float
    results: list[FreqResult] = field(default_factory=list)
    recommended_hz: int = SAFE_HZ
    max_spin_hz: int = SAFE_HZ  # 判转通过的最高档（可高于推荐默认）
    aborted_at: int | None = None

    def summary_text(self) -> str:
        lines = [
            f"port={self.port} pulse={self.pulse_us}us hold={self.hold_s:.1f}s",
            f"判转阈值: |rpm| >= {MIN_SPIN_RPM:.0f}（AS5047P 遥测）",
            "",
            f"{'Hz':>5} {'mean':>8} {'peak':>8} {'moved':>6}  verdict  note",
        ]
        for r in self.results:
            lines.append(
                f"{r.hz:5d} {r.mean_rpm:8.1f} {r.peak_rpm:8.1f} "
                f"{'Y' if r.moved else 'N':>6}  {r.verdict:<7} {r.note}"
            )
        lines.append("")
        lines.append(f"判转支持上限: {self.max_spin_hz} Hz")
        lines.append(f"推荐工作默认 FREQ: {self.recommended_hz} Hz")
        if self.aborted_at is not None:
            lines.append(f"在 {self.aborted_at} Hz FAIL → 更高频率未测")
        lines.append("")
        lines.append(self.analysis())
        return "\n".join(lines)

    def analysis(self) -> str:
        if not self.results:
            return "分析: 无结果"
        first = self.results[0]
        if first.hz == SAFE_HZ and first.verdict == "FAIL":
            return (
                "分析: 50Hz 开环给油在时限内编码器未见转动。\n"
                "  这不是「电调不支持高频」，而是基线就有问题。请查：\n"
                "  1) 电调动力电是否已上、行程校准是否完成\n"
                "  2) 探点脉宽是否过低（学习台阶能转的脉宽再试 --pulse）\n"
                "  3) 编码器接线/磁铁；看遥测 rpm 是否一直接近 0\n"
                "  4) 串口是否被 UI 占用、PWM 是否被其它模式抢走\n"
                "  5) 须用 MEASURE/开环锁存；旧 OPEN+PWM 会被控制环盖回怠速"
            )
        if self.aborted_at is not None:
            prev = [
                r for r in self.results if r.verdict == "PASS" and r.hz < self.aborted_at
            ]
            if prev:
                return (
                    f"分析: {self.aborted_at} Hz 时限内未转起来，"
                    f"而 {prev[-1].hz} Hz 已通过。\n"
                    f"  判转支持上限 {self.max_spin_hz} Hz；"
                    f"工作默认用 {self.recommended_hz} Hz。\n"
                    f"  更高档已按规则跳过，无需再测。"
                )
            return (
                f"分析: {self.aborted_at} Hz FAIL，且无更低档 PASS。"
                "请先保证 50Hz 通过。"
            )
        passes = [r for r in self.results if r.verdict == "PASS"]
        if not passes:
            return "分析: 全部未通过"
        base = next((r for r in passes if r.hz == SAFE_HZ), passes[0])
        hints = []
        for r in passes:
            if r.hz == base.hz or base.mean_rpm < MIN_SPIN_RPM:
                continue
            tol = max(50.0, abs(base.mean_rpm) * 0.10)
            if abs(r.mean_rpm - base.mean_rpm) > tol:
                hints.append(
                    f"  {r.hz}Hz 均值 {r.mean_rpm:.0f} 相对 {base.hz}Hz "
                    f"{base.mean_rpm:.0f} 偏差偏大（>{tol:.0f}）→ MARGINAL，"
                    f"不据此抬高工作默认"
                )
        msg = (
            f"分析: 判转通过至 {self.max_spin_hz} Hz。\n"
            f"  工作默认 FREQ={self.recommended_hz} Hz"
            f"（相对 50Hz 基线偏差 ≤10%/±50RPM 的最高档；否则保持 50）。"
        )
        if hints:
            msg += "\n" + "\n".join(hints)
        return msg

    def finalize_recommendation(self) -> None:
        passes = [r for r in self.results if r.verdict == "PASS"]
        if not passes:
            self.recommended_hz = SAFE_HZ
            self.max_spin_hz = SAFE_HZ
            return
        self.max_spin_hz = max(r.hz for r in passes)
        base = next((r for r in passes if r.hz == SAFE_HZ), passes[0])
        rec = SAFE_HZ
        for r in passes:
            if base.mean_rpm < MIN_SPIN_RPM:
                break
            tol = max(50.0, abs(base.mean_rpm) * 0.10)
            if abs(r.mean_rpm - base.mean_rpm) <= tol:
                rec = r.hz
        self.recommended_hz = rec

def drain(ser: serial.Serial, budget_s: float = 0.12) -> tuple[list[str], list[dict]]:
    metas: list[str] = []
    telems: list[dict] = []
    for line in read_lines(ser, budget_s):
        if line.startswith("#"):
            metas.append(line)
        else:
            t = parse_telem(line)
            if t:
                telems.append(t)
    return metas, telems


def safe_stop(ser: serial.Serial) -> None:
    send(ser, "PWM 1000")
    send(ser, "MEASURE ABORT")
    send(ser, "STOP")
    send(ser, f"FREQ {SAFE_HZ}")
    time.sleep(0.15)
    drain(ser, 0.2)


def wait_stopped(ser: serial.Serial, timeout_s: float = STOP_CONFIRM_S) -> bool:
    send(ser, "PWM 1000")
    send(ser, "MEASURE ABORT")
    send(ser, "STOP")
    t0 = time.time()
    last = 999.0
    while time.time() - t0 < timeout_s:
        _, telems = drain(ser, 0.1)
        if telems:
            last = abs(float(telems[-1]["rpm"]))
            if last < MIN_SPIN_RPM:
                return True
    return last < MIN_SPIN_RPM * 2  # 略宽：慢滑行也算可换频


def run_one_freq(
    ser: serial.Serial,
    hz: int,
    pulse_us: int,
    hold_s: float,
) -> FreqResult:
    wait_stopped(ser)
    time.sleep(GAP_S)

    send(ser, f"FREQ {hz}")
    time.sleep(0.2)
    drain(ser, 0.15)
    # MEASURE 模式下 PWM 由 measure_pulse 锁存，不被 target_ramped 覆盖
    # （旧固件 OPEN+PWM 会被 controlTick 立刻盖回怠速）
    send(ser, "MEASURE START")
    time.sleep(0.15)
    drain(ser, 0.1)
    send(ser, "PING")
    send(ser, f"PWM {pulse_us}")

    samples: list[float] = []
    t0 = time.time()
    last_ping = t0
    while time.time() - t0 < hold_s:
        now = time.time()
        if now - last_ping >= 0.35:
            send(ser, "PING")
            last_ping = now
        _, telems = drain(ser, 0.1)
        for t in telems:
            samples.append(abs(float(t["rpm"])))
        # 不提前结束：需跑满时限，才能与 50Hz 基线比转速（裁定是否 MARGINAL）

    # 停油并回安全频率（FAIL/PASS 后均回 50Hz，符合 T1′）
    send(ser, "PWM 1000")
    send(ser, "MEASURE ABORT")
    send(ser, "STOP")
    send(ser, f"FREQ {SAFE_HZ}")
    drain(ser, 0.2)

    if not samples:
        return FreqResult(
            hz, pulse_us, hold_s, 0.0, 0.0, False, "FAIL", "无遥测（检查串口/固件）"
        )

    # 用后半段均值，避开启动瞬态；峰值为全程
    tail = samples[len(samples) // 2 :] or samples
    mean = sum(tail) / len(tail)
    peak = max(samples)
    moved = peak >= MIN_SPIN_RPM or mean >= MIN_SPIN_RPM * 0.75
    if moved:
        return FreqResult(
            hz, pulse_us, hold_s, mean, peak, True, "PASS",
            f"编码器见转 peak={peak:.0f} mean={mean:.0f}",
        )
    return FreqResult(
        hz,
        pulse_us,
        hold_s,
        mean,
        peak,
        False,
        "FAIL",
        f"{hold_s:.0f}s 内 |rpm|≈{mean:.1f}，未见转动",
    )


def run_scan(
    ser: serial.Serial,
    *,
    port: str,
    freqs: list[int],
    pulse_us: int,
    hold_s: float,
) -> FreqReport:
    report = FreqReport(port=port, pulse_us=pulse_us, hold_s=hold_s)
    info = wait_banner(ser)
    print("板子:", info)

    send(ser, "STOP")
    send(ser, "RPM 0")
    send(ser, f"FREQ {SAFE_HZ}")
    send(ser, "PWM 1000")
    time.sleep(0.3)
    drain(ser, 0.3)

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = LOG_DIR / f"freq_scan_{ts}.csv"

    with csv_path.open("w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(
            ["hz", "pulse_us", "hold_s", "mean_rpm", "peak_rpm", "moved", "verdict", "note"]
        )

        for hz in freqs:
            print(f"\n>>> FREQ {hz} Hz  开环 PWM {pulse_us}μs × {hold_s:.1f}s …")
            res = run_one_freq(ser, hz, pulse_us, hold_s)
            report.results.append(res)
            w.writerow(
                [
                    res.hz,
                    res.pulse_us,
                    f"{res.hold_s:.1f}",
                    f"{res.mean_rpm:.2f}",
                    f"{res.peak_rpm:.2f}",
                    int(res.moved),
                    res.verdict,
                    res.note,
                ]
            )
            fp.flush()
            print(
                f"    mean={res.mean_rpm:.1f} peak={res.peak_rpm:.1f} → {res.verdict} {res.note}"
            )

            if res.verdict == "FAIL":
                report.aborted_at = hz
                report.finalize_recommendation()
                # 其余标 SKIP
                for skip_hz in freqs:
                    if skip_hz <= hz:
                        continue
                    skip = FreqResult(
                        skip_hz, pulse_us, hold_s, 0.0, 0.0, False, "SKIP",
                        f"因 {hz}Hz FAIL 未测",
                    )
                    report.results.append(skip)
                    w.writerow(
                        [
                            skip.hz,
                            skip.pulse_us,
                            f"{skip.hold_s:.1f}",
                            "0",
                            "0",
                            0,
                            skip.verdict,
                            skip.note,
                        ]
                    )
                print(f"\n中止: {hz} Hz 未通过，更高频率跳过。")
                break
        else:
            report.finalize_recommendation()

    safe_stop(ser)
    text = report.summary_text() + f"\n\ncsv={csv_path}\n"
    rep_path = csv_path.with_suffix(".report.txt")
    rep_path.write_text(text, encoding="utf-8")

    # T1′ 冻结表（写进方案同目录，便于裁定 D4）
    freeze_md = _freeze_markdown(report, csv_path)
    freeze_path = LOG_DIR / f"freq_freeze_{ts}.md"
    freeze_path.write_text(freeze_md, encoding="utf-8")
    # 副本到项目根，方便对照分歧方案
    root_copy = Path(__file__).resolve().parent.parent / f"PWM频率冻结表_{ts}.md"
    root_copy.write_text(freeze_md, encoding="utf-8")

    print("\n======== 扫频报告 ========")
    print(text)
    print(f"报告: {rep_path}")
    print(f"冻结表: {freeze_path}")
    print(f"副本: {root_copy}")
    return report


def _freeze_markdown(report: FreqReport, csv_path: Path) -> str:
    lines = [
        "# PWM 频率冻结表（T1′）",
        "",
        "**工况：空载开环探点**",
        "",
        f"- 时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 端口: {report.port}",
        f"- 探点: {report.pulse_us} μs × {report.hold_s:.1f} s",
        f"- 判转: |rpm| ≥ {MIN_SPIN_RPM:.0f}",
        f"- 判转支持上限: {report.max_spin_hz} Hz",
        f"- **冻结工作默认 FREQ: {report.recommended_hz} Hz**",
        "",
        "| Hz | 时限 s | 峰值 RPM | 均值 RPM | 转动? | 结论 | 失败后回 50Hz |",
        "|----|--------|----------|----------|-------|------|---------------|",
    ]
    for r in report.results:
        back = "—" if r.hz == SAFE_HZ else ("是" if r.verdict in ("FAIL", "SKIP") else "是（档间）")
        lines.append(
            f"| {r.hz} | {r.hold_s:.1f} | {r.peak_rpm:.1f} | {r.mean_rpm:.1f} | "
            f"{'Y' if r.moved else 'N'} | {r.verdict} | {back} |"
        )
    lines.append("")
    lines.append(report.analysis())
    lines.append("")
    lines.append(f"原始: `{csv_path.name}`")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="PWM 频率自动扫频（编码器判转）")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--pulse", type=int, default=1100, help="开环探点脉宽 μs（≤1600）")
    ap.add_argument("--hold", type=float, default=3.0, help="每档给油秒数（建议 3 或 5）")
    ap.add_argument(
        "--freqs",
        default="",
        help="逗号分隔，默认 50,100,…,600",
    )
    args = ap.parse_args()

    pulse = max(1000, min(1600, int(args.pulse)))
    hold = max(1.0, min(30.0, float(args.hold)))
    if args.freqs.strip():
        freqs = [int(x) for x in args.freqs.split(",") if x.strip()]
    else:
        freqs = list(DEFAULT_FREQS)
    freqs = [f for f in freqs if 50 <= f <= 600]
    if not freqs:
        print("无有效频率")
        return 1

    print(f"打开 {args.port} @ {BAUD} …")
    print(
        f"将扫频 {freqs}，开环 {pulse}μs × {hold:.1f}s；"
        f"FAIL 即停更高档。请确认卸桨/固定。"
    )
    try:
        acquire("auto_freq_test")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        print("请先关闭 encoder_monitor 后再试。")
        return 1

    try:
        report = run_scan(
            ser, port=args.port, freqs=freqs, pulse_us=pulse, hold_s=hold
        )
        if report.aborted_at == SAFE_HZ:
            return 2
        if report.aborted_at is not None:
            return 3
        return 0
    except KeyboardInterrupt:
        print("\n中断 → 安全停")
        try:
            safe_stop(ser)
        except Exception:  # noqa: BLE001
            pass
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"异常: {exc}")
        try:
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
