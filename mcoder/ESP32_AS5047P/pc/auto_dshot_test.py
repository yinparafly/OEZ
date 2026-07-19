#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
T2 DShot 自动扫速：DShot150 → 300 → 600（到 600 为止）。

规则（对齐 T1′）:
  - 每档：停转 → DSHOTRATE → 开环 DSHOT 探点油门 → hold 秒
  - 编码器 |rpm|≥阈值 → PASS；否则 FAIL
  - FAIL → 立即 DSHOT 0 + PROTO PWM（安全回模拟），更高档不测
  - 150 就 FAIL → 台架/电调未开 DShot/接线问题，不是「不支持 600」

用法:
  python auto_dshot_test.py --port COM10
  python auto_dshot_test.py --hold 3 --dshot 250
  python auto_dshot_test.py --rates 150,300,600
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

DEFAULT_RATES = (150, 300, 600)
MIN_SPIN_RPM = 40.0
STOP_CONFIRM_S = 1.5
GAP_S = 0.8


def pulse_to_dshot(pulse_us: int) -> int:
    """与固件 pulseUsToDshot 一致：1000→0，1000..2000→48..2047。"""
    if pulse_us <= 1000:
        return 0
    if pulse_us >= 2000:
        return 2047
    t = (pulse_us - 1000) / 1000.0
    return int(48 + t * (2047 - 48) + 0.5)


@dataclass
class DshotResult:
    rate: int
    thr: int
    hold_s: float
    mean_rpm: float
    peak_rpm: float
    moved: bool
    verdict: str
    note: str = ""


@dataclass
class DshotReport:
    port: str
    thr: int
    hold_s: float
    results: list[DshotResult] = field(default_factory=list)
    recommended_rate: int = 0
    max_ok_rate: int = 0
    aborted_at: int | None = None

    def summary_text(self) -> str:
        lines = [
            f"port={self.port} dshot_thr={self.thr} hold={self.hold_s:.1f}s",
            f"判转阈值: |rpm| >= {MIN_SPIN_RPM:.0f}",
            "",
            f"{'rate':>6} {'mean':>8} {'peak':>8} {'moved':>6}  verdict  note",
        ]
        for r in self.results:
            lines.append(
                f"{r.rate:6d} {r.mean_rpm:8.1f} {r.peak_rpm:8.1f} "
                f"{'Y' if r.moved else 'N':>6}  {r.verdict:<7} {r.note}"
            )
        lines.append("")
        lines.append(f"判转支持上限: DShot{self.max_ok_rate}" if self.max_ok_rate else "判转支持上限: 无")
        lines.append(
            f"推荐工作 DShot: {self.recommended_rate}"
            if self.recommended_rate
            else "推荐工作 DShot: 无（回 PWM）"
        )
        if self.aborted_at is not None:
            lines.append(f"在 DShot{self.aborted_at} FAIL → 更高档未测，已回 PROTO PWM")
        lines.append("")
        lines.append(self.analysis())
        return "\n".join(lines)

    def analysis(self) -> str:
        if not self.results:
            return "分析: 无结果"
        first = self.results[0]
        if first.rate == 150 and first.verdict == "FAIL":
            return (
                "分析: DShot150 时限内编码器未见转动。\n"
                "  这不是「电调不支持 600」，而是基线协议就不通。请查：\n"
                "  1) 电调是否已切到 DShot（BLHeli/AM32 配置）\n"
                "  2) 信号线仍接 GPIO9、共地；动力电是否已上\n"
                "  3) 探点油门是否过低（试 --dshot 400 或 --pulse 1200）\n"
                "  4) 固件是否已烧录含 PROTO DSHOT 的版本"
            )
        if self.aborted_at is not None:
            prev = [r for r in self.results if r.verdict == "PASS" and r.rate < self.aborted_at]
            if prev:
                return (
                    f"分析: DShot{self.aborted_at} 未转，而 DShot{prev[-1].rate} 已通过。\n"
                    f"  建议工作用 DShot{self.recommended_rate}；更高档跳过。"
                )
            return f"分析: DShot{self.aborted_at} FAIL 且无更低档 PASS。"
        if self.max_ok_rate:
            return (
                f"分析: 判转通过至 DShot{self.max_ok_rate}。\n"
                f"  工作建议 DShot{self.recommended_rate}（取最高通过档）。"
            )
        return "分析: 全部未通过"


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


def safe_pwm(ser: serial.Serial) -> None:
    send(ser, "DSHOT 0")
    send(ser, "PROTO PWM")
    send(ser, "FREQ 50")
    send(ser, "PWM 1000")
    send(ser, "STOP")
    time.sleep(0.2)
    drain(ser, 0.25)


def wait_stopped(ser: serial.Serial, timeout_s: float = STOP_CONFIRM_S) -> bool:
    send(ser, "DSHOT 0")
    send(ser, "PWM 1000")
    send(ser, "STOP")
    t0 = time.time()
    last = 999.0
    while time.time() - t0 < timeout_s:
        _, telems = drain(ser, 0.1)
        if telems:
            last = abs(float(telems[-1]["rpm"]))
            if last < MIN_SPIN_RPM:
                return True
    return last < MIN_SPIN_RPM * 2


def run_one_rate(
    ser: serial.Serial,
    rate: int,
    thr: int,
    hold_s: float,
) -> DshotResult:
    wait_stopped(ser)
    time.sleep(GAP_S)

    send(ser, f"DSHOTRATE {rate}")
    time.sleep(0.25)
    metas, _ = drain(ser, 0.2)
    joined = "\n".join(metas)
    if "ERR" in joined and "ACK DSHOTRATE" not in joined:
        return DshotResult(
            rate, thr, hold_s, 0.0, 0.0, False, "FAIL",
            f"设速失败: {joined[-120:]}",
        )

    send(ser, "PING")
    send(ser, f"DSHOT {thr}")

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

    send(ser, "DSHOT 0")
    drain(ser, 0.15)

    if not samples:
        return DshotResult(
            rate, thr, hold_s, 0.0, 0.0, False, "FAIL", "无遥测"
        )

    tail = samples[len(samples) // 2 :] or samples
    mean = sum(tail) / len(tail)
    peak = max(samples)
    moved = peak >= MIN_SPIN_RPM or mean >= MIN_SPIN_RPM * 0.75
    if moved:
        return DshotResult(
            rate, thr, hold_s, mean, peak, True, "PASS",
            f"编码器见转 peak={peak:.0f} mean={mean:.0f}",
        )
    return DshotResult(
        rate, thr, hold_s, mean, peak, False, "FAIL",
        f"{hold_s:.0f}s 内 |rpm|≈{mean:.1f}，未见转动",
    )


def run_scan(
    ser: serial.Serial,
    *,
    port: str,
    rates: list[int],
    thr: int,
    hold_s: float,
) -> DshotReport:
    report = DshotReport(port=port, thr=thr, hold_s=hold_s)
    info = wait_banner(ser)
    print("板子:", info)

    # 探测是否支持 PROTO
    send(ser, "PROTO?")
    time.sleep(0.15)
    metas, _ = drain(ser, 0.2)
    proto_lines = [m for m in metas if "PROTO" in m]
    print("PROTO:", proto_lines[-1] if proto_lines else "(无应答 — 可能未烧录 DShot 固件)")

    send(ser, "STOP")
    send(ser, "RPM 0")
    safe_pwm(ser)

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = LOG_DIR / f"dshot_scan_{ts}.csv"

    with csv_path.open("w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(
            ["rate", "thr", "hold_s", "mean_rpm", "peak_rpm", "moved", "verdict", "note"]
        )

        for rate in rates:
            print(f"\n>>> DShot{rate}  开环 thr={thr} × {hold_s:.1f}s …")
            res = run_one_rate(ser, rate, thr, hold_s)
            report.results.append(res)
            w.writerow(
                [
                    res.rate,
                    res.thr,
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
                report.aborted_at = rate
                passes = [r.rate for r in report.results if r.verdict == "PASS"]
                report.max_ok_rate = max(passes) if passes else 0
                report.recommended_rate = report.max_ok_rate
                for skip in rates:
                    if skip <= rate:
                        continue
                    skip_r = DshotResult(
                        skip, thr, hold_s, 0.0, 0.0, False, "SKIP",
                        f"因 DShot{rate} FAIL 未测",
                    )
                    report.results.append(skip_r)
                    w.writerow(
                        [skip, thr, f"{hold_s:.1f}", "0", "0", 0, "SKIP", skip_r.note]
                    )
                print(f"\n中止: DShot{rate} 未通过，更高档跳过，回 PROTO PWM。")
                break
        else:
            passes = [r.rate for r in report.results if r.verdict == "PASS"]
            report.max_ok_rate = max(passes) if passes else 0
            report.recommended_rate = report.max_ok_rate

    safe_pwm(ser)
    text = report.summary_text() + f"\n\ncsv={csv_path}\n"
    rep_path = csv_path.with_suffix(".report.txt")
    rep_path.write_text(text, encoding="utf-8")

    freeze = _freeze_md(report, csv_path)
    freeze_path = LOG_DIR / f"dshot_freeze_{ts}.md"
    freeze_path.write_text(freeze, encoding="utf-8")
    root_copy = Path(__file__).resolve().parent.parent / f"DShot冻结表_{ts}.md"
    root_copy.write_text(freeze, encoding="utf-8")

    print("\n======== DShot 扫速报告 ========")
    print(text)
    print(f"报告: {rep_path}")
    print(f"冻结表: {freeze_path}")
    print(f"副本: {root_copy}")
    return report


def _freeze_md(report: DshotReport, csv_path: Path) -> str:
    lines = [
        "# DShot 冻结表（T2）",
        "",
        "**工况：空载开环探点**",
        "",
        f"- 时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 端口: {report.port}",
        f"- 探点: DSHOT {report.thr} × {report.hold_s:.1f} s",
        f"- 判转: |rpm| ≥ {MIN_SPIN_RPM:.0f}",
        f"- 判转支持上限: DShot{report.max_ok_rate}" if report.max_ok_rate else "- 判转支持上限: 无",
        (
            f"- **冻结工作协议: DShot{report.recommended_rate}**"
            if report.recommended_rate
            else "- **冻结工作协议: 回 PWM**（DShot 未通过）"
        ),
        "",
        "| Rate | 时限 s | 峰值 RPM | 均值 RPM | 转动? | 结论 | 失败后回 PWM |",
        "|------|--------|----------|----------|-------|------|--------------|",
    ]
    for r in report.results:
        back = "—" if r.rate == 150 and r.verdict != "FAIL" else "是"
        if r.verdict == "PASS":
            back = "档间停转"
        lines.append(
            f"| {r.rate} | {r.hold_s:.1f} | {r.peak_rpm:.1f} | {r.mean_rpm:.1f} | "
            f"{'Y' if r.moved else 'N'} | {r.verdict} | {back} |"
        )
    lines.append("")
    lines.append(report.analysis())
    lines.append("")
    lines.append(f"原始: `{csv_path.name}`")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="DShot150/300/600 自动扫速（编码器判转）")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--pulse", type=int, default=1100, help="等价 PWM 探点 μs，映射为 DShot 油门")
    ap.add_argument("--dshot", type=int, default=0, help="直接指定 DShot 油门 48..2047（优先于 --pulse）")
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--rates", default="", help="默认 150,300,600")
    args = ap.parse_args()

    if args.dshot > 0:
        thr = max(0, min(2047, int(args.dshot)))
    else:
        thr = pulse_to_dshot(max(1000, min(2000, int(args.pulse))))
    if thr < 48:
        print("探点油门过低（映射后 <48），请提高 --pulse 或 --dshot")
        return 1

    hold = max(1.0, min(30.0, float(args.hold)))
    if args.rates.strip():
        rates = [int(x) for x in args.rates.split(",") if x.strip()]
    else:
        rates = list(DEFAULT_RATES)
    rates = [r for r in rates if r in (150, 300, 600)]
    if not rates:
        print("无有效 DShot 速率（仅 150/300/600）")
        return 1

    print(f"打开 {args.port} @ {BAUD} …")
    print(
        f"将扫 DShot {rates}，开环 thr={thr} × {hold:.1f}s；"
        f"FAIL 即停更高档并回 PWM。请确认卸桨/固定，电调已开 DShot。"
    )
    try:
        acquire("auto_dshot_test")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        return 1

    try:
        report = run_scan(ser, port=args.port, rates=rates, thr=thr, hold_s=hold)
        if report.aborted_at == 150:
            return 2
        if report.aborted_at is not None:
            return 3
        return 0
    except KeyboardInterrupt:
        print("\n中断 → 回 PWM")
        try:
            safe_pwm(ser)
        except Exception:  # noqa: BLE001
            pass
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"异常: {exc}")
        try:
            safe_pwm(ser)
        except Exception:  # noqa: BLE001
            pass
        return 5
    finally:
        try:
            safe_pwm(ser)
        except Exception:  # noqa: BLE001
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
