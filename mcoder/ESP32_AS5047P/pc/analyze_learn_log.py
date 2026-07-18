#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析自学习 CSV：油门是否单调、转速是否在采样极限附近折叠。"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


def latest_log(logs_dir: Path) -> Path | None:
    files = sorted(logs_dir.glob("learn_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def analyze(path: Path, ctrl_hz_hint: float = 100.0) -> str:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        return f"空日志: {path}"

    telem = [r for r in rows if r.get("kind") == "telem" and r.get("pulse_us")]
    points = []
    for r in rows:
        note = r.get("note") or ""
        if "ACK LEARN point" in note:
            parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in note.split() if "=" in p}
            try:
                points.append((int(float(parts["pulse"])), abs(float(parts["rpm"]))))
            except (KeyError, ValueError):
                pass

    lines: list[str] = []
    lines.append(f"文件: {path}")
    lines.append(f"总行数: {len(rows)}  遥测点: {len(telem)}  学习台阶点: {len(points)}")

    pulses = [int(float(r["pulse_us"])) for r in telem]
    rpms = [abs(float(r["rpm"])) for r in telem if r.get("rpm") not in ("", None)]
    if not pulses:
        lines.append("结论: 无遥测 pulse，无法分析")
        return "\n".join(lines)

    drops = []
    prev = pulses[0]
    for i, pu in enumerate(pulses):
        if pu < prev - 20:
            drops.append((i, prev, pu))
        prev = pu

    nyquist_rpm = 0.5 * ctrl_hz_hint * 60.0  # 角度差分 ±180°/样本 的理论上限
    rpm_peak = max(rpms) if rpms else 0.0
    pulse_at_peak = pulses[rpms.index(max(rpms))] if rpms else 0

    lines.append(f"油门范围: {min(pulses)} … {max(pulses)} μs")
    lines.append(f"油门回落(>20μs): {len(drops)} 次" + (" — 异常" if drops else " — 正常（单调上台阶）"))
    if drops[:5]:
        for i, a, b in drops[:5]:
            lines.append(f"  drop#{i}: {a} → {b}")

    lines.append(f"转速峰值: {rpm_peak:.1f} RPM @ pulse≈{pulse_at_peak} μs")
    lines.append(f"按 ctrl≈{ctrl_hz_hint:.0f}Hz 的理论测速上限≈{nyquist_rpm:.0f} RPM")

    # 台阶表：油门升、转速却降
    fold = False
    if len(points) >= 4:
        lines.append("学习台阶 (pulse → |rpm|):")
        peak_i = max(range(len(points)), key=lambda i: points[i][1])
        for i, (pu, rpm) in enumerate(points):
            mark = "  <<峰值" if i == peak_i else ""
            lines.append(f"  {pu:4d} μs → {rpm:7.1f}{mark}")
        # 峰值之后若油门继续升而转速持续下降 → 折叠嫌疑
        after = points[peak_i + 1 :]
        if after and points[peak_i][1] > nyquist_rpm * 0.85:
            down = sum(1 for j in range(1, min(6, len(after))) if after[j][1] < after[j - 1][1])
            if down >= 2:
                fold = True

    if len(drops) == 0 and fold:
        lines.append("")
        lines.append("结论: 油门一直在升高，没有先升后降。")
        lines.append(
            f"你看到的「升→降→再升」是实测转速在 ~{nyquist_rpm:.0f} RPM 附近折叠："
            "采样跟不上真实转角，差分被 unwrap 算错，转速显示变假低，"
            "再往后又重新锁到另一段表观转速。"
        )
        lines.append("处理: 提高编码器测速频率（例如 ctrl 250~400Hz），使上限高于电机最大 RPM。")
    elif drops:
        lines.append("")
        lines.append("结论: 油门本身有回落，需查 UI/MODE/PWM 抢控。")
    else:
        lines.append("")
        lines.append("结论: 油门单调；转速曲线需结合 Nyquist 与负载判断。")

    done = next((r for r in rows if r.get("kind") == "done"), None)
    if done and done.get("note"):
        lines.append(f"结束: {done['note'][:160]}")

    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="分析自学习 CSV")
    ap.add_argument("path", nargs="?", help="learn_*.csv；默认取 pc/logs 最新")
    ap.add_argument("--ctrl-hz", type=float, default=100.0)
    args = ap.parse_args()
    logs_dir = Path(__file__).resolve().parent / "logs"
    path = Path(args.path) if args.path else latest_log(logs_dir)
    if path is None or not path.exists():
        print("未找到学习日志。请先在界面跑一次自动学习。", file=sys.stderr)
        return 1
    text = analyze(path, ctrl_hz_hint=args.ctrl_hz)
    print(text)
    out = path.with_suffix(".report.txt")
    out.write_text(text + "\n", encoding="utf-8")
    print(f"\n报告已写: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
