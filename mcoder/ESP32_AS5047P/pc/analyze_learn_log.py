#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析自学习 CSV（LEARN START 台阶 / LEARN RAMP 斜坡均可）：
  - 油门是否单调 / Nyquist 折叠
  - 从台阶点重算 plant gain → 建议 PID（与固件 suggestPidFromMap 同公式）
  - 从台阶点逐段重算分区 PID（与固件 suggestPidZonesFromMap 同公式）→ 打印分区 PID 表
  - 对照固件 SUGGEST / GAIN point 行
  - 根据死区建议闭环起步转速（300 落死区则建议先试 1000）
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

# 与固件 suggestPidFromMap 一致
GAIN_FLOOR = 0.05
KP_NUM = 0.35
KI_RATIO = 2.5
MIN_SPIN_RPM = 40.0


def latest_log(logs_dir: Path) -> Path | None:
    files = sorted(
        list(logs_dir.glob("learn_*.csv")) + list(logs_dir.glob("learn_auto_*.csv")),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return files[0] if files else None


def parse_learn_points(rows: list[dict]) -> list[tuple[int, float]]:
    """解析台阶点 `ACK LEARN point` 或斜坡点 `LEARN RAMP sample`（两者格式一致）。"""
    points: list[tuple[int, float]] = []
    for r in rows:
        note = r.get("note") or ""
        if "ACK LEARN point" not in note and "LEARN RAMP sample" not in note:
            continue
        parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in note.split() if "=" in p}
        try:
            points.append((int(float(parts["pulse"])), abs(float(parts["rpm"]))))
        except (KeyError, ValueError):
            pass
    return points


def parse_gain_points(rows: list[dict]) -> list[dict[str, float]]:
    """解析固件 `# GAIN point rpm=... kp=... ki=... kd=...` 行（LEARN 完成后打印）。"""
    out: list[dict[str, float]] = []
    for r in rows:
        note = r.get("note") or ""
        if "GAIN point" not in note:
            continue
        parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in note.split() if "=" in p}
        try:
            out.append(
                {
                    "rpm": float(parts["rpm"]),
                    "kp": float(parts["kp"]),
                    "ki": float(parts["ki"]),
                    "kd": float(parts.get("kd", 0.0)),
                }
            )
        except (KeyError, ValueError):
            pass
    return out


def suggest_pid_zones_from_points(points: list[tuple[int, float]]) -> list[dict[str, float]]:
    """与固件 suggestPidZonesFromMap 同算法（PC 侧复核）：
    按前馈图逐段局部斜率 gain=drpm/dpulse 生成分区 PID，断点=段中点转速；
    另加 rpm=0 柔和增益作首点。"""
    zones: list[dict[str, float]] = [{"rpm": 0.0, "kp": 0.05, "ki": 0.10, "kd": 0.0}]
    if len(points) < 2:
        return zones
    pts = sorted(points, key=lambda x: x[0])
    for (p0, r0), (p1, r1) in zip(pts, pts[1:]):
        dpulse = float(p1 - p0)
        drpm = r1 - r0
        if dpulse <= 0.0 or drpm <= 1.0:
            continue
        gain = drpm / dpulse
        if gain < GAIN_FLOOR:
            gain = GAIN_FLOOR
        kp_s = KP_NUM / gain
        ki_s = kp_s * KI_RATIO
        kp_s = min(0.4, max(0.02, kp_s))
        ki_s = min(1.5, max(0.05, ki_s))
        rpm_bp = 0.5 * (r0 + r1)
        zones.append({"rpm": rpm_bp, "kp": kp_s, "ki": ki_s, "kd": 0.0})
        if len(zones) >= 12:  # GAIN_MAX_POINTS
            break
    return zones


def parse_firmware_suggest(rows: list[dict]) -> dict[str, float]:
    out: dict[str, float] = {}
    for r in rows:
        note = r.get("note") or ""
        if "SUGGEST" in note and "gain=" in note:
            m = re.search(
                r"gain=([0-9.]+).*kp=([0-9.]+).*ki=([0-9.]+)",
                note,
                re.I,
            )
            if m:
                out["fw_gain"] = float(m.group(1))
                out["fw_kp"] = float(m.group(2))
                out["fw_ki"] = float(m.group(3))
        if "HINT apply: PID" in note:
            toks = note.split("PID", 1)[1].split()
            try:
                out["fw_hint_kp"] = float(toks[0])
                out["fw_hint_ki"] = float(toks[1])
                if len(toks) > 2:
                    out["fw_hint_kd"] = float(toks[2])
            except (IndexError, ValueError):
                pass
    return out


def suggest_pid_from_points(points: list[tuple[int, float]]) -> dict[str, float]:
    """与固件 suggestPidFromMap 同算法（PC 侧复核）。"""
    if len(points) < 2:
        return {}
    n = len(points)
    i0 = n // 4
    i1 = (3 * n) // 4
    if i1 <= i0:
        i0, i1 = 0, n - 1
    dpulse = float(points[i1][0] - points[i0][0])
    drpm = float(points[i1][1] - points[i0][1])
    if dpulse < 1.0:
        dpulse = 1.0
    gain = drpm / dpulse
    if gain < GAIN_FLOOR:
        gain = GAIN_FLOOR
    kp = KP_NUM / gain
    ki = kp * KI_RATIO
    kp = min(0.4, max(0.02, kp))
    ki = min(1.5, max(0.05, ki))
    return {
        "gain": gain,
        "kp": kp,
        "ki": ki,
        "kd": 0.0,
        "i0_pulse": float(points[i0][0]),
        "i1_pulse": float(points[i1][0]),
        "i0_rpm": points[i0][1],
        "i1_rpm": points[i1][1],
    }


def interp_pulse_for_rpm(points: list[tuple[int, float]], rpm: float) -> float | None:
    """线性插值：目标转速 → 前馈脉宽。"""
    if len(points) < 2:
        return None
    pts = sorted(points, key=lambda x: x[1])
    if rpm <= pts[0][1]:
        return float(pts[0][0])
    if rpm >= pts[-1][1]:
        return float(pts[-1][0])
    for (p0, r0), (p1, r1) in zip(pts, pts[1:]):
        if r0 <= rpm <= r1 and r1 > r0:
            t = (rpm - r0) / (r1 - r0)
            return p0 + t * (p1 - p0)
    return None


def first_spin_point(points: list[tuple[int, float]]) -> tuple[int, float] | None:
    for pu, rpm in points:
        if rpm >= MIN_SPIN_RPM:
            return pu, rpm
    return None


def recommend_closed_targets(points: list[tuple[int, float]]) -> dict:
    """
    300 若落在死区（插值脉宽仍几乎不转），建议先试 1000。
    学习台阶已证明更高油门能转。
    """
    spin = first_spin_point(points)
    p300 = interp_pulse_for_rpm(points, 300.0)
    p1000 = interp_pulse_for_rpm(points, 1000.0)
    dead_300 = False
    if spin is None:
        return {
            "try_order": [1000.0],
            "dead_300": True,
            "reason": "学习曲线几乎无转动点，请先检查台架",
            "pulse_300": p300,
            "pulse_1000": p1000,
            "first_spin": None,
        }
    spin_pu, spin_rpm = spin
    # 若 300 对应脉宽明显低于首次转动脉宽 → 死区
    if p300 is not None and p300 < spin_pu - 10:
        dead_300 = True
    if spin_rpm >= 500 and (p300 is None or p300 < spin_pu):
        dead_300 = True
    if dead_300:
        return {
            "try_order": [1000.0, 1500.0],
            "dead_300": True,
            "reason": (
                f"300RPM 前馈≈{p300:.0f}μs，低于首次转动 {spin_pu}μs@{spin_rpm:.0f}RPM；"
                f"闭环易卡死区。学习已证明更高油门能转 → 先试 1000RPM"
            ),
            "pulse_300": p300,
            "pulse_1000": p1000,
            "first_spin": (spin_pu, spin_rpm),
        }
    return {
        "try_order": [300.0, 1000.0],
        "dead_300": False,
        "reason": f"300RPM 前馈≈{p300:.0f}μs，接近可转区；仍建议 300 失败后再试 1000",
        "pulse_300": p300,
        "pulse_1000": p1000,
        "first_spin": (spin_pu, spin_rpm),
    }


def analyze_pid(path: Path) -> str:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    points = parse_learn_points(rows)
    fw = parse_firmware_suggest(rows)
    fw_gain_points = parse_gain_points(rows)
    pc = suggest_pid_from_points(points)
    pc_zones = suggest_pid_zones_from_points(points) if points else []
    rec = recommend_closed_targets(points) if points else {}

    lines: list[str] = []
    lines.append("—— PID 自动调参分析（PC 复核）——")
    if not points:
        lines.append("无 LEARN point，无法重算 PID")
        return "\n".join(lines)

    lines.append(f"台阶点数: {len(points)}")
    if pc:
        lines.append(
            f"PC 重算: gain={pc['gain']:.3f} rpm/μs "
            f"(段 {pc['i0_pulse']:.0f}→{pc['i1_pulse']:.0f} μs, "
            f"{pc['i0_rpm']:.0f}→{pc['i1_rpm']:.0f} RPM)"
        )
        lines.append(
            f"PC 建议 PID:  Kp={pc['kp']:.4f}  Ki={pc['ki']:.4f}  Kd=0"
        )
        lines.append(f"下发命令:     PID {pc['kp']:.4f} {pc['ki']:.4f} 0")

    if fw.get("fw_kp") is not None:
        lines.append(
            f"固件 SUGGEST: gain={fw.get('fw_gain', float('nan')):.3f}  "
            f"Kp={fw['fw_kp']:.4f}  Ki={fw['fw_ki']:.4f}"
        )
        if pc:
            dkp = abs(pc["kp"] - fw["fw_kp"])
            dki = abs(pc["ki"] - fw["fw_ki"])
            if dkp < 0.005 and dki < 0.02:
                lines.append("对照: PC 与固件建议一致（OK）")
            else:
                lines.append(
                    f"对照: 与固件有偏差 ΔKp={dkp:.4f} ΔKi={dki:.4f} — 以学习点重算为准可再下发 PC 值"
                )
    else:
        lines.append("固件 SUGGEST: （日志中未捕获，用 PC 重算即可）")

    lines.append("")
    lines.append("—— 分区 PID（GainMap，按 rpm 插值）——")
    if pc_zones:
        lines.append(f"PC 重算分区数: {len(pc_zones)}")
        for z in pc_zones:
            lines.append(
                f"  rpm={z['rpm']:6.0f}  Kp={z['kp']:.4f}  Ki={z['ki']:.4f}  Kd={z['kd']:.2f}"
            )
        lines.append("下发: GAIN SAVE 后 GAIN ON 生效（固件学习完成会自动建表+保存）")
    else:
        lines.append("台阶点不足，无法重算分区 PID")

    if fw_gain_points:
        lines.append("")
        lines.append(f"固件 GAIN point（日志捕获）: {len(fw_gain_points)} 点")
        for z in fw_gain_points:
            lines.append(
                f"  rpm={z['rpm']:6.0f}  Kp={z['kp']:.4f}  Ki={z['ki']:.4f}  Kd={z['kd']:.2f}"
            )

    if rec:
        lines.append("")
        lines.append("—— 闭环起步建议 ——")
        fs = rec.get("first_spin")
        if fs:
            lines.append(f"开环首次转动: {fs[0]} μs → {fs[1]:.0f} RPM")
        if rec.get("pulse_300") is not None:
            lines.append(f"300RPM 前馈脉宽≈ {rec['pulse_300']:.0f} μs")
        if rec.get("pulse_1000") is not None:
            lines.append(f"1000RPM 前馈脉宽≈ {rec['pulse_1000']:.0f} μs")
        lines.append(f"试转顺序: {' → '.join(f'{t:.0f}' for t in rec['try_order'])} RPM")
        lines.append(f"原因: {rec['reason']}")

    return "\n".join(lines)


def analyze(path: Path, ctrl_hz_hint: float = 100.0) -> str:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        return f"空日志: {path}"

    telem = [r for r in rows if r.get("kind") == "telem" and r.get("pulse_us")]
    points = parse_learn_points(rows)

    lines: list[str] = []
    lines.append(f"文件: {path}")
    lines.append(f"总行数: {len(rows)}  遥测点: {len(telem)}  学习台阶点: {len(points)}")

    pulses = [int(float(r["pulse_us"])) for r in telem]
    rpms = [abs(float(r["rpm"])) for r in telem if r.get("rpm") not in ("", None)]
    if not pulses:
        lines.append("结论: 无遥测 pulse，无法分析")
        lines.append("")
        lines.append(analyze_pid(path))
        return "\n".join(lines)

    drops = []
    prev = pulses[0]
    for i, pu in enumerate(pulses):
        if pu < prev - 20:
            drops.append((i, prev, pu))
        prev = pu

    nyquist_rpm = 0.5 * ctrl_hz_hint * 60.0
    rpm_peak = max(rpms) if rpms else 0.0
    pulse_at_peak = pulses[rpms.index(max(rpms))] if rpms else 0

    lines.append(f"油门范围: {min(pulses)} … {max(pulses)} μs")
    lines.append(
        f"油门回落(>20μs): {len(drops)} 次"
        + (" — 异常" if drops else " — 正常（单调上台阶）")
    )
    if drops[:5]:
        for i, a, b in drops[:5]:
            lines.append(f"  drop#{i}: {a} → {b}")

    lines.append(f"转速峰值: {rpm_peak:.1f} RPM @ pulse≈{pulse_at_peak} μs")
    lines.append(f"按 ctrl≈{ctrl_hz_hint:.0f}Hz 的理论测速上限≈{nyquist_rpm:.0f} RPM")

    fold = False
    if len(points) >= 4:
        lines.append("学习台阶 (pulse → |rpm|):")
        peak_i = max(range(len(points)), key=lambda i: points[i][1])
        for i, (pu, rpm) in enumerate(points):
            mark = "  <<峰值" if i == peak_i else ""
            spin_mark = "  <<首次转动" if rpm >= MIN_SPIN_RPM and all(
                points[j][1] < MIN_SPIN_RPM for j in range(i)
            ) else ""
            lines.append(f"  {pu:4d} μs → {rpm:7.1f}{mark}{spin_mark}")
        after = points[peak_i + 1 :]
        if after and points[peak_i][1] > nyquist_rpm * 0.85:
            down = sum(
                1 for j in range(1, min(6, len(after))) if after[j][1] < after[j - 1][1]
            )
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
        lines.append(
            "处理: 提高编码器测速频率（例如 ctrl 250~400Hz），使上限高于电机最大 RPM。"
        )
    elif drops:
        lines.append("")
        lines.append("结论: 油门本身有回落，需查 UI/MODE/PWM 抢控。")
    else:
        lines.append("")
        lines.append("结论: 油门单调；转速曲线需结合 Nyquist 与负载判断。")

    done = next((r for r in rows if r.get("kind") == "done"), None)
    if done and done.get("note"):
        lines.append(f"结束: {done['note'][:160]}")

    lines.append("")
    lines.append(analyze_pid(path))
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="分析自学习 CSV + PID 建议")
    ap.add_argument("path", nargs="?", help="learn_*.csv；默认取 pc/logs 最新")
    ap.add_argument("--ctrl-hz", type=float, default=250.0)
    ap.add_argument("--pid-only", action="store_true", help="只输出 PID/起步分析")
    args = ap.parse_args()
    logs_dir = Path(__file__).resolve().parent / "logs"
    path = Path(args.path) if args.path else latest_log(logs_dir)
    if path is None or not path.exists():
        print("未找到学习日志。请先在界面跑一次自动学习。", file=sys.stderr)
        return 1
    text = analyze_pid(path) if args.pid_only else analyze(path, ctrl_hz_hint=args.ctrl_hz)
    print(text)
    out = path.with_suffix(".report.txt")
    out.write_text(text + "\n", encoding="utf-8")
    print(f"\n报告已写: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
