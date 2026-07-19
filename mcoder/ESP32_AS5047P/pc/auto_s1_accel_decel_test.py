#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S1 自动验证：升/降斜坡不对称（备忘 A1/A2）。

流程:
  1) 预检 ctrl_hz / 遥测
  2) 对称基线 UP=DOWN=800：A1 升速、A2 降速
  3) 不对称 UP=800 DOWN=300：再跑 A1/A2
  4) 写 CSV + 报告，对比下降段是否更跟得上

用法:
  python auto_s1_accel_decel_test.py --port COM10
  python auto_s1_accel_decel_test.py --dry
  python auto_s1_accel_decel_test.py --skip-baseline   # 只跑不对称
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
    run_dry,
    send,
    wait_banner,
)
from com_port_guard import acquire

MIN_SPIN = 40.0
BAND_FRAC = 0.10  # ±10% 进入带宽


@dataclass
class TrajResult:
    name: str
    up: float
    down: float
    rpm_from: float
    rpm_to: float
    settle_s: float
    t_in_band_s: float | None  # 首次进入带宽的时间；None=未进入
    mean_tail: float
    peak: float
    pulse_end: int
    overshoot_pct: float
    verdict: str
    note: str = ""


@dataclass
class S1Report:
    port: str
    soft_up_down: bool = False
    results: list[TrajResult] = field(default_factory=list)

    def summary(self) -> str:
        lines = [
            f"port={self.port}",
            "S1 升/降斜坡对比（备忘 A1/A2）",
            f"固件支持 SOFT RATE UP/DOWN: {'是' if self.soft_up_down else '否（本次不对称对比无效）'}",
            "",
            f"{'name':<22} {'up':>5} {'dn':>5} {'from':>5}->{'to':<5} "
            f"{'t_band':>7} {'tail':>7} {'over%':>6}  verdict",
        ]
        for r in self.results:
            tb = f"{r.t_in_band_s:.2f}" if r.t_in_band_s is not None else "—"
            lines.append(
                f"{r.name:<22} {r.up:5.0f} {r.down:5.0f} "
                f"{r.rpm_from:5.0f}->{r.rpm_to:<5.0f} "
                f"{tb:>7} {r.mean_tail:7.1f} {r.overshoot_pct:6.1f}  "
                f"{r.verdict} {r.note}"
            )
        lines.append("")
        if not self.soft_up_down:
            lines.append("—— 结论 ——")
            lines.append(
                "板子固件尚无 S1（SOFT RATE UP/DOWN）。"
                "闭环轨迹 A1/A2 能跑通（PING 保活），但升/降不对称未真正生效。"
            )
            lines.append("下一步: 烧录含 S1 的 ESP32ReadAS5047P.ino 后重跑本脚本。")
            lines.append("OVERALL: NEED_FLASH_S1")
            return "\n".join(lines)

        sym = next((x for x in self.results if x.name == "A2_sym"), None)
        asym = next((x for x in self.results if x.name == "A2_asym"), None)
        if sym and asym:
            lines.append("—— A2 降速对比 ——")
            if asym.t_in_band_s is not None and sym.t_in_band_s is not None:
                d = asym.t_in_band_s - sym.t_in_band_s
                lines.append(
                    f"进入±10%带宽时间: 对称 {sym.t_in_band_s:.2f}s → "
                    f"降慢 {asym.t_in_band_s:.2f}s (Δ={d:+.2f}s)"
                )
                lines.append(
                    "分析: 降速斜坡放慢后应更跟指令；若仍 FAIL 再查滑行/积分，勿急开 S2。"
                )
        fails = [r for r in self.results if r.verdict.startswith("FAIL")]
        if not fails and self.results:
            lines.append("OVERALL: PASS")
        elif self.results:
            lines.append(f"OVERALL: CHECK（{len(fails)} 段 FAIL）")
        return "\n".join(lines)


def drain(ser: serial.Serial, budget_s: float = 0.1):
    metas, telems = [], []
    for line in read_lines(ser, budget_s):
        if line.startswith("#"):
            metas.append(line)
        else:
            t = parse_telem(line)
            if t:
                telems.append(t)
    return metas, telems


def keep_alive(ser: serial.Serial) -> None:
    """固件 HOST_TIMEOUT≈1.5s，自动测试必须周期性 PING。"""
    send(ser, "PING")


def safe_stop(ser: serial.Serial) -> None:
    send(ser, "STOP")
    send(ser, "RPM 0")
    send(ser, "PWM 1000")
    keep_alive(ser)
    time.sleep(0.3)
    drain(ser, 0.2)


_soft_up_down_ok: bool | None = None


def probe_soft_up_down(ser: serial.Serial) -> bool:
    """探测固件是否支持 SOFT RATE UP/DOWN。"""
    global _soft_up_down_ok
    if _soft_up_down_ok is not None:
        return _soft_up_down_ok
    send(ser, "SOFT RATE UP 800")
    time.sleep(0.1)
    metas, _ = drain(ser, 0.25)
    if any("ACK SOFT RATE UP" in m for m in metas):
        _soft_up_down_ok = True
        send(ser, "SOFT RATE DOWN 300")
        drain(ser, 0.15)
        return True
    if any("ERR unknown" in m for m in metas):
        _soft_up_down_ok = False
        print("  注意: 固件无 SOFT RATE UP/DOWN — 请烧录含 S1 的固件后再做不对称对比")
        return False
    # 旧固件可能把 "SOFT RATE UP 800" 吃成 SOFT RATE=800（忽略 UP）
    if any("ACK SOFT RATE=" in m for m in metas):
        _soft_up_down_ok = False
        print("  注意: 固件将 SOFT RATE UP 当作对称 RATE — 不对称对比无效，请烧录 S1 固件")
        return False
    _soft_up_down_ok = False
    return False


def set_soft(ser: serial.Serial, up: float, down: float) -> None:
    send(ser, "SOFT ON")
    if probe_soft_up_down(ser):
        send(ser, f"SOFT RATE UP {up:.1f}")
        send(ser, f"SOFT RATE DOWN {down:.1f}")
    else:
        send(ser, f"SOFT RATE {((up + down) / 2):.1f}")
    keep_alive(ser)
    drain(ser, 0.1)


def wait_near(ser: serial.Serial, rpm: float, timeout_s: float, band: float) -> bool:
    """闭环到某转速附近；期间持续 PING 防 host_timeout。"""
    t0 = time.time()
    last_ping = 0.0
    while time.time() - t0 < timeout_s:
        now = time.time()
        if now - last_ping >= 0.4:
            keep_alive(ser)
            last_ping = now
        _, telems = drain(ser, 0.1)
        if telems and abs(abs(telems[-1]["rpm"]) - rpm) <= band:
            return True
    return False


def run_traj(
    ser: serial.Serial,
    *,
    name: str,
    up: float,
    down: float,
    rpm_from: float,
    rpm_to: float,
    settle_s: float,
    rpmmax: float,
) -> TrajResult:
    set_soft(ser, up, down)
    send(ser, "MODE CLOSED")
    send(ser, f"RPMMAX {rpmmax:.0f}")

    # 先到起点
    send(ser, f"RPM {rpm_from:.0f}")
    send(ser, "START")
    band0 = max(80.0, abs(rpm_from) * BAND_FRAC)
    ok0 = wait_near(ser, rpm_from, timeout_s=max(8.0, abs(rpm_from) / max(up, 1) + 5), band=band0)
    if not ok0 and rpm_from > MIN_SPIN:
        # 再等一会儿
        time.sleep(2.0)
        _, telems = drain(ser, 0.3)
        if not telems or abs(telems[-1]["rpm"]) < MIN_SPIN:
            safe_stop(ser)
            return TrajResult(
                name, up, down, rpm_from, rpm_to, settle_s, None, 0, 0, 1000, 0,
                "FAIL_START", f"未到起点 {rpm_from:.0f}",
            )

    time.sleep(1.0)  # 起点稍稳
    keep_alive(ser)

    # 阶跃到终点（斜坡由固件 SOFT 完成）
    send(ser, f"RPM {rpm_to:.0f}")
    samples: list[tuple[float, float, int]] = []  # t_rel, |rpm|, pulse
    t0 = time.time()
    band = max(50.0, abs(rpm_to) * BAND_FRAC)
    t_in: float | None = None
    last_ping = 0.0
    while time.time() - t0 < settle_s:
        now_wall = time.time()
        if now_wall - last_ping >= 0.4:
            keep_alive(ser)
            last_ping = now_wall
        metas, telems = drain(ser, 0.08)
        for m in metas:
            if "ESTOP" in m and "cleared" not in m.lower() and "host_timeout" in m.lower():
                # 仍可能超时：记 ABORT
                safe_stop(ser)
                return TrajResult(
                    name, up, down, rpm_from, rpm_to, settle_s, t_in, 0, 0, 1000, 0,
                    "ABORT", m[:80],
                )
            if "ESTOP" in m and "cleared" not in m.lower() and "ACK ESTOP" not in m:
                if "host_timeout" not in m.lower():
                    safe_stop(ser)
                    return TrajResult(
                        name, up, down, rpm_from, rpm_to, settle_s, t_in, 0, 0, 1000, 0,
                        "ABORT", m[:80],
                    )
        now = time.time() - t0
        for t in telems:
            rpm = abs(float(t["rpm"]))
            samples.append((now, rpm, int(t["pulse_us"])))
            if t_in is None and abs(rpm - rpm_to) <= band:
                t_in = now

    safe_stop(ser)
    time.sleep(1.2)

    if not samples:
        return TrajResult(
            name, up, down, rpm_from, rpm_to, settle_s, None, 0, 0, 1000, 0,
            "FAIL_START", "无遥测",
        )

    tail = [s for s in samples if s[0] >= settle_s - 1.5] or samples[-10:]
    mean_tail = sum(s[1] for s in tail) / len(tail)
    peak = max(s[1] for s in samples)
    pulse_end = tail[-1][2]
    # 超调：相对终点
    if rpm_to >= rpm_from:
        overshoot = max(0.0, (peak - rpm_to) / max(rpm_to, 1.0) * 100.0)
    else:
        # 降速：看是否掉过头（低于目标）
        undershoot = max(0.0, (rpm_to - min(s[1] for s in samples)) / max(rpm_to, 1.0) * 100.0)
        overshoot = undershoot

    if mean_tail < MIN_SPIN and rpm_to > MIN_SPIN:
        verdict, note = "FAIL_START", "终点未见转动"
    elif t_in is None:
        verdict, note = "FAIL_BAND", f"未进±{BAND_FRAC*100:.0f}%带 tail={mean_tail:.0f}"
    elif overshoot > 25.0:
        verdict, note = "MARGINAL", f"超调/过冲 {overshoot:.0f}%"
    elif abs(mean_tail - rpm_to) > max(120.0, rpm_to * 0.15):
        verdict, note = "MARGINAL", f"稳态偏 mean={mean_tail:.0f}"
    else:
        verdict, note = "PASS", f"t_band={t_in:.2f}s"

    return TrajResult(
        name, up, down, rpm_from, rpm_to, settle_s, t_in, mean_tail, peak,
        pulse_end, overshoot, verdict, note,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="S1 升/降斜坡自动验证 A1/A2")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    ap.add_argument("--settle", type=float, default=8.0, help="每段轨迹观察秒数")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--skip-baseline", action="store_true", help="跳过对称基线")
    ap.add_argument("--up", type=float, default=800.0)
    ap.add_argument("--down", type=float, default=300.0)
    args = ap.parse_args()

    print(f"打开 {args.port} @ {BAUD} …")
    try:
        acquire("auto_s1_accel_decel_test")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        print("请先关闭 encoder_monitor 界面（或其它自动测试）。")
        return 1

    try:
        if args.dry:
            return run_dry(ser)

        info = wait_banner(ser)
        print("板子:", info)
        ctrl = float(info.get("ctrl_hz") or 0)
        if ctrl < 200:
            print(f"FAIL: ctrl_hz={ctrl} < 200，拒绝给油（测速会折叠）")
            return 2

        print(
            "警告: 将自动闭环跑 A1(升)/A2(降)。请确认卸桨、固定、周围安全。\n"
            f"  对称基线 up=down={args.up:.0f}；不对称 up={args.up:.0f} down={args.down:.0f}"
        )

        send(ser, "STOP")
        send(ser, "MODE CLOSED")
        send(ser, f"RPMMAX {args.rpmmax:.0f}")
        probe_soft_up_down(ser)
        drain(ser, 0.3)

        report = S1Report(port=args.port, soft_up_down=bool(_soft_up_down_ok))
        jobs: list[tuple[str, float, float, float, float]] = []
        if not args.skip_baseline:
            jobs.append(("A1_sym", args.up, args.up, 1000.0, 2000.0))
            jobs.append(("A2_sym", args.up, args.up, 3000.0, 2000.0))
        jobs.append(("A1_asym", args.up, args.down, 1000.0, 2000.0))
        jobs.append(("A2_asym", args.up, args.down, 3000.0, 2000.0))

        if not _soft_up_down_ok:
            print("  → 将只验证闭环轨迹可跑通；不对称对比需烧录后再测")

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = LOG_DIR / f"s1_accel_decel_{ts}.csv"

        with csv_path.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(
                [
                    "name", "up", "down", "from", "to", "t_in_band",
                    "mean_tail", "peak", "pulse_end", "overshoot_pct",
                    "verdict", "note",
                ]
            )
            for name, up, down, f, t in jobs:
                print(f"\n>>> {name}: {f:.0f}→{t:.0f}  UP={up:.0f} DOWN={down:.0f} …")
                res = run_traj(
                    ser,
                    name=name,
                    up=up,
                    down=down,
                    rpm_from=f,
                    rpm_to=t,
                    settle_s=args.settle,
                    rpmmax=args.rpmmax,
                )
                report.results.append(res)
                w.writerow(
                    [
                        res.name,
                        f"{res.up:.0f}",
                        f"{res.down:.0f}",
                        f"{res.rpm_from:.0f}",
                        f"{res.rpm_to:.0f}",
                        f"{res.t_in_band_s:.3f}" if res.t_in_band_s is not None else "",
                        f"{res.mean_tail:.2f}",
                        f"{res.peak:.2f}",
                        res.pulse_end,
                        f"{res.overshoot_pct:.1f}",
                        res.verdict,
                        res.note,
                    ]
                )
                fp.flush()
                print(
                    f"    → {res.verdict}  t_band={res.t_in_band_s}  "
                    f"tail={res.mean_tail:.0f}  {res.note}"
                )
                if res.verdict == "ABORT":
                    break

        safe_stop(ser)
        text = report.summary() + f"\n\ncsv={csv_path}\n"
        rep = csv_path.with_suffix(".report.txt")
        rep.write_text(text, encoding="utf-8")
        print("\n======== S1 报告 ========")
        print(text)
        print(f"报告: {rep}")

        if any(r.verdict == "ABORT" for r in report.results):
            return 3
        if not report.soft_up_down:
            return 6  # 需烧录 S1
        if any(r.verdict.startswith("FAIL") for r in report.results):
            return 4
        return 0
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
