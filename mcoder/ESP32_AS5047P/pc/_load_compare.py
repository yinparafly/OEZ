# -*- coding: utf-8 -*-
"""负载对比骨架：静态读 LOAD?/ENC?/CTRL?/TELEM?，不转电机。

对比维度（硬件就绪后由操作者切换工况，本脚本只采快照）：
  - 浮点主路径 vs USE_FIXED_POINT=1（需分别烧录两版固件）
  - USB 遥测 ON@20Hz vs TELEM OFF
  - BLE ON vs BLE OFF
  - 可选：仅采集（SAFE/停转）vs 采集+输出控制（CLOSED，需保活；默认不做）

默认：只查询、不 START/RPM、不 flash。
  python _load_compare.py              # 探测 COM，读一轮
  python _load_compare.py --dwell 8    # 每工况 dwell 秒后 LOAD?
  python _load_compare.py --telem-ab   # 同固件下 TELEM ON/OFF 对比
  python _load_compare.py --ble-ab     # BLE ON/OFF 对比
  python _load_compare.py --dry-run    # 不连串口，打印命令计划

环境：COM 口空闲；勿与 encoder_monitor 抢口。
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path

os.environ.setdefault("PYTHONIOENCODING", "utf-8")

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

LOAD_RE = re.compile(
    r"enc_busy_avg_us=([\d.]+).*?enc_busy_max_us=(\d+).*?enc_cpu%=([\d.]+).*?"
    r"meas_hz=([\d.]+).*?ctrl_expect_us=(\d+).*?ctrl_period_avg_us=([\d.]+).*?"
    r"ctrl_period_max_us=(\d+).*?ctrl_busy_avg_us=([\d.]+).*?ctrl_busy_max_us=(\d+).*?"
    r"ctrl_overrun=(\d+).*?meas_out_hz=([\d.]+).*?loop_hz=([\d.]+).*?"
    r"async_log_drop=(\d+).*?fixed=(\d+).*?usb_telem=(\w+).*?ble=(\d+)",
    re.I,
)


def _pick_port(cli: str | None) -> str:
    if cli:
        return cli
    env = os.environ.get("OEZ_COM") or os.environ.get("COM_PORT")
    if env:
        return env
    return "COM10"


def parse_load(line: str) -> dict | None:
    m = LOAD_RE.search(line)
    if not m:
        return None
    keys = (
        "enc_busy_avg_us",
        "enc_busy_max_us",
        "enc_cpu_pct",
        "meas_hz",
        "ctrl_expect_us",
        "ctrl_period_avg_us",
        "ctrl_period_max_us",
        "ctrl_busy_avg_us",
        "ctrl_busy_max_us",
        "ctrl_overrun",
        "meas_out_hz",
        "loop_hz",
        "async_log_drop",
        "fixed",
        "usb_telem",
        "ble",
    )
    vals = list(m.groups())
    out: dict = {}
    for k, v in zip(keys, vals):
        if k in ("usb_telem",):
            out[k] = v
        elif k in (
            "enc_busy_max_us",
            "ctrl_expect_us",
            "ctrl_period_max_us",
            "ctrl_busy_max_us",
            "ctrl_overrun",
            "async_log_drop",
            "fixed",
            "ble",
        ):
            out[k] = int(v)
        else:
            out[k] = float(v)
    return out


def fmt_row(tag: str, d: dict | None, raw: str = "") -> str:
    if not d:
        return f"{tag}: (parse fail) {raw[:120]}"
    return (
        f"{tag}: enc_avg={d['enc_busy_avg_us']:.1f}us max={d['enc_busy_max_us']}us "
        f"cpu%={d['enc_cpu_pct']:.2f} meas_hz={d['meas_hz']:.1f} | "
        f"ctrl_period avg/max={d['ctrl_period_avg_us']:.1f}/{d['ctrl_period_max_us']}us "
        f"busy avg/max={d['ctrl_busy_avg_us']:.1f}/{d['ctrl_busy_max_us']}us "
        f"overrun={d['ctrl_overrun']} out_hz={d['meas_out_hz']:.1f} | "
        f"loop_hz={d['loop_hz']:.0f} drop={d['async_log_drop']} "
        f"fixed={d['fixed']} telem={d['usb_telem']} ble={d['ble']}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="ESP32 load snapshot compare (no motor spin)")
    ap.add_argument("--port", default=None, help="serial port (default COM10 / OEZ_COM)")
    ap.add_argument("--dwell", type=float, default=5.0, help="seconds to settle before LOAD?")
    ap.add_argument("--telem-ab", action="store_true", help="compare TELEM ON vs OFF")
    ap.add_argument("--ble-ab", action="store_true", help="compare BLE ON vs OFF")
    ap.add_argument("--dry-run", action="store_true", help="print plan only, no serial")
    ap.add_argument("--out", default="", help="optional markdown summary path")
    args = ap.parse_args()

    plan = ["PING", "LOAD RESET", "ENC?", "CTRL?", "TELEM?", "CORE?", "FIXED?", "LOAD?"]
    if args.telem_ab:
        plan += ["TELEM OFF", f"(dwell {args.dwell}s)", "LOAD?", "TELEM ON", f"(dwell {args.dwell}s)", "LOAD?"]
    if args.ble_ab:
        plan += ["BLE OFF", f"(dwell {args.dwell}s)", "LOAD?", "BLE ON", f"(dwell {args.dwell}s)", "LOAD?"]

    print("=== load compare plan (NO START/RPM; static queries only) ===", flush=True)
    for p in plan:
        print(f"  - {p}", flush=True)

    if args.dry_run:
        print("dry-run done.", flush=True)
        return 0

    from auto_learn_test import open_port, send, read_lines  # noqa: E402
    from com_port_guard import acquire  # noqa: E402

    port = _pick_port(args.port)
    acquire("load_compare")
    ser = open_port(port)

    def drain(sec: float = 0.4):
        return [ln for ln in read_lines(ser, sec)]

    def cmd(c: str, wait: float = 0.45) -> list[str]:
        send(ser, c)
        time.sleep(wait)
        lines = [ln for ln in drain(wait) if ln.startswith("#")]
        for ln in lines:
            print(f"[{c}] {ln}", flush=True)
        return lines

    def snap(tag: str) -> dict | None:
        lines = cmd("LOAD?", 0.5)
        for ln in lines:
            d = parse_load(ln)
            if d:
                print(fmt_row(tag, d, ln), flush=True)
                return d
        print(fmt_row(tag, None, lines[-1] if lines else ""), flush=True)
        return None

    rows: list[tuple[str, dict | None]] = []
    try:
        time.sleep(0.5)
        for ln in drain(0.8):
            if ln.startswith("#"):
                print(ln, flush=True)

        cmd("PING")
        cmd("LOAD RESET", 0.3)
        for c in ("ENC?", "CTRL?", "TELEM?", "CORE?", "FIXED?"):
            cmd(c)

        time.sleep(max(1.0, args.dwell * 0.5))
        rows.append(("baseline", snap("baseline")))

        if args.telem_ab:
            cmd("TELEM OFF", 0.3)
            cmd("LOAD RESET", 0.2)
            time.sleep(args.dwell)
            rows.append(("telem_off", snap("telem_off")))
            cmd("TELEM ON", 0.3)
            cmd("LOAD RESET", 0.2)
            time.sleep(args.dwell)
            rows.append(("telem_on", snap("telem_on")))

        if args.ble_ab:
            cmd("BLE OFF", 0.3)
            cmd("LOAD RESET", 0.2)
            time.sleep(args.dwell)
            rows.append(("ble_off", snap("ble_off")))
            cmd("BLE ON", 0.3)
            cmd("LOAD RESET", 0.2)
            time.sleep(args.dwell)
            rows.append(("ble_on", snap("ble_on")))

        print("--- summary ---", flush=True)
        for tag, d in rows:
            print(fmt_row(tag, d), flush=True)

        if args.out:
            outp = Path(args.out)
            lines = [
                "# LOAD compare snapshot",
                "",
                f"port={port} dwell={args.dwell}",
                "",
                "| tag | enc_avg_us | enc_max_us | cpu% | ctrl_period_avg | ctrl_period_max | ctrl_busy_avg | overrun | loop_hz | drop |",
                "|-----|------------|------------|------|-----------------|-----------------|---------------|---------|---------|------|",
            ]
            for tag, d in rows:
                if not d:
                    lines.append(f"| {tag} | - | - | - | - | - | - | - | - | - |")
                    continue
                lines.append(
                    f"| {tag} | {d['enc_busy_avg_us']:.1f} | {d['enc_busy_max_us']} | "
                    f"{d['enc_cpu_pct']:.2f} | {d['ctrl_period_avg_us']:.1f} | {d['ctrl_period_max_us']} | "
                    f"{d['ctrl_busy_avg_us']:.1f} | {d['ctrl_overrun']} | {d['loop_hz']:.0f} | "
                    f"{d['async_log_drop']} |"
                )
            outp.write_text("\n".join(lines) + "\n", encoding="utf-8")
            print(f"wrote {outp}", flush=True)
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
