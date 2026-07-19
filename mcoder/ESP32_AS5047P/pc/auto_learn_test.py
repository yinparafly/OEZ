#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无界面自动测试：连接板子 → 自学习 → 写 CSV → 分析油门/转速。
用法:
  python auto_learn_test.py              # 完整自学习
  python auto_learn_test.py --dry        # 只检查 ctrl_hz / 遥测，不给油
  python auto_learn_test.py --rpmmax 6000
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    raise SystemExit("需要 pyserial: pip install pyserial")

from analyze_learn_log import analyze
from com_port_guard import acquire

BAUD = 921600
DEFAULT_PORT = "COM10"
LOG_DIR = Path(__file__).resolve().parent / "logs"


def open_port(port: str) -> serial.Serial:
    ser = serial.Serial(
        port,
        BAUD,
        timeout=0.05,
        write_timeout=1.0,
        dsrdtr=False,
        rtscts=False,
    )
    ser.dtr = False
    ser.rts = False
    time.sleep(0.08)
    ser.reset_input_buffer()
    return ser


def send(ser: serial.Serial, cmd: str) -> None:
    ser.write((cmd.strip() + "\n").encode("utf-8"))


def read_lines(ser: serial.Serial, budget_s: float = 0.2) -> list[str]:
    end = time.time() + budget_s
    buf = getattr(ser, "_linebuf", "")
    lines: list[str] = []
    while time.time() < end:
        chunk = ser.read(1024).decode("utf-8", errors="ignore")
        if not chunk:
            continue
        buf += chunk
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if line:
                lines.append(line)
    ser._linebuf = buf  # type: ignore[attr-defined]
    return lines


def parse_telem(line: str) -> dict | None:
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) < 12:
        return None
    try:
        return {
            "t_ms": float(parts[0]),
            "rpm": float(parts[4]),
            "pulse_us": int(float(parts[9])),
            "target": float(parts[10]),
            "mode": int(float(parts[11])),
            "run": int(float(parts[16])) if len(parts) > 16 else 0,
        }
    except ValueError:
        return None


def wait_banner(ser: serial.Serial, timeout_s: float = 3.0) -> dict:
    info: dict = {}
    t0 = time.time()
    send(ser, "PING")
    while time.time() - t0 < timeout_s:
        for line in read_lines(ser, 0.15):
            if "ctrl_hz=" in line:
                try:
                    info["ctrl_hz"] = int(
                        float(line.split("ctrl_hz=", 1)[1].split()[0].rstrip(","))
                    )
                except ValueError:
                    pass
            if "ACK PING" in line:
                info["ping"] = True
            if line.startswith("#") and "Ready" in line:
                info["ready"] = True
        if info.get("ping") and info.get("ctrl_hz"):
            break
        send(ser, "PING")
    return info


def run_dry(ser: serial.Serial) -> int:
    info = wait_banner(ser)
    print("板子信息:", info)
    ctrl = int(info.get("ctrl_hz") or 0)
    ok = True
    if ctrl < 200:
        print(f"FAIL: ctrl_hz={ctrl}，期望 ≥200（否则 ~3000RPM 会折叠）")
        ok = False
    else:
        print(f"OK: ctrl_hz={ctrl}")
    # 采几帧遥测
    n = 0
    t_end = time.time() + 1.5
    while time.time() < t_end:
        for line in read_lines(ser, 0.1):
            if parse_telem(line):
                n += 1
    print(f"遥测帧: {n}")
    if n < 5:
        print("FAIL: 几乎无遥测")
        ok = False
    return 0 if ok else 2


def run_learn(ser: serial.Serial, rpmmax: float) -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = LOG_DIR / f"learn_auto_{ts}.csv"
    fp = path.open("w", newline="", encoding="utf-8")
    w = csv.writer(fp)
    w.writerow(
        ["wall_s", "t_ms", "kind", "pulse_us", "rpm", "target", "mode", "run", "note"]
    )

    def log_row(**kw):
        w.writerow(
            [
                f"{time.time():.3f}",
                kw.get("t_ms", ""),
                kw.get("kind", ""),
                kw.get("pulse_us", ""),
                kw.get("rpm", ""),
                kw.get("target", ""),
                kw.get("mode", ""),
                kw.get("run", ""),
                (kw.get("note") or "")[:200],
            ]
        )
        fp.flush()

    info = wait_banner(ser)
    print("板子信息:", info)
    ctrl_hz = float(info.get("ctrl_hz") or 250)
    log_row(kind="session_start", note=str(path))

    send(ser, "STOP")
    send(ser, "RPM 0")
    send(ser, f"RPMMAX {rpmmax:.0f}")
    send(ser, "LEARN MAXUS 2000")
    time.sleep(0.2)
    read_lines(ser, 0.3)
    send(ser, "MEASURE AUTO")
    log_row(kind="cmd", note="MEASURE AUTO")
    print("已启动 MEASURE AUTO，等待 LEARN DONE…")

    last_telem_log = 0.0
    done_note = ""
    t0 = time.time()
    # 全台阶约 21*2s ≈ 42s，留余量
    while time.time() - t0 < 120.0:
        send(ser, "PING")  # 保活（学习中固件已忽略超时，但无害）
        for line in read_lines(ser, 0.12):
            if line.startswith("#"):
                if "LEARN" in line or "MEASURE" in line or "SUGGEST" in line or "EVAL" in line:
                    kind = "settle" if "LEARN settle" in line else "meta"
                    log_row(kind=kind, note=line.lstrip("# ").strip())
                    if "ACK LEARN point" in line or "LEARN next" in line:
                        print(" ", line[:100])
                    if "ACK LEARN DONE" in line or "LEARN DONE" in line:
                        done_note = line.lstrip("# ").strip()
                        log_row(kind="done", note=done_note)
                        log_row(kind="session_end")
                        fp.close()
                        print("学习结束:", done_note[:120])
                        return path
                continue
            telem = parse_telem(line)
            if not telem:
                continue
            now = time.time()
            if now - last_telem_log >= 0.1:
                last_telem_log = now
                log_row(kind="telem", **telem)
                if int(telem["mode"]) == 3 and int(telem["pulse_us"]) % 100 == 0:
                    print(
                        f"  telem pulse={telem['pulse_us']} rpm={telem['rpm']:.1f} "
                        f"mode={telem['mode']}"
                    )

    fp.close()
    raise TimeoutError("120s 内未收到 LEARN DONE")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--rpmmax", type=float, default=6000.0)
    ap.add_argument(
        "--dry",
        action="store_true",
        help="只检查连接与 ctrl_hz，不启动学习/给油",
    )
    args = ap.parse_args()

    print(f"打开 {args.port} @ {BAUD} …")
    try:
        acquire("auto_learn_test")
    except SystemExit as exc:
        print(exc)
        return 1
    try:
        ser = open_port(args.port)
    except Exception as exc:  # noqa: BLE001
        print(f"无法打开串口: {exc}")
        print("请先关闭 encoder_monitor 界面后再试。")
        return 1

    try:
        if args.dry:
            return run_dry(ser)

        print("警告: 即将自动给油门做自学习，请确认电机已固定、周围安全。")
        path = run_learn(ser, args.rpmmax)
        # 读 ctrl_hz 再分析
        send(ser, "PING")
        info = wait_banner(ser, timeout_s=1.5)
        ctrl = float(info.get("ctrl_hz") or 250)
        send(ser, "STOP")
        send(ser, "PWM 1000")
        text = analyze(path, ctrl_hz_hint=ctrl)
        print("\n======== 自动分析 ========")
        print(text)
        rep = path.with_suffix(".report.txt")
        rep.write_text(text + "\n", encoding="utf-8")
        print(f"\n报告: {rep}")

        # 简单判定
        if "油门回落" in text and "0 次" in text and "折叠" not in text:
            print("\nVERDICT: PASS（油门单调，未见 Nyquist 折叠特征）")
            return 0
        if "折叠" in text:
            print("\nVERDICT: FAIL（仍像测速折叠，检查 ctrl_hz 是否已 ≥200）")
            return 3
        if "回落" in text and "异常" in text:
            print("\nVERDICT: FAIL（油门有回落）")
            return 4
        print("\nVERDICT: CHECK（请人工看报告）")
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"测试异常: {exc}")
        try:
            send(ser, "ESTOP")
            send(ser, "PWM 1000")
        except Exception:  # noqa: BLE001
            pass
        return 5
    finally:
        try:
            send(ser, "STOP")
            send(ser, "PWM 1000")
        except Exception:  # noqa: BLE001
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
