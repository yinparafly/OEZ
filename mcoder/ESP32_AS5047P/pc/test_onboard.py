# -*- coding: utf-8 -*-
"""板内 TEST：主机只发 START/STOP，等 # TEST DONE|OK|FAIL。

判定（在转/段通过）全在 ESP32 固件；本脚本几乎不发别的。

用法:
  python test_onboard.py                         # 默认 smoke2000 @ COM10
  python test_onboard.py --name step7500
  python test_onboard.py --name idle --timeout 15
  python test_onboard.py --port COM10 --name smoke2000
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("PYTHONIOENCODING", "utf-8")

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from auto_learn_test import open_port, read_lines, send  # noqa: E402
from com_port_guard import acquire, release  # noqa: E402

DEFAULT_PORT = os.environ.get("OEZ_COM", "COM10")
DONE_RE = re.compile(r"#\s*TEST\s+(DONE|OK|FAIL)\b", re.I)


def drain(ser, budget: float = 0.25) -> list[str]:
    return read_lines(ser, budget)


def main() -> int:
    ap = argparse.ArgumentParser(description="板内 TEST 最小通讯客户端")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--name", default="smoke2000", choices=("smoke2000", "step7500", "idle"))
    ap.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="秒；0=按段自动（smoke≈45s，step≈90s，idle≈15s）",
    )
    args = ap.parse_args()

    if args.timeout > 0:
        timeout = args.timeout
    elif args.name == "step7500":
        timeout = 90.0
    elif args.name == "idle":
        timeout = 15.0
    else:
        timeout = 50.0

    acquire(f"test_onboard:{args.name}")
    ser = open_port(args.port)
    result_line = None
    ok = None
    try:
        print(
            f"=== test_onboard {datetime.now().isoformat(timespec='seconds')} "
            f"port={args.port} name={args.name} timeout={timeout:.0f}s ===",
            flush=True,
        )
        drain(ser, 0.4)
        send(ser, "TEST?")
        for ln in drain(ser, 0.5):
            if ln.startswith("#"):
                print(f"  {ln}", flush=True)

        cmd = f"TEST START {args.name}"
        print(f"CMD {cmd}", flush=True)
        send(ser, cmd)

        t0 = time.time()
        while time.time() - t0 < timeout:
            for ln in drain(ser, 0.2):
                if not ln.startswith("#"):
                    continue
                # 状态切换行也打印，便于台架盯
                if "TEST" in ln.upper():
                    print(f"  {ln}", flush=True)
                m = DONE_RE.search(ln)
                if m:
                    result_line = ln
                    # DONE ok=1 / OK / FAIL
                    if re.search(r"\bok=1\b", ln) or m.group(1).upper() == "OK":
                        if "FAIL" not in ln.upper() or re.search(r"\bok=1\b", ln):
                            ok = True
                    if re.search(r"\bok=0\b", ln) or m.group(1).upper() == "FAIL":
                        ok = False
                    if m.group(1).upper() == "DONE":
                        om = re.search(r"\bok=(\d)\b", ln)
                        if om:
                            ok = om.group(1) == "1"
                    # 收到 DONE 即结束（OK/FAIL 可能随后一行）
                    if m.group(1).upper() == "DONE":
                        # 再吸半秒拿 OK/FAIL 行
                        for ln2 in drain(ser, 0.5):
                            if ln2.startswith("#") and "TEST" in ln2.upper():
                                print(f"  {ln2}", flush=True)
                        break
            if result_line and ("DONE" in result_line.upper() or ok is not None):
                if "DONE" in (result_line or "").upper() or ok is not None:
                    # 若只收到 OK/FAIL 再等等 DONE；若已有 DONE 退出
                    if "DONE" in result_line.upper():
                        break
            time.sleep(0.05)
        else:
            print("TIMEOUT — sending TEST STOP", flush=True)
            send(ser, "TEST STOP")
            for ln in drain(ser, 1.5):
                if ln.startswith("#") and "TEST" in ln.upper():
                    print(f"  {ln}", flush=True)
                    if DONE_RE.search(ln):
                        result_line = ln
                        om = re.search(r"\bok=(\d)\b", ln)
                        if om:
                            ok = om.group(1) == "1"
            if ok is None:
                ok = False

        print("---", flush=True)
        print(f"result: {result_line or '(none)'}", flush=True)
        print(f"verdict: {'PASS' if ok else 'FAIL'}", flush=True)
        return 0 if ok else 2
    except Exception as exc:
        print(f"ERROR: {exc}", flush=True)
        try:
            send(ser, "TEST STOP")
            send(ser, "ESTOP")
        except Exception:
            pass
        return 1
    finally:
        try:
            ser.close()
        except Exception:
            pass
        release()


if __name__ == "__main__":
    sys.exit(main())
