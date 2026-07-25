#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v26 引导测试：USB 验固件 + BLE 监控；需要拧电调时用 PC 喇叭语音提醒。
例：「已经开始监控，请加油」
"""
from __future__ import annotations

import asyncio
import queue
import sys
import threading
import time
from pathlib import Path

from voice_notify import speak, speak_test


PORT = "COM6"
BAUD = 921600
LOG = Path(__file__).resolve().parent / "serial_logs" / "smoke_ble_v26_last.txt"
MOTOR_WAIT_S = 45.0  # 武装后等用户加油的最长时间


def usb_check_fw(port: str = PORT) -> dict:
    import serial

    out: list[str] = []
    print(f"=== USB open {port} ===", flush=True)
    ser = serial.Serial(
        port, BAUD, timeout=0.15, write_timeout=2.0, dsrdtr=False, rtscts=False
    )
    try:
        try:
            ser.setDTR(False)
            ser.setRTS(True)
            time.sleep(0.05)
            ser.setRTS(False)
        except Exception:
            pass
        time.sleep(1.4)
        ser.reset_input_buffer()

        def read_for(sec: float) -> None:
            t0 = time.time()
            buf = b""
            while time.time() - t0 < sec:
                chunk = ser.read(4096)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode("utf-8", errors="ignore").strip("\r")
                        if not line or line.startswith("L,"):
                            continue
                        out.append(line)
                        print(line, flush=True)
                else:
                    time.sleep(0.01)

        def send(cmd: str) -> None:
            print(f">> {cmd}", flush=True)
            ser.write((cmd + "\n").encode("utf-8"))
            ser.flush()

        read_for(2.0)
        for cmd, w in (("PING", 0.6), ("FW?", 0.8), ("HELP", 1.0), ("ABI?", 0.8)):
            send(cmd)
            read_for(w)
    finally:
        ser.close()

    text = "\n".join(out)
    LOG.parent.mkdir(parents=True, exist_ok=True)
    LOG.write_text(text + "\n", encoding="utf-8")
    fw_ok = ("monitor-v26-ble-ram" in text) or ("DUMP BIN BLE" in text)
    return {
        "fw_v26": fw_ok,
        "help_ble": "DUMP BIN BLE" in text,
        "pong": "PONG" in text,
    }


async def ble_monitor_motor_test() -> dict:
    from ble_link import BleLink, bleak_available, scan_oez_devices_sync

    if not bleak_available():
        speak_test("蓝牙库未安装，无法继续", block=True)
        return {"ok": False, "error": "no bleak"}

    speak_test("开始扫描蓝牙设备", block=True)
    found = scan_oez_devices_sync(6.0)
    print(f"BLE found={found}", flush=True)
    if not found:
        speak("请确认板子已上电", block=True)
        return {"ok": False, "error": "no device"}

    addr, name = found[0]
    speak_test("找到设备，正在连接", block=True)
    out_q: queue.Queue = queue.Queue()
    stop_evt = threading.Event()
    status_msgs: list[str] = []

    def on_status(m: str) -> None:
        status_msgs.append(m)
        print(f"BLE status: {m}", flush=True)

    link = BleLink(addr, out_q, stop_evt, on_status=on_status)
    link.start()

    # 等连接
    t0 = time.time()
    while time.time() - t0 < 20 and not link.is_open:
        await asyncio.sleep(0.2)
    if not link.is_open:
        speak_test("蓝牙连接失败", block=True)
        stop_evt.set()
        link.join(timeout=3)
        return {"ok": False, "error": "connect fail", "status": status_msgs[-3:]}

    speak_test("蓝牙已连接", block=True)
    speak("请准备电调", block=True)
    await asyncio.sleep(2.0)

    link.send("MONITOR START")
    speak("已经开始监控，请加油", block=True)

    flags = {
        "staging": False,
        "confirm": False,
        "snap_done": False,
        "sd_ok": False,
        "ble_pull": False,
        "bin_ok": False,
        "bin_fail": False,
    }
    notes: list[str] = []
    deadline = time.time() + MOTOR_WAIT_S
    reminded = False
    dump_deadline_extra = False

    while time.time() < deadline:
        try:
            while True:
                kind, payload = out_q.get_nowait()
                if kind == "__error__":
                    notes.append(str(payload))
                    print(f"ERR {payload}", flush=True)
                elif kind == "__bindump__":
                    raw = payload
                    print(f"BIN bytes={len(raw)}", flush=True)
                    try:
                        from abi_monitor import parse_snap_bindump

                        n, hz, rows = parse_snap_bindump(raw)
                        flags["bin_ok"] = True
                        notes.append(f"BIN OK n={n} hz={hz} rows={len(rows)}")
                        speak(f"数据已取回，共{n}个点", block=True)
                    except Exception as exc:  # noqa: BLE001
                        flags["bin_fail"] = True
                        notes.append(f"BIN parse fail: {exc}")
                        speak("蓝牙数据校验失败", block=True)
                elif kind == "__raw__":
                    line = str(payload)
                    if line.startswith("L,"):
                        continue
                    print(f"<< {line}", flush=True)
                    notes.append(line)
                    if "STAGING" in line:
                        flags["staging"] = True
                        speak("探测到了", block=True)
                    if "CONFIRM" in line:
                        flags["confirm"] = True
                        speak("开始记录，请保持转动两秒", block=True)
                        if not dump_deadline_extra:
                            dump_deadline_extra = True
                            deadline = max(deadline, time.time() + 180.0)
                    if "SNAP DONE" in line:
                        flags["snap_done"] = True
                        speak("记录完毕", block=True)
                    if "SD SAVE OK" in line:
                        flags["sd_ok"] = True
                        speak("已存入存储卡", block=True)
                        speak_test("正在蓝牙取回", block=True)
                        deadline = max(deadline, time.time() + 150.0)
                    if "BLE PULL READY" in line:
                        flags["ble_pull"] = True
                        link.send("DUMP BIN BLE")
                    if "BIN BLE BEGIN" in line:
                        speak_test("正在蓝牙传输，请稍等", block=True)
                        deadline = max(deadline, time.time() + 150.0)
        except queue.Empty:
            pass

        if flags["bin_ok"] or flags["bin_fail"]:
            break
        if (not reminded) and (not flags["staging"]) and (
            time.time() > (deadline - MOTOR_WAIT_S + MOTOR_WAIT_S / 2)
            if not dump_deadline_extra
            else False
        ):
            reminded = True
            speak("请加油门", block=True)
        await asyncio.sleep(0.05)

    if not flags["staging"] and not flags["confirm"]:
        speak("超时未检测到电机转动", block=True)
        speak_test("测试结束", block=True)
    elif flags["bin_ok"]:
        speak_test("测试成功", block=True)
    elif flags["sd_ok"] and not flags["bin_ok"]:
        speak("已存卡，但蓝牙取回未完成", block=True)
    else:
        speak_test("监控流程未完整完成", block=True)

    link.send("MONITOR STOP")
    await asyncio.sleep(0.5)
    stop_evt.set()
    link.join(timeout=5)

    ok = bool(flags["confirm"] and (flags["sd_ok"] or flags["bin_ok"]))
    return {"ok": ok, "flags": flags, "notes_tail": notes[-12:], "status": status_msgs[-5:]}


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser(description="v26 USB+BLE smoke")
    ap.add_argument("--port", default=PORT, help="USB COM port")
    ap.add_argument(
        "--skip-motor",
        action="store_true",
        help="只验 USB 固件，不连 BLE、不转电机",
    )
    args = ap.parse_args()

    speak_test("开始自动测试", block=True)
    print("======== USB ========", flush=True)
    try:
        usb = usb_check_fw(args.port)
    except Exception as exc:  # noqa: BLE001
        speak_test("串口检查失败", block=True)
        print(f"USB FAIL: {exc}", flush=True)
        return 2
    print("USB:", usb, flush=True)
    if not usb.get("fw_v26"):
        speak("请重新烧录固件", block=True)
        return 3
    speak_test("固件版本正确", block=True)

    if args.skip_motor:
        speak_test("跳过电机与蓝牙，仅 USB 通过", block=True)
        print("skip-motor: USB OK", flush=True)
        return 0

    time.sleep(0.6)
    print("======== BLE + 电机 ========", flush=True)
    try:
        ble = asyncio.run(ble_monitor_motor_test())
    except Exception as exc:  # noqa: BLE001
        speak_test("蓝牙测试异常", block=True)
        print(f"BLE FAIL: {exc}", flush=True)
        return 4
    print("BLE:", ble, flush=True)

    if ble.get("ok"):
        speak_test("全部测试通过", block=True)
        return 0
    speak("请查看屏幕日志", block=True)
    speak_test("测试未通过", block=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
