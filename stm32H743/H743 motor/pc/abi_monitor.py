#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AS5047P ABI 转速监控 — PC 端（独立工程）

「开始监控」→ 板内记录 |RPM|>20
「停止监控」→ 停止记录（仍可看实时转速）
支持 USB / BLE，拉取板内 LOG 回放与保存 CSV。
记录完成后板子自动推送：USB 串口与已连接的蓝牙端（PC/手机）各收一份。
默认优先蓝牙（需 bleak）；可用 --usb 强制串口。
"""

from __future__ import annotations

import csv
import os
import queue
import struct
import threading
import time
import tkinter as tk
import zlib
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise SystemExit("请先: pip install -r requirements.txt")

try:
    from ble_link import BleLink, bleak_available, scan_oez_devices_sync
except ImportError:
    BleLink = None  # type: ignore

    def bleak_available() -> bool:
        return False

    def scan_oez_devices_sync(timeout_s: float = 5.0):
        return []


try:
    from voice_notify import speak as voice_speak
except ImportError:

    def voice_speak(text: str, **kwargs) -> None:  # type: ignore[misc]
        print(f"[语音] {text}", flush=True)

    def scan_oez_devices_sync(_timeout_s: float = 5.0):
        return []


BAUD = 921600
# 默认优先蓝牙（有 bleak 时）；可用 --usb 强制串口；--sim 进程内假板
_ARGV = __import__("sys").argv
PREFER_USB = "--usb" in _ARGV
PREFER_SIM = "--sim" in _ARGV
PREFER_BLE = ("--ble" in _ARGV) or (not PREFER_USB and not PREFER_SIM)
LIVE_WINDOW_S = 15.0
UI_HZ = 10.0
UI_PERIOD_MS = int(1000 / UI_HZ)  # 100ms → 10Hz UI 刷新

try:
    from abi_sim import SimLink
except ImportError:
    SimLink = None  # type: ignore

try:
    from snap_edit import (
        delete_time_range,
        distance_series,
        filter_by_rpm,
        index_baseline,
        integrate_rpm_distance,
        keep_time_range,
        split_at_time,
    )
except ImportError:
    delete_time_range = None  # type: ignore


@dataclass
class SegmentRecord:
    """一次接收到的数据段（可浏览回放）。"""
    label: str
    board_seg: int
    samples: list  # (t,rpm,dir,seg,index_n,t_rel,unix)
    index_ev: list = field(default_factory=list)
    recv_at: float = field(default_factory=time.time)

    @property
    def duration_s(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return (self.samples[-1][0] - self.samples[0][0]) / 1000.0



SNAP_MAGIC_V1 = 0xAB1C0001  # t_us,rpm_x10,dir,pad,index_n (12B) — 旧
SNAP_MAGIC_V2 = 0xAB1C0002  # t_us,counts,index_n (16B) — 转速由上位机算
SNAP_MAGIC = SNAP_MAGIC_V2
SNAP_POINT_SIZE_V1 = 12
SNAP_POINT_SIZE_V2 = 16
SNAP_POINT_SIZE = SNAP_POINT_SIZE_V2
SNAP_PREAMBLE = bytes([0xAA] * 10 + [0x55])
ABI_STEPS_PER_REV = 1000
# 上位机默认差分窗（与 v39 板端接近；可在曲线工作室再平滑）
HOST_VEL_WIN = 32
_SNAP_FMT_V1 = "<IhbBI"
_SNAP_FMT_V2 = "<IqI"  # t_us u32, counts i64, index_n u32
assert struct.calcsize(_SNAP_FMT_V1) == SNAP_POINT_SIZE_V1
assert struct.calcsize(_SNAP_FMT_V2) == SNAP_POINT_SIZE_V2


def app_dir() -> Path:
    """源码运行 → pc/；打包 exe → exe 所在目录（logs/serial_logs 写在旁边）。"""
    import sys

    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


SERIAL_LOG_DIR = app_dir() / "serial_logs"


def help_dir() -> Path:
    """源码 → pc/help；打包 exe → _MEIPASS/help。"""
    import sys

    if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
        return Path(sys._MEIPASS) / "help"
    return Path(__file__).resolve().parent / "help"


def help_text(name: str = "转速计算方法.md") -> str:
    p = help_dir() / name
    if not p.is_file():
        return (
            f"（未找到帮助文件：{p}）\n\n"
            "v40：BIN 存 counts；rpm = (Δc/4000)*(1e6/Δt)*60，默认窗 32 点≈16ms。"
        )
    return p.read_text(encoding="utf-8")


def snap_crc32(data: bytes) -> int:
    """与固件 snapCrc32 一致（IEEE CRC32 / zlib）。"""
    return zlib.crc32(data) & 0xFFFFFFFF


def rpm_from_counts_series(
    t_us: list[int],
    counts: list[int],
    *,
    steps: int = ABI_STEPS_PER_REV,
    vel_win: int = HOST_VEL_WIN,
) -> list[float]:
    """由 counts/时间序列算 RPM（多拍差分，无 EMA；后续平滑交给曲线工作室）。"""
    n = len(counts)
    out = [0.0] * n
    if n == 0 or steps <= 0:
        return out
    w = max(1, int(vel_win))
    for i in range(n):
        j = i - w if i >= w else 0
        if i == j:
            out[i] = 0.0
            continue
        dc = counts[i] - counts[j]
        dt = t_us[i] - t_us[j]
        if dt <= 0:
            out[i] = out[i - 1] if i else 0.0
            continue
        out[i] = (dc / steps) * (1_000_000.0 / dt) * 60.0
    return out


def row_has_counts(row: tuple) -> bool:
    """v2 行：(t_ms,rpm,dir,seg,index_n,t_rel,unix,counts)。"""
    return len(row) > 7 and row[7] is not None


def row_counts(row: tuple) -> int | None:
    if not row_has_counts(row):
        return None
    return int(row[7])


def recompute_rows_rpm_from_counts(
    rows: list[tuple],
    *,
    steps: int = ABI_STEPS_PER_REV,
    vel_win: int = HOST_VEL_WIN,
) -> list[tuple]:
    """根据行内 counts 与 t_ms 重算 rpm/dir；无 counts 则原样返回。"""
    if not rows or not row_has_counts(rows[0]):
        return list(rows)
    t_us = [int(round(float(r[0]) * 1000.0)) for r in rows]
    c_list = [int(r[7]) for r in rows]
    rpms = rpm_from_counts_series(t_us, c_list, steps=steps, vel_win=vel_win)
    out: list[tuple] = []
    for r, rpm in zip(rows, rpms):
        direc = 1 if rpm > 0.5 else (-1 if rpm < -0.5 else 0)
        lst = list(r)
        lst[1] = float(rpm)
        if len(lst) > 2:
            lst[2] = direc
        out.append(tuple(lst))
    return out


def _points_to_rows_v1(payload: bytes, n: int) -> list[tuple]:
    rows: list[tuple] = []
    for i in range(n):
        t_us, rpm_x10, direc, _pad, index_n = struct.unpack_from(
            _SNAP_FMT_V1, payload, i * SNAP_POINT_SIZE_V1
        )
        rows.append((t_us / 1000.0, rpm_x10 / 10.0, int(direc), 1, int(index_n), 0, 0))
    return rows


def _points_to_rows_v2(
    payload: bytes,
    n: int,
    *,
    steps: int = ABI_STEPS_PER_REV,
    vel_win: int = HOST_VEL_WIN,
) -> list[tuple]:
    t_list: list[int] = []
    c_list: list[int] = []
    idx_list: list[int] = []
    for i in range(n):
        t_us, counts, index_n = struct.unpack_from(
            _SNAP_FMT_V2, payload, i * SNAP_POINT_SIZE_V2
        )
        t_list.append(int(t_us))
        c_list.append(int(counts))
        idx_list.append(int(index_n))
    rpms = rpm_from_counts_series(t_list, c_list, steps=steps, vel_win=vel_win)
    rows: list[tuple] = []
    for i in range(n):
        rpm = rpms[i]
        direc = 1 if rpm > 0.5 else (-1 if rpm < -0.5 else 0)
        rows.append((t_list[i] / 1000.0, rpm, direc, 1, idx_list[i], 0, 0, c_list[i]))
    return rows


def parse_snap_bindump(raw: bytes) -> tuple[int, int, list[tuple]]:
    """解析 magic+n+hz+payload+crc（raw 已去掉 preamble）。支持 v1/v2。"""
    if len(raw) < 12:
        raise ValueError(f"bin too short: {len(raw)}")
    magic, n, hz = struct.unpack_from("<IHH", raw, 0)
    if magic == SNAP_MAGIC_V2:
        pt = SNAP_POINT_SIZE_V2
        need = 8 + n * pt + 4
        if len(raw) < need:
            raise ValueError(f"bin len {len(raw)} < need {need}")
        payload = raw[8 : 8 + n * pt]
        (crc,) = struct.unpack_from("<I", raw, 8 + n * pt)
        got = snap_crc32(payload)
        if got != crc:
            raise ValueError(f"CRC mismatch got=0x{got:08X} expect=0x{crc:08X}")
        return n, hz, _points_to_rows_v2(payload, n)
    if magic == SNAP_MAGIC_V1:
        pt = SNAP_POINT_SIZE_V1
        need = 8 + n * pt + 4
        if len(raw) < need:
            raise ValueError(f"bin len {len(raw)} < need {need}")
        payload = raw[8 : 8 + n * pt]
        (crc,) = struct.unpack_from("<I", raw, 8 + n * pt)
        got = snap_crc32(payload)
        if got != crc:
            raise ValueError(f"CRC mismatch got=0x{got:08X} expect=0x{crc:08X}")
        return n, hz, _points_to_rows_v1(payload, n)
    raise ValueError(f"bad magic 0x{magic:08X}")


def parse_snap_file(raw: bytes) -> tuple[int, int, list[tuple]]:
    """
    打开 SD 上的 /snap_*.bin（裸点阵），或 USB DUMP BIN（含 magic/CRC）。
    返回 (n, hz, rows)。v2 裸文件按 16B；旧 12B 仍可按 v1 解。
    """
    if not raw:
        raise ValueError("empty file")
    if raw.startswith(SNAP_PREAMBLE):
        raw = raw[len(SNAP_PREAMBLE) :]
    if len(raw) >= 12:
        magic = struct.unpack_from("<I", raw, 0)[0]
        if magic in (SNAP_MAGIC_V1, SNAP_MAGIC_V2):
            return parse_snap_bindump(raw)
    if len(raw) % SNAP_POINT_SIZE_V2 == 0 and len(raw) >= SNAP_POINT_SIZE_V2:
        n = len(raw) // SNAP_POINT_SIZE_V2
        return n, 2000, _points_to_rows_v2(raw, n)
    if len(raw) % SNAP_POINT_SIZE_V1 == 0 and len(raw) >= SNAP_POINT_SIZE_V1:
        n = len(raw) // SNAP_POINT_SIZE_V1
        return n, 2000, _points_to_rows_v1(raw, n)
    raise ValueError(f"raw snap size {len(raw)} not multiple of 12 or 16")


def filter_snap_rows(
    rows: list[tuple],
    *,
    t0_s: float | None = None,
    t1_s: float | None = None,
    rpm_min: float = 0.0,
    drop_zero: bool = False,
) -> list[tuple]:
    """按相对段起点时间窗 + |RPM| 下限过滤。t 单位：行内为 ms。"""
    if not rows:
        return []
    base = float(rows[0][0])
    out: list[tuple] = []
    for r in rows:
        t_ms = float(r[0])
        rpm = float(r[1])
        rel_s = (t_ms - base) / 1000.0
        if t0_s is not None and rel_s < t0_s:
            continue
        if t1_s is not None and rel_s > t1_s:
            continue
        if drop_zero and abs(rpm) < 1e-6:
            continue
        if abs(rpm) < rpm_min:
            continue
        out.append(r)
    return out


class SerialLink(threading.Thread):
    def __init__(self, port: str, out_q: queue.Queue, stop_evt: threading.Event):
        super().__init__(daemon=True)
        self.port = port
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.ser: serial.Serial | None = None
        self._tx: queue.Queue[str] = queue.Queue()
        self._last_l_emit = 0.0
        self._pending_l: str | None = None
        self._bin_hunt = False
        self._bin_deadline = 0.0
        self._bin_got_preamble = False

    def send(self, cmd: str) -> None:
        line = cmd.strip()
        if line:
            self._tx.put(line + "\n")

    def arm_bindump(self, timeout_s: float = 12.0) -> None:
        """找 AA×10+55 前导，再按 magic 头里的 n 收满（定长）。"""
        self._bin_hunt = True
        self._bin_got_preamble = False
        self._bin_deadline = time.monotonic() + timeout_s

    def cancel_bindump(self) -> None:
        self._bin_hunt = False
        self._bin_got_preamble = False

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _emit_line(self, line: str) -> None:
        if line.startswith("L,"):
            now = time.monotonic()
            if now - self._last_l_emit < 0.1:
                self._pending_l = line
                return
            self._last_l_emit = now
            self._pending_l = None
        self.out_q.put(("__raw__", line))

    def run(self) -> None:
        magic_le = struct.pack("<I", SNAP_MAGIC)
        try:
            self.ser = serial.Serial(
                self.port,
                BAUD,
                timeout=0.05,
                write_timeout=5.0,
                dsrdtr=False,
                rtscts=False,
            )
            buf = bytearray()
            while not self.stop_evt.is_set():
                try:
                    while True:
                        self.ser.write(self._tx.get_nowait().encode("utf-8"))
                except queue.Empty:
                    pass
                chunk = self.ser.read(4096)
                if chunk:
                    buf.extend(chunk)

                if self._bin_hunt:
                    if time.monotonic() > self._bin_deadline:
                        self._bin_hunt = False
                        self._bin_got_preamble = False
                        self.out_q.put(("__error__", "BIN hunt timeout (no preamble/magic)"))
                        continue
                    if not self._bin_got_preamble:
                        pre_i = buf.find(SNAP_PREAMBLE)
                        if pre_i < 0:
                            if len(buf) > len(SNAP_PREAMBLE):
                                del buf[: -(len(SNAP_PREAMBLE) - 1)]
                            elif not chunk:
                                time.sleep(0.001)
                            continue
                        if pre_i > 0:
                            del buf[:pre_i]
                        del buf[: len(SNAP_PREAMBLE)]
                        self._bin_got_preamble = True
                    if len(buf) < 8:
                        if not chunk:
                            time.sleep(0.001)
                        continue
                    if buf[0:4] != magic_le:
                        # 假同步：丢掉 1 字节，重新找前导
                        del buf[0]
                        self._bin_got_preamble = False
                        continue
                    _magic, n, _hz = struct.unpack_from("<IHH", buf, 0)
                    if n > 6000:
                        del buf[0]
                        self._bin_got_preamble = False
                        continue
                    # v2=16B；若收到旧 v1 magic 仍按 12B（极少走 USB 串口）
                    pt = SNAP_POINT_SIZE_V2 if _magic == SNAP_MAGIC_V2 else SNAP_POINT_SIZE_V1
                    need = 8 + n * pt + 4
                    if len(buf) < need:
                        if not chunk:
                            time.sleep(0.001)
                        continue
                    raw = bytes(buf[:need])
                    del buf[:need]
                    self._bin_hunt = False
                    self._bin_got_preamble = False
                    self.out_q.put(("__bindump__", raw))
                    continue

                while True:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    line_b = bytes(buf[:nl])
                    del buf[: nl + 1]
                    line = line_b.decode("utf-8", errors="ignore").strip("\r")
                    if line:
                        self._emit_line(line)
                if len(buf) > 65536:
                    buf.clear()
                if not chunk:
                    now = time.monotonic()
                    if self._pending_l is not None and (now - self._last_l_emit) >= 0.1:
                        self.out_q.put(("__raw__", self._pending_l))
                        self._pending_l = None
                        self._last_l_emit = now
                    time.sleep(0.002)
        except Exception as exc:  # noqa: BLE001
            self.out_q.put(("__error__", str(exc)))
        finally:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:  # noqa: BLE001
                pass


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("AS5047P ABI 转速监控")
        self.geometry("1100x780")
        self.minsize(900, 640)

        self.out_q: queue.Queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.link: SerialLink | BleLink | None = None
        self.live_abs: deque[tuple[float, float]] = deque()
        self.dump_rows: list[tuple[float, float, int, int, int, int, int]] = []  # t,rpm,dir,seg,index_n,t_rel,unix
        self.dump_index: list[tuple] = []  # t,index_n,rpm,rpm_i,dt,seg,t_rel,unix
        self.playback: list[tuple[float, float, int, int, int, int, int]] = []
        self.archive: list[SegmentRecord] = []
        self.cap_rows: list[tuple] = []  # t_ms,rpm,dcounts,dindex
        self._cap_expect = 0
        self._cap_end_mono: float | None = None
        self._cap_total_s = 0
        self._cap_board_n = 0
        self._cap_dumping = False
        self._board_auto_push = False
        self._read_scheduled = False
        self._auto_save_on_push = True
        self.view_seg: SegmentRecord | None = None
        self.plot_mode = "live"  # live | segment
        self.var_t0 = tk.StringVar(value="0.0")
        self.var_t1 = tk.StringVar(value="0.3")
        self.play_i = 0
        self.play_job: str | None = None
        self._ble_addr: dict[str, str] = {}
        self._auto_split_after_dump = False
        self._serial_log_fp = None
        self._serial_log_path: Path | None = None
        self._open_serial_log()

        prefer_ble = PREFER_BLE and bleak_available() and not PREFER_USB and not PREFER_SIM
        if PREFER_SIM:
            self.var_link = tk.StringVar(value="仿真SIM")
        else:
            self.var_link = tk.StringVar(value="蓝牙BLE" if prefer_ble else "USB串口")
        self.var_port = tk.StringVar(value="COM6")
        self.var_ble = tk.StringVar(value="")
        self.var_status = tk.StringVar(
            value="未连接"
            + (
                " · 仿真模式"
                if PREFER_SIM
                else (
                    " · 蓝牙就绪"
                    if bleak_available()
                    else " · 未装 bleak（pip install bleak）"
                )
            )
        )
        self._auto_save_on_push = True
        self.var_rpm = tk.StringVar(value="—")
        self.var_dir = tk.StringVar(value="—")
        self.var_revs = tk.StringVar(value="I圈数 — · A/B圈数 —")
        self.var_mon = tk.StringVar(value="监控 OFF")
        self._mon_state = "idle"  # idle|armed|staging|recording|dumping|done
        self.var_log = tk.StringVar(value="log_n=0 segs=0")
        self.var_hz = tk.StringVar(value="meas_hz=—")
        self.var_info = tk.StringVar(value="")
        self.var_dump_prog = tk.StringVar(value="")
        self.var_rec_s = tk.DoubleVar(value=2.0)
        self.var_rpm_min = tk.StringVar(value="10")
        self.var_drop_zero = tk.BooleanVar(value=False)
        self.var_i0 = tk.StringVar(value="auto")
        self.var_len_i = tk.StringVar(value="1.0")  # 每 I 脉冲对应长度（单位自定，如 mm）
        self.var_plot_y = tk.StringVar(value="rpm")  # rpm | s_i | s_rpm
        self.var_edit_info = tk.StringVar(value="剪辑：先打开 bin，用 t0/t1 裁剪或删除")
        self._snap_raw_rows: list = []  # 打开 bin 的完整点，过滤前
        self._dump_expect = 0
        self._dump_got = 0
        self._last_live_draw = 0.0  # 曲线更慢
        self._last_ui_l = 0.0  # 转速数字 ≤10Hz
        self._last_ble_status = 0.0
        self._ui_dumping = False
        self._log_lines = 0
        self._read_scheduled = False
        self._hb = 0
        self.var_hb = tk.StringVar(value="UI 10Hz")
        self._build()
        self.after(UI_PERIOD_MS, self._poll)
        self.after(200, self._refresh_ports)
        self.after(UI_PERIOD_MS, self._heartbeat)

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text="连接方式").pack(side=tk.LEFT)
        ttk.Combobox(
            top,
            textvariable=self.var_link,
            values=("USB串口", "蓝牙BLE", "仿真SIM"),
            width=10,
            state="readonly",
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT, padx=2)

        self.frm_usb = ttk.Frame(top)
        self.frm_usb.pack(side=tk.LEFT, padx=6)
        ttk.Label(self.frm_usb, text="COM").pack(side=tk.LEFT)
        self.cmb_port = ttk.Combobox(self.frm_usb, textvariable=self.var_port, width=12)
        self.cmb_port.pack(side=tk.LEFT, padx=2)

        self.frm_ble = ttk.Frame(top)
        ttk.Label(self.frm_ble, text="设备").pack(side=tk.LEFT)
        self.cmb_ble = ttk.Combobox(self.frm_ble, textvariable=self.var_ble, width=28)
        self.cmb_ble.pack(side=tk.LEFT, padx=2)
        ttk.Button(self.frm_ble, text="扫描蓝牙", command=self._ble_scan).pack(side=tk.LEFT)

        self.btn_conn = ttk.Button(top, text="连接", command=self._toggle_conn)
        self.btn_conn.pack(side=tk.LEFT, padx=8)
        ttk.Label(top, textvariable=self.var_status).pack(side=tk.LEFT, padx=8)
        ttk.Label(top, textvariable=self.var_hb, foreground="#666").pack(side=tk.LEFT, padx=4)
        self.var_link.trace_add("write", lambda *_: self._on_link_mode_changed())
        self._sync_link_ui()
        if self._is_ble_mode() and bleak_available():
            self.after(300, self._ble_scan)

        mid = ttk.Frame(self, padding=8)
        mid.pack(fill=tk.X)
        rpm_box = ttk.LabelFrame(mid, text="实时转速", padding=12)
        rpm_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ttk.Label(rpm_box, textvariable=self.var_rpm, font=("Segoe UI", 36, "bold")).pack()
        ttk.Label(rpm_box, text="RPM").pack()
        ttk.Label(rpm_box, textvariable=self.var_dir, font=("Segoe UI", 14)).pack(pady=4)
        ttk.Label(rpm_box, textvariable=self.var_revs, font=("Segoe UI", 12)).pack(pady=2)
        ttk.Label(rpm_box, textvariable=self.var_hz).pack()
        ttk.Button(rpm_box, text="清零圈数", command=lambda: self._send("REVS CLEAR")).pack(pady=4)

        mon_box = ttk.LabelFrame(
            mid,
            text="触发：武装环�?(|RPM|>10 �?I �?→ 回溯100点 + 再记2s �?SD",
            padding=12,
        )
        mon_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))
        ttk.Label(mon_box, textvariable=self.var_mon, font=("Segoe UI", 12, "bold")).pack(anchor=tk.W)
        self.lbl_mon_state = tk.Label(
            mon_box,
            text="状态：空闲（未武装）",
            font=("Segoe UI", 11, "bold"),
            fg="#222",
            bg="#e8e8e8",
            padx=10,
            pady=4,
            anchor="w",
        )
        self.lbl_mon_state.pack(fill=tk.X, pady=(4, 2))
        ttk.Label(mon_box, textvariable=self.var_log).pack(anchor=tk.W, pady=4)
        ttk.Label(mon_box, textvariable=self.var_dump_prog, foreground="#0a7").pack(anchor=tk.W)
        # 蓝牙 BIN 下载进度
        prog_fr = ttk.Frame(mon_box)
        prog_fr.pack(fill=tk.X, pady=(4, 2))
        self.var_ble_pct = tk.StringVar(value="")
        ttk.Label(prog_fr, text="蓝牙下载").pack(side=tk.LEFT)
        self.ble_prog = ttk.Progressbar(
            prog_fr, orient=tk.HORIZONTAL, mode="determinate", maximum=100, length=180
        )
        self.ble_prog.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Label(prog_fr, textvariable=self.var_ble_pct, width=6).pack(side=tk.LEFT)
        self.ble_prog["value"] = 0
        rec_fr = ttk.Frame(mon_box)
        rec_fr.pack(fill=tk.X, pady=4)
        ttk.Label(rec_fr, text="确认后记录时长").pack(side=tk.LEFT)
        self.lbl_rec = ttk.Label(rec_fr, text="1.0 s")
        self.lbl_rec.pack(side=tk.LEFT, padx=6)
        self.scl_rec = ttk.Scale(
            rec_fr,
            from_=0.5,
            to=3.0,
            orient=tk.HORIZONTAL,
            variable=self.var_rec_s,
            command=self._on_rec_scale,
        )
        self.scl_rec.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        ttk.Button(rec_fr, text="应用到板子", command=self._apply_rec_ms).pack(side=tk.LEFT, padx=4)
        ttk.Label(
            mon_box,
            text="记录结束后自动上传；回传时下方显示进度（蓝牙慢速分包，勿关窗口）",
            foreground="#444",
        ).pack(anchor=tk.W)
        bf = ttk.Frame(mon_box)
        bf.pack(fill=tk.X, pady=8)
        # tk.Button 才能稳定改底色（ttk 在 Windows 上几乎不变色）
        self.btn_mon_start = tk.Button(
            bf,
            text="① 开始武装",
            command=self._on_monitor_start,
            font=("Segoe UI", 10, "bold"),
            relief=tk.RAISED,
            bd=2,
            padx=10,
            pady=4,
        )
        self.btn_mon_start.pack(side=tk.LEFT, padx=2)
        self.btn_mon_stop = tk.Button(
            bf,
            text="停止",
            command=self._on_monitor_stop,
            font=("Segoe UI", 10),
            relief=tk.RAISED,
            bd=2,
            padx=8,
            pady=4,
        )
        self.btn_mon_stop.pack(side=tk.LEFT, padx=2)
        ttk.Button(bf, text="清空全部段", command=lambda: self._send("LOG CLEAR")).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(bf, text="拉取并按段保存", command=self._dump_and_split).pack(
            side=tk.LEFT, padx=2
        )
        self.btn_dump = ttk.Button(bf, text="② 仅拉取", command=self._dump)
        self.btn_dump.pack(side=tk.LEFT, padx=2)
        ttk.Label(bf, text="回放段").pack(side=tk.LEFT, padx=(8, 2))
        self.cmb_seg = ttk.Combobox(bf, width=8, state="readonly")
        self.cmb_seg.pack(side=tk.LEFT)
        ttk.Button(bf, text="回放该段", command=self._play_seg).pack(side=tk.LEFT, padx=2)
        self._set_mon_ui("idle")

        # 动态滤波设置（收起/展开）
        self._flt_expanded = False
        self.var_flt_toggle = tk.StringVar(value="▶ 动态滤波设置（点击展开）")
        # 测速滤波参数
        self.var_flt_ema = tk.StringVar(value="150")
        self.var_flt_dpos = tk.StringVar(value="6")
        self.var_flt_zero = tk.StringVar(value="10")
        self.var_flt_rate = tk.StringVar(value="0")
        # IC 输入滤波动态表：icf[0..4] + bnd[0..3]
        self.var_icf = tuple(tk.StringVar(value=v) for v in ("0x0F", "0x0A", "0x06", "0x04", "0x01"))
        self.var_bnd = tuple(
            tk.StringVar(value=v) for v in ("500", "1500", "4000", "8000")
        )
        # 人工范围钳制
        self.var_flt_lo = tk.StringVar(value="0")
        self.var_flt_hi = tk.StringVar(value="15")

        # 长采：默认可收起，需要时再展开
        self._cap_expanded = False
        self.var_cap_toggle = tk.StringVar(value="▶ 长采（点击展开）")
        self.var_cap = tk.StringVar(value="长采未开始（推荐采30秒：各种启停拧几下即可）")
        cap_wrap = ttk.Frame(self)
        cap_wrap.pack(fill=tk.X, padx=8, pady=2)
        self.btn_cap_toggle = ttk.Button(
            cap_wrap, textvariable=self.var_cap_toggle, command=self._toggle_cap_panel
        )
        self.btn_cap_toggle.pack(fill=tk.X)
        self.cap_box = ttk.LabelFrame(
            cap_wrap,
            text="长采：2kHz 全时段 · 无门限（调试用，与短时武装无关）",
            padding=8,
        )
        # 默认不 pack → 收起
        ttk.Label(
            self.cap_box, textvariable=self.var_cap, font=("Segoe UI", 11, "bold"), foreground="#0a5"
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(self.cap_box, text="采30秒", command=lambda: self._cap_start(30)).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(self.cap_box, text="采60秒", command=lambda: self._cap_start(60)).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(self.cap_box, text="采80秒", command=lambda: self._cap_start(80)).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(self.cap_box, text="停止长采", command=lambda: self._send("CAPTURE STOP")).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(self.cap_box, text="拉取长采并保存", command=self._cap_dump_save).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(self.cap_box, text="打开保存目录", command=self._open_captures_dir).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(self.cap_box, text="画最近长采", command=self._plot_last_cap).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(self.cap_box, text="长采状态?", command=lambda: self._send("CAPTURE?")).pack(
            side=tk.LEFT, padx=2
        )

        # ---- 动态滤波设置（可收起，Task 8）----
        flt_wrap = ttk.Frame(self)
        flt_wrap.pack(fill=tk.X, padx=8, pady=2)
        self.btn_flt_toggle = ttk.Button(
            flt_wrap, textvariable=self.var_flt_toggle, command=self._toggle_flt_panel
        )
        self.btn_flt_toggle.pack(fill=tk.X)
        self.flt_box = ttk.LabelFrame(
            flt_wrap, text="动态滤波：AUTO 全自动（按|rpm|查表·范围钳位）→ 人工可改", padding=8
        )
        # 默认收起，展开时才 pack
        row0 = ttk.Frame(self.flt_box)
        row0.pack(fill=tk.X, pady=2)
        ttk.Label(row0, text="EMA‰:").pack(side=tk.LEFT)
        ttk.Entry(row0, textvariable=self.var_flt_ema, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(row0, text="dpos×:").pack(side=tk.LEFT)
        ttk.Entry(row0, textvariable=self.var_flt_dpos, width=4).pack(side=tk.LEFT, padx=2)
        ttk.Label(row0, text="zero周期:").pack(side=tk.LEFT)
        ttk.Entry(row0, textvariable=self.var_flt_zero, width=4).pack(side=tk.LEFT, padx=2)
        ttk.Label(row0, text="rate(rpm/s):").pack(side=tk.LEFT)
        ttk.Entry(row0, textvariable=self.var_flt_rate, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Label(
            row0,
            text="(0=自动实测学习本机爬坡斜率；>0=手动指定您电机实测的爬坡斜率，超其 120% 判跳)",
            foreground="#888",
        ).pack(side=tk.LEFT, padx=4)
        ttk.Label(row0, text="范围(IC):").pack(side=tk.LEFT)
        ttk.Entry(row0, textvariable=self.var_flt_lo, width=3).pack(side=tk.LEFT, padx=2)
        ttk.Label(row0, text="~").pack(side=tk.LEFT)
        ttk.Entry(row0, textvariable=self.var_flt_hi, width=3).pack(side=tk.LEFT, padx=2)
        row1 = ttk.Frame(self.flt_box)
        row1.pack(fill=tk.X, pady=2)
        ttk.Label(row1, text="ICF 表(val 0..15): ").pack(side=tk.LEFT)
        for i, v in enumerate(self.var_icf):
            ttk.Label(row1, text=f"icf[{i}]").pack(side=tk.LEFT, padx=(4, 0))
            ttk.Entry(row1, textvariable=v, width=5).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Label(row1, text=" | 边界rpm: ").pack(side=tk.LEFT)
        for i, v in enumerate(self.var_bnd):
            ttk.Label(row1, text=f"bnd[{i}]").pack(side=tk.LEFT, padx=(4, 0))
            ttk.Entry(row1, textvariable=v, width=6).pack(side=tk.LEFT, padx=(0, 2))
        row2 = ttk.Frame(self.flt_box)
        row2.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(row2, text="应用到板子", command=self._apply_flt).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="读取当前(FILT SHOW)", command=lambda: self._send("FILT SHOW")).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(row2, text="恢复自动默认(FILT RESET)", command=lambda: self._send("FILT RESET")).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Label(
            row2,
            text="解释：低速(0~500rpm)强滤波0x0F滤毛刺；高速(>8000rpm)弱滤波0x01防丢步；rate(0=实测学习)>0则限幅超其120%的跳变",
            foreground="#666",
        ).pack(side=tk.LEFT, padx=8)
        # 展开状态由 _toggle_flt_panel 控制

        browse = ttk.LabelFrame(self, text="已接收数据 · 转速 vs 时间（可缩放）", padding=6)
        browse.pack(fill=tk.X, padx=8, pady=4)
        br_left = ttk.Frame(browse)
        br_left.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(br_left, text="数据段列表").pack(anchor=tk.W)
        self.lst_archive = tk.Listbox(br_left, height=6, width=48, exportselection=False)
        self.lst_archive.pack(fill=tk.Y)
        br_btns = ttk.Frame(br_left)
        br_btns.pack(fill=tk.X, pady=4)
        ttk.Button(br_btns, text="打开 snap.bin", command=self._open_snap_bin).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(br_btns, text="打开曲线工作室", command=self._open_curve_studio).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(br_btns, text="主窗预览", command=self._view_selected).pack(side=tk.LEFT, padx=2)
        ttk.Button(br_btns, text="回放选中", command=self._play_archived).pack(side=tk.LEFT, padx=2)
        ttk.Button(br_btns, text="导出CSV", command=self._export_filtered_csv).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(br_btns, text="删选中", command=self._delete_selected).pack(side=tk.LEFT, padx=2)

        br_right = ttk.Frame(browse)
        br_right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(12, 0))
        ttk.Label(br_right, text="显示时间窗 t0 ~ t1（秒，相对段起点）").pack(anchor=tk.W)
        win_fr = ttk.Frame(br_right)
        win_fr.pack(fill=tk.X, pady=4)
        ttk.Label(win_fr, text="t0").pack(side=tk.LEFT)
        ttk.Entry(win_fr, textvariable=self.var_t0, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Label(win_fr, text="t1").pack(side=tk.LEFT)
        ttk.Entry(win_fr, textvariable=self.var_t1, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Button(win_fr, text="应用窗口", command=self._apply_time_window).pack(side=tk.LEFT, padx=4)
        ttk.Button(win_fr, text="0~0.3s", command=lambda: self._preset_window(0.0, 0.3)).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(win_fr, text="0~1s", command=lambda: self._preset_window(0.0, 1.0)).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(win_fr, text="全段", command=self._window_full).pack(side=tk.LEFT, padx=2)
        ttk.Button(win_fr, text="切回实时", command=self._live_plot_mode).pack(side=tk.LEFT, padx=8)
        fil_fr = ttk.Frame(br_right)
        fil_fr.pack(fill=tk.X, pady=4)
        ttk.Label(fil_fr, text="|RPM|<").pack(side=tk.LEFT)
        ttk.Entry(fil_fr, textvariable=self.var_rpm_min, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(fil_fr, text="过滤掉").pack(side=tk.LEFT)
        ttk.Checkbutton(fil_fr, text="去0", variable=self.var_drop_zero).pack(side=tk.LEFT, padx=2)
        ttk.Button(fil_fr, text="应用转速过滤", command=self._apply_rpm_cut).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(fil_fr, text="恢复原文件", command=self._restore_snap_raw).pack(
            side=tk.LEFT, padx=2
        )

        cut_fr = ttk.Frame(br_right)
        cut_fr.pack(fill=tk.X, pady=2)
        ttk.Button(cut_fr, text="只保留 t0~t1", command=self._edit_keep_range).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(cut_fr, text="删除 t0~t1", command=self._edit_delete_range).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(cut_fr, text="在 t0 拆两段", command=self._edit_split_at_t0).pack(
            side=tk.LEFT, padx=2
        )

        dist_fr = ttk.Frame(br_right)
        dist_fr.pack(fill=tk.X, pady=2)
        ttk.Label(dist_fr, text="I0").pack(side=tk.LEFT)
        ttk.Entry(dist_fr, textvariable=self.var_i0, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Button(dist_fr, text="用首点", command=self._edit_i0_from_first).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Label(dist_fr, text="每I长度").pack(side=tk.LEFT, padx=(6, 0))
        ttk.Entry(dist_fr, textvariable=self.var_len_i, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(dist_fr, text="(mm/圈)").pack(side=tk.LEFT)
        ttk.Radiobutton(
            dist_fr, text="RPM-t", variable=self.var_plot_y, value="rpm", command=self._refresh_edit_plot
        ).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(
            dist_fr, text="S(I)-t", variable=self.var_plot_y, value="s_i", command=self._refresh_edit_plot
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            dist_fr, text="S(∫rpm)-t", variable=self.var_plot_y, value="s_rpm", command=self._refresh_edit_plot
        ).pack(side=tk.LEFT)
        ttk.Label(br_right, textvariable=self.var_edit_info, foreground="#333").pack(
            anchor=tk.W, pady=2
        )
        ttk.Label(
            br_right,
            text="剪辑像视频：t0/t1→只保留/删除；|RPM|<阈值过滤；I0+每I长度→S=ΔI×长度；可导出含 S 的 CSV",
        ).pack(anchor=tk.W)

        split = ttk.Panedwindow(self, orient=tk.VERTICAL)
        split.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        plot_fr = ttk.LabelFrame(split, text="曲线：RPM vs t(s)（默认关闭，防卡死）", padding=6)
        self._has_plot = False
        self.chart = None
        self._plot_enabled = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            plot_fr,
            text="启用实时曲线（可能卡顿，仅浏览已接收数据时建议打开）",
            variable=self._plot_enabled,
            command=self._toggle_plot,
        ).pack(anchor=tk.W)
        self.plot_host = ttk.Frame(plot_fr)
        self.plot_host.pack(fill=tk.BOTH, expand=True)
        ttk.Label(self.plot_host, text="曲线已关闭：监控时只更新上方数字，记完后点「仅拉取」").pack(
            anchor=tk.W, pady=8
        )

        log_fr = ttk.LabelFrame(split, text="日志（拖中间分隔条可加大）", padding=4)
        log_inner = ttk.Frame(log_fr)
        log_inner.pack(fill=tk.BOTH, expand=True)
        self.log_txt = tk.Text(log_inner, height=10, wrap=tk.WORD, undo=False)
        log_scroll = ttk.Scrollbar(log_inner, orient=tk.VERTICAL, command=self.log_txt.yview)
        self.log_txt.configure(yscrollcommand=log_scroll.set)
        self.log_txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        split.add(plot_fr, weight=3)
        split.add(log_fr, weight=2)
        self._split = split

        bot = ttk.Frame(self, padding=8)
        bot.pack(fill=tk.X)
        ttk.Button(bot, text="ABI?", command=lambda: self._send("ABI?")).pack(side=tk.LEFT)
        ttk.Button(bot, text="停止回放", command=self._stop_play).pack(side=tk.LEFT, padx=4)
        ttk.Button(bot, text="帮助·转速计算", command=self._show_rpm_help).pack(side=tk.LEFT, padx=8)
        ttk.Label(bot, textvariable=self.var_info).pack(side=tk.LEFT, padx=12)

    def _show_rpm_help(self) -> None:
        """打开打包内帮助：转速从 counts 如何计算。"""
        win = tk.Toplevel(self)
        win.title("帮助 · 转速计算方法")
        win.geometry("720x560")
        win.minsize(480, 360)
        frm = ttk.Frame(win, padding=8)
        frm.pack(fill=tk.BOTH, expand=True)
        txt = tk.Text(frm, wrap=tk.WORD, undo=False)
        sb = ttk.Scrollbar(frm, orient=tk.VERTICAL, command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        body = help_text("转速计算方法.md")
        txt.insert("1.0", body)
        txt.configure(state=tk.DISABLED)
        bar = ttk.Frame(win, padding=6)
        bar.pack(fill=tk.X)
        def _open_file() -> None:
            p = help_dir() / "转速计算方法.md"
            try:
                if p.is_file():
                    os.startfile(str(p))  # type: ignore[attr-defined]
                else:
                    messagebox.showinfo("帮助", f"文件不在磁盘旁路：{p}\n（内容已在上方窗口）")
            except Exception as exc:  # noqa: BLE001
                messagebox.showwarning("帮助", str(exc))
        ttk.Button(bar, text="用系统打开 .md", command=_open_file).pack(side=tk.LEFT)
        ttk.Button(bar, text="关闭", command=win.destroy).pack(side=tk.RIGHT)

    def _is_ble_mode(self) -> bool:
        v = self.var_link.get()
        return v in ("BLE", "蓝牙BLE")

    def _is_sim_mode(self) -> bool:
        v = self.var_link.get()
        return "仿真" in v or "SIM" in v.upper()

    def _on_link_mode_changed(self) -> None:
        self._sync_link_ui()
        if self._is_ble_mode() and bleak_available() and not self.cmb_ble["values"]:
            self._ble_scan()

    def _sync_link_ui(self) -> None:
        if self._is_sim_mode():
            self.frm_usb.pack_forget()
            self.frm_ble.pack_forget()
            if hasattr(self, "btn_dump"):
                self.btn_dump.configure(text="② 仅拉取(仿真)")
        elif self._is_ble_mode():
            self.frm_usb.pack_forget()
            self.frm_ble.pack(side=tk.LEFT, padx=6, before=self.btn_conn)
            if hasattr(self, "btn_dump"):
                self.btn_dump.configure(text="仅拉取(蓝牙)")
        else:
            self.frm_ble.pack_forget()
            self.frm_usb.pack(side=tk.LEFT, padx=6, before=self.btn_conn)
            if hasattr(self, "btn_dump"):
                self.btn_dump.configure(text="仅拉取(USB全速)")

    def _dump_cmd(self) -> str:
        """短时 snap：BLE→DUMP BIN BLE；USB→DUMP BIN。旧长 LOG 仍可用 LOG DUMP。"""
        if self._is_sim_mode():
            return "LOG DUMP USB"
        if self._is_ble_mode():
            return "DUMP BIN BLE"
        return "DUMP BIN"

    def _refresh_ports(self) -> None:
        if self._is_ble_mode():
            self._ble_scan()
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb_port["values"] = ports
        if ports and not self.var_port.get():
            self.var_port.set(ports[0])

    def _ble_scan(self) -> None:
        if not bleak_available():
            messagebox.showerror(
                "蓝牙",
                "未安装 bleak。\n请在 pc 目录执行:\npip install bleak",
            )
            return
        self.var_status.set("正在扫描蓝牙 OEZ-ABI…")
        self._append("# 扫描蓝牙…")

        def work() -> None:
            try:
                found = scan_oez_devices_sync(5.0)
            except Exception as exc:  # noqa: BLE001
                self.after(0, lambda: messagebox.showerror("蓝牙扫描失败", str(exc)))
                self.after(0, lambda: self.var_status.set("扫描失败"))
                return

            def apply() -> None:
                labels = [f"{n} [{a}]" for a, n in found]
                self._ble_addr = {f"{n} [{a}]": a for a, n in found}
                self.cmb_ble["values"] = labels
                if labels:
                    self.var_ble.set(labels[0])
                    self.var_status.set(f"扫描到 {len(labels)} 台，可点连接")
                else:
                    self.var_status.set("未扫到 OEZ-ABI（确认板子上电且未连手机）")
                self._append(f"# BLE scan: {labels or 'none'}")

            self.after(0, apply)

        threading.Thread(target=work, daemon=True).start()

    def _disconnect(self) -> None:
        self.stop_evt.set()
        link = self.link
        self.link = None
        self.btn_conn.configure(text="连接")
        self.var_status.set("已断开")
        if link is not None and link.is_alive():
            # 给 BLE 线程一点时间退出
            def _join() -> None:
                link.join(timeout=2.0)

            threading.Thread(target=_join, daemon=True).start()

    def _toggle_conn(self) -> None:
        if self.link is not None:
            self._disconnect()
            return
        self.stop_evt = threading.Event()
        self.out_q = queue.Queue()
        if self._is_ble_mode():
            if BleLink is None or not bleak_available():
                messagebox.showerror("蓝牙", "请先: pip install bleak")
                return
            label = self.var_ble.get().strip()
            addr = self._ble_addr.get(label)
            if not addr and "[" in label and label.endswith("]"):
                addr = label[label.rfind("[") + 1 : -1]
            if not addr:
                messagebox.showwarning("蓝牙", "请先点「扫描蓝牙」并选择 OEZ-ABI")
                return
            self.link = BleLink(
                addr, self.out_q, self.stop_evt, on_status=self._ble_status
            )
        elif self._is_sim_mode():
            if SimLink is None:
                messagebox.showerror("仿真", "找不到 abi_sim.py")
                return
            self.link = SimLink(self.out_q, self.stop_evt, auto_launch=True)
            self.var_status.set("仿真已连接 — 点①开始监控将自动弹射假 ABI")
        else:
            port = self.var_port.get().strip()
            if not port:
                messagebox.showwarning("USB", "请选择串口")
                return
            self.link = SerialLink(port, self.out_q, self.stop_evt)
        self.link.start()
        self.btn_conn.configure(text="断开")
        if not self._is_sim_mode():
            self.var_status.set("连接中…")
        if getattr(self, "_serial_log_path", None):
            self._append(f"# serial log file: {self._serial_log_path}")
        # 对时：SD 文件名用墙钟；连上后发两次，武装前再刷新
        self.after(200, self._sync_board_time)
        self.after(1200, self._sync_board_time)
        self.after(500, self._apply_rec_ms)
        self.after(900, lambda: self._queue_cmd("ABI?"))
        if self._is_sim_mode():
            self.var_info.set("仿真：连接后点「①开始监控」，约 0.5s 后自动假弹射")

    def _sync_board_time(self) -> None:
        """把 PC 当前时间发给板子，供 SD 存档文件名/元数据使用。"""
        self._queue_cmd(f"TIME {int(time.time() * 1000)}")

    def _on_rec_scale(self, _v: str | None = None) -> None:
        s = round(float(self.var_rec_s.get()) * 10) / 10.0
        if s < 0.5:
            s = 0.5
        if s > 3.0:
            s = 3.0
        self.var_rec_s.set(s)
        self.lbl_rec.configure(text=f"{s:.1f} s")

    def _apply_rec_ms(self) -> None:
        s = float(self.var_rec_s.get())
        ms = int(round(s * 1000))
        if ms < 500:
            ms = 500
        if ms > 3000:
            ms = 3000
        ms = (ms // 100) * 100
        self.var_rec_s.set(ms / 1000.0)
        self.lbl_rec.configure(text=f"{ms/1000.0:.1f} s")
        if self.link:
            self._queue_cmd(f"REC MS {ms}")
            self.var_info.set(f"已设置记录时长 {ms} ms")

    def _ble_status(self, msg: str) -> None:
        # 限流：避免 bleak 线程狂刷 after(0) 把 Tk 事件队列撑爆
        now = time.monotonic()
        # 进度类状态更勤刷新
        is_prog = "BLE BIN" in msg and "%" in msg
        gap = 0.12 if is_prog else 0.25
        if now - self._last_ble_status < gap and "error" not in msg.lower():
            return
        self._last_ble_status = now
        self.after(0, lambda m=msg: self.var_status.set(m))
        if is_prog:
            self.after(0, lambda m=msg: self._apply_ble_prog_from_status(m))

    def _apply_ble_prog_from_status(self, msg: str) -> None:
        """解析 'BLE BIN 1234/5678 (45%)' → 进度条。"""
        try:
            if "(" in msg and "%" in msg:
                pct_s = msg.rsplit("(", 1)[1].split("%", 1)[0].strip()
                pct = max(0, min(100, int(pct_s)))
                self.ble_prog["value"] = pct
                self.var_ble_pct.set(f"{pct}%")
                if "BLE BIN" in msg and "/" in msg:
                    mid = msg.split("BLE BIN", 1)[1].strip()
                    frac = mid.split("(", 1)[0].strip()
                    self.var_dump_prog.set(f"蓝牙下载 {frac} · {pct}%")
        except Exception:  # noqa: BLE001
            pass

    def _set_ble_bin_progress(self, got: int, expect: int, pct: int) -> None:
        pct = max(0, min(100, int(pct)))
        self.ble_prog["value"] = pct
        self.var_ble_pct.set(f"{pct}%")
        if expect > 0:
            self.var_dump_prog.set(f"蓝牙下载 {got}/{expect} 字节 · {pct}%")
            self.var_mon.set(f"蓝牙回传中… {pct}%")
        else:
            self.var_dump_prog.set(f"蓝牙下载 {got} 字节…")

    def _queue_cmd(self, cmd: str) -> None:
        """连接中也可排队发送（不弹未连接框）。"""
        if not self.link:
            return
        self.link.send(cmd)
        self._append(f">> {cmd}")

    def _toggle_plot(self) -> None:
        for w in self.plot_host.winfo_children():
            w.destroy()
        self.chart = None
        self._has_plot = False
        if not self._plot_enabled.get():
            ttk.Label(
                self.plot_host, text="曲线已关闭：监控时只更新上方数字，记完后点「仅拉取」"
            ).pack(anchor=tk.W, pady=8)
            return
        try:
            from rpm_chart import RpmChart

            self.chart = RpmChart(self.plot_host)
            self.chart.pack(fill=tk.BOTH, expand=True)
            self.chart.set_labels("t (s)", "|RPM|")
            self.chart.set_title("live")
            self._has_plot = True
        except Exception as exc:  # noqa: BLE001
            ttk.Label(self.plot_host, text=f"曲线加载失败: {exc}").pack()

    def _cap_start(self, sec: int) -> None:
        if not self.link or not getattr(self.link, "is_open", False):
            messagebox.showwarning("未连接", "请先 USB 连接板子")
            return
        self.cap_rows.clear()
        self._cap_total_s = int(sec)
        self._cap_end_mono = time.monotonic() + float(sec)
        self._cap_board_n = 0
        self._cap_dumping = False
        self.var_cap.set(f"长采中 剩余 {sec}s · 已采 0 点（可随意启停/等待）")
        self._queue_cmd(f"CAPTURE START {sec}")

    def _cap_dump_save(self) -> None:
        if not self.link or not getattr(self.link, "is_open", False):
            messagebox.showwarning("未连接", "请先连接")
            return
        if self._cap_dumping:
            return
        self._cap_dumping = True
        self._cap_end_mono = None
        self.cap_rows.clear()
        self.var_cap.set("正在拉取长采…（stride=10 ACK分块，勿拔线）")
        self._queue_cmd("CAPTURE DUMP")

    def _save_cap_file(self) -> None:
        if not self.cap_rows:
            self.var_cap.set("未收到 C 点（板内可能为空或拉取失败）")
            messagebox.showwarning("长采", "没有收到任何长采数据点")
            return
        from pathlib import Path

        folder = app_dir() / "captures"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / datetime.now().strftime("hand_twist_%Y%m%d_%H%M%S.csv")
        try:
            with path.open("w", encoding="utf-8", newline="") as f:
                f.write("t_ms,rpm,dcounts,dindex\n")
                for t, rpm, dc, di in self.cap_rows:
                    f.write(f"{t},{rpm},{dc},{di}\n")
            self._last_cap_path = str(path)
            t0 = self.cap_rows[0][0] / 1000.0
            t1 = self.cap_rows[-1][0] / 1000.0
            self.var_cap.set(
                f"已保存 {len(self.cap_rows)} 点 · {t1 - t0:.1f}s → {path.name}"
            )
            self.var_info.set(str(path))
            self._append(f"# CAP saved {path} n={len(self.cap_rows)}")
            self._plot_last_cap()
            messagebox.showinfo(
                "长采已保存",
                f"共 {len(self.cap_rows)} 点\n时长约 {t1 - t0:.1f} s\n\n{path}",
            )
        except Exception as exc:  # noqa: BLE001
            self.var_cap.set(f"保存失败: {exc}")

    def _open_captures_dir(self) -> None:
        from pathlib import Path
        import subprocess

        folder = app_dir() / "captures"
        folder.mkdir(parents=True, exist_ok=True)
        subprocess.Popen(["explorer", str(folder)])

    def _plot_last_cap(self) -> None:
        rows = self.cap_rows
        if not rows:
            # 尝试读最近文件
            from pathlib import Path

            folder = app_dir() / "captures"
            files = sorted(folder.glob("hand_twist_*.csv")) if folder.exists() else []
            if not files:
                messagebox.showinfo("长采", "还没有长采数据/文件")
                return
            path = files[-1]
            rows = []
            with path.open("r", encoding="utf-8") as f:
                next(f, None)
                for line in f:
                    p = line.strip().split(",")
                    if len(p) >= 4:
                        rows.append((float(p[0]), float(p[1]), int(float(p[2])), int(float(p[3]))))
            self.cap_rows = rows
            self.var_info.set(str(path))
        if not self._plot_enabled.get():
            self._plot_enabled.set(True)
            self._toggle_plot()
        if not self._has_plot or self.chart is None:
            return
        # 降采样画图，避免卡顿
        step = max(1, len(rows) // 4000)
        xs = [rows[i][0] / 1000.0 for i in range(0, len(rows), step)]
        ys = [abs(rows[i][1]) for i in range(0, len(rows), step)]
        self.plot_mode = "segment"
        self.chart.set_data(xs, ys, title=f"长采 {len(rows)}点", xlabel="t (s)")
        self.var_cap.set(f"已显示长采曲线 · {len(rows)} 点（每{step}点抽1）")

    def _set_mon_ui(self, state: str) -> None:
        """监控按钮/色条：idle→armed→staging→recording→dumping→done。"""
        try:
            self._mon_state = state
            # (条文字, 条bg, 条fg, 开始按钮文字, 开始bg, 开始fg, 停止bg)
            styles = {
            "idle": (
                "状态：空闲 — 点①武装；可静止，转起来后自动判",
                "#e8e8e8",
                "#222",
                "① 开始武装",
                "#dfe6e9",
                "#111",
                "#ececec",
            ),
            "armed": (
                "状态：已武装 · 环缓临时存 · 等 |RPM|>10",
                "#ffeaa7",
                "#5d4e00",
                "① 已武装·等待中…",
                "#fdcb6e",
                "#111",
                "#fab1a0",
            ),
            "staging": (
                "状态：|RPM|>10 · 等 I 过完 1 圈",
                "#fdcb6e",
                "#4a3000",
                "① 等 I 一圈…",
                "#e17055",
                "#fff",
                "#e17055",
            ),
            "recording": (
                "状态：已触发 · 回溯400 + 记2s → SD",
                "#ff7675",
                "#fff",
                "① 记录中…",
                "#d63031",
                "#fff",
                "#d63031",
            ),
            "dumping": (
                "状态：SD 存档中 / 可读 U 盘",
                "#74b9ff",
                "#003d7a",
                "① 存档中…",
                "#0984e3",
                "#fff",
                "#0984e3",
            ),
            "done": (
                "状态：本段完成 · 可再武装",
                "#55efc4",
                "#004d40",
                "① 开始武装",
                "#00b894",
                "#fff",
                "#ececec",
            ),
        }
            text, bg, fg, btn_t, btn_bg, btn_fg, stop_bg = styles.get(state, styles["idle"])
            if hasattr(self, "lbl_mon_state"):
                self.lbl_mon_state.configure(text=text, bg=bg, fg=fg)
            # Windows：先 NORMAL 再改色，再按需 DISABLED，避免 TclError 把整窗打崩
            start_state = tk.NORMAL if state in ("idle", "done") else tk.DISABLED
            stop_state = tk.NORMAL if state not in ("idle", "done", "dumping") else tk.DISABLED
            if hasattr(self, "btn_mon_start"):
                self.btn_mon_start.configure(state=tk.NORMAL)
                self.btn_mon_start.configure(
                    text=btn_t,
                    bg=btn_bg,
                    fg=btn_fg,
                    activebackground=btn_bg,
                    activeforeground=btn_fg,
                    disabledforeground="#666",
                )
                self.btn_mon_start.configure(state=start_state)
            if hasattr(self, "btn_mon_stop"):
                self.btn_mon_stop.configure(state=tk.NORMAL)
                self.btn_mon_stop.configure(
                    bg=stop_bg,
                    fg="#111",
                    activebackground=stop_bg,
                    disabledforeground="#666",
                )
                self.btn_mon_stop.configure(state=stop_state)
        except Exception as exc:  # noqa: BLE001
            try:
                self._append(f"# WARN mon_ui: {exc}")
            except Exception:  # noqa: BLE001
                pass

    def _on_monitor_stop(self) -> None:
        self._queue_cmd("MONITOR STOP")
        self._set_mon_ui("idle")
        self.var_mon.set("已停止 · 仅实时显示")
        self.var_info.set("停止后板内已记数据仍在；可点②拉取")

    def _on_rec_now(self) -> None:
        """电机已转起来后：立刻采满 → SNAP DONE → SD SAVE →（BLE 则拉 RAM）。"""
        if not self.link or not getattr(self.link, "is_open", False):
            messagebox.showwarning("未连接", "请先连接设备")
            return
        self._read_scheduled = False
        self._ui_dumping = False
        self._board_auto_push = False
        self.dump_rows.clear()
        self.dump_index.clear()
        self._plot_enabled.set(False)
        self._toggle_plot()
        self._set_mon_ui("recording")
        self.var_mon.set("直采中… 2s@2kHz（RAM）")
        if self._is_ble_mode():
            self.var_info.set("RAM→SD 存档后，蓝牙从 RAM 拉（不读 SD SPI）")
        else:
            self.var_info.set("RAM 记满 → ALIVE → 自动 SD SAVE；可选再 DUMP BIN / U盘")
        self.var_dump_prog.set("记录中（INTERNAL RAM）…")
        self.after(20, lambda: self._queue_cmd("REC NOW"))

    def _do_bin_pull(self) -> None:
        """USB: DUMP BIN；BLE: DUMP BIN BLE（读板内 RAM，与 SD 内容一致）。"""
        if not self.link or not getattr(self.link, "is_open", False):
            self.var_dump_prog.set("DUMP BIN 失败：未连接")
            return
        self.dump_rows.clear()
        self.dump_index.clear()
        self.playback.clear()
        self._ui_dumping = True
        self._awaiting_bindump = True
        self._auto_split_after_dump = True
        self._dump_watch_t0 = time.monotonic()
        self._dump_watch_n = 0
        self._set_mon_ui("dumping")
        if self._is_ble_mode():
            self.var_dump_prog.set("BLE BIN 拉取中（src=RAM）… 0%")
            self.var_mon.set("蓝牙回传 RAM snap…")
            self.ble_prog["value"] = 0
            self.var_ble_pct.set("0%")
            self._queue_cmd("SNAP?")
            self.after(80, lambda: self._queue_cmd("DUMP BIN BLE"))
        else:
            self.var_dump_prog.set("DUMP BIN 拉取中（preamble+magic）…")
            self.var_mon.set("二进制回传中")
            self._queue_cmd("SNAP?")
            if isinstance(self.link, SerialLink):
                self.link.arm_bindump(timeout_s=15.0)
            self.after(50, lambda: self._queue_cmd("DUMP BIN"))

    def _handle_bindump(self, raw: bytes) -> None:
        self._awaiting_bindump = False
        try:
            n, hz, rows = parse_snap_bindump(raw)
        except Exception as exc:  # noqa: BLE001
            head = raw[:16].hex(" ") if raw else ""
            self._append(f"# BIN CRC/parse FAIL: {exc} | head16={head} len={len(raw)}")
            self.var_mon.set(f"二进制校验失败: {exc}")
            self.var_dump_prog.set("BIN 失败 — 可再点①重采")
            self._set_mon_ui("idle")
            self._ui_dumping = False
            return
        self.dump_rows = list(rows)
        self.playback = list(rows)
        self._dump_expect = n
        self._dump_got = n
        self._append(f"# BIN OK n={n} hz={hz} bytes={len(raw)} CRC ok")
        self.var_dump_prog.set(f"BIN OK {n} 点 @ {hz}Hz")
        self.var_mon.set(f"二进制收齐 {n} 点 · 正在保存")
        try:
            self.ble_prog["value"] = 100
            self.var_ble_pct.set("100%")
        except Exception:  # noqa: BLE001
            pass
        voice_speak(f"数据已取回，共{n}个点")
        self._force_finish_dump = True
        self.after(50, self._finish_auto_recv)

    def _on_monitor_start(self) -> None:
        """武装：环缓临时存 → |RPM|>10 且 I+1圈 → 回溯400 + 记2s → SD。"""
        if not self.link or not getattr(self.link, "is_open", False):
            messagebox.showwarning("未连接", "请先连接设备")
            return
        self._read_scheduled = False
        self._ui_dumping = False
        self._board_auto_push = False
        self._plot_enabled.set(False)
        self._toggle_plot()
        self._set_mon_ui("armed")
        self.var_mon.set("已武装 · 环缓中… 等 |RPM|>10 且 I 过1圈")
        if self._is_ble_mode():
            self.var_info.set("触发→RAM→SD 存档→自动蓝牙从 RAM 取回（SD 仅存档）")
        else:
            self.var_info.set("触发后：回溯400 + 再记2s → SD SAVE → USB DISK ON 读卡")
        self.var_dump_prog.set("armed / ring…")
        self._sync_board_time()
        self.after(40, lambda: self._queue_cmd("MONITOR START"))
        voice_speak("已经开始监控，请加油")

    def _schedule_read_once(self, delay_ms: int = 200) -> None:
        """记完后：PC 立刻拉取；板子 1.2s 无动作也会自推。"""
        if getattr(self, "_read_scheduled", False):
            return
        if self._ui_dumping and len(self.dump_rows) > 50:
            return
        self._read_scheduled = True
        self._auto_split_after_dump = True
        self._dump_retry = 0
        self._set_mon_ui("dumping")
        self.var_dump_prog.set(f"记录完成 — {delay_ms}ms 后拉取（含触发前追点）…")
        self.var_mon.set("记录完成 · 即将拉取 backtrack+RECORD")
        self.var_info.set("板内已冻结；PC 拉取 / 板子超时自推")
        self.after(delay_ms, self._do_auto_pull)

    def _do_auto_pull_backup(self) -> None:
        self._do_auto_pull()

    def _do_auto_pull(self) -> None:
        self._read_scheduled = False
        if self._ui_dumping and self.dump_rows and len(self.dump_rows) > 200:
            return
        if not self.link or not getattr(self.link, "is_open", False):
            self.var_dump_prog.set("自动拉取失败：未连接 — 等板子 FALLBACK 自推")
            self._ui_dumping = True
            self._board_auto_push = True
            return
        # 若已在收板子 FALLBACK 推送，不要清掉
        if self.dump_rows and len(self.dump_rows) > 50:
            self._ui_dumping = True
            self._board_auto_push = True
            return
        self.dump_rows.clear()
        self.dump_index.clear()
        self.playback.clear()
        self._auto_split_after_dump = True
        self._dump_got = 0
        self._ui_dumping = True
        self._board_auto_push = True
        self._dump_watch_t0 = time.monotonic()
        self._dump_watch_n = 0
        seg = int(getattr(self, "_pull_seg", 0) or 0)
        if self._is_ble_mode():
            cmd = f"LOG DUMP BLE SEG {seg}" if seg else "READ"
        else:
            cmd = f"LOG DUMP USB SEG {seg}" if seg else "LOG DUMP USB"
        self.var_dump_prog.set(f"拉取中… ({cmd}) expect≈{self._dump_expect or '?'}")
        self.var_mon.set("正在拉取完整段（触发前追点 + 确认后记录）…")
        self._queue_cmd(cmd)

    def _finish_auto_recv(self) -> None:
        """拉取结束后：归档、存盘、画图、分析。"""
        try:
            if getattr(self, "_finishing_auto", False):
                return
            # 若 expect 很大但点太少，多半是误触发收尾 —— 继续等
            n_now = len(self.dump_rows)
            exp = int(getattr(self, "_dump_expect", 0) or 0)
            if (
                getattr(self, "_ui_dumping", False)
                and exp >= 200
                and n_now > 0
                and n_now < max(100, exp // 5)
                and not getattr(self, "_force_finish_dump", False)
            ):
                self._append(f"# WARN finish too early n={n_now} expect={exp} — keep waiting")
                return
            self._force_finish_dump = False
            self._finishing_auto = True
            self._board_auto_push = False
            self._ui_dumping = False
            self._auto_split_after_dump = False
            if not self.dump_rows:
                self._set_mon_ui("idle")
                self.var_mon.set("拉取结束：收到 0 个点（传输失败，不是转速为0）")
                self.var_dump_prog.set("0 点 = 空缓冲/回传失败")
                self.var_info.set("转速为0也是数据；若板内有记录却收到0点，属回传问题")
                self.after(1500, lambda: setattr(self, "_finishing_auto", False))
                try:
                    messagebox.showwarning(
                        "空缓冲（异常）",
                        "收到 0 个采样点。\n\n"
                        "说明：转速=0 也是合法数据，不应出现空缓冲。\n"
                        "这是回传/拉取失败，请再①开始监控并转动一次。",
                    )
                except Exception:  # noqa: BLE001
                    pass
                return
            # 即使点数偏少也保存：0RPM 采样同样有效，残片也比「空」强，便于对照
            n = len(self.dump_rows)
            if n < 100:
                self._append(
                    f"# WARN short dump n={n} (still saving; 0RPM samples are valid data)"
                )
                self.var_info.set(
                    f"点数偏少 {n}（完整约 backtrack+1s）；已保存。转速0≠空数据"
                )
            self.playback = list(self.dump_rows)
            self._refresh_seg_combo()
            self._archive_current()
            self.var_dump_prog.set(f"回传完成 {n} 点")
            self._set_mon_ui("done")
            self.var_mon.set("已自动接收 · 正在保存并分析…")
            # 不强制重建曲线（易把窗口打崩）；有图才画，无图用状态即可
            try:
                if self._plot_enabled.get() and not self._has_plot:
                    self._toggle_plot()
                elif not self._plot_enabled.get():
                    self._plot_enabled.set(True)
                    self._toggle_plot()
            except Exception as exc:  # noqa: BLE001
                self._append(f"# WARN plot enable: {exc}")
            self.after(100, self._safe_view_selected)
            self.after(200, self._safe_auto_save_and_analyze)
            self.after(3000, lambda: setattr(self, "_finishing_auto", False))
            self.after(
                4000,
                lambda: self._set_mon_ui("idle")
                if getattr(self, "_mon_state", "") == "done"
                else None,
            )
        except Exception as exc:  # noqa: BLE001
            self._finishing_auto = False
            try:
                self._append(f"# ERR finish_auto_recv: {exc}")
                self.var_mon.set(f"收尾异常（窗口应仍在）: {exc}")
            except Exception:  # noqa: BLE001
                pass

    def _safe_view_selected(self) -> None:
        try:
            self._view_selected_silent()
        except Exception as exc:  # noqa: BLE001
            self._append(f"# WARN view: {exc}")

    def _safe_auto_save_and_analyze(self) -> None:
        try:
            self._auto_save_dump()
        except Exception as exc:  # noqa: BLE001
            self._append(f"# ERR auto-save: {exc}")
        try:
            self._analyze_last_segment()
        except Exception as exc:  # noqa: BLE001
            self._append(f"# ERR analyze: {exc}")
            self.var_mon.set(f"分析异常: {exc}")

    def _auto_save_and_analyze(self) -> None:
        self._safe_auto_save_and_analyze()

    def _analyze_last_segment(self) -> None:
        rows = self.dump_rows or (self.archive[0].samples if self.archive else [])
        if not rows:
            return
        rpms = [abs(float(r[1])) for r in rows]
        t0 = float(rows[0][0])
        t1 = float(rows[-1][0])
        dur = max(0.0, (t1 - t0) / 1000.0)
        peak = max(rpms) if rpms else 0.0
        mean = sum(rpms) / len(rpms) if rpms else 0.0
        idx0 = int(rows[0][4]) if len(rows[0]) > 4 else 0
        idx1 = int(rows[-1][4]) if len(rows[-1]) > 4 else idx0
        d_idx = abs(idx1 - idx0)
        revs_est = 0.0
        for i in range(1, len(rows)):
            dt = (float(rows[i][0]) - float(rows[i - 1][0])) / 1000.0
            if dt <= 0:
                continue
            revs_est += abs(float(rows[i][1])) / 60.0 * dt
        # 0 RPM 也是有效点
        n_zero = sum(1 for r in rows if abs(float(r[1])) < 1e-6)
        warn = ""
        if len(rows) < 200 or dur < 0.2:
            warn = (
                "\n\n[注意] 点数偏少（直采1s@2kHz 应≈2000点）。"
                "这与「转速为0」无关：0也是数据；偏少=回传不完整。"
            )
        summary = (
            f"分析: {len(rows)}点(其中0RPM={n_zero}) · {dur:.3f}s · 峰值|{peak:.0f}|RPM · "
            f"均值|{mean:.0f}| · ΔI圈≈{d_idx} · 积分≈{revs_est:.2f}圈"
        )
        self.var_info.set(summary)
        self.var_mon.set(f"自动完成 · {len(rows)}点 · 峰值 {peak:.0f} · ≈{revs_est:.2f}圈")
        self._append(f"# ANALYZE {summary}")
        # 非阻塞提示：避免 modal 对话框叠在重绘上导致窗口退出
        self.after(
            50,
            lambda: self._show_analyze_info(
                len(rows), dur, peak, mean, d_idx, revs_est, n_zero, warn
            ),
        )

    def _show_analyze_info(
        self,
        n: int,
        dur: float,
        peak: float,
        mean: float,
        d_idx: int,
        revs_est: float,
        n_zero: int,
        warn: str,
    ) -> None:
        try:
            messagebox.showinfo(
                "自动分析完成",
                f"点数: {n}（其中转速≈0 的点: {n_zero}，同属有效数据）\n"
                f"时长: {dur:.3f} s\n"
                f"峰值 |RPM|: {peak:.1f}\n"
                f"均值 |RPM|: {mean:.1f}\n"
                f"I 圈增量: {d_idx}\n"
                f"转速积分估圈: {revs_est:.2f}\n"
                f"{warn}\n"
                f"已保存到 pc/logs/ 目录",
            )
        except Exception as exc:  # noqa: BLE001
            self._append(f"# WARN analyze dialog: {exc}")

    def _send(self, cmd: str) -> None:
        if not self.link or not getattr(self.link, "is_open", False):
            messagebox.showwarning("未连接", "请先连接设备")
            return
        self._queue_cmd(cmd)

    def _rows_by_seg(self) -> dict[int, list]:
        rows = self.playback or list(self.dump_rows)
        by_seg: dict[int, list] = {}
        for row in rows:
            seg = int(row[3])
            by_seg.setdefault(seg, []).append(row)
        return by_seg

    def _refresh_seg_combo(self) -> None:
        by = self._rows_by_seg()
        labels = [str(s) for s in sorted(by.keys())]
        self.cmb_seg["values"] = labels
        if labels:
            self.cmb_seg.set(labels[0])
        else:
            self.cmb_seg.set("")

    def _dump(self) -> None:
        """② 仅拉取：短时 snap 走二进制（BLE/USB），与自动路径一致。"""
        if not self.link or not getattr(self.link, "is_open", False):
            messagebox.showwarning("未连接", "请先连接设备")
            return
        if self._is_sim_mode():
            self.dump_rows.clear()
            self.dump_index.clear()
            self.playback.clear()
            self._auto_split_after_dump = False
            cmd = self._dump_cmd()
            self.var_info.set(f"正在拉取… ({cmd})")
            self._send(cmd)
            return
        self._do_bin_pull()

    def _dump_and_split(self) -> None:
        if not self.link or not getattr(self.link, "is_open", False):
            messagebox.showwarning("未连接", "请先连接设备")
            return
        if self._is_sim_mode():
            self.dump_rows.clear()
            self.dump_index.clear()
            self.playback.clear()
            self._auto_split_after_dump = True
            cmd = self._dump_cmd()
            self.var_info.set(f"正在拉取并保存… ({cmd})")
            self._send(cmd)
            return
        self._auto_split_after_dump = True
        self._do_bin_pull()

    def _save_csv_split(self, folder: str | None = None) -> None:
        from pathlib import Path

        by_seg = self._rows_by_seg()
        if not by_seg and not self.dump_index:
            messagebox.showinfo("保存", "请先拉取数据")
            return
        if not folder:
            folder = filedialog.askdirectory(title="选择目录（每段 CSV + I事件）")
        if not folder:
            return
        out = Path(folder)
        out.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        for seg, seg_rows in sorted(by_seg.items()):
            path = out / f"abi_seg{seg:03d}_{stamp}.csv"
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["seg", "t_ms", "rpm", "dir", "index_n", "t_rel_ms", "unix_ms"])
                for row in seg_rows:
                    t, r, d, s, idxn = row[0], row[1], row[2], row[3], row[4]
                    t_rel = int(row[5]) if len(row) > 5 else 0
                    unix = int(row[6]) if len(row) > 6 else 0
                    w.writerow([int(s), int(t), f"{r:.2f}", d, int(idxn), t_rel, unix])
        # I 过零事件：按段拆文件，便于分析每转间隔/转速规律
        by_i: dict[int, list] = {}
        for row in self.dump_index:
            by_i.setdefault(int(row[5]), []).append(row)
        for seg, rows in sorted(by_i.items()):
            path = out / f"abi_seg{seg:03d}_I_{stamp}.csv"
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(
                    ["seg", "t_ms", "index_n", "rpm_ab", "rpm_i", "dt_ms", "t_rel_ms", "unix_ms"]
                )
                for t, idxn, rpm, rpm_i, dt, s, t_rel, unix in rows:
                    w.writerow(
                        [
                            int(s),
                            int(t),
                            int(idxn),
                            f"{rpm:.2f}",
                            f"{rpm_i:.2f}",
                            int(dt),
                            int(t_rel),
                            int(unix),
                        ]
                    )
        self.var_info.set(
            f"已保存 {len(by_seg)} 段转速 + {len(by_i)} 段I事件 → {out}"
        )
        self._append(f"# saved under {out} (D samples + I edges)")

    def _play_seg(self) -> None:
        by = self._rows_by_seg()
        if not by:
            messagebox.showinfo("回放", "请先拉取")
            return
        sel = self.cmb_seg.get().strip()
        if not sel:
            messagebox.showinfo("回放", "请选择段号")
            return
        try:
            seg = int(sel)
        except ValueError:
            return
        rows = by.get(seg)
        if not rows:
            messagebox.showinfo("回放", f"没有 seg={seg}")
            return
        self.playback = list(rows)
        self._play()

    def _play(self) -> None:
        if not self.playback and self.dump_rows:
            self.playback = list(self.dump_rows)
        if not self.playback:
            messagebox.showinfo("回放", "请先拉取板内 LOG")
            return
        self._stop_play()
        self.play_i = 0
        self.live_abs.clear()
        seg = int(self.playback[0][3]) if self.playback else 0

        def step() -> None:
            if self.play_i >= len(self.playback):
                self.var_info.set(f"回放结束 seg={seg}")
                self.play_job = None
                return
            row = self.playback[self.play_i]
            t_ms, rpm, direction, s, idxn = row[0], row[1], row[2], row[3], row[4]
            unix = int(row[6]) if len(row) > 6 else 0
            self.play_i += 1
            self.var_rpm.set(f"{rpm:.1f}")
            self.var_dir.set(
                "正转 +" if direction > 0 else ("反转 −" if direction < 0 else "静止")
            )
            if unix > 0:
                ts = datetime.fromtimestamp(unix / 1000.0).strftime("%H:%M:%S.%f")[:-3]
                self.var_revs.set(f"I圈数 {idxn} · {ts}")
            else:
                self.var_revs.set(f"I圈数 {idxn}")
            now = time.time()
            self.live_abs.append((now, rpm))
            while self.live_abs and self.live_abs[-1][0] - self.live_abs[0][0] > LIVE_WINDOW_S:
                self.live_abs.popleft()
            self._redraw()
            delay = 20
            if self.play_i < len(self.playback):
                dt = int(self.playback[self.play_i][0] - t_ms)
                delay = max(5, min(200, dt if dt > 0 else 20))
            self.var_info.set(f"回放 seg={s} {self.play_i}/{len(self.playback)}")
            self.play_job = self.after(delay, step)

        self.var_info.set(f"回放 seg={seg} · {len(self.playback)} 点…")
        step()

    def _stop_play(self) -> None:
        if self.play_job:
            self.after_cancel(self.play_job)
            self.play_job = None

    def _open_serial_log(self) -> None:
        """会话级串口全文 txt，便于把 log 发给朋友分析。"""
        try:
            SERIAL_LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = SERIAL_LOG_DIR / f"serial_{stamp}.txt"
            self._serial_log_fp = open(path, "a", encoding="utf-8", newline="\n")
            self._serial_log_path = path
            self._serial_log_fp.write(
                f"# serial log start {stamp} baud={BAUD}\n"
                f"# FW expect monitor-v19-snapdiag\n"
            )
            self._serial_log_fp.flush()
        except Exception:
            self._serial_log_fp = None
            self._serial_log_path = None

    def _tee_serial_log(self, s: str) -> None:
        fp = getattr(self, "_serial_log_fp", None)
        if not fp:
            return
        try:
            fp.write(s + "\n")
            fp.flush()
        except Exception:
            pass

    def _append(self, s: str) -> None:
        self._tee_serial_log(s)
        if "DUMP PROG" in s or s.startswith("# KEEPALIVE") or s.startswith("D,"):
            return
        # 关键记录事件：不过滤、不限流
        important = (
            "CAPTURE PROG" in s
            or "CAPTURE DONE" in s
            or "CAPTURE START" in s
            or "MONITOR" in s
            or "CONFIRM" in s
            or "RECORD" in s
            or "SALVAGE" in s
            or "AUTO DUMP" in s
            or "LOG DUMP" in s
            or "LOG empty" in s
            or "SNAP" in s
            or "ALIVE" in s
            or "HEX" in s
            or "BIN" in s
            or "D END" in s
            or "I END" in s
            or "DATA_START" in s
            or "DATA_END" in s
            or "ANALYZE" in s
            or "Guru" in s
            or "panic" in s
            or "FW=" in s
        )
        if "CAPTURE PROG" in s or "CAPTURE DONE" in s or "CAPTURE START" in s or important:
            try:
                self.log_txt.insert(tk.END, s + "\n")
                self.log_txt.see(tk.END)
            except Exception:  # noqa: BLE001
                pass
            if important and "CAPTURE PROG" not in s:
                return
            if "CAPTURE PROG" in s or "CAPTURE DONE" in s or "CAPTURE START" in s:
                return
        # 日志限流：每秒最多 8 行
        now = time.monotonic()
        if not hasattr(self, "_append_bucket"):
            self._append_bucket = 0.0
            self._append_n = 0
        if now - self._append_bucket >= 1.0:
            self._append_bucket = now
            self._append_n = 0
        self._append_n += 1
        if self._append_n > 8:
            return
        try:
            self.log_txt.insert(tk.END, s + "\n")
            self._log_lines += 1
            if self._log_lines > 200:
                self.log_txt.delete("1.0", "80.0")
                self._log_lines = 120
            self.log_txt.see(tk.END)
        except Exception:  # noqa: BLE001
            pass


    def _open_snap_bin(self) -> None:
        """打开 SD/U盘拷出的 snap_*.bin → 列表 + 曲线 + 可过滤。"""
        path = filedialog.askopenfilename(
            title="打开 snap_*.bin",
            filetypes=[
                ("Snap binary", "*.bin"),
                ("All files", "*.*"),
            ],
        )
        if not path:
            return
        try:
            raw = Path(path).read_bytes()
            n, hz, rows = parse_snap_file(raw)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("打开失败", str(exc))
            return
        self._snap_raw_rows = list(rows)
        self.dump_rows = list(rows)
        self.dump_index.clear()
        self.playback = list(rows)
        name = Path(path).name
        dur = 0.0
        if len(rows) >= 2:
            dur = (float(rows[-1][0]) - float(rows[0][0])) / 1000.0
        peak = max((abs(float(r[1])) for r in rows), default=0.0)
        label = f"文件 {name} · {n}点@{hz}Hz · {dur:.2f}s · peak|{peak:.0f}|"
        self.archive.insert(
            0,
            SegmentRecord(label=label, board_seg=1, samples=list(rows), index_ev=[]),
        )
        self._refresh_archive_list()
        self.lst_archive.selection_clear(0, tk.END)
        self.lst_archive.selection_set(0)
        self.lst_archive.activate(0)
        self.var_t0.set("0.0")
        self.var_t1.set(f"{max(dur, 0.05):.3f}")
        self.var_rpm_min.set("10")
        self.var_drop_zero.set(False)
        self.var_i0.set("auto")
        self.var_plot_y.set("rpm")
        try:
            if not self._plot_enabled.get():
                self._plot_enabled.set(True)
                self._toggle_plot()
        except Exception:  # noqa: BLE001
            pass
        self._view_selected_silent()
        self._edit_i0_from_first()
        self.var_mon.set(f"已打开 {name}")
        self.var_info.set(f"打开 {name}: {n}点 · {dur:.3f}s · 峰值|{peak:.0f}|RPM")
        self.var_edit_info.set(f"原文件 {n} 点 — 用 t0/t1 裁剪，或过滤 |RPM|<{self.var_rpm_min.get()}")
        self._append(f"# OPEN SNAP {path} n={n} hz={hz} bytes={len(raw)}")

    def _edit_working_rows(self) -> list | None:
        rec = self._selected_archive()
        if rec is not None and rec.samples:
            return list(rec.samples)
        if self.dump_rows:
            return list(self.dump_rows)
        return None

    def _read_t01(self) -> tuple[float, float]:
        try:
            t0 = float(self.var_t0.get().strip())
            t1 = float(self.var_t1.get().strip())
        except ValueError:
            t0, t1 = 0.0, 999.0
        return t0, t1

    def _read_i0_len(self, rows: list) -> tuple[int, float]:
        try:
            length = float(self.var_len_i.get().strip() or "1")
        except ValueError:
            length = 1.0
        s = (self.var_i0.get() or "auto").strip().lower()
        if s in ("", "auto"):
            i0 = index_baseline(rows) if index_baseline else 0
        else:
            try:
                i0 = int(float(s))
            except ValueError:
                i0 = index_baseline(rows) if index_baseline else 0
        return i0, length

    def _commit_edit_rows(self, rows: list, note: str) -> None:
        if not rows:
            messagebox.showinfo("剪辑", "结果为空，未改动")
            return
        rec = self._selected_archive()
        dur = 0.0
        if len(rows) >= 2:
            dur = (float(rows[-1][0]) - float(rows[0][0])) / 1000.0
        peak = max((abs(float(r[1])) for r in rows), default=0.0)
        i0, length = self._read_i0_len(rows)
        series = distance_series(rows, i0=i0, length_per_i=length) if distance_series else []
        s_end = series[-1][2] if series else 0.0
        label = f"{note} · {len(rows)}点 · {dur:.2f}s · peak|{peak:.0f}| · S≈{s_end:.2f}"
        self.dump_rows = list(rows)
        self.playback = list(rows)
        idx = 0
        if rec is not None:
            sel = self.lst_archive.curselection()
            idx = int(sel[0]) if sel else 0
            self.archive[idx] = SegmentRecord(
                label=label, board_seg=rec.board_seg, samples=list(rows), index_ev=[]
            )
        else:
            self.archive.insert(
                0, SegmentRecord(label=label, board_seg=1, samples=list(rows), index_ev=[])
            )
            idx = 0
        self._refresh_archive_list()
        self.lst_archive.selection_clear(0, tk.END)
        self.lst_archive.selection_set(idx)
        self.view_seg = self.archive[idx]
        self.plot_mode = "segment"
        self.var_t0.set("0.0")
        self.var_t1.set(f"{max(dur, 0.05):.3f}")
        self.var_edit_info.set(
            f"{note}: {len(rows)}点 · {dur:.3f}s · I0={i0} · 每I={length} → S末={s_end:.3f}"
        )
        self.var_info.set(self.var_edit_info.get())
        self._append(f"# EDIT {note} n={len(rows)} I0={i0} len/I={length} S_end={s_end:.3f}")
        self._refresh_edit_plot()

    def _edit_keep_range(self) -> None:
        rows = self._edit_working_rows()
        if not rows or keep_time_range is None:
            messagebox.showinfo("剪辑", "请先打开/选择一段")
            return
        t0, t1 = self._read_t01()
        self._commit_edit_rows(keep_time_range(rows, t0, t1), f"保留{t0:.3f}~{t1:.3f}s")

    def _edit_delete_range(self) -> None:
        rows = self._edit_working_rows()
        if not rows or delete_time_range is None:
            messagebox.showinfo("剪辑", "请先打开/选择一段")
            return
        t0, t1 = self._read_t01()
        self._commit_edit_rows(delete_time_range(rows, t0, t1), f"删{t0:.3f}~{t1:.3f}s")

    def _edit_split_at_t0(self) -> None:
        rows = self._edit_working_rows()
        if not rows or split_at_time is None:
            messagebox.showinfo("剪辑", "请先打开/选择一段")
            return
        t0, _ = self._read_t01()
        left, right = split_at_time(rows, t0)
        if not left or not right:
            messagebox.showinfo("拆分", "拆分后有一段为空，换个 t0")
            return
        # 当前段变左段，右段插入列表
        self._commit_edit_rows(left, f"拆分左≤{t0:.3f}s")
        dur = 0.0
        if len(right) >= 2:
            dur = (float(right[-1][0]) - float(right[0][0])) / 1000.0
        peak = max((abs(float(r[1])) for r in right), default=0.0)
        label = f"拆分右>{t0:.3f}s · {len(right)}点 · {dur:.2f}s · peak|{peak:.0f}|"
        self.archive.insert(
            0, SegmentRecord(label=label, board_seg=2, samples=list(right), index_ev=[])
        )
        self._refresh_archive_list()
        self.var_edit_info.set(f"已拆成两段：左{len(left)} + 右{len(right)}（右段在列表顶）")

    def _apply_rpm_cut(self) -> None:
        """过滤掉 |RPM| < 阈值 的点。"""
        rows = self._edit_working_rows()
        if not rows or filter_by_rpm is None:
            messagebox.showinfo("过滤", "请先打开/选择一段")
            return
        try:
            thr = float(self.var_rpm_min.get().strip() or "0")
        except ValueError:
            thr = 0.0
        # 语义：小于 thr 的丢掉 → 保留 abs(rpm) >= thr
        out = filter_by_rpm(rows, rpm_min=thr, drop_zero=bool(self.var_drop_zero.get()))
        self._commit_edit_rows(out, f"滤|RPM|<{thr}")

    def _edit_i0_from_first(self) -> None:
        rows = self._edit_working_rows()
        if not rows:
            return
        i0 = index_baseline(rows) if index_baseline else int(rows[0][4])
        self.var_i0.set(str(i0))
        self.var_edit_info.set(f"I0={i0}（首点 index_n）· S=(I-I0)×每I长度")
        self._refresh_edit_plot()

    def _refresh_edit_plot(self) -> None:
        rec = self._selected_archive()
        if rec is None:
            return
        self.view_seg = rec
        self.plot_mode = "segment"
        self._plot_segment(rec, None, None)

    def _restore_snap_raw(self) -> None:
        if not self._snap_raw_rows:
            messagebox.showinfo("恢复", "没有打开过的完整 bin（请先打开 snap.bin）")
            return
        rows = list(self._snap_raw_rows)
        self.dump_rows = rows
        self.playback = list(rows)
        dur = 0.0
        if len(rows) >= 2:
            dur = (float(rows[-1][0]) - float(rows[0][0])) / 1000.0
        peak = max((abs(float(r[1])) for r in rows), default=0.0)
        label = f"文件(全点) · {len(rows)}点 · {dur:.2f}s · peak|{peak:.0f}|"
        self.archive.insert(
            0, SegmentRecord(label=label, board_seg=1, samples=list(rows), index_ev=[])
        )
        self._refresh_archive_list()
        self.lst_archive.selection_set(0)
        self.var_t0.set("0.0")
        self.var_t1.set(f"{max(dur, 0.05):.3f}")
        self.var_plot_y.set("rpm")
        self._edit_i0_from_first()
        self._view_selected_silent()
        self.var_info.set(f"已恢复全点 {len(rows)}")
        self.var_edit_info.set(f"已恢复原文件 {len(rows)} 点")

    def _export_filtered_csv(self) -> None:
        rec = self._selected_archive()
        rows = rec.samples if rec else (self.dump_rows or self.playback)
        if not rows:
            messagebox.showinfo("导出", "没有可导出的点")
            return
        i0, length = self._read_i0_len(rows)
        series = distance_series(rows, i0=i0, length_per_i=length) if distance_series else []
        s_rpm = (
            integrate_rpm_distance(rows, length_per_rev=length) if integrate_rpm_distance else []
        )
        path = filedialog.asksaveasfilename(
            title="导出 CSV（含 S）",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile=f"snap_{len(rows)}_S.csv",
        )
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(
                    [
                        "t_ms",
                        "t_rel_s",
                        "rpm",
                        "dir",
                        "index_n",
                        "i_rel",
                        "S_I",
                        "S_rpm_integ",
                        "I0",
                        "length_per_I",
                    ]
                )
                for i, r in enumerate(rows):
                    t_ms = float(r[0])
                    tr = series[i][0] if i < len(series) else 0.0
                    i_rel = series[i][4] if i < len(series) else 0
                    s_i = series[i][2] if i < len(series) else 0.0
                    s_r = s_rpm[i][2] if i < len(s_rpm) else 0.0
                    w.writerow(
                        [
                            f"{t_ms:.3f}",
                            f"{tr:.6f}",
                            f"{float(r[1]):.2f}",
                            int(r[2]),
                            int(r[4]),
                            i_rel,
                            f"{s_i:.6f}",
                            f"{s_r:.6f}",
                            i0,
                            length,
                        ]
                    )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("导出失败", str(exc))
            return
        self.var_info.set(f"已导出 {len(rows)} 点(+S) → {path}")
        self._append(f"# EXPORT CSV {path} n={len(rows)} I0={i0} len/I={length}")

    def _archive_current(self) -> None:
        """把当前 dump_rows 按 seg 归档到可浏览列表。"""
        if not self.dump_rows:
            return
        by_seg: dict[int, list] = {}
        for row in self.dump_rows:
            by_seg.setdefault(int(row[3]), []).append(row)
        by_i: dict[int, list] = {}
        for row in self.dump_index:
            by_i.setdefault(int(row[5]), []).append(row)
        stamp = datetime.now().strftime("%H:%M:%S")
        for seg, rows in sorted(by_seg.items()):
            dur = 0.0
            if len(rows) >= 2:
                dur = (rows[-1][0] - rows[0][0]) / 1000.0
            ie = by_i.get(seg, [])
            label = f"#{len(self.archive)+1} 板seg{seg} · {len(rows)}点 · {dur:.2f}s · I{len(ie)} · {stamp}"
            self.archive.insert(
                0,
                SegmentRecord(label=label, board_seg=seg, samples=list(rows), index_ev=list(ie)),
            )
        self._refresh_archive_list()
        if self.archive:
            self.lst_archive.selection_clear(0, tk.END)
            self.lst_archive.selection_set(0)
            self.lst_archive.activate(0)

    def _refresh_archive_list(self) -> None:
        if not hasattr(self, "lst_archive"):
            return
        self.lst_archive.delete(0, tk.END)
        for rec in self.archive:
            self.lst_archive.insert(tk.END, rec.label)

    def _selected_archive(self) -> SegmentRecord | None:
        sel = self.lst_archive.curselection()
        if not sel:
            return None
        i = int(sel[0])
        if 0 <= i < len(self.archive):
            return self.archive[i]
        return None

    def _delete_selected(self) -> None:
        sel = self.lst_archive.curselection()
        if not sel:
            return
        i = int(sel[0])
        del self.archive[i]
        self._refresh_archive_list()
        self.var_info.set(f"已删除，剩余 {len(self.archive)} 段")

    def _toggle_cap_panel(self) -> None:
        """展开/收起长采区域（默认收起）。"""
        self._cap_expanded = not getattr(self, "_cap_expanded", False)
        if self._cap_expanded:
            self.cap_box.pack(fill=tk.X, pady=(4, 0))
            self.var_cap_toggle.set("▼ 长采（点击收起）")
        else:
            self.cap_box.pack_forget()
            self.var_cap_toggle.set("▶ 长采（点击展开）")

    def _toggle_flt_panel(self) -> None:
        """展开/收起动态滤波设置（默认收起 → 全自动）。"""
        self._flt_expanded = not getattr(self, "_flt_expanded", False)
        if self._flt_expanded:
            self.flt_box.pack(fill=tk.X, pady=(4, 0))
            self.var_flt_toggle.set("▼ 动态滤波设置（点击收起）")
        else:
            self.flt_box.pack_forget()
            self.var_flt_toggle.set("▶ 动态滤波设置（点击展开）")

    def _parse_hex(self, s: str) -> int | None:
        try:
            return int(s.strip(), 0)
        except ValueError:
            return None

    def _apply_flt(self) -> None:
        """把面板值逐项下发（FILT …）：范围校验在固件端，成功后回显。"""
        if not self.link:
            messagebox.showwarning("滤波", "未连接")
            return
        try:
            self._queue_cmd(f"FILT EMA {int(self.var_flt_ema.get())}")
            self._queue_cmd(f"FILT DPOS {int(self.var_flt_dpos.get())}")
            self._queue_cmd(f"FILT ZERO {int(self.var_flt_zero.get())}")
            self._queue_cmd(f"FILT RATE {int(self.var_flt_rate.get())}")
            lo = self._parse_hex(self.var_flt_lo.get())
            hi = self._parse_hex(self.var_flt_hi.get())
            if lo is None or hi is None:
                messagebox.showwarning("滤波", "范围需为 0..15 整数")
                return
            self._queue_cmd(f"FILT RANGE {lo} {hi}")
            for i, v in enumerate(self.var_icf):
                val = self._parse_hex(v.get())
                if val is None:
                    messagebox.showwarning("滤波", f"icf[{i}] 需 0..15（可用 0x 前缀）")
                    return
                self._queue_cmd(f"FILT ICF {i} {val}")
            for i, v in enumerate(self.var_bnd):
                b = int(v.get())
                self._queue_cmd(f"FILT BND {i} {b}")
            self._queue_cmd("FILT SHOW")
            self.var_info.set("动态滤波参数已下发（回显见日志）")
        except ValueError:
            messagebox.showwarning("滤波", "参数含非法数字")
            self.var_info.set("滤波下发失败：参数非数字")

    def _open_curve_studio(self) -> None:
        """独立影视剪辑风格窗口：RPM / S积分 / 选区加速度。"""
        rec = self._selected_archive()
        rows = rec.samples if rec else (self.dump_rows or self._snap_raw_rows)
        if not rows:
            messagebox.showinfo("曲线工作室", "请先打开 snap.bin 或选择一段")
            return
        try:
            from curve_studio import open_curve_studio
        except ImportError as exc:
            messagebox.showerror("曲线工作室", f"无法加载: {exc}")
            return

        def _commit(new_rows: list) -> None:
            if rec is None:
                self.dump_rows = list(new_rows)
                return
            sel = self.lst_archive.curselection()
            idx = int(sel[0]) if sel else 0
            dur = 0.0
            if len(new_rows) >= 2:
                dur = (float(new_rows[-1][0]) - float(new_rows[0][0])) / 1000.0
            peak = max((abs(float(r[1])) for r in new_rows), default=0.0)
            label = f"工作室写回 · {len(new_rows)}点 · {dur:.2f}s · peak|{peak:.0f}|"
            self.archive[idx] = SegmentRecord(
                label=label, board_seg=rec.board_seg, samples=list(new_rows), index_ev=[]
            )
            self._refresh_archive_list()
            self.lst_archive.selection_set(idx)
            self.dump_rows = list(new_rows)
            self.playback = list(new_rows)
            self.var_info.set(label)

        title = rec.label if rec else f"snap {len(rows)}点"
        open_curve_studio(self, rows, title=title, on_commit=_commit)
        self.var_info.set("已打开独立曲线工作室窗口")

    def _view_selected(self) -> None:
        rec = self._selected_archive()
        if rec is None:
            messagebox.showinfo("浏览", "请先选择一段已接收数据")
            return
        self._stop_play()
        self.view_seg = rec
        self.plot_mode = "segment"
        self.var_t0.set("0.0")
        self.var_t1.set(f"{max(rec.duration_s, 0.05):.3f}")
        try:
            if not self._plot_enabled.get():
                self._plot_enabled.set(True)
                self._toggle_plot()
        except Exception:  # noqa: BLE001
            pass
        self._plot_segment(rec, None, None)
        self.var_info.set(f"主窗预览 {rec.label}")

    def _play_archived(self) -> None:
        rec = self._selected_archive()
        if rec is None:
            messagebox.showinfo("回放", "请先选择一段")
            return
        self.view_seg = rec
        self.plot_mode = "segment"
        self.playback = list(rec.samples)
        self._plot_segment(rec, None, None)
        self._play()

    def _preset_window(self, t0: float, t1: float) -> None:
        self.var_t0.set(f"{t0:.3f}")
        self.var_t1.set(f"{t1:.3f}")
        self._apply_time_window()

    def _window_full(self) -> None:
        rec = self.view_seg or self._selected_archive()
        if rec is None:
            messagebox.showinfo("窗口", "请先查看一段数据")
            return
        self.var_t0.set("0.0")
        self.var_t1.set(f"{max(rec.duration_s, 0.05):.3f}")
        self._apply_time_window()

    def _apply_time_window(self) -> None:
        rec = self.view_seg or self._selected_archive()
        if rec is None:
            messagebox.showinfo("窗口", "请先选择并查看一段数据")
            return
        try:
            t0 = float(self.var_t0.get().strip())
            t1 = float(self.var_t1.get().strip())
        except ValueError:
            messagebox.showerror("窗口", "时间格式错误，例如 0.0 与 0.3")
            return
        self.view_seg = rec
        self.plot_mode = "segment"
        self._plot_segment(rec, t0, t1)

    def _live_plot_mode(self) -> None:
        self.plot_mode = "live"
        self.view_seg = None
        self.var_info.set("已切回实时曲线")
        self._redraw()

    def _plot_segment(
        self, rec: SegmentRecord, t0: float | None, t1: float | None
    ) -> None:
        if not self._has_plot or self.chart is None or not rec.samples:
            return
        mode = (self.var_plot_y.get() if hasattr(self, "var_plot_y") else "rpm") or "rpm"
        i0, length = self._read_i0_len(rec.samples)
        t_base = rec.samples[0][0]
        xs = [(r[0] - t_base) / 1000.0 for r in rec.samples]
        ylabel = "|RPM|"
        if mode == "s_i" and distance_series:
            series = distance_series(rec.samples, i0=i0, length_per_i=length)
            ys = [abs(float(p[2])) for p in series]
            ylabel = f"|S| (I0={i0}, ×{length})"
        elif mode == "s_rpm" and integrate_rpm_distance:
            series = integrate_rpm_distance(rec.samples, length_per_rev=length)
            ys = [abs(float(p[2])) for p in series]
            ylabel = f"|S| ∫rpm (×{length}/rev)"
        else:
            ys = [abs(float(r[1])) for r in rec.samples]
            ylabel = "|RPM|"
        if t0 is None:
            t0 = 0.0
        if t1 is None:
            t1 = xs[-1] if xs else 0.05
        if t1 < t0:
            t0, t1 = t1, t0
        if t1 - t0 < 0.01:
            t1 = t0 + 0.01
        y_win = [y for x, y in zip(xs, ys) if t0 <= x <= t1]
        if mode.startswith("s"):
            ymax = (max(y_win) * 1.1 + 1e-6) if y_win else 1.0
            ymin = 0.0
        else:
            ymax = (max(y_win) * 1.15 + 10) if y_win else 50.0
            ymin = 0.0
        self.chart.set_data(
            xs,
            ys,
            xlim=(t0, t1),
            ylim=(ymin, max(ymax, 1e-3)),
            title=rec.label,
            xlabel="t (s) 相对段起点",
            ylabel=ylabel,
        )
        # 仅在显式窗口操作时改 t0/t1；刷新 S 图时保留用户输入
        if not hasattr(self, "_plot_keep_t") or not self._plot_keep_t:
            self.var_t0.set(f"{t0:.3f}")
            self.var_t1.set(f"{t1:.3f}")

    def _redraw(self) -> None:
        if not self._has_plot or self.chart is None:
            return
        if self.plot_mode == "segment":
            return
        if not self.live_abs:
            return
        t0 = self.live_abs[0][0]
        xs = [t - t0 for t, _ in self.live_abs]
        ys = [abs(r) for _, r in self.live_abs]
        self.chart.set_data(xs, ys, title="live", xlabel="t (s)")

    def _heartbeat(self) -> None:
        self._hb = (self._hb + 1) % 10000
        self.var_hb.set(f"UI 10Hz · {self._hb}")
        # 长采倒计时（本地 + 板报 n）
        if self._cap_end_mono is not None:
            rem = max(0.0, self._cap_end_mono - time.monotonic())
            rem_s = int(rem + 0.999)
            elapsed = max(0, self._cap_total_s - rem_s)
            self.var_cap.set(
                f"长采中 剩余 {rem_s}s / 共{self._cap_total_s}s · 已过 {elapsed}s · "
                f"板内 n={self._cap_board_n}（可启停/等待，倒计时不停）"
            )
            if rem <= 0:
                self._cap_end_mono = None
                self.var_cap.set(
                    f"长采时间到 · 板内 n≈{self._cap_board_n} — 0.5s 后自动拉取…"
                )
                # 到时自动上传（发 CAPTURE DUMP）
                if self.link and getattr(self.link, "is_open", False):
                    self.after(500, self._cap_dump_save)
        self.after(UI_PERIOD_MS, self._heartbeat)

    def _poll(self) -> None:
        try:
            latest_l: str | None = None
            # 监控 DUMP 时必须吃光 D 行，旧 d_budget=20 会丢点且可能永远等不到结束
            d_budget = 10000 if (self._ui_dumping or self._board_auto_push) else 200
            c_budget = 8000 if self._cap_dumping else 2000
            n = 0
            d_got_this = 0
            while n < 12000:
                n += 1
                try:
                    kind, payload = self.out_q.get_nowait()
                except queue.Empty:
                    break
                if kind == "__error__":
                    self.var_status.set(f"错误: {payload}")
                    self._append(str(payload))
                elif kind == "__bindump__":
                    self._set_ble_bin_progress(
                        len(payload) if isinstance(payload, (bytes, bytearray)) else 0,
                        len(payload) if isinstance(payload, (bytes, bytearray)) else 0,
                        100,
                    )
                    self._handle_bindump(payload)
                elif kind == "__bin_prog__":
                    got, exp, pct = payload
                    self._set_ble_bin_progress(int(got), int(exp or 0), int(pct))
                elif kind == "__dump_batch__":
                    for line in payload:
                        self._on_dump_sample_line(str(line))
                    self.var_dump_prog.set(
                        f"回传中… 已收 {len(self.dump_rows)} / {self._dump_expect or '?'}"
                    )
                elif kind == "__raw__":
                    line = str(payload)
                    self._tee_serial_log(line)
                    if line.startswith("L,"):
                        if not self._ui_dumping:
                            latest_l = line
                    elif line.startswith("C,"):
                        if c_budget <= 0:
                            continue
                        c_budget -= 1
                        self._on_cap_line(line)
                    elif line.startswith("D,") and not line.startswith("D END"):
                        if d_budget <= 0:
                            # 绝不丢弃：放回队列头处理不了，改为强制处理
                            self._on_dump_sample_line(line)
                            d_got_this += 1
                            continue
                        d_budget -= 1
                        self._on_dump_sample_line(line)
                        d_got_this += 1
                    else:
                        self._on_line(line)
            if d_got_this and (self._ui_dumping or self._board_auto_push):
                self.var_dump_prog.set(
                    f"自动接收中… 已收 {len(self.dump_rows)} 点"
                )
                self.var_mon.set(f"自动接收中… {len(self.dump_rows)} 点")
            # 卡死保护：推送开始后长时间无结束标记
            if self._ui_dumping or self._board_auto_push:
                if getattr(self, "_dump_watch_t0", None) is None:
                    self._dump_watch_t0 = time.monotonic()
                    self._dump_watch_n = len(self.dump_rows)
                else:
                    now = time.monotonic()
                    if len(self.dump_rows) != getattr(self, "_dump_watch_n", -1):
                        self._dump_watch_t0 = now
                        self._dump_watch_n = len(self.dump_rows)
                    elif now - self._dump_watch_t0 > (
                        120.0
                        if (
                            getattr(self, "_awaiting_bindump", False)
                            and self._is_ble_mode()
                        )
                        else 8.0
                    ):
                        # 等二进制时不要改走 LOG DUMP 文本路径（BLE BIN 可达数分钟）
                        if getattr(self, "_awaiting_bindump", False):
                            self._append(
                                "# WARN BIN stall — still waiting "
                                + ("BLE hex/CRC" if self._is_ble_mode() else "magic/CRC")
                            )
                            self._dump_watch_t0 = now
                        else:
                            n = len(self.dump_rows)
                            exp = int(getattr(self, "_dump_expect", 0) or 0)
                            retries = int(getattr(self, "_dump_retry", 0) or 0)
                            # 残片（如 9 点）禁止收尾：重拉一次
                            if n < max(100, exp // 5 if exp else 100) and retries < 2:
                                self._dump_retry = retries + 1
                                self._append(
                                    f"# WARN dump stall n={n} expect={exp} → retry #{self._dump_retry}"
                                )
                                self._dump_watch_t0 = time.monotonic()
                                self._ui_dumping = False
                                self._board_auto_push = False
                                self._read_scheduled = False
                                self.after(300, self._do_auto_pull)
                            elif n < 100:
                                self._append(
                                    f"# WARN dump stall short n={n} — still finish+save "
                                    f"(0RPM is valid; empty=0 points only)"
                                )
                                self._dump_watch_t0 = now + 999
                                self._force_finish_dump = True
                                self.after(50, self._finish_auto_recv)
                            else:
                                self._append(
                                    f"# WARN dump stall → force finish n={n}"
                                )
                                self._dump_watch_t0 = now + 999
                                self._force_finish_dump = True
                                self.after(50, self._finish_auto_recv)
            else:
                self._dump_watch_t0 = None
                self._awaiting_bindump = False
            # 每拍最多处理 1 条 L（配合 100ms poll = 10Hz）
            if latest_l is not None:
                now = time.monotonic()
                if now - self._last_ui_l >= (1.0 / UI_HZ) - 0.005:
                    self._last_ui_l = now
                    self._on_line(latest_l)
        except Exception as exc:  # noqa: BLE001
            try:
                self._append(f"# UI poll err: {exc}")
            except Exception:  # noqa: BLE001
                pass
        self.after(UI_PERIOD_MS, self._poll)

    def _on_cap_line(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) < 5:
            return
        try:
            self.cap_rows.append(
                (
                    int(float(parts[1])),
                    float(parts[2]),
                    int(float(parts[3])),
                    int(float(parts[4])),
                )
            )
            if len(self.cap_rows) % 2000 == 0:
                self.var_cap.set(f"长采接收中… {len(self.cap_rows)} 点")
        except ValueError:
            pass

    def _on_dump_sample_line(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) < 5:
            return
        try:
            # 短帧: D,t_ms,rpm,dir,seg[,index_n]
            if len(parts) in (5, 6):
                idxn = int(float(parts[5])) if len(parts) >= 6 else 0
                self.dump_rows.append(
                    (
                        float(parts[1]),
                        float(parts[2]),
                        int(float(parts[3])),
                        int(float(parts[4])),
                        idxn,
                        0,
                        0,
                    )
                )
                if not self._ui_dumping:
                    self._ui_dumping = True
                    self._board_auto_push = True
                return
            # 旧长帧: D,idx,t_ms,rpm,dir,seg,...
            seg = int(float(parts[5])) if len(parts) >= 6 else 1
            idxn = int(float(parts[6])) if len(parts) >= 7 else 0
            t_rel = int(float(parts[7])) if len(parts) >= 8 else 0
            unix = int(float(parts[8])) if len(parts) >= 9 else 0
            self.dump_rows.append(
                (float(parts[2]), float(parts[3]), int(float(parts[4])), seg, idxn, t_rel, unix)
            )
        except ValueError:
            pass

    def _on_line(self, line: str) -> None:
        if line.startswith("L,"):
            parts = line.split(",")
            if len(parts) < 3:
                return
            try:
                # 兼容两种格式：
                # 短帧(BLE): L,rpm,dir,armed,phase,remain,log_n
                # 长帧(USB): L,t,rpm,dir,armed,log_n,drop,hz,counts,seg,phase,...
                if len(parts) <= 8 and "." in parts[1]:
                    rpm = float(parts[1])
                    direction = int(float(parts[2])) if len(parts) > 2 else 0
                    armed = int(float(parts[3])) if len(parts) > 3 else 0
                    phase = int(float(parts[4])) if len(parts) > 4 else 0
                    remain = int(float(parts[5])) if len(parts) > 5 else 0
                    log_n = int(float(parts[6])) if len(parts) > 6 else 0
                    log_drop = 0
                    meas_hz = 0.0
                    counts = 0
                    seg = 0
                    pool_n = 0
                    revs = 0.0
                    segs = 0
                    revs_abi = revs_abs = 0.0
                    index_n = index_signed = 0
                    t_rel = 0
                    unix_ms = 0
                else:
                    rpm = float(parts[2])
                    direction = int(float(parts[3])) if len(parts) > 3 else 0
                    armed = int(float(parts[4])) if len(parts) > 4 else 0
                    log_n = int(float(parts[5])) if len(parts) > 5 else 0
                    log_drop = int(float(parts[6])) if len(parts) > 6 else 0
                    meas_hz = float(parts[7]) if len(parts) > 7 else 0.0
                    counts = int(float(parts[8])) if len(parts) > 8 else 0
                    seg = int(float(parts[9])) if len(parts) > 9 else 0
                    phase = int(float(parts[10])) if len(parts) > 10 else 0
                    pool_n = int(float(parts[11])) if len(parts) > 11 else 0
                    revs = float(parts[12]) if len(parts) > 12 else 0.0
                    remain = int(float(parts[13])) if len(parts) > 13 else 0
                    segs = int(float(parts[14])) if len(parts) > 14 else 0
                    revs_abi = float(parts[15]) if len(parts) > 15 else 0.0
                    revs_abs = float(parts[16]) if len(parts) > 16 else 0.0
                    index_n = int(float(parts[17])) if len(parts) > 17 else 0
                    index_signed = int(float(parts[18])) if len(parts) > 18 else 0
                    t_rel = int(float(parts[19])) if len(parts) > 19 else 0
                    unix_ms = int(float(parts[20])) if len(parts) > 20 else 0
            except ValueError:
                return
            self.var_rpm.set(f"{rpm:.1f}")
            self.var_dir.set(
                "正转 +" if direction > 0 else ("反转 −" if direction < 0 else "静止")
            )
            # 监控记录中 UI 只做极简更新
            if armed and phase in (1, 2):
                if phase == 1:
                    self._set_mon_ui("staging")
                    self.var_mon.set("记录中·确认转动…")
                else:
                    self._set_mon_ui("recording")
                    self.var_mon.set(f"记录中·已确认 · 剩余约 {remain} ms")
                self.var_log.set(f"log_n={log_n}")
                return
            # 轻量刷新：圈数/状态每拍最多一次（已由 10Hz poll 保证）
            self.var_revs.set(
                f"I圈数 {index_n}（有符号 {index_signed}） · A/B {revs_abi:.2f} / 路程 {revs_abs:.2f}"
                f" · counts={counts}"
            )
            if not armed:
                if self._mon_state not in ("dumping", "done"):
                    self._set_mon_ui("idle")
                if abs(rpm) >= 20:
                    self.var_mon.set(
                        f"高速中 |RPM|≈{rpm:.0f} 但未武装 — 点「①开始监控」才会写入"
                    )
                else:
                    self.var_mon.set("仅实时显示 · 未记录（点开始监控才写入）")
            else:
                if self._mon_state not in ("staging", "recording", "dumping"):
                    self._set_mon_ui("armed")
                self.var_mon.set("已武装 · 等待 |RPM|>20")
            self.var_log.set(f"log_n={log_n} pool={pool_n} drop={log_drop} segs={segs}")
            if meas_hz > 0:
                self.var_hz.set(f"meas_hz≈{meas_hz:.0f}")
            if self.play_job is None and not self._ui_dumping and self._has_plot:
                now = time.time()
                self.live_abs.append((now, rpm))
                while self.live_abs and self.live_abs[-1][0] - self.live_abs[0][0] > LIVE_WINDOW_S:
                    self.live_abs.popleft()
                if now - self._last_live_draw >= 1.0:
                    self._last_live_draw = now
                    self._redraw()
            return
        if line.startswith("D,") and not line.startswith("D END"):
            self._on_dump_sample_line(line)
            return
        if line.startswith("D END"):
            self._append(line)
            try:
                # D END N segs=...
                parts = line.replace(",", " ").split()
                if len(parts) >= 3 and parts[1].isdigit():
                    got = int(parts[1])
                    if got > 0 and self._dump_expect <= 0:
                        self._dump_expect = got
            except Exception:  # noqa: BLE001
                pass
            # 无论有无点都结束「自动接收」状态，避免一直卡住
            self.after(100, self._finish_auto_recv)
            return
        if line.startswith("I,") and not line.startswith("I END"):
            parts = line.split(",")
            if len(parts) >= 8:
                try:
                    t = float(parts[2])
                    idxn = int(float(parts[3]))
                    rpm = float(parts[4])
                    rpm_i = float(parts[5])
                    dt = int(float(parts[6]))
                    seg = int(float(parts[7]))
                    t_rel = int(float(parts[8])) if len(parts) >= 9 else 0
                    unix = int(float(parts[9])) if len(parts) >= 10 else 0
                    self.dump_index.append((t, idxn, rpm, rpm_i, dt, seg, t_rel, unix))
                except ValueError:
                    pass
            return
        if line.startswith("I END"):
            self._append(line)
            if self.archive and self.dump_index:
                by_i: dict[int, list] = {}
                for row in self.dump_index:
                    by_i.setdefault(int(row[5]), []).append(row)
                for rec in self.archive:
                    if rec.board_seg in by_i and not rec.index_ev:
                        rec.index_ev = list(by_i[rec.board_seg])
            # USB 自动推送通常以 I END 收尾
            if getattr(self, "_board_auto_push", False) or getattr(
                self, "_auto_split_after_dump", False
            ):
                self.after(100, self._finish_auto_recv)
            return
        if line.startswith("S BEGIN") or line.startswith("S END"):
            return
        if line.startswith("#"):
            if "DUMP PROG" not in line and "KEEPALIVE" not in line:
                self._append(line)
            if "REC NOW" in line and "SNAP DONE" not in line:
                self._set_mon_ui("recording")
                self.var_mon.set("直采中… 1s@2kHz SNAP")
            elif "SNAP DONE" in line:
                self._set_mon_ui("recording")
                self.var_mon.set("SNAP 完成 · 等 ALIVE（约2秒）")
                voice_speak("记录完毕")
                try:
                    for tok in line.replace("#", " ").split():
                        if tok.startswith("n="):
                            self._dump_expect = int(tok[2:])
                except Exception:  # noqa: BLE001
                    pass
                self.var_dump_prog.set(
                    f"板内 n={self._dump_expect or '?'} — 等 ALIVE，勿急着 DUMP"
                )
                # 不在此处自动 DUMP：等 SNAP DUMP READY（ALIVE 满 5s）
            elif "ALIVE" in line:
                self.var_dump_prog.set(line.strip())
            elif "SNAP DUMP READY" in line:
                self._set_mon_ui("dumping")
                self.var_mon.set("RAM 就绪 · 板子自动 SD SAVE…")
                try:
                    for tok in line.replace("#", " ").split():
                        if tok.startswith("n="):
                            self._dump_expect = int(tok[2:])
                except Exception:  # noqa: BLE001
                    pass
                self.var_dump_prog.set(f"READY n={self._dump_expect or '?'} → SD archive")
                # 板子会自动 SD SAVE；可选再拉 USB 二进制
                # self.after(100, self._do_bin_pull)
            elif "SD SAVE OK" in line:
                self._set_mon_ui("done")
                self.var_mon.set("已存档到 SD（RAM→SD）")
                self.var_dump_prog.set(line.strip())
                voice_speak("已存入存储卡")
                if self._is_ble_mode():
                    self.var_info.set("SD 已存档 · 即将蓝牙从 RAM 取回（不读卡）")
                else:
                    self.var_info.set("发 USB DISK ON 用 Native USB 拷 snap_*.bin；或拔卡读")
            elif "BLE PULL READY" in line:
                self.var_mon.set("SD 已存 · RAM 可拉")
                self.var_info.set("蓝牙从 RAM 取数（与 SD 文件一致，更稳）")
                self.ble_prog["value"] = 0
                self.var_ble_pct.set("0%")
                self.var_dump_prog.set("即将蓝牙下载… 0%")
                if self._is_ble_mode() and not getattr(self, "_awaiting_bindump", False):
                    self.after(200, self._do_bin_pull)
            elif "BIN BLE BEGIN" in line:
                self.ble_prog["value"] = 0
                self.var_ble_pct.set("0%")
                self.var_dump_prog.set("蓝牙下载开始… 0%")
                self.var_mon.set("蓝牙回传中… 0%")
                try:
                    for tok in line.replace("#", " ").split():
                        if tok.startswith("bytes="):
                            self._dump_expect = int(tok[6:])
                except Exception:  # noqa: BLE001
                    pass
            elif "STAGING" in line and "I+1" in line:
                self._set_mon_ui("staging")
                self.var_mon.set("|RPM|>10 · 等待 I 过完 1 圈")
                voice_speak("探测到了")
            elif "CONFIRM" in line and "I+1" in line:
                self._set_mon_ui("recording")
                self.var_mon.set("已触发 · 回溯400 + 记2s…")
                voice_speak("开始记录，请保持转动两秒")
            elif "SD SAVE fail" in line:
                self._set_mon_ui("idle")
                self.var_mon.set("SD 存档失败 · RAM 数据仍在")
                if self._is_ble_mode():
                    self.var_info.set("可点②发 DUMP BIN BLE（读 RAM）；或插卡后 SD SAVE")
                    voice_speak("存卡失败，请检查存储卡")
                else:
                    self.var_info.set("可发 DUMP BIN 或检查 SD 后 SD SAVE")
            elif "BIN BLE END" in line and "ok=0" in line:
                self._awaiting_bindump = False
                self._ui_dumping = False
                self.var_mon.set("蓝牙 BIN 传输中断")
                self.var_dump_prog.set(line.strip())
                voice_speak("蓝牙取回失败")
            elif "DUMP BIN BLE fail" in line or "DUMP BIN BLE wait" in line:
                self._awaiting_bindump = False
                self._ui_dumping = False
                self.var_mon.set(line.strip())
                self.var_dump_prog.set(line.strip())
            elif "BIN END" in line:
                pass  # 真正收齐靠 __bindump__
            elif "MONITOR armed" in line:
                self._set_mon_ui("armed")
                self.var_mon.set("已武装（可静止）… 等待弹射 |RPM|>20")
                self.var_info.set("开始监控已生效 — 转起来后会 CONFIRM→RECORD→自动推送")
            elif "CONFIRM" in line:
                self._set_mon_ui("recording")
                bt = ""
                if "backtrack=" in line:
                    try:
                        bt = line.split("backtrack=")[1].split()[0]
                    except Exception:  # noqa: BLE001
                        bt = ""
                self.var_mon.set(
                    f"已确认 · 已追加触发前 {bt or '?'} 点，继续记满时长"
                    if bt
                    else "已确认真实转动（已追溯提交临时池）"
                )
            elif "NOISE" in line:
                self._set_mon_ui("armed")
                self.var_mon.set("噪声：临时池已丢弃")
            elif "TRIGGER" in line and "record started" in line:
                self._set_mon_ui("recording")
                self.var_mon.set("已触发 · 记录中…（暂停刷新，完成后自动取回）")
                self.var_info.set("H743 专注记录；完成后 UI 等 10s 自动下载")
                self._plot_enabled.set(False)
                self._toggle_plot()
            elif "DUMP BUSY" in line or "RECORD done" in line or line.startswith("# RECORD_DONE"):
                # snap 路径：`RECORD done … → ALIVE then SD SAVE` — 等 SD/BLE PULL，勿发 LOG DUMP
                if "RECORD done" in line and "SD SAVE" in line:
                    self._set_mon_ui("recording")
                    self.var_mon.set("记录完毕 · 等存卡后蓝牙/U盘取数")
                    voice_speak("记录完毕")
                    try:
                        for tok in line.replace("#", " ").replace("→", " ").split():
                            if tok.startswith("n="):
                                self._dump_expect = int(tok[2:].rstrip(","))
                    except Exception:  # noqa: BLE001
                        pass
                    self.var_dump_prog.set(
                        f"snap n={self._dump_expect or '?'} → ALIVE → SD → BLE/USB"
                    )
                else:
                    self._set_mon_ui("dumping")
                    self.var_mon.set("记录完成 · 即将拉取完整段")
                    if "RECORD done" in line or line.startswith("# RECORD_DONE"):
                        voice_speak("记录完毕")
                    try:
                        if "seg=" in line:
                            self._pull_seg = int(line.split("seg=")[1].split()[0])
                        if line.startswith("# RECORD_DONE"):
                            n = int(line.split("|", 1)[1].strip().split()[0])
                            self._dump_expect = n
                    except Exception:  # noqa: BLE001
                        pass
                    self._schedule_read_once(10000)
            elif "AUTO DUMP PREP" in line or "AUTO DUMP FALLBACK" in line:
                self._set_mon_ui("dumping")
                self._ui_dumping = True
                self._board_auto_push = True
                self.var_mon.set("板子导出中…")
            elif "SALVAGE keep" in line:
                self._set_mon_ui("dumping")
                self.var_mon.set("已保留板内数据 · 即将自动读取")
                self._schedule_read_once(150)
            elif "AUTO DUMP READY" in line:
                try:
                    if "expect_D=" in line:
                        part = line.split("expect_D=")[1].split()[0]
                        self._dump_expect = int(float(part))
                        self._dump_got = 0
                    if "seg=" in line:
                        self._pull_seg = int(line.split("seg=")[1].split()[0])
                except Exception:  # noqa: BLE001
                    pass
                if self._dump_expect == 0:
                    self.var_mon.set("板内 expect=0 — 本段无点，请再①并转动")
                    self.var_info.set("确认/记录可能未写入；看是否有 CONFIRM / RECORD done")
                else:
                    self.var_mon.set(
                        f"板内就绪 expect≈{self._dump_expect}（含触发前追点）· 拉取中"
                    )
                    self._schedule_read_once(80)
            elif "expect_D=0" in line or "log empty for seg" in line:
                self.var_mon.set("板内该段无数据")
                self._set_mon_ui("idle")
            elif "LOG empty" in line:
                self.var_mon.set("DUMP 空 — 板内 log_n=0（未记上或已复位）")
                self._set_mon_ui("idle")
            elif "DATA_START|" in line:
                self._ui_dumping = True
                self._board_auto_push = True
                self._set_mon_ui("dumping")
                self.dump_rows.clear()
                self.dump_index.clear()
                try:
                    self._dump_expect = int(line.split("|", 1)[1].strip().split()[0])
                except Exception:  # noqa: BLE001
                    pass
                self.var_dump_prog.set(f"回传 0/{self._dump_expect or '?'}")
                self.var_mon.set("记录完成 · 正在自动接收…")
            elif "DATA_END" in line:
                self.after(100, self._finish_auto_recv)
            elif "expect_D=" in line and "READY" not in line:
                try:
                    part = line.split("expect_D=")[1].split()[0]
                    self._dump_expect = int(float(part))
                    self._dump_got = 0
                    self.var_dump_prog.set(f"预计约 {self._dump_expect} 点（蓝牙会降采样）")
                except Exception:  # noqa: BLE001
                    pass
            elif "AUTO DUMP BEGIN" in line:
                self._ui_dumping = True
                self._board_auto_push = True
                self._read_scheduled = True
                self._set_mon_ui("dumping")
                self.dump_rows.clear()
                self.dump_index.clear()
                self.playback.clear()
                self._dump_got = 0
                self._auto_split_after_dump = True
                try:
                    if "expect_D=" in line:
                        self._dump_expect = int(line.split("expect_D=")[1].split()[0])
                except Exception:  # noqa: BLE001
                    pass
                self.var_info.set("板子自动推送中…")
                self.var_dump_prog.set(
                    f"自动回传中… 预计 {self._dump_expect or '?'} 点"
                )
                self.var_mon.set("记录完成 · 正在自动接收…")
            elif "DUMP PROG" in line:
                n = tot = pct = None
                try:
                    for tok in line.replace("#", " ").split():
                        if tok.startswith("n="):
                            n = int(tok[2:])
                        elif tok.startswith("total="):
                            tot = int(tok[6:])
                        elif tok.startswith("pct="):
                            pct = int(tok[4:])
                    if n is not None:
                        self._dump_got = n
                    if tot:
                        self._dump_expect = tot
                    if pct is None and self._dump_expect:
                        pct = int(self._dump_got * 100 / self._dump_expect)
                    self.var_dump_prog.set(
                        f"回传进度 {self._dump_got}/{self._dump_expect or '?'}  ({pct or 0}%)"
                    )
                    self.var_info.set(self.var_dump_prog.get())
                except Exception:  # noqa: BLE001
                    self.var_info.set(line.strip())
            elif (self._ui_dumping or self._board_auto_push) and (
                "FW=monitor" in line or "Guru Meditation" in line or "panic" in line
            ):
                self._append("# WARN board reboot/panic during dump")
                self.after(80, self._finish_auto_recv)
            elif "AUTO DUMP END" in line:
                self.after(100, self._finish_auto_recv)
            elif "disarm" in line.lower():
                pass  # RECORD done 已调度自动拉取
            elif "CAPTURE DONE" in line or "CAPTURE done" in line:
                self._cap_end_mono = None
                try:
                    if "n=" in line:
                        self._cap_board_n = int(line.split("n=")[1].split()[0])
                except Exception:  # noqa: BLE001
                    pass
                self.var_cap.set(
                    f"长采完成 n={self._cap_board_n} — 0.5s 后自动拉取并保存…"
                )
                self.var_mon.set("长采完成 · 即将上传")
                if self.link and getattr(self.link, "is_open", False):
                    self.after(500, self._cap_dump_save)
            elif "CAPTURE PROG" in line:
                try:
                    if "remain_s=" in line:
                        rem_s = int(line.split("remain_s=")[1].split()[0])
                        self._cap_end_mono = time.monotonic() + float(rem_s)
                    if "n=" in line:
                        self._cap_board_n = int(line.split("n=")[1].split()[0])
                except Exception:  # noqa: BLE001
                    pass
            elif "CAPTURE START" in line and "PROG" not in line:
                self.var_cap.set(
                    f"长采已开始 · 剩余约 {self._cap_total_s}s（可随意启停/等待）"
                )
            elif "CAPTURE stop" in line.lower() or line.startswith("# CAPTURE stop"):
                self._cap_end_mono = None
                try:
                    if "n=" in line:
                        self._cap_board_n = int(line.split("n=")[1].split()[0])
                except Exception:  # noqa: BLE001
                    pass
                self.var_cap.set(f"已手动停止 · n={self._cap_board_n} — 可拉取保存")
            elif "CAP DUMP" in line:
                self._cap_dumping = True
                self.var_cap.set("板子开始回传长采…")
                try:
                    if "n=" in line:
                        self._cap_expect = int(line.split("n=")[1].split()[0])
                        self.var_cap.set(f"回传长采 预计 {self._cap_expect} 点…")
                except Exception:  # noqa: BLE001
                    pass
            elif "CAP CHUNK" in line:
                self._cap_dumping = True
                try:
                    self.var_cap.set(
                        f"回传中 {line.split('CAP CHUNK', 1)[1].strip()} · PC已收 {len(self.cap_rows)}"
                    )
                except Exception:  # noqa: BLE001
                    self.var_cap.set(f"回传中… PC已收 {len(self.cap_rows)} 点")
                if "need=NEXT" in line:
                    self._queue_cmd("CAPTURE NEXT")
            elif line.startswith("CAP END"):
                self._cap_end_mono = None
                self._cap_dumping = False
                self.var_cap.set(f"长采已收 {len(self.cap_rows)} 点")
                self._save_cap_file()
            elif "CAP empty" in line:
                self._cap_dumping = False
                self.var_cap.set("长采为空（板内 n=0）— 重新点采30秒")
            elif self._cap_dumping and (
                "FW=capture" in line or "ready. CAPTURE START" in line
            ):
                # 回传中途板子复位：尽量保存已收到的点
                n_got = len(self.cap_rows)
                self._cap_dumping = False
                self._append(f"# WARN dump aborted by reboot; partial n={n_got}")
                if n_got > 100:
                    self.var_cap.set(f"回传中断(板复位) · 已保存部分 {n_got} 点")
                    self._save_cap_file()
                else:
                    self.var_cap.set("回传中断(板复位) · 请重采后拉取（需 capture-v4）")
            elif "SALVAGE keep" in line:
                # 上面 # 分支已 schedule；此处兜底（非 # 前缀时）
                self.var_mon.set("已保留板内数据 · 即将自动读取")
                self._schedule_read_once(500)
            elif "NOISE discard" in line:
                self.var_mon.set("噪声丢弃（几乎无转动）· 主记录未改")
            elif "LOG empty" in line:
                self.var_mon.set("板内无记录 — 请先点「①开始监控」再弹射")
                self.var_dump_prog.set("空：实时转速不会自动写入")
                self.var_info.set("顺序：连接→①武装→弹射→自动拉取分析")
                messagebox.showinfo(
                    "没有记录数据",
                    "板内 log_n=0。\n\n"
                    "请先点「① 开始监控」，再弹射；\n"
                    "成功时会自动拉取并弹出分析结果。",
                )
            elif "armed" in line.lower() or "MONITOR armed" in line:
                self.var_mon.set("已武装：等待 |RPM|>20（可静止等待弹射）")
            elif "INDEX EVENTS" in line:
                pass

    def _view_selected_silent(self) -> None:
        rec = self._selected_archive()
        if rec is None:
            return
        self._stop_play()
        self.view_seg = rec
        self.plot_mode = "segment"
        self.var_t0.set("0.0")
        self.var_t1.set(f"{max(rec.duration_s, 0.05):.3f}")
        self._plot_segment(rec, None, None)

    def _auto_save_dump(self) -> None:
        if not self.dump_rows:
            return
        from pathlib import Path

        folder = app_dir() / "logs" / datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )
        try:
            self._save_csv_split(str(folder))
            self._append(f"# auto-saved push → {folder}")
        except Exception as exc:  # noqa: BLE001
            self._append(f"# auto-save failed: {exc}")

    def destroy(self) -> None:  # type: ignore[override]
        self._stop_play()
        self.stop_evt.set()
        super().destroy()


if __name__ == "__main__":
    import socket
    import sys
    import traceback
    from pathlib import Path

    _crash_log = app_dir() / "ui_crash.log"

    def _excepthook(exc_type, exc, tb):
        try:
            with _crash_log.open("a", encoding="utf-8") as f:
                f.write("\n=== " + datetime.now().isoformat() + " ===\n")
                traceback.print_exception(exc_type, exc, tb, file=f)
        except Exception:
            pass
        traceback.print_exception(exc_type, exc, tb)
        try:
            messagebox.showerror("UI 异常", f"{exc_type.__name__}: {exc}\n\n详见 pc/ui_crash.log")
        except Exception:
            pass

    sys.excepthook = _excepthook

    # 单实例：仿真用不同端口，可与真机窗口并存对照
    _lock_port = 47330 if PREFER_SIM else 47329
    _lock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        _lock.bind(("127.0.0.1", _lock_port))
        _lock.listen(1)
    except OSError:
        # 已有实例在运行：提示而不是无声退出，避免"双击没反应"
        import tkinter.messagebox as _mb

        _mb.showerror(
            "ABI 监控已打开",
            "另一扇窗已经在运行。\n"
            "请先关闭已打开的 ABI 监控窗口（或在任务管理器结束残留 python 进程），再重试。",
        )
        sys.exit(0)
    # 保持引用，避免 GC 释放端口导致误开第二实例
    globals()["_ABI_UI_LOCK"] = _lock
    try:
        App().mainloop()
    except Exception as exc:
        _excepthook(type(exc), exc, exc.__traceback__)
        raise


