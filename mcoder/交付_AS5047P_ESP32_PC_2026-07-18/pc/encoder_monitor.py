#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AS5047P 磁编码器 PC 监视器
- 实时显示角度 0~360° / 0~2π，钟表式相位表盘（0° 在上方，顺时针）
- 开始/停止记录，绘制转速(RPM) vs 时间
- 可导出记录为 CSV
"""

from __future__ import annotations

import csv
import math
import os
import queue
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
from matplotlib import font_manager
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.font_manager import FontProperties

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise SystemExit("请先安装依赖: pip install -r requirements.txt")


BAUD = 115200
DEFAULT_PORT = "COM18"
AUTO_CONNECT = True  # 启动时若有默认口则自动连接
LIVE_WINDOW_S = 10.0  # 实时转速曲线窗口长度（秒）

# Windows 常见中文字体（文件名 / 族名），避免表盘汉字变成方框
_CN_FONT_FILES = (
    "msyh.ttc",  # Microsoft YaHei
    "msyhbd.ttc",
    "simhei.ttf",  # SimHei
    "simsun.ttc",  # SimSun
    "SIMSUN.TTC",
)
_CN_FONT_NAMES = (
    "Microsoft YaHei",
    "SimHei",
    "SimSun",
    "Noto Sans CJK SC",
    "Source Han Sans SC",
    "Arial Unicode MS",
)


def _configure_matplotlib_chinese_font() -> FontProperties | None:
    """配置 matplotlib 中文字体，返回可用的 FontProperties（供显式指定）。"""
    matplotlib.rcParams["axes.unicode_minus"] = False

    fonts_dir = Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts"
    for name in _CN_FONT_FILES:
        path = fonts_dir / name
        if not path.is_file():
            continue
        try:
            font_manager.fontManager.addfont(str(path))
        except (OSError, RuntimeError, ValueError):
            pass
        prop = FontProperties(fname=str(path))
        family = prop.get_name()
        matplotlib.rcParams["font.sans-serif"] = [family, *_CN_FONT_NAMES, "DejaVu Sans"]
        matplotlib.rcParams["font.family"] = "sans-serif"
        return prop

    # 无字体文件时仍按族名回退
    matplotlib.rcParams["font.sans-serif"] = [*_CN_FONT_NAMES, "DejaVu Sans"]
    matplotlib.rcParams["font.family"] = "sans-serif"
    return None


_CN_FONT = _configure_matplotlib_chinese_font()


def list_serial_ports() -> list[str]:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    # 优先 COM18，其余按设备名排序
    preferred = [p for p in ports if p.upper() == DEFAULT_PORT.upper()]
    others = sorted(p for p in ports if p.upper() != DEFAULT_PORT.upper())
    return preferred + others


def parse_line(line: str) -> tuple | None:
    """解析: t_ms,raw,deg,rad,rpm[,ef,agc,magL,magH]"""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) < 4:
        return None
    try:
        t_ms = float(parts[0])
        raw = int(float(parts[1]))
        deg = float(parts[2])
        rad = float(parts[3])
        rpm = float(parts[4]) if len(parts) >= 5 else 0.0
        ef = int(float(parts[5])) if len(parts) >= 6 else 0
        agc = int(float(parts[6])) if len(parts) >= 7 else -1
        mag_l = int(float(parts[7])) if len(parts) >= 8 else 0
        mag_h = int(float(parts[8])) if len(parts) >= 9 else 0
        return t_ms, raw, deg, rad, rpm, ef, agc, mag_l, mag_h
    except ValueError:
        return None


def unwrap_delta_deg(cur: float, prev: float) -> float:
    d = cur - prev
    if d > 180.0:
        d -= 360.0
    if d < -180.0:
        d += 360.0
    return d


class SerialReader(threading.Thread):
    def __init__(self, port: str, out_q: queue.Queue, stop_evt: threading.Event):
        super().__init__(daemon=True)
        self.port = port
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.error: str | None = None

    def run(self) -> None:
        try:
            # dsrdtr/rtscts=False：避免 Windows 上一直拉 DTR/RTS 导致 ESP32 卡在 bootloader
            with serial.Serial(
                self.port,
                BAUD,
                timeout=0.2,
                dsrdtr=False,
                rtscts=False,
            ) as ser:
                ser.dtr = False
                ser.rts = False
                time.sleep(0.05)
                ser.reset_input_buffer()
                buf = ""
                while not self.stop_evt.is_set():
                    chunk = ser.read(256)
                    if not chunk:
                        continue
                    buf += chunk.decode("utf-8", errors="ignore")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        parsed = parse_line(line)
                        if parsed is not None:
                            self.out_q.put(parsed)
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            self.out_q.put(("__error__", str(exc)))


class EncoderMonitorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("AS5047P 磁编码器监视器")
        root.geometry("1100x680")
        root.minsize(900, 560)

        self.q: queue.Queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.reader: SerialReader | None = None

        self.recording = False
        self.rec_t0_ms: float | None = None
        self.rec_rows: list[tuple[float, float, float, float, float]] = []

        self.live_t: deque[float] = deque()
        self.live_rpm: deque[float] = deque()
        self.prev_deg: float | None = None
        self.prev_t_ms: float | None = None
        self.current_deg: float | None = None

        self._build_ui()
        self.root.after(50, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(
            top, textvariable=self.port_var, width=18, state="readonly"
        )
        self.port_cb.pack(side=tk.LEFT, padx=(4, 4))
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT)

        self.btn_connect = ttk.Button(top, text="连接", command=self._toggle_connect)
        self.btn_connect.pack(side=tk.LEFT, padx=(12, 0))

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=12)

        mid = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        mid.pack(fill=tk.X)

        self.deg_var = tk.StringVar(value="— °")
        self.rad_var = tk.StringVar(value="— rad")
        self.rpm_var = tk.StringVar(value="— RPM")
        self.raw_var = tk.StringVar(value="raw: —")
        self.diag_var = tk.StringVar(value="诊断: —")

        big = ("Segoe UI", 28, "bold")
        ttk.Label(mid, textvariable=self.deg_var, font=big).pack(side=tk.LEFT, padx=(0, 24))
        ttk.Label(mid, textvariable=self.rad_var, font=big).pack(side=tk.LEFT, padx=(0, 24))
        ttk.Label(mid, textvariable=self.rpm_var, font=("Segoe UI", 22)).pack(
            side=tk.LEFT, padx=(0, 24)
        )
        side = ttk.Frame(mid)
        side.pack(side=tk.LEFT)
        ttk.Label(side, textvariable=self.raw_var).pack(anchor=tk.W)
        ttk.Label(side, textvariable=self.diag_var).pack(anchor=tk.W)

        ctrl = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        ctrl.pack(fill=tk.X)
        self.btn_rec = ttk.Button(
            ctrl, text="开始记录", command=self._toggle_record, state=tk.DISABLED
        )
        self.btn_rec.pack(side=tk.LEFT)
        self.btn_export = ttk.Button(
            ctrl, text="导出 CSV", command=self._export_csv, state=tk.DISABLED
        )
        self.btn_export.pack(side=tk.LEFT, padx=8)
        self.btn_clear = ttk.Button(ctrl, text="清空曲线", command=self._clear_plot)
        self.btn_clear.pack(side=tk.LEFT)

        self.rec_info = tk.StringVar(value="记录: 未开始")
        ttk.Label(ctrl, textvariable=self.rec_info).pack(side=tk.LEFT, padx=16)

        plot_frame = ttk.Frame(self.root, padding=8)
        plot_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(10, 4.2), dpi=100)
        gs = self.fig.add_gridspec(1, 2, width_ratios=[3.2, 1.0], wspace=0.28)
        self.ax = self.fig.add_subplot(gs[0, 0])
        fp = dict(fontproperties=_CN_FONT) if _CN_FONT is not None else {}
        self.ax.set_title("转速 vs 时间", **fp)
        self.ax.set_xlabel("时间 (s)", **fp)
        self.ax.set_ylabel("转速 (RPM)", **fp)
        self.ax.grid(True, alpha=0.3)
        (self.line,) = self.ax.plot([], [], color="#1f77b4", linewidth=1.5)

        # 相位表盘：0° 在上方（12 点），顺时针增加（钟表/电机相位习惯）
        self.ax_dial = self.fig.add_subplot(gs[0, 1], projection="polar")
        self._setup_phase_dial()

        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._refresh_ports(prefer_default=True)
        if AUTO_CONNECT and self.port_var.get().upper() == DEFAULT_PORT.upper():
            # 稍延后，等窗口先画出来
            self.root.after(200, self._connect)

    def _setup_phase_dial(self) -> None:
        """相位表盘：0° 在 12 点方向，顺时针为正。"""
        ax = self.ax_dial
        fp = dict(fontproperties=_CN_FONT) if _CN_FONT is not None else {}
        ax.set_theta_zero_location("N")  # 0° 在上方
        ax.set_theta_direction(-1)  # 顺时针
        ax.set_ylim(0, 1.15)
        ax.set_yticks([])
        ax.set_xticks([math.radians(a) for a in (0, 90, 180, 270)])
        ax.set_xticklabels(["0°", "90°", "180°", "270°"], fontsize=9, **fp)
        ax.set_title("相位\n0°↑ 顺时针", fontsize=10, pad=12, **fp)
        # 外圈参考圆
        theta_c = [math.radians(a) for a in range(0, 361, 2)]
        ax.plot(theta_c, [1.0] * len(theta_c), color="#888888", linewidth=1.2)
        # 刻度短线（每 30°）
        for a in range(0, 360, 30):
            th = math.radians(a)
            r0, r1 = (0.88, 1.0) if a % 90 == 0 else (0.94, 1.0)
            ax.plot([th, th], [r0, r1], color="#555555", linewidth=1.4 if a % 90 == 0 else 0.8)
        (self.needle,) = ax.plot(
            [0, 0], [0, 0.92], color="#d62728", linewidth=2.4, solid_capstyle="round"
        )
        self.needle_tip = ax.plot(
            [0], [0.92], marker="o", markersize=6, color="#d62728"
        )[0]
        ax.plot([0], [0], marker="o", markersize=5, color="#333333")

    def _update_phase_dial(self, deg: float | None) -> None:
        if deg is None:
            self.needle.set_data([0, 0], [0, 0])
            self.needle_tip.set_data([0], [0])
            return
        th = math.radians(deg % 360.0)
        self.needle.set_data([th, th], [0.0, 0.92])
        self.needle_tip.set_data([th], [0.92])

    def _refresh_ports(self, prefer_default: bool = False) -> None:
        ports = list_serial_ports()
        self.port_cb["values"] = ports
        if not ports:
            self.port_var.set("")
            return
        cur = self.port_var.get()
        if prefer_default or not cur:
            default = next(
                (p for p in ports if p.upper() == DEFAULT_PORT.upper()),
                ports[0],
            )
            self.port_var.set(default)
        elif cur not in ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self) -> None:
        if self.reader and self.reader.is_alive():
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择串口")
            return
        self.stop_evt.clear()
        self.reader = SerialReader(port, self.q, self.stop_evt)
        self.reader.start()
        self.btn_connect.configure(text="断开")
        self.btn_rec.configure(state=tk.NORMAL)
        self.status_var.set(f"已连接 {port} @ {BAUD}")
        self.prev_deg = None
        self.prev_t_ms = None

    def _disconnect(self) -> None:
        if self.recording:
            self._stop_record()
        self.stop_evt.set()
        self.reader = None
        self.btn_connect.configure(text="连接")
        self.btn_rec.configure(state=tk.DISABLED)
        self.status_var.set("未连接")

    def _toggle_record(self) -> None:
        if self.recording:
            self._stop_record()
        else:
            self._start_record()

    def _start_record(self) -> None:
        self.recording = True
        self.rec_t0_ms = None
        self.rec_rows.clear()
        self.live_t.clear()
        self.live_rpm.clear()
        self.btn_rec.configure(text="停止记录")
        self.btn_export.configure(state=tk.DISABLED)
        self.rec_info.set("记录中…")
        self._redraw()

    def _stop_record(self) -> None:
        self.recording = False
        self.btn_rec.configure(text="开始记录")
        n = len(self.rec_rows)
        self.rec_info.set(f"记录结束: {n} 点")
        if n > 0:
            self.btn_export.configure(state=tk.NORMAL)

    def _clear_plot(self) -> None:
        if self.recording:
            messagebox.showinfo("提示", "请先停止记录再清空")
            return
        self.live_t.clear()
        self.live_rpm.clear()
        self.rec_rows.clear()
        self.btn_export.configure(state=tk.DISABLED)
        self.rec_info.set("记录: 未开始")
        self._redraw()

    def _export_csv(self) -> None:
        if not self.rec_rows:
            messagebox.showinfo("提示", "没有可导出的数据")
            return
        default = datetime.now().strftime("as5047p_rpm_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default,
            filetypes=[("CSV", "*.csv"), ("All", "*.*")],
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "deg", "rad", "rpm", "raw"])
            for t_s, deg, rad, rpm, raw in self.rec_rows:
                w.writerow([f"{t_s:.4f}", f"{deg:.3f}", f"{rad:.6f}", f"{rpm:.2f}", int(raw)])
        messagebox.showinfo("完成", f"已保存: {path}")

    def _poll(self) -> None:
        updated = False
        try:
            while True:
                item = self.q.get_nowait()
                if isinstance(item, tuple) and item and item[0] == "__error__":
                    messagebox.showerror("串口错误", item[1])
                    self._disconnect()
                    break
                self._on_sample(item)
                updated = True
        except queue.Empty:
            pass

        if updated:
            self._redraw()
        self.root.after(50, self._poll)

    def _on_sample(self, sample: tuple) -> None:
        t_ms, raw, deg, rad, rpm_fw, ef, agc, mag_l, mag_h = sample

        # 优先用本机角度差分算转速（更稳）；固件 RPM 作后备
        rpm = rpm_fw
        if self.prev_deg is not None and self.prev_t_ms is not None:
            dt = (t_ms - self.prev_t_ms) * 0.001
            if dt > 0.0005:
                ddeg = unwrap_delta_deg(deg, self.prev_deg)
                rpm = (ddeg / 360.0) / dt * 60.0
        self.prev_deg = deg
        self.prev_t_ms = t_ms

        self.deg_var.set(f"{deg:7.2f} °")
        self.rad_var.set(f"{rad:7.4f} rad  (0~{2*math.pi:.4f})")
        self.rpm_var.set(f"{rpm:8.1f} RPM")
        self.raw_var.set(f"raw: {raw}")
        self.current_deg = deg

        warns = []
        if ef:
            warns.append("SPI_EF")
        if mag_l:
            warns.append("磁场过弱MAGL")
        if mag_h:
            warns.append("磁场过强MAGH")
        agc_txt = f"AGC={agc}" if agc >= 0 else "AGC=—"
        self.diag_var.set(
            f"诊断: {agc_txt}" + ((" | " + ", ".join(warns)) if warns else " | OK")
        )

        if self.recording:
            if self.rec_t0_ms is None:
                self.rec_t0_ms = t_ms
            t_s = (t_ms - self.rec_t0_ms) * 0.001
            self.rec_rows.append((t_s, deg, rad, rpm, float(raw)))
            self.live_t.append(t_s)
            self.live_rpm.append(rpm)
            self.rec_info.set(f"记录中… {len(self.rec_rows)} 点  t={t_s:.2f}s")
        else:
            # 未记录时也显示最近 LIVE_WINDOW_S 的转速曲线
            now_wall = time.time()
            self.live_t.append(now_wall)
            self.live_rpm.append(rpm)
            while self.live_t and (now_wall - self.live_t[0]) > LIVE_WINDOW_S:
                self.live_t.popleft()
                self.live_rpm.popleft()

    def _redraw(self) -> None:
        if self.recording or self.rec_rows:
            xs = list(self.live_t) if self.recording else [r[0] for r in self.rec_rows]
            ys = list(self.live_rpm) if self.recording else [r[3] for r in self.rec_rows]
            xlabel = "时间 (s) — 记录段"
        else:
            if not self.live_t:
                xs, ys = [], []
            else:
                t0 = self.live_t[0]
                xs = [t - t0 for t in self.live_t]
                ys = list(self.live_rpm)
            xlabel = f"时间 (s) — 最近 {LIVE_WINDOW_S:.0f}s"

        self.line.set_data(xs, ys)
        fp = dict(fontproperties=_CN_FONT) if _CN_FONT is not None else {}
        self.ax.set_xlabel(xlabel, **fp)
        if xs:
            self.ax.relim()
            self.ax.autoscale_view()
        self._update_phase_dial(self.current_deg)
        self.canvas.draw_idle()

    def _on_close(self) -> None:
        self.stop_evt.set()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    try:
        style = ttk.Style()
        if "vista" in style.theme_names():
            style.theme_use("vista")
    except tk.TclError:
        pass
    EncoderMonitorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
