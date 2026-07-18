#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AS5047P + 航模电调 PC 监视/控制
- 编码器相位/转速监视与记录
- 电机：4 圈圆盘设定 0~6000 RPM、窗口输入、启动/停止/急停、缓启动
- 模式：开环 / 闭环 / 学习；profile noload|flap；PID / 自适应
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


BAUD = 921600
DEFAULT_PORT = "COM10"
AUTO_CONNECT = True
LIVE_WINDOW_S = 10.0
UI_REFRESH_MS = 40
MAX_REC_POINTS = 120000
EXPECTED_CTRL_HZ = 250
RPM_MAX = 6000.0  # 电机允许最大（6S）；3S 调试通常达不到
# 圆盘 4 圈 = 6000 RPM → 1500 RPM/圈
DIAL_RPM_PER_DEG = RPM_MAX / (4.0 * 360.0)
ESC_PULSE_MAX_US = 2000
MOTOR_RPM_LIMIT_DEFAULT = 6000.0

MODE_NAMES = {0: "SAFE", 1: "OPEN", 2: "CLOSED", 3: "LEARN", 4: "MEASURE"}
RUN_NAMES = {0: "IDLE", 1: "RUN", 2: "ESTOP"}
PROF_NAMES = {0: "noload", 1: "flap"}

_CN_FONT_FILES = (
    "msyh.ttc",
    "msyhbd.ttc",
    "simhei.ttf",
    "simsun.ttc",
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
    matplotlib.rcParams["font.sans-serif"] = [*_CN_FONT_NAMES, "DejaVu Sans"]
    matplotlib.rcParams["font.family"] = "sans-serif"
    return None


_CN_FONT = _configure_matplotlib_chinese_font()


def list_serial_ports() -> list[str]:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    preferred = [p for p in ports if p.upper() == DEFAULT_PORT.upper()]
    others = sorted(p for p in ports if p.upper() != DEFAULT_PORT.upper())
    return preferred + others


def parse_meta_line(line: str) -> dict | None:
    line = line.strip()
    if not line.startswith("#"):
        return None
    meta: dict = {"raw": line}
    for key in ("sample_hz", "ctrl_hz", "oversample", "baud", "esc_pwm_hz"):
        token = f"{key}="
        if token not in line:
            continue
        try:
            frag = line.split(token, 1)[1].split()[0].rstrip(",")
            meta[key] = int(float(frag))
        except (ValueError, IndexError):
            pass
    return meta


def parse_line(line: str) -> tuple | None:
    """t_ms,raw,deg,rad,rpm,ef,agc,magL,magH,pulse,target,mode,kp,ki,kd,profile,run"""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) < 5:
        return None
    try:
        t_ms = float(parts[0])
        raw = int(float(parts[1]))
        deg = float(parts[2])
        rad = float(parts[3])
        rpm = float(parts[4])
        ef = int(float(parts[5])) if len(parts) >= 6 else 0
        agc = int(float(parts[6])) if len(parts) >= 7 else -1
        mag_l = int(float(parts[7])) if len(parts) >= 8 else 0
        mag_h = int(float(parts[8])) if len(parts) >= 9 else 0
        pulse = int(float(parts[9])) if len(parts) >= 10 else 1000
        target = float(parts[10]) if len(parts) >= 11 else 0.0
        mode = int(float(parts[11])) if len(parts) >= 12 else 1
        kp = float(parts[12]) if len(parts) >= 13 else 0.0
        ki = float(parts[13]) if len(parts) >= 14 else 0.0
        kd = float(parts[14]) if len(parts) >= 15 else 0.0
        profile = int(float(parts[15])) if len(parts) >= 16 else 0
        run = int(float(parts[16])) if len(parts) >= 17 else 0
        return (
            t_ms,
            raw,
            deg,
            rad,
            rpm,
            ef,
            agc,
            mag_l,
            mag_h,
            pulse,
            target,
            mode,
            kp,
            ki,
            kd,
            profile,
            run,
        )
    except ValueError:
        return None


def unwrap_delta_deg(cur: float, prev: float) -> float:
    d = cur - prev
    if d > 180.0:
        d -= 360.0
    if d < -180.0:
        d += 360.0
    return d


class SerialLink(threading.Thread):
    """双向串口：读遥测 + 写指令。"""

    def __init__(self, port: str, out_q: queue.Queue, stop_evt: threading.Event):
        super().__init__(daemon=True)
        self.port = port
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.error: str | None = None
        self._ser: serial.Serial | None = None
        self._wlock = threading.Lock()
        self._tx_q: queue.Queue[str] = queue.Queue()

    def send(self, cmd: str) -> None:
        line = cmd.strip()
        if not line:
            return
        self._tx_q.put(line + "\n")

    def run(self) -> None:
        try:
            with serial.Serial(
                self.port,
                BAUD,
                timeout=0.05,
                write_timeout=0.5,
                dsrdtr=False,
                rtscts=False,
            ) as ser:
                self._ser = ser
                ser.dtr = False
                ser.rts = False
                time.sleep(0.05)
                ser.reset_input_buffer()
                buf = ""
                while not self.stop_evt.is_set():
                    try:
                        while True:
                            tx = self._tx_q.get_nowait()
                            with self._wlock:
                                ser.write(tx.encode("utf-8"))
                    except queue.Empty:
                        pass

                    chunk = ser.read(512)
                    if not chunk:
                        continue
                    buf += chunk.decode("utf-8", errors="ignore")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        if line.startswith("#"):
                            self.out_q.put(("__meta__", parse_meta_line(line) or {"raw": line}))
                            continue
                        parsed = parse_line(line)
                        if parsed is not None:
                            self.out_q.put(parsed)
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            self.out_q.put(("__error__", str(exc)))
        finally:
            self._ser = None


class SpeedDial(tk.Canvas):
    """按住旋转：累计角位移，4 圈 = 6000 RPM。"""

    def __init__(self, master, on_change, size: int = 180, **kw):
        super().__init__(
            master, width=size, height=size, highlightthickness=1, bg="#f7f7f7", **kw
        )
        self.size = size
        self.on_change = on_change
        self.rpm = 0.0
        self._dragging = False
        self._last_ang: float | None = None
        cx = cy = size / 2
        r = size * 0.42
        self._cx, self._cy, self._r = cx, cy, r
        self.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#444", width=2)
        self.create_text(cx, cy + r * 0.55, text="4圈=6000", fill="#666", font=("Segoe UI", 8))
        self._needle = self.create_line(cx, cy, cx, cy - r * 0.85, fill="#c0392b", width=3)
        self._label = self.create_text(
            cx, cy, text="0", font=("Segoe UI", 16, "bold"), fill="#222"
        )
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<B1-Motion>", self._motion)
        self.bind("<ButtonRelease-1>", self._release)

    def set_rpm(self, rpm: float, notify: bool = False) -> None:
        self.rpm = max(0.0, min(RPM_MAX, float(rpm)))
        # 指针角度：用 rpm 映射到视觉角度（仅显示，非绝对一圈）
        vis = (self.rpm / RPM_MAX) * 360.0 * 4.0  # 可超过 360，取模显示
        ang = math.radians(vis % 360.0)
        # 0 在上方，顺时针
        x = self._cx + self._r * 0.85 * math.sin(ang)
        y = self._cy - self._r * 0.85 * math.cos(ang)
        self.coords(self._needle, self._cx, self._cy, x, y)
        self.itemconfigure(self._label, text=f"{self.rpm:.0f}")
        if notify:
            self.on_change(self.rpm)

    def _angle_at(self, event) -> float:
        dx = event.x - self._cx
        dy = self._cy - event.y
        return math.degrees(math.atan2(dx, dy))  # 0=上，顺时针为正约 -180..180

    def _press(self, event) -> None:
        self._dragging = True
        self._last_ang = self._angle_at(event)

    def _motion(self, event) -> None:
        if not self._dragging or self._last_ang is None:
            return
        ang = self._angle_at(event)
        d = ang - self._last_ang
        if d > 180:
            d -= 360
        if d < -180:
            d += 360
        self._last_ang = ang
        self.set_rpm(self.rpm + d * DIAL_RPM_PER_DEG, notify=True)

    def _release(self, _event) -> None:
        self._dragging = False
        self._last_ang = None


class EncoderMonitorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("AS5047P + 电调转速控制")
        root.geometry("1500x860")
        root.minsize(1100, 700)

        self.q: queue.Queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.link: SerialLink | None = None

        self.recording = False
        self.rec_t0_ms: float | None = None
        self.rec_rows: list[tuple] = []

        self.live_t: deque[float] = deque()
        self.live_rpm: deque[float] = deque()
        self.prev_deg: float | None = None
        self.prev_t_ms: float | None = None
        self.current_deg: float | None = None
        self.ctrl_hz = EXPECTED_CTRL_HZ
        self._ui_dirty = False
        self._last_ui_draw = 0.0
        self._rx_count = 0
        self._rx_t0 = time.time()
        self._setpoint = 0.0
        self._syncing_sp = False
        self._last_ping = 0.0
        self._learn_points: list[tuple[int, float]] = []
        self._learn_suggest: dict[str, float] = {}
        self._learn_done_pending = False
        self._learn_dialog_open = False
        self._learn_log_path: Path | None = None
        self._learn_log_fp = None
        self._learn_log_writer = None
        self._learn_log_last_t = 0.0
        self._exp_step = 1  # 1校准 2学习 3闭环试跑 4调参/换载重学
        self._await_sense = False
        self._await_phase_zero = False
        self._await_move = False
        self._await_rpmmax = False
        self._quiet_cmd_until = 0.0  # 连接初期不弹 unknown 类错误
        self._op_dialog_open = False
        self._last_op_popup = ""  # 防同一结果连弹

        self._build_ui()
        self.root.after(UI_REFRESH_MS, self._poll)
        self.root.bind("<Escape>", lambda _e: self._estop())
        self.root.bind("<space>", self._on_space)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _learn_log_dir(self) -> Path:
        d = Path(__file__).resolve().parent / "logs"
        d.mkdir(parents=True, exist_ok=True)
        return d

    def _learn_log_open(self) -> None:
        self._learn_log_close()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self._learn_log_dir() / f"learn_{ts}.csv"
        fp = open(path, "w", newline="", encoding="utf-8")
        w = csv.writer(fp)
        w.writerow(
            [
                "wall_s",
                "t_ms",
                "kind",
                "pulse_us",
                "rpm",
                "target",
                "mode",
                "run",
                "note",
            ]
        )
        fp.flush()
        self._learn_log_path = path
        self._learn_log_fp = fp
        self._learn_log_writer = w
        self._learn_log_last_t = 0.0
        self._learn_log_row(kind="session_start", note=str(path))

    def _learn_log_close(self) -> None:
        if self._learn_log_fp is not None:
            try:
                self._learn_log_row(kind="session_end")
                self._learn_log_fp.flush()
                self._learn_log_fp.close()
            except OSError:
                pass
        self._learn_log_fp = None
        self._learn_log_writer = None

    def _learn_log_row(
        self,
        *,
        kind: str,
        pulse_us: float | int | str = "",
        rpm: float | int | str = "",
        target: float | int | str = "",
        mode: float | int | str = "",
        run: float | int | str = "",
        t_ms: float | int | str = "",
        note: str = "",
    ) -> None:
        w = self._learn_log_writer
        if w is None:
            return
        try:
            w.writerow(
                [
                    f"{time.time():.3f}",
                    t_ms,
                    kind,
                    pulse_us,
                    rpm,
                    target,
                    mode,
                    run,
                    note.replace("\n", " ")[:200],
                ]
            )
            if self._learn_log_fp is not None:
                self._learn_log_fp.flush()
        except OSError:
            pass

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(
            top, textvariable=self.port_var, width=14, state="readonly"
        )
        self.port_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT)
        self.btn_connect = ttk.Button(top, text="连接", command=self._toggle_connect)
        self.btn_connect.pack(side=tk.LEFT, padx=(12, 0))
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=12)

        body = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(1, weight=1)

        mid = ttk.Frame(body)
        mid.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))

        self.deg_var = tk.StringVar(value="— °")
        self.rpm_var = tk.StringVar(value="实测: — RPM")
        self.rpm_target_disp = tk.StringVar(value="目标: — RPM")
        self.enc_info_var = tk.StringVar(
            value="编码器: AS5047P 一圈 16384 点（14bit）"
        )
        self.thr_var = tk.StringVar(value="油门: — μs / — %")
        self.raw_var = tk.StringVar(value="raw: —")
        self.diag_var = tk.StringVar(value="诊断: —")
        self.esc_var = tk.StringVar(value="ESC: —")
        self.ack_var = tk.StringVar(value="指令反馈: —")

        ttk.Label(mid, textvariable=self.deg_var, font=("Segoe UI", 24, "bold")).pack(
            side=tk.LEFT, padx=(0, 16)
        )
        rpm_box = ttk.Frame(mid)
        rpm_box.pack(side=tk.LEFT, padx=(0, 16))
        ttk.Label(
            rpm_box, textvariable=self.rpm_var, font=("Segoe UI", 20, "bold")
        ).pack(anchor=tk.W)
        ttk.Label(
            rpm_box, textvariable=self.rpm_target_disp, font=("Segoe UI", 12)
        ).pack(anchor=tk.W)
        ttk.Label(
            rpm_box, textvariable=self.enc_info_var, font=("Segoe UI", 9), foreground="#666"
        ).pack(anchor=tk.W)
        ttk.Label(
            mid,
            textvariable=self.thr_var,
            font=("Segoe UI", 18, "bold"),
            foreground="#0a5a9c",
        ).pack(side=tk.LEFT, padx=(0, 16))
        side = ttk.Frame(mid)
        side.pack(side=tk.LEFT)
        ttk.Label(side, textvariable=self.raw_var).pack(anchor=tk.W)
        ttk.Label(side, textvariable=self.diag_var).pack(anchor=tk.W)
        ttk.Label(side, textvariable=self.esc_var).pack(anchor=tk.W)
        ttk.Label(
            side, textvariable=self.ack_var, foreground="#0a7a2f", wraplength=520
        ).pack(anchor=tk.W)

        # 左曲线 | 右设置（可拖动）；设置区两列、三块可调大小
        main_pw = ttk.Panedwindow(body, orient=tk.HORIZONTAL)
        main_pw.grid(row=1, column=0, columnspan=2, sticky="nsew")

        left = ttk.Frame(main_pw, padding=0)
        right = ttk.Frame(main_pw, padding=0)
        main_pw.add(left, weight=3)
        main_pw.add(right, weight=2)

        ctrl = ttk.Frame(left)
        ctrl.pack(fill=tk.X)
        self.btn_rec = ttk.Button(
            ctrl, text="开始记录", command=self._toggle_record, state=tk.DISABLED
        )
        self.btn_rec.pack(side=tk.LEFT)
        self.btn_export = ttk.Button(
            ctrl, text="导出 CSV", command=self._export_csv, state=tk.DISABLED
        )
        self.btn_export.pack(side=tk.LEFT, padx=8)
        ttk.Button(ctrl, text="清空曲线", command=self._clear_plot).pack(side=tk.LEFT)
        self.rec_info = tk.StringVar(value="记录: 未开始")
        ttk.Label(ctrl, textvariable=self.rec_info).pack(side=tk.LEFT, padx=12)

        plot_frame = ttk.Frame(left)
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        self.fig = Figure(figsize=(8, 4), dpi=100)
        gs = self.fig.add_gridspec(1, 2, width_ratios=[3.0, 1.0], wspace=0.3)
        self.ax = self.fig.add_subplot(gs[0, 0])
        fp = dict(fontproperties=_CN_FONT) if _CN_FONT is not None else {}
        self.ax.set_title("转速 vs 时间", **fp)
        self.ax.set_xlabel("时间 (s)", **fp)
        self.ax.set_ylabel("转速 (RPM)", **fp)
        self.ax.grid(True, alpha=0.3)
        (self.line,) = self.ax.plot([], [], color="#1f77b4", linewidth=1.4)
        self.ax_dial = self.fig.add_subplot(gs[0, 1], projection="polar")
        self._setup_phase_dial()
        self.fig.subplots_adjust(left=0.07, right=0.98, top=0.90, bottom=0.12, wspace=0.35)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 右：两列 — 左=转速控制；右竖分=测量校准 | 频率调试
        right_h = ttk.Panedwindow(right, orient=tk.HORIZONTAL)
        right_h.pack(fill=tk.BOTH, expand=True)

        pane_run = ttk.LabelFrame(right_h, text="① 转速控制", padding=6)
        pane_right = ttk.Frame(right_h)
        right_h.add(pane_run, weight=1)
        right_h.add(pane_right, weight=1)

        right_v = ttk.Panedwindow(pane_right, orient=tk.VERTICAL)
        right_v.pack(fill=tk.BOTH, expand=True)
        pane_meas = ttk.LabelFrame(right_v, text="② 校准 / 测量 / PID", padding=6)
        pane_freq = ttk.LabelFrame(right_v, text="③ 频率调试", padding=6)
        right_v.add(pane_meas, weight=1)
        right_v.add(pane_freq, weight=1)

        run_inner = self._scrollable(pane_run)
        meas_inner = self._scrollable(pane_meas)
        freq_inner = self._scrollable(pane_freq)

        self._build_run_panel(run_inner)
        self._build_measure_panel(meas_inner)
        self._build_freq_debug_panel(freq_inner)

        self._refresh_ports(prefer_default=True)
        if AUTO_CONNECT and self.port_var.get().upper() == DEFAULT_PORT.upper():
            self.root.after(200, self._connect)

    def _scrollable(self, parent: ttk.LabelFrame) -> ttk.Frame:
        """LabelFrame 内嵌可滚动区域。"""
        wrap = ttk.Frame(parent)
        wrap.pack(fill=tk.BOTH, expand=True)
        canvas = tk.Canvas(wrap, highlightthickness=0, borderwidth=0)
        vsb = ttk.Scrollbar(wrap, orient=tk.VERTICAL, command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind(
            "<Configure>",
            lambda e, c=canvas: c.configure(scrollregion=c.bbox("all")),
        )
        win = canvas.create_window((0, 0), window=inner, anchor=tk.NW)

        def _on_canvas_configure(event, c=canvas, w=win):
            c.itemconfigure(w, width=max(event.width, 1))

        canvas.bind("<Configure>", _on_canvas_configure)
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)

        def _wheel(event, c=canvas):
            c.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind("<Enter>", lambda e, c=canvas: c.bind_all("<MouseWheel>", _wheel))
        canvas.bind("<Leave>", lambda e, c=canvas: c.unbind_all("<MouseWheel>"))
        return inner

    def _build_run_panel(self, parent: ttk.Frame) -> None:
        self.speed_dial = SpeedDial(parent, on_change=self._on_dial_rpm, size=170)
        self.speed_dial.pack(pady=(0, 6))

        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="目标 RPM").pack(side=tk.LEFT)
        self.sp_var = tk.StringVar(value="0")
        self.sp_entry = ttk.Entry(row, textvariable=self.sp_var, width=8)
        self.sp_entry.pack(side=tk.LEFT, padx=6)
        self.sp_entry.bind("<Return>", self._on_sp_entry)
        self.sp_entry.bind("<FocusOut>", self._on_sp_entry)

        btn_row = ttk.Frame(parent)
        btn_row.pack(fill=tk.X, pady=6)
        self.btn_start = ttk.Button(btn_row, text="启动", command=self._start)
        self.btn_start.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self.btn_stop = ttk.Button(btn_row, text="停止", command=self._stop)
        self.btn_stop.pack(side=tk.LEFT, expand=True, fill=tk.X)
        # 防止空格在停止按钮上二次触发
        self.btn_stop.bind("<Key-space>", lambda e: "break")
        self.btn_start.bind("<Key-space>", lambda e: "break")

        self.btn_estop = tk.Button(
            parent,
            text="急停 ESC",
            command=self._estop,
            bg="#c0392b",
            fg="white",
            activebackground="#922b21",
            font=("Segoe UI", 11, "bold"),
            relief=tk.RAISED,
            height=2,
        )
        self.btn_estop.pack(fill=tk.X, pady=(0, 6))

        self.soft_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            parent,
            text="缓启动",
            variable=self.soft_var,
            command=self._on_soft_toggle,
        ).pack(anchor=tk.W)

        soft_row = ttk.Frame(parent)
        soft_row.pack(fill=tk.X, pady=2)
        ttk.Label(soft_row, text="斜率 RPM/s").pack(side=tk.LEFT)
        self.soft_rate_var = tk.StringVar(value="500")
        ttk.Entry(soft_row, textvariable=self.soft_rate_var, width=7).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(soft_row, text="应用", command=self._apply_soft_rate).pack(side=tk.LEFT)

        lim_row = ttk.Frame(parent)
        lim_row.pack(fill=tk.X, pady=4)
        ttk.Label(lim_row, text="电机最大RPM").pack(side=tk.LEFT)
        self.rpm_limit_var = tk.StringVar(value=f"{int(MOTOR_RPM_LIMIT_DEFAULT)}")
        ttk.Entry(lim_row, textvariable=self.rpm_limit_var, width=7).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(lim_row, text="下发", command=self._apply_rpm_limit).pack(side=tk.LEFT)
        ttk.Label(
            parent,
            text="学习规则：未到此转速就继续加油门；\n油门到 2000μs 仍未到则记录实测最高转速",
            foreground="#666",
            wraplength=220,
        ).pack(anchor=tk.W)
        self.learn_peak_var = tk.StringVar(value="学习峰值: 尚未学习")
        ttk.Label(
            parent, textvariable=self.learn_peak_var, foreground="#0a5a9c", wraplength=220
        ).pack(anchor=tk.W, pady=(2, 0))

        self.motor_status = tk.StringVar(value="电机: Idle")
        ttk.Label(parent, textvariable=self.motor_status, wraplength=200).pack(
            anchor=tk.W, pady=(10, 0)
        )

        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)
        ttk.Label(parent, text="相位 / 停机角", font=("Segoe UI", 9, "bold")).pack(
            anchor=tk.W
        )
        self.phase_info = tk.StringVar(value="相对相位: —°  |  绝对: —°")
        ttk.Label(parent, textvariable=self.phase_info, wraplength=220).pack(anchor=tk.W)
        self.sense_info = tk.StringVar(value="电调转向: 未辨识（请先点辨识）")
        ttk.Label(parent, textvariable=self.sense_info, foreground="#0a5a9c", wraplength=220).pack(
            anchor=tk.W
        )
        ttk.Label(
            parent,
            text="单向电调：选 CW/CCW 表示「期望转到的方位」，\n"
            "系统自动换算成电调唯一能转的方向路径",
            foreground="#666",
            wraplength=220,
        ).pack(anchor=tk.W, pady=(2, 4))

        ttk.Button(
            parent, text="当前位置设为 0°", command=self._phase_zero
        ).pack(fill=tk.X, pady=2)
        ttk.Button(
            parent, text="辨识电调转向（短给油）", command=self._sense_auto
        ).pack(fill=tk.X, pady=2)

        move_row = ttk.Frame(parent)
        move_row.pack(fill=tk.X, pady=2)
        ttk.Label(move_row, text="期望").pack(side=tk.LEFT)
        self.phase_dir_var = tk.StringVar(value="CW")
        ttk.Combobox(
            move_row,
            textvariable=self.phase_dir_var,
            values=["CW", "CCW"],
            width=5,
            state="readonly",
        ).pack(side=tk.LEFT, padx=4)
        self.phase_move_deg_var = tk.StringVar(value="90")
        ttk.Entry(move_row, textvariable=self.phase_move_deg_var, width=6).pack(
            side=tk.LEFT
        )
        ttk.Label(move_row, text="°").pack(side=tk.LEFT)

        crawl_row = ttk.Frame(parent)
        crawl_row.pack(fill=tk.X, pady=2)
        ttk.Label(crawl_row, text="爬行RPM").pack(side=tk.LEFT)
        self.phase_crawl_var = tk.StringVar(value="300")
        ttk.Entry(crawl_row, textvariable=self.phase_crawl_var, width=7).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(crawl_row, text="开始转动", command=self._phase_move).pack(
            side=tk.LEFT
        )
        ttk.Label(
            parent,
            text="辨识默认 1400μs≈40% 油门；仍不转把爬行栏改成 1500~1600 再辨识",
            foreground="#666",
            wraplength=220,
        ).pack(anchor=tk.W)

        stop_row = ttk.Frame(parent)
        stop_row.pack(fill=tk.X, pady=4)
        ttk.Label(stop_row, text="停机相位").pack(side=tk.LEFT)
        self.stopat_deg_var = tk.StringVar(value="0")
        ttk.Entry(stop_row, textvariable=self.stopat_deg_var, width=6).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Label(stop_row, text="°").pack(side=tk.LEFT)
        ttk.Button(stop_row, text="启用", command=self._stopat_on).pack(
            side=tk.LEFT, padx=(6, 2)
        )
        ttk.Button(stop_row, text="关", command=lambda: self._send("STOPAT OFF")).pack(
            side=tk.LEFT
        )

        goto_row = ttk.Frame(parent)
        goto_row.pack(fill=tk.X, pady=2)
        ttk.Button(goto_row, text="转到停机相位", command=self._phase_goto_stop).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(goto_row, text="停止转动", command=self._stop).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )

        ttk.Label(
            parent,
            text="空格=停止  Esc=急停\n拖分隔条可调各区大小",
            foreground="#666",
        ).pack(anchor=tk.W, pady=(6, 0))

    def _build_measure_panel(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="电调行程校准", font=("Segoe UI", 9, "bold")).pack(
            anchor=tk.W
        )
        ttk.Label(
            parent,
            text="高油门→上电→低油门→完成",
            foreground="#666",
            wraplength=220,
        ).pack(anchor=tk.W)
        esc_row = ttk.Frame(parent)
        esc_row.pack(fill=tk.X, pady=4)
        ttk.Button(esc_row, text="高油门", command=self._esccal_high).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2)
        )
        ttk.Button(esc_row, text="低油门", command=self._esccal_low).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        ttk.Button(esc_row, text="完成", command=self._esccal_done).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0)
        )
        self.cal_pulse_var = tk.StringVar(value="校准脉宽: 未开始（看顶部 ESC: pulse=）")
        ttk.Label(parent, textvariable=self.cal_pulse_var, foreground="#0a7a2f").pack(
            anchor=tk.W, pady=(2, 0)
        )

        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        ttk.Label(parent, text="自动学习（脉宽台阶上升）", font=("Segoe UI", 9, "bold")).pack(
            anchor=tk.W
        )
        self.exp_step_var = tk.StringVar(
            value="实验步骤: ①电调校准 → ②自动学习 → ③低转速闭环试跑"
        )
        ttk.Label(
            parent, textvariable=self.exp_step_var, foreground="#0a5a9c", wraplength=220
        ).pack(anchor=tk.W, pady=(0, 4))
        ttk.Label(
            parent,
            text="点「开始自动学习」：油门台阶升高，\n"
            "直到接近「电机最大RPM」或顶到 2000μs",
            foreground="#666",
            wraplength=220,
        ).pack(anchor=tk.W)
        mbtn = ttk.Frame(parent)
        mbtn.pack(fill=tk.X, pady=2)
        ttk.Button(mbtn, text="开始自动学习", command=self._measure_auto).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4)
        )
        ttk.Button(mbtn, text="仅手动", command=self._measure_manual).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        ttk.Button(mbtn, text="停止学习", command=self._measure_stop).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )

        self.pulse_var = tk.IntVar(value=1000)
        self.pulse_lbl = tk.StringVar(value="油门: 1000 μs / 0.0%（学习中会自动变）")
        self._pulse_scale_busy = True
        ttk.Label(parent, text="当前脉宽 μs（也可手动微调）").pack(anchor=tk.W, pady=(4, 0))
        self.pulse_scale = ttk.Scale(
            parent,
            from_=1000,
            to=2000,
            orient=tk.HORIZONTAL,
            command=self._on_pulse_scale,
        )
        self.pulse_scale.set(1000)
        self.pulse_scale.pack(fill=tk.X)
        ttk.Label(parent, textvariable=self.pulse_lbl).pack(anchor=tk.W)
        self._pulse_scale_busy = False

        pstep = ttk.Frame(parent)
        pstep.pack(fill=tk.X, pady=2)
        ttk.Button(pstep, text="−50", command=lambda: self._pulse_nudge(-50)).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2)
        )
        ttk.Button(pstep, text="+50", command=lambda: self._pulse_nudge(50)).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        ttk.Button(pstep, text="手动HOLD", command=lambda: self._send("MEASURE HOLD")).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0)
        )

        m2 = ttk.Frame(parent)
        m2.pack(fill=tk.X, pady=4)
        ttk.Button(m2, text="清空", command=lambda: self._send("MEASURE CLEAR")).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2)
        )
        ttk.Button(
            m2, text="保存+建议PID", command=lambda: self._send("MEASURE SAVE")
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))

        mode_row = ttk.Frame(parent)
        mode_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(mode_row, text="模式").pack(side=tk.LEFT)
        self.mode_var = tk.StringVar(value="CLOSED")
        self.mode_cb = ttk.Combobox(
            mode_row,
            textvariable=self.mode_var,
            values=["OPEN", "CLOSED", "SAFE", "MEASURE"],
            width=9,
            state="readonly",
        )
        self.mode_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(mode_row, text="切换", command=self._apply_mode).pack(side=tk.LEFT)

        prof_row = ttk.Frame(parent)
        prof_row.pack(fill=tk.X, pady=4)
        ttk.Label(prof_row, text="Profile").pack(side=tk.LEFT)
        self.prof_var = tk.StringVar(value="noload")
        ttk.Combobox(
            prof_row,
            textvariable=self.prof_var,
            values=["noload", "flap"],
            width=9,
            state="readonly",
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(prof_row, text="切换", command=self._apply_profile).pack(side=tk.LEFT)

        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        ttk.Label(parent, text="PID", font=("Segoe UI", 9, "bold")).pack(anchor=tk.W)
        pid_row = ttk.Frame(parent)
        pid_row.pack(fill=tk.X)
        self.kp_var = tk.StringVar(value="0.08")
        self.ki_var = tk.StringVar(value="0.25")
        self.kd_var = tk.StringVar(value="0")
        for lab, var in (("Kp", self.kp_var), ("Ki", self.ki_var), ("Kd", self.kd_var)):
            ttk.Label(pid_row, text=lab).pack(side=tk.LEFT)
            ttk.Entry(pid_row, textvariable=var, width=5).pack(side=tk.LEFT, padx=(2, 4))
        pid_btn = ttk.Frame(parent)
        pid_btn.pack(fill=tk.X, pady=4)
        ttk.Button(pid_btn, text="下发 PID", command=self._apply_pid).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(pid_btn, text="保存", command=lambda: self._send("PID SAVE")).pack(
            side=tk.LEFT
        )
        self.adapt_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            parent,
            text="自适应 ADAPT",
            variable=self.adapt_var,
            command=self._on_adapt_toggle,
        ).pack(anchor=tk.W, pady=4)

        self._pulse_scale_busy = False
        self._suggest_kp = None
        self._suggest_ki = None

    def _build_freq_debug_panel(self, parent: ttk.Frame) -> None:
        """T1：低转速逐频测试。给油 N 秒→停→恢复 50Hz→人工确认下一频。"""
        self._freq_list = [50, 100, 150, 200, 250, 300, 350, 400]
        self._freq_idx = 0
        self._freq_testing = False
        self._freq_job = None
        self._freq_results: list[str] = []

        ttk.Label(
            parent,
            text="PWM 刷新率摸底（默认安全回 50Hz）",
            font=("Segoe UI", 9, "bold"),
        ).pack(anchor=tk.W)
        ttk.Label(
            parent,
            text="流程：设频→低转速给油→到时停止→恢复50Hz\n"
            "须人工点「下一频率」才继续。先低速！",
            foreground="#666",
            wraplength=240,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(2, 8))

        self.freq_status = tk.StringVar(value="当前步骤: 50 Hz（待测）")
        ttk.Label(parent, textvariable=self.freq_status, wraplength=240).pack(anchor=tk.W)

        row_p = ttk.Frame(parent)
        row_p.pack(fill=tk.X, pady=4)
        ttk.Label(row_p, text="低转速脉宽 μs").pack(side=tk.LEFT)
        self.freq_pulse_var = tk.StringVar(value="1100")
        ttk.Entry(row_p, textvariable=self.freq_pulse_var, width=7).pack(
            side=tk.LEFT, padx=6
        )

        row_t = ttk.Frame(parent)
        row_t.pack(fill=tk.X, pady=4)
        ttk.Label(row_t, text="给油时长(秒)").pack(side=tk.LEFT)
        self.freq_hold_var = tk.StringVar(value="3")
        ttk.Entry(row_t, textvariable=self.freq_hold_var, width=7).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Label(row_t, text="默认3，可设5", foreground="#888").pack(side=tk.LEFT)

        btn1 = ttk.Frame(parent)
        btn1.pack(fill=tk.X, pady=8)
        self.btn_freq_run = ttk.Button(
            btn1, text="开始本频测试", command=self._freq_run_current
        )
        self.btn_freq_run.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self.btn_freq_next = ttk.Button(
            btn1, text="下一频率", command=self._freq_next, state=tk.DISABLED
        )
        self.btn_freq_next.pack(side=tk.LEFT, expand=True, fill=tk.X)

        ttk.Button(parent, text="中止并恢复50Hz", command=self._freq_abort).pack(
            fill=tk.X, pady=2
        )

        ttk.Label(parent, text="结果记录").pack(anchor=tk.W, pady=(8, 0))
        self.freq_pass_var = tk.StringVar(value="PASS")
        ttk.Combobox(
            parent,
            textvariable=self.freq_pass_var,
            values=["PASS", "MARGINAL", "FAIL"],
            width=12,
            state="readonly",
        ).pack(anchor=tk.W)
        ttk.Button(parent, text="记下本频结论", command=self._freq_log_result).pack(
            fill=tk.X, pady=4
        )

        self.freq_log = tk.Text(parent, height=12, width=28, font=("Consolas", 9))
        self.freq_log.pack(fill=tk.BOTH, expand=True, pady=4)
        self.freq_log.insert(tk.END, "Hz,结论,脉宽,时长,备注\n")
        self._freq_refresh_status()

    def _freq_refresh_status(self) -> None:
        hz = self._freq_list[self._freq_idx]
        n = len(self._freq_list)
        self.freq_status.set(
            f"步骤 {self._freq_idx + 1}/{n}: 待测 {hz} Hz"
            + (" | 测试中…" if self._freq_testing else "")
        )

    def _freq_parse_hold(self) -> float:
        try:
            t = float(self.freq_hold_var.get().strip())
        except ValueError:
            t = 3.0
        return max(0.5, min(30.0, t))

    def _freq_parse_pulse(self) -> int:
        try:
            us = int(float(self.freq_pulse_var.get().strip()))
        except ValueError:
            us = 1100
        return max(1000, min(1600, us))

    def _freq_run_current(self) -> None:
        if not self.link or not self.link.is_alive():
            messagebox.showwarning("提示", "请先连接串口")
            return
        if self._freq_testing:
            messagebox.showinfo("提示", "本频测试进行中")
            return
        hz = self._freq_list[self._freq_idx]
        hold = self._freq_parse_hold()
        pulse = self._freq_parse_pulse()
        if not messagebox.askokcancel(
            "频率测试",
            f"将设置 FREQ={hz} Hz，给油 {pulse} μs，保持 {hold:.1f} s 后自动停止，\n"
            f"并恢复 FREQ=50 Hz。\n确认电机侧安全后继续。",
        ):
            return

        self._freq_testing = True
        self.btn_freq_run.configure(state=tk.DISABLED)
        self.btn_freq_next.configure(state=tk.DISABLED)
        self._freq_refresh_status()
        self.motor_status.set(f"频率调试: {hz}Hz 给油中…")

        self._send(f"FREQ {hz}")
        self.root.after(150, lambda p=pulse: self._send(f"PWM {p}"))

        if self._freq_job is not None:
            try:
                self.root.after_cancel(self._freq_job)
            except Exception:  # noqa: BLE001
                pass
        self._freq_job = self.root.after(int(hold * 1000), self._freq_on_hold_done)

    def _freq_on_hold_done(self) -> None:
        self._freq_job = None
        if self.link and self.link.is_alive():
            self.link.send("PWM 1000")
            self.link.send("STOP")
            self.link.send("FREQ 50")
        self._freq_testing = False
        self.btn_freq_run.configure(state=tk.NORMAL)
        if self._freq_idx < len(self._freq_list) - 1:
            self.btn_freq_next.configure(state=tk.NORMAL)
        else:
            self.btn_freq_next.configure(state=tk.DISABLED)
        hz = self._freq_list[self._freq_idx]
        self.freq_status.set(
            f"步骤 {self._freq_idx + 1}/{len(self._freq_list)}: {hz} Hz 已停，已回 50Hz\n"
            "请观察后「记下本频结论」，再「下一频率」"
        )
        self.motor_status.set(f"频率调试: {hz}Hz 结束，已恢复 50Hz")

    def _freq_next(self) -> None:
        if self._freq_testing:
            return
        if self._freq_idx >= len(self._freq_list) - 1:
            messagebox.showinfo("完成", "已是最后一个频率 400 Hz")
            return
        nxt = self._freq_list[self._freq_idx + 1]
        if not messagebox.askokcancel(
            "下一频率",
            f"确认进入 {nxt} Hz？\n（不会自动给油，需再点「开始本频测试」）",
        ):
            return
        self._freq_idx += 1
        self.btn_freq_next.configure(state=tk.DISABLED)
        self._freq_refresh_status()
        self._send("FREQ 50")
        self._send("PWM 1000")

    def _freq_abort(self) -> None:
        if self._freq_job is not None:
            try:
                self.root.after_cancel(self._freq_job)
            except Exception:  # noqa: BLE001
                pass
            self._freq_job = None
        self._freq_testing = False
        self.btn_freq_run.configure(state=tk.NORMAL)
        self.btn_freq_next.configure(state=tk.DISABLED)
        if self.link and self.link.is_alive():
            self.link.send("ESTOP")
            self.link.send("FREQ 50")
            self.link.send("PWM 1000")
        self._freq_refresh_status()
        self.motor_status.set("频率调试: 已中止，FREQ=50")

    def _freq_log_result(self) -> None:
        hz = self._freq_list[self._freq_idx]
        hold = self._freq_parse_hold()
        pulse = self._freq_parse_pulse()
        verdict = self.freq_pass_var.get()
        line = f"{hz},{verdict},{pulse},{hold:.1f},\n"
        self.freq_log.insert(tk.END, line)
        self.freq_log.see(tk.END)
        self._freq_results.append(line)

    def _setup_phase_dial(self) -> None:
        ax = self.ax_dial
        fp = dict(fontproperties=_CN_FONT) if _CN_FONT is not None else {}
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_ylim(0, 1.15)
        ax.set_yticks([])
        ax.set_xticks([math.radians(a) for a in (0, 90, 180, 270)])
        ax.set_xticklabels(["0°", "90°", "180°", "270°"], fontsize=8, **fp)
        ax.set_title("相位", fontsize=10, pad=10, **fp)
        theta_c = [math.radians(a) for a in range(0, 361, 2)]
        ax.plot(theta_c, [1.0] * len(theta_c), color="#888", linewidth=1.0)
        (self.needle,) = ax.plot([0, 0], [0, 0.92], color="#d62728", linewidth=2.2)
        self.needle_tip = ax.plot([0], [0.92], marker="o", markersize=5, color="#d62728")[0]

    def _update_phase_dial(self, deg: float | None) -> None:
        if deg is None:
            self.needle.set_data([0, 0], [0, 0])
            self.needle_tip.set_data([0], [0])
            return
        th = math.radians(deg % 360.0)
        self.needle.set_data([th, th], [0.0, 0.92])
        self.needle_tip.set_data([th], [0.92])

    def _popup_op(self, title: str, msg: str, kind: str = "info") -> None:
        """操作结果弹窗（延后一拍，避免在串口轮询里重入）。"""
        key = f"{title}|{msg[:80]}"
        if self._op_dialog_open or key == self._last_op_popup:
            return
        self._last_op_popup = key
        self._op_dialog_open = True

        def _show() -> None:
            try:
                if kind == "error":
                    messagebox.showerror(title, msg)
                elif kind == "warning":
                    messagebox.showwarning(title, msg)
                else:
                    messagebox.showinfo(title, msg)
            finally:
                self._op_dialog_open = False
                # 稍后再清，避免固件连发两行重复弹
                self.root.after(800, lambda: setattr(self, "_last_op_popup", ""))

        self.root.after(50, _show)

    def _handle_op_feedback(self, raw: str) -> None:
        """解析板子 ACK/ERR，给用户明确成功/失败交互。"""
        text = raw.lstrip("# ").strip()

        if "ACK RPMMAX" in raw:
            self._await_rpmmax = False
            lim = "?"
            try:
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                lim = parts.get("RPMMAX", parts.get("rpm_limit", "?"))
                # ACK RPMMAX=6000 ...
                for p in raw.split():
                    if p.startswith("RPMMAX="):
                        lim = p.split("=", 1)[1]
            except (ValueError, KeyError):
                pass
            if hasattr(self, "rpm_limit_var"):
                try:
                    self.rpm_limit_var.set(str(int(float(str(lim)))))
                except ValueError:
                    pass
            self.ack_var.set(f"指令反馈: 最大转速已设为 {lim} RPM")
            self._popup_op(
                "最大转速已更新",
                f"电机允许最大转速已设为 {lim} RPM。\n\n"
                "自动学习：未接近该转速前会继续加油门；\n"
                "油门到 2000μs 仍未达到，则记录实测最高转速。",
                "info",
            )
            return

        # ---- 转向辨识 ----
        if "ERR SENSE" in raw or ("SENSE AUTO" in raw and "little motion" in raw):
            self._await_sense = False
            self.sense_info.set("电调转向: 识别失败（几乎无转动）")
            self.ack_var.set("指令反馈: 转向识别失败")
            self._popup_op(
                "转向识别失败",
                "没有检测到明显转动。\n\n"
                "可能原因：\n"
                "• 油门仍偏低（爬行栏填 1500~1600 再试）\n"
                "• 电调未解锁 / 未上动力电\n"
                "• 编码器未读到\n\n"
                f"板子回报:\n{text}",
                "error",
            )
            return

        if "ACK SENSE=" in raw and "saved" in raw:
            self._await_sense = False
            sense_neg = "SENSE=-1" in raw or "CCW/encoder-" in raw
            if sense_neg:
                self.sense_info.set("电调转向: 识别成功 → 给油时编码器减少（走 CCW 路径）")
                dir_txt = "编码器角度减少（相对用户坐标系的 CCW 路径）"
            else:
                self.sense_info.set("电调转向: 识别成功 → 给油时编码器增加（走 CW 路径）")
                dir_txt = "编码器角度增加（相对用户坐标系的 CW 路径）"
            accum = ""
            pulse = ""
            try:
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                if "accum" in parts:
                    accum = f"\n累计转角 ≈ {float(parts['accum']):.1f}°"
                if "pulse" in parts:
                    pulse = f"\n辨识油门 = {parts['pulse']} μs"
            except (ValueError, KeyError):
                pass
            self.ack_var.set(f"指令反馈: 转向识别成功 — {dir_txt}")
            self._popup_op(
                "转向识别成功",
                f"已识别电调给油时的转动方向：\n\n{dir_txt}{accum}{pulse}\n\n"
                "之后「期望 CW/CCW」会自动换算成该单向路径。",
                "info",
            )
            return

        if "ACK SENSE AUTO pulse=" in raw:
            # 开始辨识，不弹窗，只更新状态
            self.sense_info.set("电调转向: 识别中…（电机应在转）")
            return

        # ---- 相位零点 ----
        if "ACK PHASE ZERO" in raw:
            self._await_phase_zero = False
            abs_v = ""
            try:
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                if "abs" in parts:
                    abs_v = f"\n（原绝对角 {float(parts['abs']):.2f}° → 现相对 0°）"
            except (ValueError, KeyError):
                pass
            self.ack_var.set("指令反馈: 相位零点已设置")
            self._popup_op(
                "相位零点已设置",
                f"当前位置已设为 0°，并已保存到板子。{abs_v}\n\n"
                "之后相位显示、停机相位都以该零点为准。",
                "info",
            )
            return

        # ---- 转动 / 到位 ----
        if "ACK MOVE DONE" in raw:
            self._await_move = False
            reason = "完成"
            if "reason=reached" in raw:
                reason = "已转到目标相位"
            elif "reason=wrong_dir" in raw:
                reason = "方向异常已处理"
            accum = ""
            try:
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                if "accum" in parts and "target" in parts:
                    accum = f"\n累计 {float(parts['accum']):.1f}° / 目标 {float(parts['target']):.1f}°"
            except (ValueError, KeyError):
                pass
            self.ack_var.set(f"指令反馈: {reason}")
            self._popup_op("转动完成", f"{reason}。{accum}\n\n电机已停在最低油门。", "info")
            return

        if "ACK MOVE skip" in raw or "already at" in raw:
            self._await_move = False
            self._popup_op("无需转动", "已在目标相位附近，未再给油。", "info")
            return

        if "UNI " in raw and "travel=" in raw:
            # 换算结果：简短确认弹窗
            try:
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.replace("→", " ").split() if "=" in p}
                travel = parts.get("travel", "?")
                via = "CW" if "via=CW" in raw else ("CCW" if "via=CCW" in raw else "?")
                target = parts.get("target", "?")
                self.ack_var.set(f"指令反馈: 单向换算 travel={travel}° via={via} → 目标 {target}°")
            except (ValueError, KeyError):
                pass
            return

        if "ACK STOPAT hit" in raw:
            self._popup_op(
                "到达停机相位",
                f"已检测到停机相位，电机已停止。\n\n{text}",
                "info",
            )
            return

        if "ACK STOPAT ON" in raw:
            self._popup_op("停机相位已启用", f"运行中经过该相位会自动停。\n\n{text}", "info")
            return

        if "ACK STOPAT OFF" in raw:
            self._popup_op("停机相位已关闭", "已取消按相位自动停机。", "info")
            return

        if "ACK ESCCAL DONE" in raw:
            self._popup_op(
                "电调校准完成",
                "行程校准流程已结束，可进行自动学习。",
                "info",
            )
            return

        # ---- 通用错误 ----
        if "ERR" in raw and ("# ERR" in raw or raw.lstrip("# ").startswith("ERR")):
            if "ERR SENSE" in raw or "little motion" in raw:
                return  # 已在上面处理
            # unknown / 连接期杂讯：只写状态栏，绝不弹窗（弹窗会叮一声）
            if "unknown:" in raw or time.time() < self._quiet_cmd_until:
                self.ack_var.set(f"指令反馈: {text[:140]}")
                return
            if self._await_rpmmax and "RPMMAX" not in raw:
                self.ack_var.set(f"指令反馈: {text[:140]}")
                return
            # 仅在用户明确等待某操作时弹错误
            if not (self._await_sense or self._await_move or self._await_phase_zero or self._await_rpmmax):
                self.ack_var.set(f"指令反馈: {text[:140]}")
                return
            self._await_sense = False
            self._await_move = False
            self._await_phase_zero = False
            if self._await_rpmmax:
                self._await_rpmmax = False
            self._popup_op("操作失败", f"板子返回错误：\n\n{text}", "error")
            return

        if "ACK STOP" in raw:
            self.ack_var.set("指令反馈: 已停止")
            self.motor_status.set("电机: 已停止")
            return

        if "WARN flipped esc_sense" in raw:
            self._popup_op(
                "已自动纠正转向",
                f"检测到转向与记录不符，已翻转并保存。\n\n{text}",
                "warning",
            )
            return

    def _phase_zero(self) -> None:
        if not messagebox.askokcancel(
            "相位零点",
            "将把「当前编码器位置」记为 0°。\n"
            "之后界面相位、停机相位都以该零点为参考。\n继续？",
        ):
            return
        self.ack_var.set("指令反馈: 设置相位零点…")
        self._await_phase_zero = True
        self._send("PHASE ZERO")

    def _sense_auto(self) -> None:
        # 默认用固定脉宽 1400μs；过低会转不动
        pulse = 1400
        try:
            v = float(self.phase_crawl_var.get())
            if v >= 1000:
                pulse = int(max(1250, min(1700, v)))
            else:
                pulse = 1400
        except ValueError:
            pulse = 1400
        if not messagebox.askokcancel(
            "辨识电调转向",
            f"将直接输出约 {pulse} μs（约 {(pulse-1000)/10:.0f}% 油门）约 1.8 秒，\n"
            "根据编码器增减判断转向。请确认机械安全。继续？",
        ):
            return
        self._await_sense = True
        self.ack_var.set(f"指令反馈: 正在识别转向… pulse={pulse}μs")
        self.sense_info.set("电调转向: 识别中…（看顶部油门是否≈1400）")
        self._send(f"SENSE AUTO {pulse}")
        # 超时未回报
        self.root.after(4000, self._sense_timeout_check)

    def _sense_timeout_check(self) -> None:
        if not self._await_sense:
            return
        self._await_sense = False
        self.sense_info.set("电调转向: 识别超时（无收到结果）")
        self._popup_op(
            "转向识别超时",
            "超过约 4 秒未收到板子识别结果。\n"
            "请看顶部油门是否曾升到约 1400μs，并确认串口仍连接。",
            "warning",
        )

    def _phase_move(self) -> None:
        try:
            deg = float(self.phase_move_deg_var.get())
            rpm = float(self.phase_crawl_var.get())
        except ValueError:
            messagebox.showwarning("提示", "角度 / 爬行 RPM 无效")
            return
        if deg <= 0 or deg > 7200:
            messagebox.showwarning("提示", "角度范围 0.5° ~ 7200°")
            return
        if rpm >= 1000:
            # 用户填了脉宽，爬行仍用 RPM 默认
            rpm = 300.0
        direction = self.phase_dir_var.get().strip().upper()
        if direction not in ("CW", "CCW"):
            direction = "CW"
        if not messagebox.askokcancel(
            "转到期望方位",
            f"期望相对 {direction} {deg:.1f}°。\n"
            "单向电调：板子会自动换算成唯一转向路径并停在目标相位。\n"
            f"爬行约 {rpm:.0f} RPM。继续？",
        ):
            return
        self._await_move = True
        self.ack_var.set(f"指令反馈: 正在转动（期望 {direction} {deg:.1f}°）…")
        self._send(f"MOVE {direction} {deg:.2f} {rpm:.1f}")

    def _stopat_on(self) -> None:
        try:
            deg = float(self.stopat_deg_var.get())
        except ValueError:
            messagebox.showwarning("提示", "停机相位无效")
            return
        deg = deg % 360.0
        self.stopat_deg_var.set(f"{deg:.1f}")
        self.ack_var.set(f"指令反馈: 正在启用停机相位 {deg:.1f}°…")
        self._send(f"STOPAT {deg:.2f}")

    def _phase_goto_stop(self) -> None:
        try:
            deg = float(self.stopat_deg_var.get())
            rpm = float(self.phase_crawl_var.get())
        except ValueError:
            messagebox.showwarning("提示", "停机相位 / 爬行 RPM 无效")
            return
        if rpm >= 1000:
            rpm = 300.0
        deg = deg % 360.0
        if not messagebox.askokcancel(
            "转到停机相位",
            f"将沿电调单向路径转到相对相位 {deg:.1f}° 后停止。\n继续？",
        ):
            return
        self._await_move = True
        self._send(f"STOPAT {deg:.2f}")
        self.ack_var.set(f"指令反馈: 正在转到 {deg:.1f}°…")
        self._send(f"GOTO {deg:.2f} {rpm:.1f}")

    def _esccal_high(self) -> None:
        self.ack_var.set("指令反馈: 正在发送 ESCCAL HIGH…")
        self.cal_pulse_var.set("校准脉宽: 期望 2000 μs（等待板子 ACK）")
        self._send("ESCCAL HIGH")

    def _esccal_low(self) -> None:
        self.ack_var.set("指令反馈: 正在发送 ESCCAL LOW…")
        self.cal_pulse_var.set("校准脉宽: 期望 1000 μs（等待板子 ACK）")
        self._send("ESCCAL LOW")

    def _esccal_done(self) -> None:
        self.ack_var.set("指令反馈: 正在发送 ESCCAL DONE…")
        self._send("ESCCAL DONE")

    def _send(self, cmd: str, *, quiet: bool = False) -> None:
        if not self.link or not self.link.is_alive():
            if not quiet:
                messagebox.showwarning("提示", "请先连接串口")
            else:
                self.ack_var.set("指令反馈: 未连接串口")
            return
        self.link.send(cmd)

    @staticmethod
    def _throttle_pct(pulse_us: int | float) -> float:
        """1000 μs = 0%，2000 μs = 100%。"""
        return max(0.0, min(100.0, (float(pulse_us) - 1000.0) / 10.0))

    def _format_throttle(self, pulse_us: int | float, tag: str = "") -> str:
        pct = self._throttle_pct(pulse_us)
        base = f"油门: {int(round(float(pulse_us)))} μs / {pct:.1f}%"
        return f"{base}（{tag}）" if tag else base

    def _update_throttle_display(self, pulse: int | float, mode: int | None = None) -> None:
        """顶部大字 + 测量区滑条旁，始终同步当前油门。"""
        tag = ""
        learning = mode == 3 or getattr(self, "_learn_done_pending", False)
        if mode == 3:
            tag = "学习中"
        elif mode == 4:
            tag = "测量"
        if hasattr(self, "thr_var"):
            self.thr_var.set(self._format_throttle(pulse, tag if learning else ""))
        if hasattr(self, "pulse_lbl"):
            self.pulse_lbl.set(self._format_throttle(pulse, tag))
        if hasattr(self, "pulse_scale"):
            # 学习中：只更新显示，不带动滑条（避免误发 PWM 抢油门）
            if learning:
                try:
                    self.pulse_scale.state(["disabled"])
                except tk.TclError:
                    pass
                return
            try:
                self.pulse_scale.state(["!disabled"])
            except tk.TclError:
                pass
            self._pulse_scale_busy = True
            try:
                self.pulse_scale.set(float(pulse))
            finally:
                # 延后清 busy，避免 ttk.Scale 异步 command 误发 PWM
                self.root.after_idle(self._clear_pulse_scale_busy)

    def _clear_pulse_scale_busy(self) -> None:
        self._pulse_scale_busy = False

    def _set_setpoint(self, rpm: float, send: bool = True) -> None:
        rpm = max(0.0, min(RPM_MAX, rpm))
        self._setpoint = rpm
        self._syncing_sp = True
        self.sp_var.set(f"{rpm:.0f}")
        self.speed_dial.set_rpm(rpm, notify=False)
        self._syncing_sp = False
        if send and self.link and self.link.is_alive():
            self.link.send(f"RPM {rpm:.1f}")

    def _on_dial_rpm(self, rpm: float) -> None:
        if self._syncing_sp:
            return
        self._set_setpoint(rpm, send=True)

    def _on_sp_entry(self, _event=None) -> None:
        if self._syncing_sp:
            return
        try:
            rpm = float(self.sp_var.get().strip())
        except ValueError:
            messagebox.showwarning("提示", f"请输入 0~{int(RPM_MAX)} 整数/小数")
            self.sp_var.set(f"{self._setpoint:.0f}")
            return
        if rpm < 0 or rpm > RPM_MAX:
            messagebox.showwarning("提示", f"目标转速范围 0~{int(RPM_MAX)}")
            self.sp_var.set(f"{self._setpoint:.0f}")
            return
        self._set_setpoint(rpm, send=True)

    def _start(self) -> None:
        self._on_sp_entry()
        self._on_soft_toggle()
        # 要转速跟上目标，应用闭环；开环只是按表给油门，实测会偏
        if self.mode_var.get() == "OPEN":
            if messagebox.askyesno(
                "开环模式",
                "当前是 OPEN 开环：目标 RPM 只用来查油门表，\n"
                "实测转速通常不会等于设定值（例如设 600、测到 3000）。\n\n"
                "是否切换到 CLOSED 闭环再启动？",
            ):
                self.mode_var.set("CLOSED")
                self._send("MODE CLOSED")
        self._send("START")

    def _stop(self) -> None:
        """立即停转。不弹窗（避免系统叮一声）。"""
        self._syncing_sp = True
        try:
            self._setpoint = 0.0
            self.sp_var.set("0")
            if hasattr(self, "speed_dial"):
                self.speed_dial.set_rpm(0.0, notify=False)
        finally:
            self._syncing_sp = False
        # 发 STOP（固件侧已改为立即拉最低油门）；不走会弹窗的 _send
        if self.link and self.link.is_alive():
            self.link.send("STOP")
            self.link.send("RPM 0")
            self.ack_var.set("指令反馈: 已发送停止")
            self.motor_status.set("电机: 停止中…")
        else:
            self.ack_var.set("指令反馈: 未连接，无法停止")

    def _estop(self) -> None:
        self._set_setpoint(0.0, send=False)
        if self.link and self.link.is_alive():
            self.link.send("ESTOP")
        self.motor_status.set("电机: ESTOP")

    def _on_space(self, event) -> None:
        w = event.widget
        # 避免焦点在按钮上时空格既触发按钮又触发绑定 → 系统叮一声
        try:
            cls = w.winfo_class()
        except Exception:
            cls = ""
        if cls in ("TButton", "Button", "TEntry", "Entry", "TCombobox", "Text"):
            return
        if isinstance(w, (ttk.Entry, tk.Entry, ttk.Button, tk.Button)):
            return
        self._stop()
        return "break"

    def _on_soft_toggle(self) -> None:
        self._send("SOFT ON" if self.soft_var.get() else "SOFT OFF")

    def _apply_rpm_limit(self) -> None:
        try:
            v = float(self.rpm_limit_var.get())
        except ValueError:
            messagebox.showwarning("提示", "最大转速无效")
            return
        if v < 100 or v > MOTOR_RPM_LIMIT_DEFAULT:
            messagebox.showwarning("提示", f"最大转速范围 100~{int(MOTOR_RPM_LIMIT_DEFAULT)}")
            return
        self.rpm_limit_var.set(f"{v:.0f}")
        self._await_rpmmax = True
        self._send(f"RPMMAX {v:.0f}")
        self.ack_var.set(f"指令反馈: 正在下发电机最大转速 {v:.0f} RPM…")
        # 成功弹窗等 ACK RPMMAX，避免把别的 ERR 误当成这次操作

    def _apply_soft_rate(self) -> None:
        try:
            r = float(self.soft_rate_var.get())
        except ValueError:
            messagebox.showwarning("提示", "斜率无效")
            return
        self._send(f"SOFT RATE {r:.1f}")

    def _apply_mode(self) -> None:
        self._send(f"MODE {self.mode_var.get()}")

    def _apply_profile(self) -> None:
        self._send(f"PROFILE {self.prof_var.get()}")

    def _measure_manual(self) -> None:
        """仅进入手动测量（不自动升油门）。"""
        self._apply_profile()
        self.ack_var.set("指令反馈: 进入手动测量，请用滑条/+50 调脉宽")
        self._send("MEASURE START")
        self._send("PWM 1000")

    def _measure_stop(self) -> None:
        self.ack_var.set("指令反馈: 停止学习…")
        self._learn_log_row(kind="cmd", note="LEARN ABORT")
        self._send("LEARN ABORT")
        self._send("MEASURE ABORT")
        self._send("PWM 1000")
        self._learn_done_pending = False
        self._learn_log_close()

    def _measure_auto(self) -> None:
        if getattr(self, "_learn_done_pending", False):
            messagebox.showwarning("提示", "正在自动学习中，请先点「停止学习」")
            return
        try:
            lim = float(self.rpm_limit_var.get())
        except (ValueError, AttributeError):
            lim = MOTOR_RPM_LIMIT_DEFAULT
        if not messagebox.askokcancel(
            "自动学习",
            "将自动台阶升高油门并记录转速：\n"
            f"从 1000 μs 起，每步 +50 μs、约 2 秒，\n"
            f"直到接近电机最大转速 {lim:.0f} RPM，\n"
            "或油门顶到 2000 μs 后结束。\n\n"
            "过程会自动保存 CSV 记录到 pc/logs/。\n"
            "请确认已卸桨/固定、选对 profile。\n继续？",
        ):
            return
        self._apply_profile()
        self._learn_points.clear()
        self._learn_suggest.clear()
        self._learn_done_pending = True
        self._learn_log_open()
        self._exp_step = 2
        if hasattr(self, "exp_step_var"):
            self.exp_step_var.set(
                f"实验步骤: ②自动学习中…（未到 {lim:.0f} RPM 则继续加油门，最高 2000μs）"
            )
        # 转速上限 + 油门硬顶 2000μs（固件也会强制）
        self._send("STOP")
        self._send("RPM 0")
        self._send(f"RPMMAX {lim:.0f}")
        self._send("LEARN MAXUS 2000")
        log_hint = (
            f"记录→ {self._learn_log_path.name}"
            if self._learn_log_path
            else "记录已开"
        )
        self.ack_var.set(
            f"指令反馈: 启动学习（上限 {lim:.0f}RPM / 2000μs）{log_hint}"
        )
        self.pulse_lbl.set(self._format_throttle(1000, "学习中…"))
        self._send("MEASURE AUTO")
        self._learn_log_row(kind="cmd", note="MEASURE AUTO")

    def _parse_learn_meta(self, raw: str) -> None:
        """解析学习过程串口行，学习结束弹出结果与下一步。"""
        if self._learn_log_writer is not None and (
            "LEARN" in raw or "MEASURE" in raw or "SUGGEST" in raw or "EVAL" in raw
        ):
            self._learn_log_row(kind="meta", note=raw.lstrip("# ").strip())

        if "ACK LEARN point" in raw:
            try:
                # ... pulse=1050 rpm=120.5 n=3 ...
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                pu = int(float(parts.get("pulse", "0")))
                rpm = float(parts.get("rpm", "0"))
                self._learn_points.append((pu, rpm))
                if hasattr(self, "exp_step_var"):
                    self.exp_step_var.set(
                        f"实验步骤: ②学习中 pulse={pu} rpm={rpm:.0f} 点={len(self._learn_points)}"
                    )
            except (ValueError, KeyError):
                pass
            return

        if "SUGGEST" in raw and "PID" in raw:
            try:
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                if "gain" in parts:
                    self._learn_suggest["gain"] = float(parts["gain"])
                # "PID kp=.. ki=.." 或 HINT 行
                toks = raw.replace(",", " ").split()
                for i, t in enumerate(toks):
                    if t.lower() == "kp" and i + 1 < len(toks):
                        self._learn_suggest["kp"] = float(toks[i + 1].lstrip("="))
                    if t.startswith("kp="):
                        self._learn_suggest["kp"] = float(t.split("=", 1)[1])
                    if t.startswith("ki="):
                        self._learn_suggest["ki"] = float(t.split("=", 1)[1])
            except (ValueError, IndexError):
                pass
            return

        if "HINT apply: PID" in raw:
            try:
                parts = raw.split("PID", 1)[1].split()
                self._learn_suggest["kp"] = float(parts[0])
                self._learn_suggest["ki"] = float(parts[1])
                if len(parts) > 2:
                    self._learn_suggest["kd"] = float(parts[2])
                self.kp_var.set(f"{self._learn_suggest['kp']:.4f}")
                self.ki_var.set(f"{self._learn_suggest['ki']:.4f}")
                self.kd_var.set(f"{self._learn_suggest.get('kd', 0):.4f}")
            except (IndexError, ValueError):
                pass
            return

        if "ACK LEARN DONE" in raw or (
            self._learn_done_pending and "LEARN DONE" in raw
        ):
            self._learn_done_pending = False
            self._learn_log_row(kind="done", note=raw.lstrip("# ").strip())
            self._learn_log_close()
            # 延后一帧，等 SUGGEST/HINT 行到齐
            self.root.after(200, lambda: self._show_learn_done_dialog(raw))

    def _show_learn_done_dialog(self, raw: str) -> None:
        if self._learn_dialog_open:
            return
        self._learn_dialog_open = True
        try:
            profile = self.prof_var.get()
            pts = self._learn_points
            n = len(pts)
            rpm_max = max((r for _, r in pts), default=0.0)
            pulse_max = max((p for p, _ in pts), default=1000)
            kp = self._learn_suggest.get("kp")
            ki = self._learn_suggest.get("ki")
            gain = self._learn_suggest.get("gain")

            reason = ""
            rpm_limit = MOTOR_RPM_LIMIT_DEFAULT
            try:
                kv = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                if "points" in kv:
                    n = max(n, int(float(kv["points"])))
                if "rpm_max" in kv:
                    rpm_max = float(kv["rpm_max"])
                if "pulse_max" in kv:
                    pulse_max = int(float(kv["pulse_max"]))
                if "profile" in kv:
                    profile = kv["profile"]
                if "reason" in kv:
                    reason = kv["reason"]
                if "rpm_limit" in kv:
                    rpm_limit = float(kv["rpm_limit"])
            except (ValueError, KeyError):
                pass
            try:
                rpm_limit = float(self.rpm_limit_var.get())
            except (ValueError, AttributeError):
                pass

            table = ""
            if pts:
                show = pts if len(pts) <= 8 else pts[:: max(1, len(pts) // 8)]
                table = "\n".join(f"  {pu:4d} μs → {rpm:7.1f} RPM" for pu, rpm in show)
                if len(pts) > len(show):
                    table += f"\n  … 共 {len(pts)} 点"

            pid_line = (
                f"建议 PID: Kp={kp:.4f}  Ki={ki:.4f}"
                if kp is not None and ki is not None
                else "建议 PID: （未收到，可稍后手动测）"
            )
            gain_line = f"\n估计增益: {gain:.3f} RPM/μs" if gain is not None else ""
            cover = (100.0 * rpm_max / rpm_limit) if rpm_limit > 1 else 0.0

            if reason == "rpm_cap":
                eval_txt = (
                    f"停止原因：已接近设定最大转速 {rpm_limit:.0f} RPM\n"
                    f"（油门停在 {pulse_max} μs，不必再推高）"
                )
            elif reason == "pulse_cap" or pulse_max >= 1950:
                eval_txt = (
                    f"停止原因：油门已到最大 {pulse_max} μs\n"
                    f"在此油门下实测最高转速 = {rpm_max:.0f} RPM\n"
                    f"（设定上限 {rpm_limit:.0f}，覆盖 {cover:.0f}%）\n"
                    f"当前电池/负载下这就是可达到的峰值，已记录。"
                )
            elif reason == "map_full":
                eval_txt = f"停止原因：采样点已满；最高 {rpm_max:.0f} RPM @ {pulse_max} μs"
            else:
                eval_txt = (
                    f"最高实测 {rpm_max:.0f} RPM @ {pulse_max} μs\n"
                    f"设定上限 {rpm_limit:.0f} RPM（覆盖 {cover:.0f}%）"
                )

            if hasattr(self, "learn_peak_var"):
                self.learn_peak_var.set(
                    f"学习峰值: {rpm_max:.0f} RPM @ {pulse_max} μs"
                    f"（上限 {rpm_limit:.0f}）"
                )

            log_line = (
                f"\n转速记录已保存:\n{self._learn_log_path}\n"
                if self._learn_log_path
                else ""
            )
            analysis = ""
            try:
                from analyze_learn_log import analyze as _analyze_learn

                log_p = self._learn_log_path
                if log_p is not None and Path(log_p).exists():
                    analysis = "\n—— 自动分析 ——\n" + _analyze_learn(
                        Path(log_p), ctrl_hz_hint=float(self.ctrl_hz or 100)
                    )
                    rep = Path(log_p).with_suffix(".report.txt")
                    rep.write_text(analysis.strip() + "\n", encoding="utf-8")
            except Exception as exc:  # noqa: BLE001
                analysis = f"\n（自动分析失败: {exc}）"

            msg = (
                f"自动学习完成（profile={profile}）\n\n"
                f"{eval_txt}\n\n"
                f"采样点: {n}\n"
                f"{pid_line}{gain_line}\n"
                f"{log_line}"
                f"{analysis}\n"
            )
            if table:
                msg += f"\n部分测点:\n{table}\n"
            msg += (
                "\n—— 下一步（闭环试跑）——\n"
                "1. 应用建议 PID\n"
                "2. 设目标约 300 RPM，确认缓启动 ON\n"
                "3. 点「启动」试闭环\n\n"
                "是否现在应用 PID，并把目标设为 300 RPM？"
            )

            self.ack_var.set(
                f"指令反馈: 学习完成 峰值 {rpm_max:.0f}RPM @ {pulse_max}μs"
            )
            if hasattr(self, "exp_step_var"):
                self.exp_step_var.set(
                    f"实验步骤: ②完成({n}点) → ③点启动试闭环 300RPM"
                )
            self._exp_step = 3
            self.mode_var.set("CLOSED")

            ok = messagebox.askyesno("学习完成 — 下一步", msg)
            if ok:
                if kp is not None and ki is not None:
                    self.kp_var.set(f"{kp:.4f}")
                    self.ki_var.set(f"{ki:.4f}")
                    self.kd_var.set("0")
                    self._send(f"PID {kp:.4f} {ki:.4f} 0")
                self._send("MODE CLOSED")
                self.soft_var.set(True)
                self._send("SOFT ON")
                self._set_rpm(300.0)
                log_txt = (
                    f"\n记录已保存: {self._learn_log_path}"
                    if self._learn_log_path
                    else ""
                )
                self.ack_var.set(
                    "指令反馈: 已应用 PID + 目标300RPM。确认安全后点「启动」"
                    + log_txt
                )
                if hasattr(self, "exp_step_var"):
                    self.exp_step_var.set("实验步骤: ③目标已设300 — 点「启动」试闭环")
            else:
                log_txt = (
                    f" 记录: {self._learn_log_path.name}"
                    if self._learn_log_path
                    else ""
                )
                self.ack_var.set(
                    "指令反馈: 学习完成。可手动改 PID 后「下发」，再设 RPM 启动"
                    + log_txt
                )
        finally:
            self._learn_dialog_open = False

    def _set_rpm(self, rpm: float) -> None:
        rpm = max(0.0, min(RPM_MAX, float(rpm)))
        self._setpoint = rpm
        self._syncing_sp = True
        try:
            self.sp_var.set(f"{rpm:.0f}")
            if hasattr(self, "dial"):
                self.dial.set_rpm(rpm, notify=False)
        finally:
            self._syncing_sp = False
        self._send(f"RPM {rpm:.1f}")

    def _on_pulse_scale(self, _val=None) -> None:
        if getattr(self, "_pulse_scale_busy", False):
            return
        # 自动学习中禁止滑条改油门
        if getattr(self, "_learn_done_pending", False):
            return
        if not hasattr(self, "pulse_lbl") or not hasattr(self, "pulse_scale"):
            return
        us = int(float(self.pulse_scale.get()))
        us = max(1000, min(2000, us))
        self.pulse_var.set(us)
        self.pulse_lbl.set(self._format_throttle(us))
        if hasattr(self, "thr_var"):
            self.thr_var.set(self._format_throttle(us))
        # 松手式：拖动过程中节流发送
        now = time.time()
        if not hasattr(self, "_last_pulse_send"):
            self._last_pulse_send = 0.0
        if now - self._last_pulse_send < 0.08:
            return
        self._last_pulse_send = now
        if self.link and self.link.is_alive():
            self.link.send(f"PWM {us}")

    def _pulse_nudge(self, delta: int) -> None:
        if getattr(self, "_learn_done_pending", False):
            self.ack_var.set("指令反馈: 学习中，请用「停止学习」中止后再手动调油门")
            return
        if not hasattr(self, "pulse_scale"):
            return
        us = int(float(self.pulse_scale.get())) + delta
        us = max(1000, min(2000, us))
        self._pulse_scale_busy = True
        self.pulse_scale.set(us)
        self.root.after_idle(self._clear_pulse_scale_busy)
        if hasattr(self, "pulse_lbl"):
            self.pulse_lbl.set(self._format_throttle(us))
        if hasattr(self, "thr_var"):
            self.thr_var.set(self._format_throttle(us))
        self._send(f"PWM {us}")

    def _apply_pid(self) -> None:
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
        except ValueError:
            messagebox.showwarning("提示", "PID 数值无效")
            return
        self._send(f"PID {kp} {ki} {kd}")

    def _on_adapt_toggle(self) -> None:
        self._send("ADAPT ON" if self.adapt_var.get() else "ADAPT OFF")

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
        if self.link and self.link.is_alive():
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择串口")
            return
        self.stop_evt.clear()
        self.link = SerialLink(port, self.q, self.stop_evt)
        self.link.start()
        self.btn_connect.configure(text="断开")
        self.btn_rec.configure(state=tk.NORMAL)
        self.status_var.set(f"已连接 {port} @ {BAUD}")
        self.prev_deg = None
        self.prev_t_ms = None
        self._rx_count = 0
        self._rx_t0 = time.time()
        self._quiet_cmd_until = time.time() + 3.0
        # 等板子复位/就绪后再发配置，避免竞态
        self.root.after(800, lambda: self._send("SOFT ON" if self.soft_var.get() else "SOFT OFF"))
        self.root.after(900, lambda: self._send("FREQ 50"))
        self.root.after(1000, lambda: self._send(f"RPM {self._setpoint:.1f}"))
        self.root.after(1100, lambda: self._send("PHASE?"))
        self.root.after(
            1200,
            lambda: self._send(f"RPMMAX {float(self.rpm_limit_var.get() or 6000):.0f}"),
        )
        self.root.after(1300, lambda: self._send("LEARN MAXUS 2000"))

    def _disconnect(self) -> None:
        if self.recording:
            self._stop_record()
        if self.link and self.link.is_alive():
            try:
                self.link.send("ESTOP")
                self.link.send("FREQ 50")
            except Exception:  # noqa: BLE001
                pass
        self.stop_evt.set()
        self.link = None
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
            w.writerow(["t_s", "deg", "rad", "rpm", "raw", "pulse_us", "target_rpm"])
            for row in self.rec_rows:
                w.writerow(row)
        messagebox.showinfo("完成", f"已保存: {path}")

    def _poll(self) -> None:
        try:
            while True:
                item = self.q.get_nowait()
                if isinstance(item, tuple) and item and item[0] == "__error__":
                    messagebox.showerror("串口错误", item[1])
                    self._disconnect()
                    break
                if isinstance(item, tuple) and item and item[0] == "__meta__":
                    meta = item[1] if len(item) > 1 else {}
                    if isinstance(meta, dict):
                        if "ctrl_hz" in meta:
                            self.ctrl_hz = int(meta["ctrl_hz"])
                        raw = meta.get("raw")
                        if isinstance(raw, str):
                            if "LEARN settle" in raw and self._learn_log_writer is not None:
                                self._learn_log_row(
                                    kind="settle", note=raw.lstrip("# ").strip()
                                )
                            # 独立反馈行，不被遥测覆盖
                            if any(
                                k in raw
                                for k in (
                                    "ACK",
                                    "ESCCAL",
                                    "LEARN",
                                    "MEASURE",
                                    "HOLD",
                                    "SUGGEST",
                                    "ESTOP",
                                    "ERR",
                                    "PHASE",
                                    "MOVE",
                                    "STOPAT",
                                    "GOTO",
                                    "SENSE",
                                    "UNI",
                                    "EVAL",
                                    "LEARN RESULT",
                                )
                            ) and "LEARN settle" not in raw:
                                self.ack_var.set(f"指令反馈: {raw.lstrip('# ').strip()[:160]}")
                                if "ESCCAL HIGH" in raw:
                                    self.cal_pulse_var.set(
                                        "校准脉宽: 板子确认高油门（应见 pulse≈2000）"
                                    )
                                elif "ESCCAL LOW" in raw:
                                    self.cal_pulse_var.set(
                                        "校准脉宽: 板子确认低油门（应见 pulse≈1000）"
                                    )
                                elif "ESCCAL DONE" in raw:
                                    self.cal_pulse_var.set("校准脉宽: 完成")
                                    self._exp_step = 2
                                    if hasattr(self, "exp_step_var"):
                                        self.exp_step_var.set(
                                            "实验步骤: ①校准完成 → ②开始自动学习"
                                        )
                                self._parse_learn_meta(raw)
                                self._handle_op_feedback(raw)
                                if "HINT apply: PID" in raw and not self._learn_done_pending:
                                    try:
                                        parts = raw.split("PID", 1)[1].split()
                                        self.kp_var.set(parts[0])
                                        self.ki_var.set(parts[1])
                                        if len(parts) > 2:
                                            self.kd_var.set(parts[2])
                                    except (IndexError, ValueError):
                                        pass
                        port = self.port_var.get().strip()
                        self.status_var.set(
                            f"已连接 {port} @ {BAUD} | ctrl {self.ctrl_hz}Hz"
                        )
                    continue
                self._on_sample(item)
                self._ui_dirty = True
        except queue.Empty:
            pass

        # 心跳，避免固件 host_timeout
        now = time.time()
        if self.link and self.link.is_alive() and (now - self._last_ping) > 0.5:
            self.link.send("PING")
            self._last_ping = now

        if self._ui_dirty and (now - self._last_ui_draw) * 1000.0 >= UI_REFRESH_MS:
            self._redraw()
            self._ui_dirty = False
            self._last_ui_draw = now
        self.root.after(UI_REFRESH_MS, self._poll)

    def _on_sample(self, sample: tuple) -> None:
        (
            t_ms,
            raw,
            deg,
            rad,
            rpm_fw,
            ef,
            agc,
            mag_l,
            mag_h,
            pulse,
            target,
            mode,
            kp,
            ki,
            kd,
            profile,
            run,
        ) = sample

        self._rx_count += 1
        elapsed = time.time() - self._rx_t0
        if elapsed >= 1.0:
            self.root.title(
                f"AS5047P+ESC | ctrl{self.ctrl_hz}Hz | 收{self._rx_count / elapsed:.0f}Hz"
            )
            self._rx_count = 0
            self._rx_t0 = time.time()

        # 直接用固件锁定符号后的转速；不再本地相位差分（跨 0° 会正负乱跳）
        rpm_meas = float(rpm_fw)
        self.prev_deg = deg
        self.prev_t_ms = t_ms
        self.current_deg = deg

        self.deg_var.set(f"{deg:7.2f} °")
        abs_deg = (float(raw) * 360.0) / 16384.0
        if hasattr(self, "phase_info"):
            self.phase_info.set(
                f"相对相位: {deg:.2f}°  |  绝对: {abs_deg:.2f}°"
            )
        # 600=目标设定；3000=编码器实测 —— 二者不同！
        if abs(rpm_meas) < 5.0:
            self.rpm_var.set("实测: 0.0 RPM")
        else:
            dir_tag = "+" if rpm_meas >= 0 else "−"
            self.rpm_var.set(f"实测: {dir_tag}{abs(rpm_meas):.1f} RPM")
        self.rpm_target_disp.set(f"目标: {target:.0f} RPM")
        self.enc_info_var.set(
            f"编码器: 16384点/圈 | raw={raw} | 若实测≠目标=未跟上（查模式/学习图）"
        )
        self.raw_var.set(f"raw: {raw} / 16384")
        rpm = abs(rpm_meas)  # 曲线用大小
        self.raw_var.set(f"raw: {raw}")
        warns = []
        if ef:
            warns.append("SPI_EF")
        if mag_l:
            warns.append("MAGL")
        if mag_h:
            warns.append("MAGH")
        agc_txt = f"AGC={agc}" if agc >= 0 else "AGC=—"
        self.diag_var.set(
            f"诊断: {agc_txt}" + ((" | " + ", ".join(warns)) if warns else " | OK")
        )
        pct = self._throttle_pct(pulse)
        self.thr_var.set(self._format_throttle(pulse))
        self.esc_var.set(
            f"ESC: {int(pulse)}μs ({pct:.1f}%)  tgt={target:.0f}  "
            f"{MODE_NAMES.get(mode, mode)}/{RUN_NAMES.get(run, run)}  "
            f"{PROF_NAMES.get(profile, profile)}"
        )
        if pulse >= 1950:
            self.cal_pulse_var.set(f"校准脉宽: 当前实测 pulse={pulse} μs（高油门）")
        elif hasattr(self, "cal_pulse_var") and "期望 2000" in self.cal_pulse_var.get():
            # 已点高油门但遥测还不是 2000，提示可能未生效
            if pulse < 1500:
                self.cal_pulse_var.set(
                    f"校准脉宽: 警告 当前 pulse={pulse} μs，未到 2000（检查连接/是否收到命令）"
                )

        # 任意模式都同步油门显示（含学习/测量/开环/闭环）
        self._update_throttle_display(pulse, mode)

        # 自学习过程：约 10Hz 写 CSV，便于事后核对「升→降→升」
        if self._learn_log_writer is not None and (
            mode == 3 or self._learn_done_pending
        ):
            now_wall = time.time()
            if now_wall - self._learn_log_last_t >= 0.1:
                self._learn_log_last_t = now_wall
                self._learn_log_row(
                    kind="telem",
                    t_ms=t_ms,
                    pulse_us=pulse,
                    rpm=f"{rpm_meas:.2f}",
                    target=f"{target:.1f}",
                    mode=mode,
                    run=run,
                )

        # 不再覆盖指令反馈；PID/模式写到转速控制区状态
        self.motor_status.set(
            f"Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f} | {MODE_NAMES.get(mode)} {RUN_NAMES.get(run)}"
        )

        if self.recording:
            if self.rec_t0_ms is None:
                self.rec_t0_ms = t_ms
            if len(self.rec_rows) >= MAX_REC_POINTS:
                self._stop_record()
                messagebox.showinfo("提示", "记录已达上限，已停止")
                return
            t_s = (t_ms - self.rec_t0_ms) * 0.001
            self.rec_rows.append(
                (f"{t_s:.4f}", f"{deg:.3f}", f"{rad:.6f}", f"{rpm:.2f}", int(raw), pulse, f"{target:.1f}")
            )
            self.live_t.append(t_s)
            self.live_rpm.append(rpm)
            self.rec_info.set(f"记录中… {len(self.rec_rows)} 点  t={t_s:.2f}s")
        else:
            now_wall = time.time()
            self.live_t.append(now_wall)
            self.live_rpm.append(rpm)
            while self.live_t and (now_wall - self.live_t[0]) > LIVE_WINDOW_S:
                self.live_t.popleft()
                self.live_rpm.popleft()

    def _redraw(self) -> None:
        if self.recording or self.rec_rows:
            if self.recording:
                xs = list(self.live_t)
                ys = list(self.live_rpm)
            else:
                xs = [float(r[0]) for r in self.rec_rows]
                ys = [float(r[3]) for r in self.rec_rows]
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
        self._learn_log_close()
        if self.link and self.link.is_alive():
            try:
                self.link.send("ESTOP")
                time.sleep(0.05)
            except Exception:  # noqa: BLE001
                pass
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
