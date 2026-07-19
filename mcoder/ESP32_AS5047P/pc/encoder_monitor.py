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
import sys
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
# 启动后这段时间内不弹「请先连接串口」（等自动连接 / 用户点连接）
CONNECT_WARN_GRACE_S = 2.0
# 同类未连接提示最短间隔，避免多处 _send 连弹
CONNECT_WARN_COOLDOWN_S = 2.0
# 自动连接延迟，给 UI 建完 + 串口枚举一点时间
AUTO_CONNECT_DELAY_MS = 2000
# 启动参数 --auto-s1：连接后自动开始 S1 旁观测试
AUTO_S1_ON_START = "--auto-s1" in sys.argv
AUTO_S1_DELAY_MS = 4500
LIVE_WINDOW_S = 10.0
# 主循环要勤：保证按钮/拖窗及时进事件队列（本身几乎不干活）
POLL_MS = 33
# 数字/状态刷新（轻量 StringVar）
LABEL_REFRESH_MS = 100  # 10 Hz
# 左侧转速曲线（Matplotlib 最贵）单独更慢
PLOT_REFRESH_MS = 200  # 5 Hz
# 曲线缓冲最大写入速率，避免 250Hz 全塞进 deque
LIVE_APPEND_MIN_S = 0.02  # ≤50 Hz
# 画图前按像素降采样的目标点数
PLOT_MAX_POINTS = 800
# 单次 poll 最多处理遥测条数，避免饿死 UI
POLL_SAMPLE_BUDGET = 80
RPM_UI_DEADZONE = 0.5  # 均值 |RPM| < 此值显示为 0
# 有符号转速滑动均值窗口（正负噪声在此窗口内抵消）
RPM_MEAN_WINDOW_S = 0.20
MAX_REC_POINTS = 120000
EXPECTED_CTRL_HZ = 250
# 兼容旧名（其它注释/日志）
UI_REFRESH_MS = LABEL_REFRESH_MS
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

    @property
    def is_open(self) -> bool:
        """线程活着且串口已真正打开（is_alive 在打开失败前也会为 True）。"""
        return self.is_alive() and self._ser is not None

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
        # 先按屏幕算初始大小，避免开到接近全屏后「只能拖、拖不大」
        root.update_idletasks()
        sw = max(root.winfo_screenwidth(), 1024)
        sh = max(root.winfo_screenheight(), 700)
        init_w = min(1500, int(sw * 0.88))
        init_h = min(860, int(sh * 0.82))
        root.geometry(f"{init_w}x{init_h}")
        root.minsize(min(800, init_w), min(500, init_h))
        root.maxsize(sw, sh)
        root.resizable(True, True)

        self.q: queue.Queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.link: SerialLink | None = None

        self.recording = False
        self.rec_t0_ms: float | None = None
        self.rec_rows: list[tuple] = []

        self.live_t: deque[float] = deque()
        self.live_rpm: deque[float] = deque()
        # 有符号转速滑动窗：(wall_s, rpm_signed)，均值消正负噪声
        self._rpm_signed_hist: deque[tuple[float, float]] = deque()
        self._rpm_disp = 0.0  # 只存大小（≥0）——数字与曲线共用
        self._rpm_dir = "—"  # 正 / 逆 / —
        self.prev_deg: float | None = None
        self.prev_t_ms: float | None = None
        self.current_deg: float | None = None
        self.ctrl_hz = EXPECTED_CTRL_HZ
        self._ui_dirty = False
        self._plot_dirty = False
        self._last_label_draw = 0.0
        self._last_plot_draw = 0.0
        self._last_live_append = 0.0
        self._resize_quiet_until = 0.0
        self._last_telem: tuple | None = None
        self._rx_count = 0
        self._rx_t0 = time.time()
        self._setpoint = 0.0
        self._syncing_sp = False
        self._last_ping = 0.0
        self._learn_points: list[tuple[int, float]] = []
        self._learn_suggest: dict[str, float] = {}
        self._learn_gain_points: list[dict[str, float]] = []
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
        self._boot_after_ids: list[str] = []
        self._connect_warn_until = time.time() + CONNECT_WARN_GRACE_S
        self._last_connect_warn_at = 0.0
        self._atest_win: tk.Toplevel | None = None
        self._atest_active = False
        self._atest_abort = False
        self._atest_job = None
        self._atest_steps: list[dict] = []
        self._atest_idx = 0
        self._atest_phase = ""
        self._atest_t0 = 0.0
        self._atest_t_in: float | None = None
        self._atest_samples: list[tuple[float, float]] = []
        self._atest_soft_up_down: bool | None = None
        self._atest_last_ping = 0.0
        self._atest_results: list[str] = []
        self.atest_banner_var = tk.StringVar(value="自动测试: 空闲")

        self._build_ui()
        self.root.after(POLL_MS, self._poll)
        self.root.bind("<Escape>", lambda _e: self._estop())
        self.root.bind("<space>", self._on_space)
        self.root.bind("<Configure>", self._on_root_configure)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_root_configure(self, event) -> None:
        # 拖窗口时暂停曲线重绘，把 CPU 留给窗口管理器
        if event.widget is self.root:
            self._resize_quiet_until = time.time() + 0.20

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
        # 根布局用 grid，保证窗口拉伸时 body 跟着变
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="ew")

        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(
            top, textvariable=self.port_var, width=14, state="readonly"
        )
        self.port_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT)
        self.btn_connect = ttk.Button(top, text="连接", command=self._toggle_connect)
        self.btn_connect.pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(top, text="自动测试", command=self._open_auto_test_win).pack(
            side=tk.LEFT, padx=(8, 0)
        )
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=12)
        ttk.Label(
            top, textvariable=self.atest_banner_var, foreground="#0a5a9c"
        ).pack(side=tk.LEFT, padx=(8, 0))

        body = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)

        mid = ttk.Frame(body)
        mid.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))

        self.deg_var = tk.StringVar(value="— °")
        self.rpm_var = tk.StringVar(value="实测: — RPM")
        self.rpm_dir_var = tk.StringVar(value="—")  # 正=顺时针 逆=逆时针
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
        rpm_row = ttk.Frame(rpm_box)
        rpm_row.pack(anchor=tk.W)
        ttk.Label(
            rpm_row, textvariable=self.rpm_var, font=("Segoe UI", 20, "bold")
        ).pack(side=tk.LEFT)
        ttk.Label(
            rpm_row,
            textvariable=self.rpm_dir_var,
            font=("Segoe UI", 18, "bold"),
            foreground="#0a5a9c",
        ).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Label(
            rpm_box,
            text="正=顺时针  逆=逆时针（转速只显示大小）",
            font=("Segoe UI", 8),
            foreground="#888",
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
        self._main_pw = main_pw

        left = ttk.Frame(main_pw, padding=0)
        right = ttk.Frame(main_pw, padding=0)
        main_pw.add(left, weight=3)
        main_pw.add(right, weight=2)
        try:
            main_pw.pane(left, weight=3)
            main_pw.pane(right, weight=2)
        except tk.TclError:
            pass

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
        (self.line,) = self.ax.plot([], [], color="#1f77b4", linewidth=3.0, solid_capstyle="round")
        self.ax_dial = self.fig.add_subplot(gs[0, 1], projection="polar")
        self._setup_phase_dial()
        self.fig.subplots_adjust(left=0.07, right=0.98, top=0.90, bottom=0.12, wspace=0.35)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        plot_widget = self.canvas.get_tk_widget()
        # 与滚动区相同：不要按 Figure 像素锁定窗口最小/最大尺寸
        try:
            plot_widget.configure(width=1, height=1)
        except tk.TclError:
            pass
        plot_widget.pack(fill=tk.BOTH, expand=True)
        self._plot_frame = plot_frame

        # 右：两列 — 左=转速控制；右竖分=测量校准 | 频率调试
        right_h = ttk.Panedwindow(right, orient=tk.HORIZONTAL)
        right_h.pack(fill=tk.BOTH, expand=True)
        self._right_h = right_h

        pane_run = ttk.LabelFrame(right_h, text="① 转速控制", padding=6)
        pane_right = ttk.Frame(right_h)
        right_h.add(pane_run, weight=1)
        right_h.add(pane_right, weight=1)

        right_v = ttk.Panedwindow(pane_right, orient=tk.VERTICAL)
        right_v.pack(fill=tk.BOTH, expand=True)
        self._right_v = right_v
        pane_meas = ttk.LabelFrame(right_v, text="② 校准 / 测量 / PID", padding=6)
        pane_freq = ttk.LabelFrame(right_v, text="③ 频率调试", padding=6)
        pane_atest = ttk.LabelFrame(right_v, text="④ 自动测试监视", padding=6)
        right_v.add(pane_meas, weight=1)
        right_v.add(pane_freq, weight=1)
        right_v.add(pane_atest, weight=1)

        run_inner = self._scrollable(pane_run)
        meas_inner = self._scrollable(pane_meas)
        freq_inner = self._scrollable(pane_freq)
        atest_inner = self._scrollable(pane_atest)

        self._build_run_panel(run_inner)
        self._build_measure_panel(meas_inner)
        self._build_freq_debug_panel(freq_inner)
        self._build_auto_test_panel(atest_inner)

        # 给分隔条留可拖空间（避免子面板把 minsize 撑死）
        self.root.after_idle(self._relax_pane_minsizes)
        self.root.after_idle(self._unlock_window_resize)

        self._refresh_ports(prefer_default=True)
        if AUTO_CONNECT and self.port_var.get().upper() == DEFAULT_PORT.upper():
            self.root.after(AUTO_CONNECT_DELAY_MS, self._connect)
            if AUTO_S1_ON_START:
                self.root.after(AUTO_S1_DELAY_MS, lambda: self._atest_start_s1(skip_confirm=True))

    def _unlock_window_resize(self) -> None:
        """布局完成后再次声明可缩放，覆盖控件撑出的隐式最小尺寸。"""
        try:
            sw = max(self.root.winfo_screenwidth(), 1024)
            sh = max(self.root.winfo_screenheight(), 700)
            self.root.resizable(True, True)
            self.root.maxsize(sw, sh)
            self.root.minsize(800, 500)
            # 保持当前位置，尺寸若已超过屏幕则略缩
            self.root.update_idletasks()
            w = min(self.root.winfo_width(), sw)
            h = min(self.root.winfo_height(), sh)
            if w < 100:
                w = min(1500, int(sw * 0.88))
            if h < 100:
                h = min(860, int(sh * 0.82))
            x = self.root.winfo_x()
            y = self.root.winfo_y()
            self.root.geometry(f"{w}x{h}+{x}+{y}")
        except tk.TclError:
            pass

    def _relax_pane_minsizes(self) -> None:
        """限制各 Panedwindow 子面板最小尺寸，分隔条才能拖动。"""
        for pw in (
            getattr(self, "_main_pw", None),
            getattr(self, "_right_h", None),
            getattr(self, "_right_v", None),
        ):
            if pw is None:
                continue
            try:
                children = pw.panes()
            except tk.TclError:
                continue
            for child in children:
                try:
                    orient = str(pw.cget("orient"))
                    if orient == "horizontal":
                        pw.pane(child, minsize=120)
                    else:
                        pw.pane(child, minsize=80)
                except tk.TclError:
                    try:
                        pw.paneconfig(child, minsize=100)
                    except tk.TclError:
                        pass

    def _scrollable(self, parent: ttk.LabelFrame) -> ttk.Frame:
        """LabelFrame 内嵌可滚动区域。

        注意：Canvas 不能按内容撑开尺寸，否则会把窗口/分隔条的最小尺寸锁死，
        表现为「看起来能拖、实际拖不动」。
        """
        wrap = ttk.Frame(parent)
        wrap.pack(fill=tk.BOTH, expand=True)
        # width/height=1：尺寸由父容器决定，不跟内部控件走
        canvas = tk.Canvas(wrap, highlightthickness=0, borderwidth=0, width=1, height=1)
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
        ttk.Label(soft_row, text="升 RPM/s").pack(side=tk.LEFT)
        self.soft_rate_up_var = tk.StringVar(value="800")
        ttk.Entry(soft_row, textvariable=self.soft_rate_up_var, width=6).pack(
            side=tk.LEFT, padx=(4, 2)
        )
        ttk.Label(soft_row, text="降").pack(side=tk.LEFT)
        self.soft_rate_down_var = tk.StringVar(value="300")
        ttk.Entry(soft_row, textvariable=self.soft_rate_down_var, width=6).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(soft_row, text="应用", command=self._apply_soft_rate).pack(
            side=tk.LEFT, padx=(4, 0)
        )
        # 兼容旧变量名（其它代码若引用 soft_rate_var）
        self.soft_rate_var = self.soft_rate_up_var
        ka_row = ttk.Frame(parent)
        ka_row.pack(fill=tk.X, pady=2)
        ttk.Label(ka_row, text="Ka μs/(rpm/s)").pack(side=tk.LEFT)
        self.ka_var = tk.StringVar(value="0")
        ttk.Entry(ka_row, textvariable=self.ka_var, width=7).pack(side=tk.LEFT, padx=4)
        ttk.Button(ka_row, text="下发", command=self._apply_ka).pack(side=tk.LEFT)
        ttk.Label(
            parent,
            text="升/降斜坡可不同；Ka=加速度前馈（S2）",
            foreground="#666",
            wraplength=220,
        ).pack(anchor=tk.W)

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
        ttk.Button(mbtn, text="斜坡学习 LEARN RAMP", command=self._measure_ramp).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
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
        self._freq_list = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600]
        self._freq_idx = 0
        self._freq_testing = False
        self._freq_job = None
        self._freq_results: list[str] = []
        self._freq_auto = False
        self._freq_auto_abort = False
        self._freq_auto_samples: list[float] = []
        self._freq_auto_job = None
        self._freq_baseline_mean: float | None = None
        self._freq_min_spin = 40.0

        ttk.Label(
            parent,
            text="PWM 刷新率摸底（默认安全回 50Hz）",
            font=("Segoe UI", 9, "bold"),
        ).pack(anchor=tk.W)
        ttk.Label(
            parent,
            text="手动：设频→低转速给油→到时停止→恢复50Hz\n"
            "自动：编码器判转；某档 FAIL 则更高频跳过",
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
        ttk.Button(
            parent, text="一键自动扫频（编码器判转）", command=self._freq_auto_start
        ).pack(fill=tk.X, pady=(6, 2))

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
        if not self._link_ready():
            self._warn_connect_once("请先连接串口")
            return
        if self._freq_testing or self._freq_auto:
            self._dlg_info("提示", "本频测试进行中")
            return
        hz = self._freq_list[self._freq_idx]
        hold = self._freq_parse_hold()
        pulse = self._freq_parse_pulse()
        if not self._dlg_askokcancel(
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
            self._dlg_info("完成", "已是最后一个频率 600 Hz")
            return
        nxt = self._freq_list[self._freq_idx + 1]
        if not self._dlg_askokcancel(
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
        if self._freq_auto:
            self._freq_auto_abort = True
        if self._freq_job is not None:
            try:
                self.root.after_cancel(self._freq_job)
            except Exception:  # noqa: BLE001
                pass
            self._freq_job = None
        if self._freq_auto_job is not None:
            try:
                self.root.after_cancel(self._freq_auto_job)
            except Exception:  # noqa: BLE001
                pass
            self._freq_auto_job = None
        self._freq_testing = False
        self._freq_auto = False
        self.btn_freq_run.configure(state=tk.NORMAL)
        self.btn_freq_next.configure(state=tk.DISABLED)
        if self.link and self.link.is_alive():
            self.link.send("ESTOP")
            self.link.send("FREQ 50")
            self.link.send("PWM 1000")
        self._freq_refresh_status()
        self.motor_status.set("频率调试: 已中止，FREQ=50")

    def _freq_auto_start(self) -> None:
        """开环扫频：编码器见转=PASS；FAIL 则更高频不测。"""
        if not self._link_ready():
            self._warn_connect_once("请先连接串口")
            return
        if self._freq_testing or self._freq_auto:
            self._dlg_info("提示", "频率测试进行中")
            return
        hold = self._freq_parse_hold()
        pulse = self._freq_parse_pulse()
        if not self._dlg_askokcancel(
            "自动扫频",
            f"将从 50Hz 起自动扫到 600Hz：\n"
            f"每档开环 {pulse} μs × {hold:.1f} s，用编码器 |rpm|≥40 判是否转起来。\n"
            f"某档 FAIL → 更高频率全部跳过，并恢复 50Hz。\n\n"
            f"请确认卸桨/固定。继续？",
        ):
            return
        self._freq_auto = True
        self._freq_auto_abort = False
        self._freq_idx = 0
        self._freq_baseline_mean = None
        self.btn_freq_run.configure(state=tk.DISABLED)
        self.btn_freq_next.configure(state=tk.DISABLED)
        self.freq_log.insert(
            tk.END,
            f"# AUTO start pulse={pulse} hold={hold:.1f}s min_spin=40\n",
        )
        self.ack_var.set("指令反馈: 自动扫频开始…")
        self._freq_auto_prep_step()

    def _freq_auto_cancel_job(self) -> None:
        if self._freq_auto_job is not None:
            try:
                self.root.after_cancel(self._freq_auto_job)
            except Exception:  # noqa: BLE001
                pass
            self._freq_auto_job = None

    def _freq_auto_prep_step(self) -> None:
        if not self._freq_auto or self._freq_auto_abort:
            self._freq_auto_finish("已中止")
            return
        if self._freq_idx >= len(self._freq_list):
            self._freq_auto_finish("全部测完")
            return
        # 先停转再给油
        if self.link and self.link.is_alive():
            self.link.send("PWM 1000")
            self.link.send("STOP")
            self.link.send("FREQ 50")
        self._freq_refresh_status()
        self.motor_status.set(
            f"自动扫频: 准备 {self._freq_list[self._freq_idx]} Hz…"
        )
        self._freq_auto_cancel_job()
        self._freq_auto_job = self.root.after(900, self._freq_auto_run_step)

    def _freq_auto_run_step(self) -> None:
        self._freq_auto_job = None
        if not self._freq_auto or self._freq_auto_abort:
            self._freq_auto_finish("已中止")
            return
        hz = self._freq_list[self._freq_idx]
        hold = self._freq_parse_hold()
        pulse = self._freq_parse_pulse()
        self._freq_testing = True
        self._freq_auto_samples = []
        self.freq_status.set(
            f"自动 {self._freq_idx + 1}/{len(self._freq_list)}: {hz} Hz 给油中…"
        )
        self.motor_status.set(f"自动扫频: {hz}Hz {pulse}μs…")
        self._send(f"FREQ {hz}")
        self.root.after(150, lambda p=pulse: self._send(f"PWM {p}"))
        # 采样窗口：poll 里若 _freq_auto 则追加 _rpm_disp
        self._freq_auto_cancel_job()
        self._freq_auto_job = self.root.after(
            int(hold * 1000), lambda h=hz, p=pulse, t=hold: self._freq_auto_on_hold(h, p, t)
        )

    def _freq_auto_on_hold(self, hz: int, pulse: int, hold: float) -> None:
        self._freq_auto_job = None
        self._freq_testing = False
        if self.link and self.link.is_alive():
            self.link.send("PWM 1000")
            self.link.send("STOP")
            self.link.send("FREQ 50")

        if not self._freq_auto or self._freq_auto_abort:
            self._freq_auto_finish("已中止")
            return

        samples = list(self._freq_auto_samples)
        # 若采样空，用当前显示转速兜底
        if not samples and getattr(self, "_rpm_disp", 0) > 0:
            samples = [float(self._rpm_disp)]
        peak = max(samples) if samples else 0.0
        tail = samples[len(samples) // 2 :] if samples else [0.0]
        mean = sum(tail) / len(tail) if tail else 0.0
        moved = peak >= self._freq_min_spin or mean >= self._freq_min_spin * 0.75
        verdict = "PASS" if moved else "FAIL"
        note = f"peak={peak:.0f}" if moved else f"{hold:.0f}s未见转 mean={mean:.1f}"
        line = f"{hz},{verdict},{pulse},{hold:.1f},{note}\n"
        self.freq_log.insert(tk.END, line)
        self.freq_log.see(tk.END)
        self._freq_results.append(line.strip())

        if hz == 50 and moved:
            self._freq_baseline_mean = mean

        if not moved:
            # FAIL → 更高频 SKIP
            for skip_hz in self._freq_list[self._freq_idx + 1 :]:
                self.freq_log.insert(
                    tk.END, f"{skip_hz},SKIP,{pulse},{hold:.1f},因{hz}Hz FAIL未测\n"
                )
            self.freq_log.see(tk.END)
            analysis = self._freq_auto_analysis(hz, fail=True)
            self.freq_log.insert(tk.END, f"# {analysis}\n")
            self.freq_log.see(tk.END)
            self._freq_auto_finish(f"{hz}Hz FAIL，更高频已跳过")
            self._dlg_warning("扫频中止", analysis)
            return

        self._freq_idx += 1
        if self._freq_idx >= len(self._freq_list):
            analysis = self._freq_auto_analysis(hz, fail=False)
            self.freq_log.insert(tk.END, f"# {analysis}\n")
            self.freq_log.see(tk.END)
            self._freq_auto_finish("全部 PASS")
            self._dlg_info("扫频完成", analysis)
            return
        self._freq_auto_prep_step()

    def _freq_auto_analysis(self, last_hz: int, *, fail: bool) -> str:
        passes = []
        for row in self._freq_results:
            parts = row.split(",")
            if len(parts) >= 2 and parts[1] == "PASS":
                try:
                    passes.append(int(parts[0]))
                except ValueError:
                    pass
        if fail and last_hz == 50:
            return (
                "分析: 50Hz 开环时限内编码器未见转动。"
                "请查电调供电/行程校准/探点脉宽/编码器；不是高频不支持问题。"
            )
        if fail:
            rec = max(passes) if passes else 50
            return (
                f"分析: {last_hz}Hz 未转起来；推荐默认 FREQ={rec}Hz。"
                f"更高频率已跳过。"
            )
        rec = max(passes) if passes else last_hz
        return f"分析: 扫频通过至 {rec}Hz；建议默认 FREQ={rec}Hz。"

    def _freq_auto_finish(self, reason: str) -> None:
        self._freq_auto_cancel_job()
        self._freq_auto = False
        self._freq_testing = False
        self.btn_freq_run.configure(state=tk.NORMAL)
        self.btn_freq_next.configure(state=tk.DISABLED)
        if self.link and self.link.is_alive():
            self.link.send("FREQ 50")
            self.link.send("PWM 1000")
            self.link.send("STOP")
        self.freq_status.set(f"自动扫频结束: {reason}（已回 50Hz）")
        self.motor_status.set(f"频率调试: {reason}")
        self.ack_var.set(f"指令反馈: 自动扫频结束 — {reason}")

    def _freq_log_result(self) -> None:
        hz = self._freq_list[self._freq_idx]
        hold = self._freq_parse_hold()
        pulse = self._freq_parse_pulse()
        verdict = self.freq_pass_var.get()
        line = f"{hz},{verdict},{pulse},{hold:.1f},\n"
        self.freq_log.insert(tk.END, line)
        self.freq_log.see(tk.END)
        self._freq_results.append(line)

    # ---------- ④ 自动测试监视（用户旁观，不干预） ----------
    def _build_auto_test_panel(self, parent: ttk.Frame) -> None:
        ttk.Label(
            parent,
            text="旁观式自动测试",
            font=("Segoe UI", 9, "bold"),
        ).pack(anchor=tk.W)
        ttk.Label(
            parent,
            text="点一次开始后自动跑完；\n本页显示「将做啥 / 正在做啥」。\n"
            "同时只能开一个监视窗（勿多开抢 COM）。\n可点中止或 Esc 急停。",
            foreground="#666",
            wraplength=220,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(2, 6))

        self.atest_plan_var = tk.StringVar(value="计划: （尚未开始）")
        ttk.Label(
            parent, textvariable=self.atest_plan_var, wraplength=220, justify=tk.LEFT
        ).pack(anchor=tk.W)

        self.atest_now_var = tk.StringVar(value="当前: 空闲")
        ttk.Label(
            parent,
            textvariable=self.atest_now_var,
            foreground="#0a5a9c",
            font=("Segoe UI", 9, "bold"),
            wraplength=220,
        ).pack(anchor=tk.W, pady=(6, 0))

        self.atest_detail_var = tk.StringVar(value="详情: —")
        ttk.Label(
            parent, textvariable=self.atest_detail_var, wraplength=220, foreground="#444"
        ).pack(anchor=tk.W, pady=(2, 4))

        self.atest_progress = ttk.Progressbar(parent, mode="determinate", maximum=100)
        self.atest_progress.pack(fill=tk.X, pady=4)

        btn = ttk.Frame(parent)
        btn.pack(fill=tk.X, pady=4)
        self.btn_atest_start = ttk.Button(
            btn, text="开始 S1 自动测试", command=self._atest_start_s1
        )
        self.btn_atest_start.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self.btn_atest_abort = ttk.Button(
            btn, text="中止", command=self._atest_abort_run, state=tk.DISABLED
        )
        self.btn_atest_abort.pack(side=tk.LEFT, expand=True, fill=tk.X)

        ttk.Button(parent, text="打开大监视窗", command=self._open_auto_test_win).pack(
            fill=tk.X, pady=(4, 2)
        )

        ttk.Label(parent, text="步骤清单").pack(anchor=tk.W, pady=(6, 0))
        self.atest_list = tk.Listbox(parent, height=8, font=("Consolas", 9))
        self.atest_list.pack(fill=tk.BOTH, expand=True, pady=2)

        ttk.Label(parent, text="运行日志").pack(anchor=tk.W)
        self.atest_log = tk.Text(parent, height=8, width=28, font=("Consolas", 8))
        self.atest_log.pack(fill=tk.BOTH, expand=True, pady=2)
        self.atest_log.insert(tk.END, "等待开始…\n")

    def _open_auto_test_win(self) -> None:
        """大监视窗：计划 + 当前 + 日志（与侧栏同步）。"""
        if self._atest_win is not None and self._atest_win.winfo_exists():
            self._atest_win.lift()
            self._atest_win.focus_force()
            return
        win = tk.Toplevel(self.root)
        win.title("自动测试监视 — 旁观即可")
        win.geometry("520x560")
        win.minsize(420, 400)
        self._atest_win = win

        ttk.Label(
            win,
            text="自动测试监视",
            font=("Segoe UI", 12, "bold"),
        ).pack(anchor=tk.W, padx=12, pady=(12, 4))
        ttk.Label(
            win,
            text="无需操作。下方显示将执行的步骤与正在执行的步骤。Esc=急停。",
            foreground="#666",
        ).pack(anchor=tk.W, padx=12)

        self.atest_win_now = tk.StringVar(value=self.atest_now_var.get())
        ttk.Label(
            win,
            textvariable=self.atest_win_now,
            font=("Segoe UI", 11, "bold"),
            foreground="#0a5a9c",
        ).pack(anchor=tk.W, padx=12, pady=8)

        self.atest_win_detail = tk.StringVar(value=self.atest_detail_var.get())
        ttk.Label(win, textvariable=self.atest_win_detail, wraplength=480).pack(
            anchor=tk.W, padx=12
        )

        fr = ttk.Frame(win, padding=12)
        fr.pack(fill=tk.BOTH, expand=True)
        ttk.Label(fr, text="步骤计划").pack(anchor=tk.W)
        self.atest_win_list = tk.Listbox(fr, height=10, font=("Consolas", 10))
        self.atest_win_list.pack(fill=tk.BOTH, expand=True, pady=4)
        # 同步已有清单
        for i in range(self.atest_list.size()):
            self.atest_win_list.insert(tk.END, self.atest_list.get(i))

        ttk.Label(fr, text="日志").pack(anchor=tk.W)
        self.atest_win_log = tk.Text(fr, height=12, font=("Consolas", 9))
        self.atest_win_log.pack(fill=tk.BOTH, expand=True)
        self.atest_win_log.insert(tk.END, self.atest_log.get("1.0", tk.END))

        bf = ttk.Frame(win, padding=12)
        bf.pack(fill=tk.X)
        ttk.Button(bf, text="开始 S1", command=self._atest_start_s1).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(bf, text="中止", command=self._atest_abort_run).pack(side=tk.LEFT)

        def _on_close() -> None:
            self._atest_win = None
            win.destroy()

        win.protocol("WM_DELETE_WINDOW", _on_close)

    def _atest_log_line(self, msg: str) -> None:
        line = f"{datetime.now().strftime('%H:%M:%S')}  {msg}\n"
        if hasattr(self, "atest_log"):
            self.atest_log.insert(tk.END, line)
            self.atest_log.see(tk.END)
        if (
            self._atest_win is not None
            and self._atest_win.winfo_exists()
            and hasattr(self, "atest_win_log")
        ):
            self.atest_win_log.insert(tk.END, line)
            self.atest_win_log.see(tk.END)

    def _atest_set_now(self, text: str) -> None:
        self.atest_now_var.set(f"当前: {text}")
        self.atest_banner_var.set(f"自动测试: {text}")
        if hasattr(self, "atest_win_now") and self._atest_win is not None:
            try:
                self.atest_win_now.set(f"当前: {text}")
            except tk.TclError:
                pass

    def _atest_set_detail(self, text: str) -> None:
        self.atest_detail_var.set(f"详情: {text}")
        if hasattr(self, "atest_win_detail") and self._atest_win is not None:
            try:
                self.atest_win_detail.set(f"详情: {text}")
            except tk.TclError:
                pass

    def _atest_refresh_list(self) -> None:
        if not hasattr(self, "atest_list"):
            return
        self.atest_list.delete(0, tk.END)
        for i, st in enumerate(self._atest_steps):
            mark = {"pending": "○", "running": "●", "pass": "✓", "fail": "✗", "skip": "–"}.get(
                st.get("status", "pending"), "○"
            )
            self.atest_list.insert(
                tk.END,
                f"{mark} {st['title']}",
            )
            if st.get("status") == "running":
                self.atest_list.selection_clear(0, tk.END)
                self.atest_list.selection_set(i)
                self.atest_list.see(i)
        if (
            self._atest_win is not None
            and self._atest_win.winfo_exists()
            and hasattr(self, "atest_win_list")
        ):
            self.atest_win_list.delete(0, tk.END)
            for i in range(self.atest_list.size()):
                self.atest_win_list.insert(tk.END, self.atest_list.get(i))
            try:
                sel = self.atest_list.curselection()
                if sel:
                    self.atest_win_list.selection_set(sel[0])
                    self.atest_win_list.see(sel[0])
            except tk.TclError:
                pass

    def _atest_ping(self) -> None:
        now = time.time()
        if now - self._atest_last_ping >= 0.4:
            if self.link and self.link.is_alive():
                self.link.send("PING")
            self._atest_last_ping = now

    def _atest_cancel_job(self) -> None:
        if self._atest_job is not None:
            try:
                self.root.after_cancel(self._atest_job)
            except Exception:  # noqa: BLE001
                pass
            self._atest_job = None

    def _atest_abort_run(self) -> None:
        if not self._atest_active:
            return
        self._atest_abort = True
        self._atest_log_line("用户中止")
        self._atest_finish("已中止")

    def _atest_start_s1(self, skip_confirm: bool = False) -> None:
        if self._atest_active:
            if not skip_confirm:
                self._dlg_info("提示", "自动测试已在进行")
            return
        if not self._link_ready():
            if skip_confirm:
                # 连接可能尚未完成，稍后再试
                self.root.after(1000, lambda: self._atest_start_s1(skip_confirm=True))
                self.atest_banner_var.set("自动测试: 等待连接…")
                return
            self._warn_connect_once("请先连接串口")
            return
        if self._freq_auto or self._freq_testing or getattr(self, "_learn_done_pending", False):
            if not skip_confirm:
                self._dlg_warning("提示", "请先结束学习/扫频再开自动测试")
            return
        if not skip_confirm and not self._dlg_askokcancel(
            "开始 S1 自动测试",
            "将自动执行：\n"
            "1) 预检 ctrl_hz\n"
            "2) 对称斜坡 A1(1000→2000) A2(3000→2000)\n"
            "3) 不对称斜坡（若固件支持 UP/DOWN）再跑 A1/A2\n\n"
            "过程无需操作，侧栏/监视窗会显示进度。\n"
            "确认卸桨固定后继续？",
        ):
            return
        self._open_auto_test_win()

        up, down = 800.0, 300.0
        self._atest_steps = [
            {"id": "preflight", "title": "预检连接与 ctrl_hz", "status": "pending"},
            {
                "id": "A1_sym",
                "title": "A1 升速 1000→2000（对称 800/800）",
                "status": "pending",
                "from": 1000.0,
                "to": 2000.0,
                "up": up,
                "down": up,
            },
            {
                "id": "A2_sym",
                "title": "A2 降速 3000→2000（对称 800/800）",
                "status": "pending",
                "from": 3000.0,
                "to": 2000.0,
                "up": up,
                "down": up,
            },
            {
                "id": "probe_soft",
                "title": "探测固件是否支持升/降斜坡",
                "status": "pending",
            },
            {
                "id": "A1_asym",
                "title": "A1 升速 1000→2000（升800/降300）",
                "status": "pending",
                "from": 1000.0,
                "to": 2000.0,
                "up": up,
                "down": down,
            },
            {
                "id": "A2_asym",
                "title": "A2 降速 3000→2000（升800/降300）",
                "status": "pending",
                "from": 3000.0,
                "to": 2000.0,
                "up": up,
                "down": down,
            },
            {"id": "report", "title": "汇总报告", "status": "pending"},
        ]
        self._atest_idx = 0
        self._atest_results = []
        self._atest_active = True
        self._atest_abort = False
        self._atest_soft_up_down = None
        self.btn_atest_start.configure(state=tk.DISABLED)
        self.btn_atest_abort.configure(state=tk.NORMAL)
        self.atest_plan_var.set(
            "计划: 预检 → A1/A2 对称 → 探测 UP/DOWN → A1/A2 不对称 → 报告"
        )
        self.atest_progress["value"] = 0
        if hasattr(self, "atest_log"):
            self.atest_log.delete("1.0", tk.END)
        self._atest_refresh_list()
        self._atest_log_line("开始 S1 自动测试序列" + ("（--auto-s1）" if skip_confirm else ""))
        self._atest_set_now("启动中…")
        self._atest_run_next_step()

    def _atest_mark(self, idx: int, status: str) -> None:
        if 0 <= idx < len(self._atest_steps):
            self._atest_steps[idx]["status"] = status
        self._atest_refresh_list()
        n = len(self._atest_steps)
        self.atest_progress["value"] = (
            100.0 * sum(1 for s in self._atest_steps if s["status"] in ("pass", "fail", "skip")) / max(n, 1)
        )

    def _atest_run_next_step(self) -> None:
        self._atest_cancel_job()
        if self._atest_abort or not self._atest_active:
            return
        if self._atest_idx >= len(self._atest_steps):
            self._atest_finish("全部完成")
            return
        st = self._atest_steps[self._atest_idx]
        self._atest_mark(self._atest_idx, "running")
        self._atest_set_now(st["title"])
        self._atest_log_line(f"→ 开始: {st['title']}")
        sid = st["id"]
        if sid == "preflight":
            self._atest_do_preflight()
        elif sid == "probe_soft":
            self._atest_do_probe_soft()
        elif sid == "report":
            self._atest_do_report()
        elif sid.startswith("A"):
            # 若不支持 UP/DOWN 且是 asym 步，跳过
            if "asym" in sid and self._atest_soft_up_down is False:
                self._atest_mark(self._atest_idx, "skip")
                self._atest_log_line(f"跳过 {sid}（固件无 SOFT RATE UP/DOWN，请烧录 S1）")
                self._atest_idx += 1
                self._atest_job = self.root.after(200, self._atest_run_next_step)
                return
            self._atest_phase = "goto_from"
            self._atest_t0 = time.time()
            self._atest_t_in = None
            self._atest_samples = []
            up, down = float(st["up"]), float(st["down"])
            if self.link and self.link.is_alive():
                self.link.send("SOFT ON")
                if self._atest_soft_up_down:
                    self.link.send(f"SOFT RATE UP {up:.1f}")
                    self.link.send(f"SOFT RATE DOWN {down:.1f}")
                else:
                    self.link.send(f"SOFT RATE {((up + down) / 2):.1f}")
                self.link.send("MODE CLOSED")
                self.link.send(f"RPM {st['from']:.0f}")
                self.link.send("START")
            self._atest_set_detail(f"前往起点 {st['from']:.0f} RPM…")
            self._atest_job = self.root.after(200, self._atest_tick_traj)

    def _atest_step_done(self, ok: bool, note: str = "") -> None:
        self._atest_mark(self._atest_idx, "pass" if ok else "fail")
        st = self._atest_steps[self._atest_idx]
        self._atest_results.append(f"{st['id']}: {'PASS' if ok else 'FAIL'} {note}")
        self._atest_log_line(f"← 结束: {st['title']} → {'PASS' if ok else 'FAIL'} {note}")
        if self.link and self.link.is_alive():
            self.link.send("STOP")
            self.link.send("RPM 0")
        self._atest_idx += 1
        self._atest_job = self.root.after(1200, self._atest_run_next_step)

    def _atest_do_preflight(self) -> None:
        self._atest_set_detail(f"ctrl_hz={self.ctrl_hz:.0f}")
        self._atest_ping()
        ok = float(self.ctrl_hz or 0) >= 200
        if not ok:
            self._atest_log_line(f"预检失败 ctrl_hz={self.ctrl_hz}")
            self._atest_mark(self._atest_idx, "fail")
            self._atest_finish("预检失败")
            return
        self._atest_log_line(f"预检通过 ctrl_hz={self.ctrl_hz}")
        self._atest_step_done(True, f"ctrl_hz={self.ctrl_hz}")

    def _atest_do_probe_soft(self) -> None:
        """发 SOFT RATE UP，看 ACK 是否含 UP。"""
        self._atest_soft_up_down = False
        self._atest_set_detail("发送 SOFT RATE UP 800 …")
        self._atest_probe_deadline = time.time() + 1.2
        self._atest_probe_seen = False
        if self.link and self.link.is_alive():
            self.link.send("SOFT RATE UP 800")
        self._atest_job = self.root.after(150, self._atest_tick_probe_soft)

    def _atest_on_meta_for_probe(self, raw: str) -> None:
        if not self._atest_active or self._atest_idx >= len(self._atest_steps):
            return
        if self._atest_steps[self._atest_idx]["id"] != "probe_soft":
            return
        if "ACK SOFT RATE UP" in raw:
            self._atest_soft_up_down = True
            self._atest_probe_seen = True
        elif "ERR unknown" in raw and "SOFT" in raw.upper():
            self._atest_soft_up_down = False
            self._atest_probe_seen = True
        elif "ACK SOFT RATE=" in raw and "UP" not in raw:
            self._atest_soft_up_down = False
            self._atest_probe_seen = True

    def _atest_tick_probe_soft(self) -> None:
        self._atest_job = None
        if self._atest_abort:
            return
        self._atest_ping()
        if self._atest_probe_seen or time.time() >= self._atest_probe_deadline:
            ok = bool(self._atest_soft_up_down)
            note = "支持 UP/DOWN" if ok else "旧固件，asym 将跳过"
            self._atest_log_line(note)
            if ok and self.link and self.link.is_alive():
                self.link.send("SOFT RATE DOWN 300")
            self._atest_step_done(True, note)
            return
        self._atest_job = self.root.after(150, self._atest_tick_probe_soft)

    def _atest_tick_traj(self) -> None:
        self._atest_job = None
        if self._atest_abort or not self._atest_active:
            return
        self._atest_ping()
        st = self._atest_steps[self._atest_idx]
        rpm = float(getattr(self, "_rpm_disp", 0.0) or 0.0)
        elapsed = time.time() - self._atest_t0
        band_from = max(80.0, abs(st["from"]) * 0.10)
        band_to = max(50.0, abs(st["to"]) * 0.10)

        if self._atest_phase == "goto_from":
            self._atest_set_detail(
                f"前往起点 {st['from']:.0f}  实测 {rpm:.0f}  t={elapsed:.1f}s"
            )
            if abs(rpm - st["from"]) <= band_from:
                self._atest_phase = "hold_from"
                self._atest_t0 = time.time()
                self._atest_set_detail(f"起点到位，稳定 1s… rpm={rpm:.0f}")
            elif elapsed > max(12.0, abs(st["from"]) / 80.0 + 6):
                self._atest_step_done(False, f"未到起点 实测{rpm:.0f}")
                return
            self._atest_job = self.root.after(200, self._atest_tick_traj)
            return

        if self._atest_phase == "hold_from":
            if elapsed >= 1.0:
                self._atest_phase = "goto_to"
                self._atest_t0 = time.time()
                self._atest_t_in = None
                self._atest_samples = []
                if self.link and self.link.is_alive():
                    self.link.send(f"RPM {st['to']:.0f}")
                self._atest_set_detail(f"斜坡前往 {st['to']:.0f} RPM…")
            self._atest_job = self.root.after(200, self._atest_tick_traj)
            return

        if self._atest_phase == "goto_to":
            self._atest_samples.append((elapsed, rpm))
            if self._atest_t_in is None and abs(rpm - st["to"]) <= band_to:
                self._atest_t_in = elapsed
            self._atest_set_detail(
                f"{st['from']:.0f}→{st['to']:.0f}  实测 {rpm:.0f}  "
                f"t={elapsed:.1f}s  "
                f"入带={'%.2fs' % self._atest_t_in if self._atest_t_in is not None else '…'}"
            )
            settle = 8.0
            if elapsed >= settle:
                ok = self._atest_t_in is not None and rpm > 40
                note = (
                    f"t_band={self._atest_t_in:.2f}s tail={rpm:.0f}"
                    if self._atest_t_in is not None
                    else f"未入带 tail={rpm:.0f}"
                )
                self._atest_step_done(ok, note)
                return
            self._atest_job = self.root.after(200, self._atest_tick_traj)

    def _atest_do_report(self) -> None:
        lines = ["======== S1 自动测试汇总 ========", *self._atest_results]
        if self._atest_soft_up_down is False:
            lines.append("固件无 SOFT RATE UP/DOWN → 请烧录 S1 后重测不对称段")
        elif self._atest_soft_up_down:
            lines.append("固件已支持升/降斜坡")
        text = "\n".join(lines)
        self._atest_log_line(text)
        self._atest_set_detail("报告已写入日志")
        try:
            log_dir = Path(__file__).resolve().parent / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            path = log_dir / f"s1_ui_{datetime.now().strftime('%Y%m%d_%H%M%S')}.report.txt"
            path.write_text(text + "\n", encoding="utf-8")
            self._atest_log_line(f"报告文件: {path.name}")
        except OSError as exc:
            self._atest_log_line(f"写报告失败: {exc}")
        self._atest_step_done(True, "已汇总")

    def _atest_finish(self, reason: str) -> None:
        self._atest_cancel_job()
        self._atest_active = False
        self._atest_phase = ""
        if self.link and self.link.is_alive():
            self.link.send("STOP")
            self.link.send("RPM 0")
            self.link.send("PWM 1000")
        self.btn_atest_start.configure(state=tk.NORMAL)
        self.btn_atest_abort.configure(state=tk.DISABLED)
        self._atest_set_now(f"结束 — {reason}")
        self.atest_banner_var.set(f"自动测试: {reason}")
        self._atest_log_line(f"序列结束: {reason}")
        self.atest_progress["value"] = 100

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

    def _bring_dialog_front(self) -> None:
        """把主窗抬到前台，但不要 -topmost：Windows 上 topmost 父窗会盖住 messagebox。"""
        try:
            self.root.deiconify()
            self.root.attributes("-topmost", False)
            self.root.lift()
            self.root.focus_force()
            self.root.update_idletasks()
        except tk.TclError:
            pass

    def _release_dialog_front(self) -> None:
        try:
            self.root.attributes("-topmost", False)
        except tk.TclError:
            pass

    def _dlg_warning(self, title: str, message: str, **kw):
        self._bring_dialog_front()
        try:
            return messagebox.showwarning(title, message, parent=self.root, **kw)
        finally:
            self._release_dialog_front()

    def _dlg_info(self, title: str, message: str, **kw):
        self._bring_dialog_front()
        try:
            return messagebox.showinfo(title, message, parent=self.root, **kw)
        finally:
            self._release_dialog_front()

    def _dlg_error(self, title: str, message: str, **kw):
        self._bring_dialog_front()
        try:
            return messagebox.showerror(title, message, parent=self.root, **kw)
        finally:
            self._release_dialog_front()

    def _dlg_askokcancel(self, title: str, message: str, **kw):
        self._bring_dialog_front()
        try:
            return messagebox.askokcancel(title, message, parent=self.root, **kw)
        finally:
            self._release_dialog_front()

    def _dlg_askyesno(self, title: str, message: str, **kw):
        self._bring_dialog_front()
        try:
            return messagebox.askyesno(title, message, parent=self.root, **kw)
        finally:
            self._release_dialog_front()

    def _popup_op(self, title: str, msg: str, kind: str = "info") -> None:
        """操作结果弹窗：尽快显示，避免铃声响了却看不到框。"""
        key = f"{title}|{msg[:80]}"
        if self._op_dialog_open or key == self._last_op_popup:
            return
        self._last_op_popup = key
        self._op_dialog_open = True

        def _show() -> None:
            try:
                if kind == "error":
                    self._dlg_error(title, msg)
                elif kind == "warning":
                    self._dlg_warning(title, msg)
                else:
                    self._dlg_info(title, msg)
            finally:
                self._op_dialog_open = False
                self.root.after(800, lambda: setattr(self, "_last_op_popup", ""))

        # after_idle：等当前串口轮询帧结束立刻弹，不要人为再拖几百毫秒
        self.root.after_idle(_show)

    def _handle_op_feedback(self, raw: str) -> None:
        """解析板子 ACK/ERR，给用户明确成功/失败交互。"""
        text = raw.lstrip("# ").strip()

        if "ACK RPMMAX" in raw:
            was_user = self._await_rpmmax
            self._await_rpmmax = False
            lim = "?"
            try:
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                lim = parts.get("RPMMAX", parts.get("rpm_limit", "?"))
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
            # 连接时静默下发的 RPMMAX 不弹窗；仅用户点「下发」才提示
            if was_user:
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
        if not self._dlg_askokcancel(
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
        if not self._dlg_askokcancel(
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
            self._dlg_warning("提示", "角度 / 爬行 RPM 无效")
            return
        if deg <= 0 or deg > 7200:
            self._dlg_warning("提示", "角度范围 0.5° ~ 7200°")
            return
        if rpm >= 1000:
            # 用户填了脉宽，爬行仍用 RPM 默认
            rpm = 300.0
        direction = self.phase_dir_var.get().strip().upper()
        if direction not in ("CW", "CCW"):
            direction = "CW"
        if not self._dlg_askokcancel(
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
            self._dlg_warning("提示", "停机相位无效")
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
            self._dlg_warning("提示", "停机相位 / 爬行 RPM 无效")
            return
        if rpm >= 1000:
            rpm = 300.0
        deg = deg % 360.0
        if not self._dlg_askokcancel(
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

    def _link_ready(self) -> bool:
        return self.link is not None and self.link.is_open

    def _warn_connect_once(self, detail: str = "请先连接串口") -> None:
        """统一未连接提示：启动宽限 + 冷却，避免多处同时弹窗。"""
        now = time.time()
        if now < self._connect_warn_until:
            self.ack_var.set(f"指令反馈: {detail}（等待连接…）")
            return
        if now - self._last_connect_warn_at < CONNECT_WARN_COOLDOWN_S:
            self.ack_var.set(f"指令反馈: {detail}")
            return
        self._last_connect_warn_at = now
        self._dlg_warning("提示", detail)

    def _send(self, cmd: str, *, quiet: bool = False) -> None:
        if not self._link_ready():
            if quiet:
                self.ack_var.set("指令反馈: 未连接串口")
            else:
                self._warn_connect_once("请先连接串口")
            return
        self.link.send(cmd)

    def _schedule_boot_cmd(self, delay_ms: int, cmd: str) -> None:
        """连接后下发配置：静默，且断开时可取消。"""

        def _go(c: str = cmd) -> None:
            self._send(c, quiet=True)

        aid = self.root.after(delay_ms, _go)
        self._boot_after_ids.append(aid)

    def _cancel_boot_cmds(self) -> None:
        for aid in self._boot_after_ids:
            try:
                self.root.after_cancel(aid)
            except (tk.TclError, ValueError):
                pass
        self._boot_after_ids.clear()

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
        if send and self._link_ready():
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
            self._dlg_warning("提示", f"请输入 0~{int(RPM_MAX)} 整数/小数")
            self.sp_var.set(f"{self._setpoint:.0f}")
            return
        if rpm < 0 or rpm > RPM_MAX:
            self._dlg_warning("提示", f"目标转速范围 0~{int(RPM_MAX)}")
            self.sp_var.set(f"{self._setpoint:.0f}")
            return
        self._set_setpoint(rpm, send=True)

    def _start(self) -> None:
        self._on_sp_entry()
        self._on_soft_toggle()
        # 要转速跟上目标，应用闭环；开环只是按表给油门，实测会偏
        if self.mode_var.get() == "OPEN":
            if self._dlg_askyesno(
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
        if self._atest_active:
            self._atest_abort = True
            self._atest_finish("急停中止")
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
        # 建界面时 Checkbutton 可能误触发；未连上只改本地状态，不弹窗
        if not self._link_ready():
            return
        self._send("SOFT ON" if self.soft_var.get() else "SOFT OFF")

    def _apply_rpm_limit(self) -> None:
        try:
            v = float(self.rpm_limit_var.get())
        except ValueError:
            self._dlg_warning("提示", "最大转速无效")
            return
        if v < 100 or v > MOTOR_RPM_LIMIT_DEFAULT:
            self._dlg_warning("提示", f"最大转速范围 100~{int(MOTOR_RPM_LIMIT_DEFAULT)}")
            return
        self.rpm_limit_var.set(f"{v:.0f}")
        self._await_rpmmax = True
        self._send(f"RPMMAX {v:.0f}")
        self.ack_var.set(f"指令反馈: 正在下发电机最大转速 {v:.0f} RPM…")
        # 成功弹窗等 ACK RPMMAX，避免把别的 ERR 误当成这次操作

    def _apply_soft_rate(self) -> None:
        try:
            up = float(self.soft_rate_up_var.get())
            down = float(self.soft_rate_down_var.get())
        except (ValueError, AttributeError):
            self._dlg_warning("提示", "升/降斜率无效")
            return
        self._send(f"SOFT RATE UP {up:.1f}")
        self._send(f"SOFT RATE DOWN {down:.1f}")
        self.ack_var.set(f"指令反馈: 缓启动升={up:.0f} 降={down:.0f} RPM/s")

    def _apply_ka(self) -> None:
        try:
            ka = float(self.ka_var.get())
        except (ValueError, AttributeError):
            self._dlg_warning("提示", "Ka 无效")
            return
        if ka < 0 or ka > 2.0:
            self._dlg_warning("提示", "Ka 范围 0~2 μs/(rpm/s)")
            return
        self._send(f"KA {ka:.4f}")
        self.ack_var.set(f"指令反馈: 加速度前馈 Ka={ka:.4f}")

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
            self._dlg_warning("提示", "正在自动学习中，请先点「停止学习」")
            return
        try:
            lim = float(self.rpm_limit_var.get())
        except (ValueError, AttributeError):
            lim = MOTOR_RPM_LIMIT_DEFAULT
        if not self._dlg_askokcancel(
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
        self._learn_gain_points.clear()
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

    def _measure_ramp(self) -> None:
        """LEARN RAMP：开环脉宽连续斜坡上升（比台阶快），同样建图+建议分区 PID。"""
        if getattr(self, "_learn_done_pending", False):
            self._dlg_warning("提示", "正在自动学习中，请先点「停止学习」")
            return
        try:
            lim = float(self.rpm_limit_var.get())
        except (ValueError, AttributeError):
            lim = MOTOR_RPM_LIMIT_DEFAULT
        if not self._dlg_askokcancel(
            "斜坡学习 LEARN RAMP",
            "将开环连续升高油门（斜坡而非台阶）并记录转速：\n"
            f"从 1000 μs 起，斜坡速率约 80 μs/s，\n"
            f"直到接近电机最大转速 {lim:.0f} RPM，\n"
            "或油门顶到 2000 μs 后结束。\n\n"
            "结束后会自动生成前馈图 + 分区 PID（GainMap）。\n"
            "过程会自动保存 CSV 记录到 pc/logs/。\n"
            "请确认已卸桨/固定、选对 profile。\n继续？",
        ):
            return
        self._apply_profile()
        self._learn_points.clear()
        self._learn_suggest.clear()
        self._learn_gain_points.clear()
        self._learn_done_pending = True
        self._learn_log_open()
        self._exp_step = 2
        if hasattr(self, "exp_step_var"):
            self.exp_step_var.set(
                f"实验步骤: ②斜坡学习中…（未到 {lim:.0f} RPM 则继续加油门，最高 2000μs）"
            )
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
            f"指令反馈: 启动斜坡学习（上限 {lim:.0f}RPM / 2000μs）{log_hint}"
        )
        self.pulse_lbl.set(self._format_throttle(1000, "斜坡学习中…"))
        self._send("MEASURE RAMP")
        self._learn_log_row(kind="cmd", note="MEASURE RAMP")

    def _parse_learn_meta(self, raw: str) -> None:
        """解析学习过程串口行，学习结束弹出结果与下一步。"""
        if self._learn_log_writer is not None and (
            "LEARN" in raw or "MEASURE" in raw or "SUGGEST" in raw or "EVAL" in raw
        ):
            self._learn_log_row(kind="meta", note=raw.lstrip("# ").strip())

        if "ACK LEARN point" in raw or "LEARN RAMP sample" in raw:
            try:
                # ... pulse=1050 rpm=120.5 n=3 ...（台阶/斜坡格式一致）
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                pu = int(float(parts.get("pulse", "0")))
                rpm = float(parts.get("rpm", "0"))
                self._learn_points.append((pu, rpm))
                tag = "斜坡" if "RAMP" in raw else ""
                if hasattr(self, "exp_step_var"):
                    self.exp_step_var.set(
                        f"实验步骤: ②{tag}学习中 pulse={pu} rpm={rpm:.0f} 点={len(self._learn_points)}"
                    )
            except (ValueError, KeyError):
                pass
            return

        if "GAIN point" in raw:
            try:
                parts = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in raw.split() if "=" in p}
                self._learn_gain_points.append(
                    {
                        "rpm": float(parts.get("rpm", "0")),
                        "kp": float(parts.get("kp", "0")),
                        "ki": float(parts.get("ki", "0")),
                        "kd": float(parts.get("kd", "0")),
                    }
                )
            except (ValueError, KeyError):
                pass
            return

        if "SUGGEST GAINMAP" in raw or "GAIN BEGIN" in raw:
            self._learn_gain_points.clear()
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
            zone_pts = self._learn_gain_points
            if zone_pts:
                zone_line = (
                    f"\n分区 PID（GainMap，共 {len(zone_pts)} 段，按 rpm 插值）:\n"
                    + "\n".join(
                        f"  rpm={z['rpm']:.0f}  Kp={z['kp']:.4f}  Ki={z['ki']:.4f}"
                        for z in zone_pts
                    )
                )
            else:
                zone_line = "\n分区 PID（GainMap）: （未收到，可稍后 GAIN? 查询）"
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
                f"{zone_line}\n"
                f"{log_line}"
                f"{analysis}\n"
            )
            if table:
                msg += f"\n部分测点:\n{table}\n"
            msg += (
                "\n—— 下一步（闭环试跑）——\n"
                "1. 应用建议 PID（全局）+ 分区 PID 表（GAIN ON）\n"
                "2. 设目标约 300 RPM，确认缓启动 ON\n"
                "3. 点「启动」试闭环\n\n"
                "是否现在应用 PID + 开启分区 PID，并把目标设为 300 RPM？"
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

            ok = self._dlg_askyesno("学习完成 — 下一步", msg)
            if ok:
                if kp is not None and ki is not None:
                    self.kp_var.set(f"{kp:.4f}")
                    self.ki_var.set(f"{ki:.4f}")
                    self.kd_var.set("0")
                    self._send(f"PID {kp:.4f} {ki:.4f} 0")
                if zone_pts:
                    self._send("GAIN ON")
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
                    "指令反馈: 已应用 PID"
                    + ("+分区PID(GAIN ON)" if zone_pts else "")
                    + " + 目标300RPM。确认安全后点「启动」"
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
            self._dlg_warning("提示", "PID 数值无效")
            return
        self._send(f"PID {kp} {ki} {kd}")

    def _on_adapt_toggle(self) -> None:
        if not self._link_ready():
            return
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
            self._dlg_warning("提示", "请选择串口")
            return
        self._cancel_boot_cmds()
        self.stop_evt.clear()
        self.link = SerialLink(port, self.q, self.stop_evt)
        self.link.start()
        self.btn_connect.configure(text="断开")
        self.btn_rec.configure(state=tk.NORMAL)
        self.status_var.set(f"正在打开 {port} @ {BAUD}…")
        self.prev_deg = None
        self.prev_t_ms = None
        self._rx_count = 0
        self._rx_t0 = time.time()
        self._quiet_cmd_until = time.time() + 3.0
        # 再给 2s 宽限：串口线程打开端口期间，控件误触发不弹窗
        self._connect_warn_until = time.time() + CONNECT_WARN_GRACE_S
        # 等串口真正打开后再下发；失败则由 __error__ 断开并取消
        self._schedule_boot_cmd(600, "SOFT ON" if self.soft_var.get() else "SOFT OFF")
        try:
            up = float(self.soft_rate_up_var.get())
            down = float(self.soft_rate_down_var.get())
        except (ValueError, AttributeError):
            up, down = 800.0, 300.0
        self._schedule_boot_cmd(650, f"SOFT RATE UP {up:.1f}")
        self._schedule_boot_cmd(680, f"SOFT RATE DOWN {down:.1f}")
        self._schedule_boot_cmd(700, "FREQ 50")
        self._schedule_boot_cmd(800, f"RPM {self._setpoint:.1f}")
        self._schedule_boot_cmd(900, "PHASE?")
        try:
            rpmmax = float(self.rpm_limit_var.get() or 6000)
        except ValueError:
            rpmmax = 6000.0
        self._schedule_boot_cmd(1000, f"RPMMAX {rpmmax:.0f}")
        self._schedule_boot_cmd(1100, "LEARN MAXUS 2000")
        self.root.after(400, self._check_link_opened)

    def _check_link_opened(self) -> None:
        if self.link is None:
            return
        if self.link.is_open:
            port = self.port_var.get().strip()
            self.status_var.set(f"已连接 {port} @ {BAUD}")
            return
        if self.link.is_alive():
            # 仍在打开中，稍后再看
            self.root.after(200, self._check_link_opened)
            return
        # 线程已退出且未打开：错误会走队列；这里只取消静默下发
        self._cancel_boot_cmds()

    def _disconnect(self) -> None:
        self._cancel_boot_cmds()
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
        # 断开后短宽限，避免残留 after 回调连弹
        self._connect_warn_until = time.time() + CONNECT_WARN_GRACE_S

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
        self._rpm_signed_hist.clear()
        self._rpm_disp = 0.0
        self._rpm_dir = "—"
        self.rpm_var.set("实测: 0.0 RPM")
        if hasattr(self, "rpm_dir_var"):
            self.rpm_dir_var.set("—")
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
            self._dlg_info("提示", "请先停止记录再清空")
            return
        self.live_t.clear()
        self.live_rpm.clear()
        self._rpm_signed_hist.clear()
        self._rpm_disp = 0.0
        self._rpm_dir = "—"
        self.rpm_var.set("实测: 0.0 RPM")
        if hasattr(self, "rpm_dir_var"):
            self.rpm_dir_var.set("—")
        self.rec_rows.clear()
        self.btn_export.configure(state=tk.DISABLED)
        self.rec_info.set("记录: 未开始")
        self._redraw()

    def _export_csv(self) -> None:
        if not self.rec_rows:
            self._dlg_info("提示", "没有可导出的数据")
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
        self._dlg_info("完成", f"已保存: {path}")

    def _poll(self) -> None:
        # 1) 限量消化队列，保证本帧还能响应按钮/拖窗
        budget = POLL_SAMPLE_BUDGET
        try:
            while budget > 0:
                item = self.q.get_nowait()
                budget -= 1
                if isinstance(item, tuple) and item and item[0] == "__error__":
                    self._dlg_error("串口错误", item[1])
                    self._disconnect()
                    break
                if isinstance(item, tuple) and item and item[0] == "__meta__":
                    meta = item[1] if len(item) > 1 else {}
                    if isinstance(meta, dict):
                        if "ctrl_hz" in meta:
                            self.ctrl_hz = int(meta["ctrl_hz"])
                        raw = meta.get("raw")
                        if isinstance(raw, str):
                            if (
                                "LEARN settle" in raw or "LEARN RAMP settle" in raw
                            ) and self._learn_log_writer is not None:
                                self._learn_log_row(
                                    kind="settle", note=raw.lstrip("# ").strip()
                                )
                            if any(
                                k in raw
                                for k in (
                                    "ACK",
                                    "ESCCAL",
                                    "LEARN",
                                    "MEASURE",
                                    "HOLD",
                                    "SUGGEST",
                                    "GAIN",
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
                            ) and "LEARN settle" not in raw and "LEARN RAMP settle" not in raw:
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
                                self._atest_on_meta_for_probe(raw)
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
                self._plot_dirty = True
        except queue.Empty:
            pass

        now = time.time()
        if self.link and self.link.is_alive() and (now - self._last_ping) > 0.5:
            self.link.send("PING")
            self._last_ping = now

        # 2) 轻量：数字/状态 10Hz（有符号滑动均值）
        if (self._ui_dirty or self._rpm_signed_hist) and (
            now - self._last_label_draw
        ) * 1000.0 >= LABEL_REFRESH_MS:
            self._update_rpm_display_mean()
            self._flush_labels()
            self._ui_dirty = False
            self._last_label_draw = now

        # 3) 重量：曲线 5Hz；拖窗期间跳过
        resizing = now < self._resize_quiet_until
        if (
            self._plot_dirty
            and not resizing
            and (now - self._last_plot_draw) * 1000.0 >= PLOT_REFRESH_MS
        ):
            self._redraw()
            self._plot_dirty = False
            self._last_plot_draw = now

        self.root.after(POLL_MS, self._poll)

    def _update_rpm_display_mean(self) -> None:
        """有符号滑动均值消噪声；数字与曲线都用 |均值| + 正/逆。"""
        now = time.time()
        while self._rpm_signed_hist and (
            now - self._rpm_signed_hist[0][0] > RPM_MEAN_WINDOW_S
        ):
            self._rpm_signed_hist.popleft()
        if self._rpm_signed_hist:
            mean = sum(v for _t, v in self._rpm_signed_hist) / len(self._rpm_signed_hist)
            if abs(mean) < RPM_UI_DEADZONE:
                self._rpm_disp = 0.0
                self._rpm_dir = "—"
            else:
                self._rpm_disp = abs(mean)
                self._rpm_dir = "正" if mean >= 0 else "逆"
        self.rpm_var.set(f"实测: {self._rpm_disp:.1f} RPM")
        if hasattr(self, "rpm_dir_var"):
            self.rpm_dir_var.set(self._rpm_dir)

        # 曲线只吃均值后的平滑点（不再吃瞬时 |rpm|，避免齿状）
        if now - self._last_live_append >= LIVE_APPEND_MIN_S:
            self._last_live_append = now
            if self.recording and self.rec_t0_ms is not None and self._last_telem is not None:
                t_ms = float(self._last_telem[0])
                t_s = (t_ms - self.rec_t0_ms) * 0.001
                self.live_t.append(t_s)
                self.live_rpm.append(self._rpm_disp)
            else:
                self.live_t.append(now)
                self.live_rpm.append(self._rpm_disp)
                while self.live_t and (now - self.live_t[0]) > LIVE_WINDOW_S:
                    self.live_t.popleft()
                    self.live_rpm.popleft()
            self._plot_dirty = True

    def _flush_labels(self) -> None:
        """把最近一帧遥测刷到界面（不在每条采样上改 StringVar）。"""
        s = self._last_telem
        if s is None:
            return
        (
            _t_ms,
            raw,
            deg,
            _rad,
            _rpm_fw,
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
        ) = s
        self.deg_var.set(f"{deg:7.2f} °")
        abs_deg = (float(raw) * 360.0) / 16384.0
        if hasattr(self, "phase_info"):
            self.phase_info.set(
                f"相对相位: {deg:.2f}°  |  绝对: {abs_deg:.2f}°"
            )
        self.rpm_target_disp.set(f"目标: {target:.0f} RPM")
        self.enc_info_var.set(
            f"编码器: 16384点/圈 | raw={raw} | "
            f"数字{1000 // LABEL_REFRESH_MS}Hz / 曲线{1000 // PLOT_REFRESH_MS}Hz"
        )
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
            if pulse < 1500:
                self.cal_pulse_var.set(
                    f"校准脉宽: 警告 当前 pulse={pulse} μs，未到 2000（检查连接/是否收到命令）"
                )
        self._update_throttle_display(pulse, mode)
        self.motor_status.set(
            f"Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f} | {MODE_NAMES.get(mode)} {RUN_NAMES.get(run)}"
        )

    def _on_sample(self, sample: tuple) -> None:
        """只做缓冲与记录；不刷新 Tk 控件（留给分频 flush）。"""
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

        self._last_telem = sample
        self._rx_count += 1
        elapsed = time.time() - self._rx_t0
        if elapsed >= 1.0:
            self.root.title(
                f"AS5047P+ESC | ctrl{self.ctrl_hz}Hz | 收{self._rx_count / elapsed:.0f}Hz"
            )
            self._rx_count = 0
            self._rx_t0 = time.time()

        rpm_meas = float(rpm_fw)
        self.prev_deg = deg
        self.prev_t_ms = t_ms
        self.current_deg = deg
        now_wall = time.time()
        self._rpm_signed_hist.append((now_wall, rpm_meas))
        while self._rpm_signed_hist and (
            now_wall - self._rpm_signed_hist[0][0] > RPM_MEAN_WINDOW_S
        ):
            self._rpm_signed_hist.popleft()
        # 自动扫频：用磁编码器瞬时转速采样判转
        if getattr(self, "_freq_auto", False) and self._freq_testing:
            self._freq_auto_samples.append(abs(rpm_meas))

        # 学习日志仍限 10Hz
        if self._learn_log_writer is not None and (
            mode == 3 or self._learn_done_pending
        ):
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

        # 记录 CSV 仍写瞬时有符号值；曲线点只在均值更新时写入（见 _update_rpm_display_mean）
        if self.recording:
            if self.rec_t0_ms is None:
                self.rec_t0_ms = t_ms
            if len(self.rec_rows) >= MAX_REC_POINTS:
                self._stop_record()
                self._dlg_info("提示", "记录已达上限，已停止")
                return
            t_s = (t_ms - self.rec_t0_ms) * 0.001
            self.rec_rows.append(
                (
                    f"{t_s:.4f}",
                    f"{deg:.3f}",
                    f"{rad:.6f}",
                    f"{rpm_meas:.2f}",
                    int(raw),
                    pulse,
                    f"{target:.1f}",
                )
            )
            self.rec_info.set(f"记录中… {len(self.rec_rows)} 点  t={t_s:.2f}s")

    @staticmethod
    def _downsample_xy(
        xs: list[float], ys: list[float], max_points: int
    ) -> tuple[list[float], list[float]]:
        """按桶取 min/max，保留尖峰，点数≈屏幕宽度。"""
        n = len(xs)
        if n <= max_points or max_points < 4:
            return xs, ys
        # 每桶至少输出 2 点（min+max），所以桶数 = max_points//2
        buckets = max(2, max_points // 2)
        out_x: list[float] = []
        out_y: list[float] = []
        for b in range(buckets):
            i0 = (b * n) // buckets
            i1 = ((b + 1) * n) // buckets
            if i1 <= i0:
                continue
            chunk_y = ys[i0:i1]
            chunk_x = xs[i0:i1]
            jmin = min(range(len(chunk_y)), key=lambda j: chunk_y[j])
            jmax = max(range(len(chunk_y)), key=lambda j: chunk_y[j])
            if chunk_x[jmin] <= chunk_x[jmax]:
                out_x.extend((chunk_x[jmin], chunk_x[jmax]))
                out_y.extend((chunk_y[jmin], chunk_y[jmax]))
            else:
                out_x.extend((chunk_x[jmax], chunk_x[jmin]))
                out_y.extend((chunk_y[jmax], chunk_y[jmin]))
        return out_x, out_y

    def _redraw(self) -> None:
        if self.recording or self.rec_rows:
            if self.recording:
                xs = list(self.live_t)
                ys = list(self.live_rpm)
            else:
                # 回放记录时也限点数，避免卡死
                raw_xs = [float(r[0]) for r in self.rec_rows]
                raw_ys = [abs(float(r[3])) for r in self.rec_rows]
                xs, ys = self._downsample_xy(raw_xs, raw_ys, PLOT_MAX_POINTS)
            xlabel = "时间 (s) — 记录段"
        else:
            if not self.live_t:
                xs, ys = [], []
            else:
                t0 = self.live_t[0]
                xs = [t - t0 for t in self.live_t]
                ys = list(self.live_rpm)
                # 按当前画布宽度估点数
                try:
                    w_px = max(int(self.canvas.get_tk_widget().winfo_width()), 200)
                except tk.TclError:
                    w_px = PLOT_MAX_POINTS
                xs, ys = self._downsample_xy(xs, ys, min(PLOT_MAX_POINTS, w_px))
            xlabel = f"时间 (s) — 最近 {LIVE_WINDOW_S:.0f}s"

        self.line.set_data(xs, ys)
        fp = dict(fontproperties=_CN_FONT) if _CN_FONT is not None else {}
        self.ax.set_xlabel(xlabel, **fp)
        self.ax.set_ylabel("转速 (RPM，大小)", **fp)
        if xs and ys:
            x0, x1 = min(xs), max(xs)
            if x1 <= x0:
                x1 = x0 + 1.0
            y_max = max(ys)
            # 显式跟数据涨：set_ylim 后 autoscale 会失效，必须自己算上限
            y_top = max(y_max * 1.15, y_max + 30.0, 50.0)
            self.ax.set_xlim(x0, x1)
            self.ax.set_ylim(0.0, y_top)
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
    from com_port_guard import acquire, status_message

    try:
        acquire("encoder_monitor")
    except SystemExit as exc:
        # 尽量弹窗；无显示时打印
        try:
            root = tk.Tk()
            root.withdraw()
            messagebox.showerror("串口已被占用", str(exc))
            root.destroy()
        except Exception:  # noqa: BLE001
            print(exc, file=sys.stderr)
        raise SystemExit(1) from exc

    print(status_message())
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
