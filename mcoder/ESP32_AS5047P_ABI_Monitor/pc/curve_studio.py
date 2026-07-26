#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
曲线工作室（Sony Vegas + MATLAB/Origin 式毛刺处理）：
  - Cut / 选择 / 圈毛刺
  - 圈毛刺：单击 / 矩形框 / 套索；动作=排除(掩码红叉) 或 删除点(其余 xy 不变)
  - 平滑重算在后台线程，避免 2kHz 大数据卡 UI
  - 导出：同一 X + Y原始 + Y平滑
"""

from __future__ import annotations

import csv
import threading
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Sequence

from rpm_chart import RpmChart
from snap_edit import (
    RPM_SMOOTH_METHODS,
    accel_from_rows,
    apply_rpm_replacements,
    distance_series,
    filter_by_rpm,
    find_startup_t_s,
    index_baseline,
    integrate_rpm_distance,
    load_timeline_csv,
    replace_indices_neighbor_avg,
    smooth_rpm_series,
    split_at_time,
    t_base_ms,
    trim_before_t,
)

try:
    from abi_monitor import (
        recompute_rows_rpm_from_counts,
        row_counts,
        row_has_counts,
    )
except Exception:  # noqa: BLE001 — 独立跑 curve_studio 时可能无完整 abi_monitor
    def row_has_counts(row: tuple) -> bool:
        return len(row) > 7 and row[7] is not None

    def row_counts(row: tuple):
        return int(row[7]) if row_has_counts(row) else None

    def recompute_rows_rpm_from_counts(rows: list, **_kw) -> list:
        return list(rows)


def _point_in_poly(x: float, y: float, poly: Sequence[tuple[float, float]]) -> bool:
    """射线法：点是否在多边形内（含边界近似）。"""
    n = len(poly)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-30) + xi
        ):
            inside = not inside
        j = i
    return inside


@dataclass
class Clip:
    rows: list
    name: str = ""
    uid: int = 0
    # 排除索引（相对本段 rows）；不改 rows 数值，仅拟合时跳过
    excluded: set = field(default_factory=set)


def _norm01(ys: list[float], *, signed: bool = False) -> tuple[list[float], float, float]:
    if not ys:
        return [], 0.0, 1.0
    lo, hi = min(ys), max(ys)
    if signed:
        peak = max(abs(lo), abs(hi), 1e-12)
        return [100.0 * y / peak for y in ys], lo, hi
    span = hi - lo
    if abs(span) < 1e-12:
        return [50.0 for _ in ys], lo, hi
    return [100.0 * (y - lo) / span for y in ys], lo, hi


def _clip_dur_s(rows: list) -> float:
    if len(rows) < 2:
        return 0.0
    return max(0.0, (float(rows[-1][0]) - float(rows[0][0])) / 1000.0)


def concat_timeline(clips: Sequence[Clip]) -> tuple[list, list[float], list[tuple[int, float, float]]]:
    """
    把多段拼成连续显示时间轴（ripple）。
    返回: display_rows, boundaries(秒), [(clip_i, t0, t1), ...]
    """
    out: list = []
    bounds = [0.0]
    spans: list[tuple[int, float, float]] = []
    t_cursor = 0.0
    for i, clip in enumerate(clips):
        rows = clip.rows
        if not rows:
            spans.append((i, t_cursor, t_cursor))
            bounds.append(t_cursor)
            continue
        base = t_base_ms(rows)
        dur = _clip_dur_s(rows)
        t0 = t_cursor
        for r in rows:
            rel = (float(r[0]) - base) / 1000.0
            t_ms = (t_cursor + rel) * 1000.0
            # 保留原字段，仅改显示用 t
            out.append((t_ms, r[1], r[2], r[3], r[4]) + tuple(r[5:]))
        t_cursor += dur
        spans.append((i, t0, t_cursor))
        bounds.append(t_cursor)
    return out, bounds, spans


class CurveStudio(tk.Toplevel):
    _uid = 1

    def __init__(
        self,
        master,
        rows: Sequence[tuple],
        *,
        title: str = "曲线工作室",
        on_commit: Callable[[list], None] | None = None,
    ) -> None:
        super().__init__(master)
        self.title(f"曲线工作室 · {title}")
        self.geometry("1120x780")
        self.minsize(900, 600)
        self._raw = list(rows)
        self._on_commit = on_commit
        self._title = title
        self._clips: list[Clip] = [
            Clip(rows=list(rows), name="Clip1", uid=CurveStudio._uid, excluded=set())
        ]
        CurveStudio._uid += 1
        self._active = 0
        self._tool = tk.StringVar(value="cut")  # cut | select | exclude
        self._exclude_action = tk.StringVar(value="mask")  # mask | delete
        self._select_shape = tk.StringVar(value="rect")  # rect | lasso
        self._drag_start_px: tuple[float, float] | None = None
        self._lasso_px: list[tuple[float, float]] = []
        self._lasso_item: int | None = None
        # 撤销：每项 = [(rows, excluded), ...] 各 clip
        self._spike_undo: list[list[tuple[list, set]]] = []
        self._refresh_seq = 0
        self._refresh_lock = threading.Lock()
        self._refreshing = False

        self.var_i0 = tk.StringVar(value="auto")
        self.var_len = tk.StringVar(value="1.0")
        self.var_s_mode = tk.StringVar(value="rpm")  # 默认 ∫平滑转速 → 距离
        self.var_rpm_cut = tk.StringVar(value="10")
        self.var_smooth = tk.StringVar(value="21")  # @2kHz ≈10ms，数字转速默认
        self.var_diff_method = tk.StringVar(value="ma_central")
        self.var_post_smooth = tk.StringVar(value="5")
        self.var_rpm_method = tk.StringVar(value="hampel_ma")
        self.var_despike_win = tk.StringVar(value="11")
        self.var_despike_sigma = tk.StringVar(value="3.0")
        self.var_rpm_smooth = tk.StringVar(value="21")  # 平滑窗 / SG 窗
        self.var_ema_alpha = tk.StringVar(value="0.08")
        self.var_iir_fc = tk.StringVar(value="50")
        self.var_max_rpm_per_s = tk.StringVar(value="80000")  # rate-clamp @2kHz≈40RPM/点
        self.var_startup_rpm = tk.StringVar(value="10")
        self.var_startup_need_i = tk.BooleanVar(value=True)
        self.var_startup_t = tk.StringVar(value="—")
        self.var_show_raw = tk.BooleanVar(value=True)
        self.var_show_smooth = tk.BooleanVar(value=True)
        self.var_view = tk.StringVar(value="rpm")
        self.var_status = tk.StringVar(value="")
        self.var_cut_t = tk.StringVar(value="—")
        self.var_playhead = 0.0
        self._last_plot_view: str | None = None
        self._smooth_refresh_job: str | None = None

        self._build()
        self._bind_smooth_auto_refresh()
        self._i0_first(refresh=False)
        self.refresh()
        self.bind("<KeyPress-c>", lambda e: self._set_tool("cut"))
        self.bind("<KeyPress-C>", lambda e: self._set_tool("cut"))
        self.bind("<KeyPress-v>", lambda e: self._set_tool("select"))
        self.bind("<KeyPress-V>", lambda e: self._set_tool("select"))
        self.bind("<KeyPress-e>", lambda e: self._set_tool("exclude"))
        self.bind("<KeyPress-E>", lambda e: self._set_tool("exclude"))
        self.bind("<KeyPress-b>", lambda e: self._set_tool("exclude"))  # 兼容旧快捷键
        self.bind("<KeyPress-B>", lambda e: self._set_tool("exclude"))
        self.bind("<Control-z>", lambda e: self._undo_spike())
        self.bind("<Control-Z>", lambda e: self._undo_spike())
        self.bind("<Delete>", lambda e: self._on_delete_key())
        self.bind("<BackSpace>", lambda e: self._on_delete_key())
        self.focus_set()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text=self._title, font=("Segoe UI", 12, "bold")).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.var_status, foreground="#333").pack(side=tk.LEFT, padx=12)

        tools = ttk.LabelFrame(self, text="工具", padding=6)
        tools.pack(fill=tk.X, padx=8, pady=2)
        # 两行布局，避免窄屏把「圈毛刺」挤出窗口
        row1 = ttk.Frame(tools)
        row1.pack(fill=tk.X, pady=2)
        row2 = ttk.Frame(tools)
        row2.pack(fill=tk.X, pady=2)

        ttk.Label(row1, text="模式:").pack(side=tk.LEFT, padx=(0, 4))
        # tk.Button 才能稳定改底色，突出「圈毛刺」
        self.btn_tool_cut = tk.Button(
            row1, text="Cut刀片 (C)", width=12, command=lambda: self._set_tool("cut")
        )
        self.btn_tool_cut.pack(side=tk.LEFT, padx=3)
        self.btn_tool_sel = tk.Button(
            row1, text="选择 (V)", width=10, command=lambda: self._set_tool("select")
        )
        self.btn_tool_sel.pack(side=tk.LEFT, padx=3)
        self.btn_tool_excl = tk.Button(
            row1,
            text="圈毛刺 (E)",
            width=12,
            command=lambda: self._set_tool("exclude"),
            font=("Segoe UI", 10, "bold"),
        )
        self.btn_tool_excl.pack(side=tk.LEFT, padx=6)
        ttk.Label(row1, text="刀口 t=").pack(side=tk.LEFT, padx=(16, 0))
        ttk.Label(row1, textvariable=self.var_cut_t, width=10).pack(side=tk.LEFT)

        row1b = ttk.Frame(tools)
        row1b.pack(fill=tk.X, pady=2)
        ttk.Label(row1b, text="圈毛刺动作:").pack(side=tk.LEFT)
        ttk.Radiobutton(
            row1b, text="排除(红叉/掩码)", variable=self._exclude_action, value="mask",
            command=self._tool_hint,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(
            row1b,
            text="删除点(其余xy不变)",
            variable=self._exclude_action,
            value="delete",
            command=self._tool_hint,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Label(
            row1b,
            text="  建议：先切「counts-t」删异常，再回看转速",
            foreground="#555",
        ).pack(side=tk.LEFT, padx=8)
        ttk.Label(row1b, text="  选区:").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Radiobutton(
            row1b, text="矩形", variable=self._select_shape, value="rect",
            command=self._tool_hint,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(
            row1b, text="套索", variable=self._select_shape, value="lasso",
            command=self._tool_hint,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Label(
            row1b,
            text="单击切换排除；框/套索批量；删除=从序列去掉该点",
            foreground="#555",
        ).pack(side=tk.LEFT, padx=10)

        ttk.Button(row2, text="在刀口切开", command=self._cut_at_playhead).pack(side=tk.LEFT, padx=3)
        ttk.Button(row2, text="删除选中段", command=self._delete_active).pack(side=tk.LEFT, padx=3)
        ttk.Button(row2, text="合并全部段", command=self._merge_all).pack(side=tk.LEFT, padx=3)
        ttk.Button(row2, text="清除排除", command=self._clear_exclusions).pack(side=tk.LEFT, padx=3)
        ttk.Button(row2, text="删除已排除点", command=self._delete_excluded_points).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(row2, text="固化排除→邻点", command=self._bake_exclusions).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(row2, text="撤销 Ctrl+Z", command=self._undo_spike).pack(side=tk.LEFT, padx=3)
        self._paint_tool_buttons()

        view = ttk.LabelFrame(self, text="显示", padding=4)
        view.pack(fill=tk.X, padx=8, pady=2)
        for text, val in (
            ("仅转速", "rpm"),
            ("counts-t", "counts"),
            ("仅距离 S", "s"),
            ("仅加速度 a", "a"),
            ("三线同图", "all"),
        ):
            ttk.Radiobutton(
                view, text=text, variable=self.var_view, value=val, command=self.refresh
            ).pack(side=tk.LEFT, padx=6)
        ttk.Label(
            view,
            text="counts=原始累计步数，便于圈删台阶/跳变",
            foreground="#555",
        ).pack(side=tk.LEFT, padx=8)

        overlay = ttk.LabelFrame(self, text="曲线叠加（勾选要画的线）", padding=4)
        overlay.pack(fill=tk.X, padx=8, pady=2)
        ttk.Checkbutton(
            overlay,
            text="平滑前（原始）",
            variable=self.var_show_raw,
            command=self.refresh,
        ).pack(side=tk.LEFT, padx=8)
        ttk.Checkbutton(
            overlay,
            text="平滑后",
            variable=self.var_show_smooth,
            command=self.refresh,
        ).pack(side=tk.LEFT, padx=8)
        ttk.Button(overlay, text="只看原始", command=self._show_only_raw).pack(side=tk.LEFT, padx=4)
        ttk.Button(overlay, text="只看平滑", command=self._show_only_smooth).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(overlay, text="两者都显示", command=self._show_both_curves).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(overlay, text="复位视图", command=self._reset_chart_view).pack(
            side=tk.LEFT, padx=12
        )
        ttk.Label(
            overlay,
            text="滚轮：光标处放大/缩小 · 双击图复位",
            foreground="#555",
        ).pack(side=tk.LEFT, padx=8)

        sm = ttk.LabelFrame(self, text="转速平滑（电机常用）", padding=4)
        sm.pack(fill=tk.X, padx=8, pady=2)
        ttk.Label(sm, text="方法").pack(side=tk.LEFT)
        self.cmb_rpm_method = ttk.Combobox(
            sm,
            textvariable=self.var_rpm_method,
            width=18,
            state="readonly",
            values=tuple(k for k, _ in RPM_SMOOTH_METHODS),
        )
        self.cmb_rpm_method.pack(side=tk.LEFT, padx=2)
        self.cmb_rpm_method.bind("<<ComboboxSelected>>", lambda _e: self._on_rpm_method_change())
        self.lbl_method_hint = ttk.Label(sm, text="", foreground="#555", width=22)
        self.lbl_method_hint.pack(side=tk.LEFT, padx=4)
        ttk.Label(sm, text="窗").pack(side=tk.LEFT, padx=(6, 0))
        self.cmb_rpm_win = ttk.Combobox(
            sm,
            textvariable=self.var_rpm_smooth,
            width=4,
            values=("7", "11", "21", "41", "81"),
        )
        self.cmb_rpm_win.pack(side=tk.LEFT, padx=2)
        ttk.Label(sm, text="Hampel窗").pack(side=tk.LEFT, padx=(6, 0))
        self.ent_despike_win = ttk.Combobox(
            sm,
            textvariable=self.var_despike_win,
            width=4,
            values=("7", "11", "15", "21"),
        )
        self.ent_despike_win.pack(side=tk.LEFT, padx=2)
        ttk.Label(sm, text="σ").pack(side=tk.LEFT)
        self.ent_despike_sigma = ttk.Entry(sm, textvariable=self.var_despike_sigma, width=4)
        self.ent_despike_sigma.pack(side=tk.LEFT, padx=2)
        ttk.Label(sm, text="EMA α").pack(side=tk.LEFT, padx=(6, 0))
        self.ent_ema = ttk.Entry(sm, textvariable=self.var_ema_alpha, width=5)
        self.ent_ema.pack(side=tk.LEFT, padx=2)
        ttk.Label(sm, text="IIR fcHz").pack(side=tk.LEFT, padx=(6, 0))
        self.ent_iir = ttk.Entry(sm, textvariable=self.var_iir_fc, width=5)
        self.ent_iir.pack(side=tk.LEFT, padx=2)
        ttk.Label(sm, text="限幅RPM/s").pack(side=tk.LEFT, padx=(6, 0))
        self.ent_rate = ttk.Entry(sm, textvariable=self.var_max_rpm_per_s, width=7)
        self.ent_rate.pack(side=tk.LEFT, padx=2)
        ttk.Label(sm, text="改参数即重绘", foreground="#555").pack(side=tk.LEFT, padx=6)
        ttk.Button(sm, text="❓", width=3, command=self._show_smooth_help).pack(side=tk.LEFT, padx=2)
        ttk.Button(sm, text="转速算法", command=self._show_counts_rpm_help).pack(side=tk.LEFT, padx=4)
        self._on_rpm_method_change(refresh=False)

        gate = ttk.LabelFrame(self, text="启动段门控（裁掉脏启动，2kHz 原始点保留到裁切前）", padding=4)
        gate.pack(fill=tk.X, padx=8, pady=2)
        ttk.Label(gate, text="|RPM|>").pack(side=tk.LEFT)
        ttk.Entry(gate, textvariable=self.var_startup_rpm, width=5).pack(side=tk.LEFT, padx=2)
        ttk.Checkbutton(gate, text="须 I+1", variable=self.var_startup_need_i).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(gate, text="检测启动点", command=self._detect_startup).pack(side=tk.LEFT, padx=4)
        ttk.Label(gate, text="t=").pack(side=tk.LEFT)
        ttk.Label(gate, textvariable=self.var_startup_t, width=10).pack(side=tk.LEFT)
        ttk.Button(gate, text="裁掉启动前(当前段)", command=self._trim_startup_active).pack(
            side=tk.LEFT, padx=8
        )
        ttk.Button(gate, text="刀口←启动点", command=self._playhead_to_startup).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Label(
            gate,
            text="建议：先检测 → 确认 t≈0.15~0.2s → 裁切后再算 S/a",
            foreground="#555",
        ).pack(side=tk.LEFT, padx=8)

        opts = ttk.Frame(self, padding=4)
        opts.pack(fill=tk.X, padx=8)
        ttk.Label(opts, text="|RPM|<").pack(side=tk.LEFT)
        ttk.Entry(opts, textvariable=self.var_rpm_cut, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Button(opts, text="过滤慢速(当前段)", command=self._filter_rpm_active).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Label(opts, text="I0").pack(side=tk.LEFT, padx=(10, 0))
        ttk.Entry(opts, textvariable=self.var_i0, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Button(opts, text="首点I0", command=self._i0_first).pack(side=tk.LEFT, padx=2)
        ttk.Label(opts, text="每I长度").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Entry(opts, textvariable=self.var_len, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(
            opts, text="S←I", variable=self.var_s_mode, value="i", command=self.refresh
        ).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(
            opts, text="S←∫rpm", variable=self.var_s_mode, value="rpm", command=self.refresh
        ).pack(side=tk.LEFT)
        ttk.Label(opts, text="微分").pack(side=tk.LEFT, padx=(10, 0))
        self.cmb_diff = ttk.Combobox(
            opts,
            textvariable=self.var_diff_method,
            width=12,
            state="readonly",
            values=("ma_central", "savgol", "central", "forward"),
        )
        self.cmb_diff.pack(side=tk.LEFT, padx=2)
        self.cmb_diff.bind("<<ComboboxSelected>>", lambda _e: self.refresh())
        ttk.Label(opts, text="窗").pack(side=tk.LEFT, padx=(6, 0))
        self.cmb_smooth = ttk.Combobox(
            opts,
            textvariable=self.var_smooth,
            width=4,
            values=("11", "21", "41", "81"),
        )
        self.cmb_smooth.pack(side=tk.LEFT, padx=2)
        self.cmb_smooth.bind("<<ComboboxSelected>>", lambda _e: self.refresh())
        ttk.Label(opts, text="后再滑").pack(side=tk.LEFT, padx=(4, 0))
        ttk.Entry(opts, textvariable=self.var_post_smooth, width=3).pack(side=tk.LEFT, padx=2)
        ttk.Button(opts, text="重算a", command=self.refresh).pack(side=tk.LEFT, padx=4)
        ttk.Button(opts, text="❓", width=3, command=self._show_diff_help).pack(side=tk.LEFT, padx=4)
        ttk.Button(opts, text="导出重算曲线", command=self._export_csv).pack(side=tk.LEFT, padx=8)
        ttk.Button(opts, text="写回主列表", command=self._commit).pack(side=tk.LEFT, padx=2)
        ttk.Button(opts, text="恢复原片", command=self._restore).pack(side=tk.LEFT, padx=2)

        # 主曲线
        chart_fr = ttk.LabelFrame(
            self,
            text="监视器 · 先点上方「圈毛刺」再点选/拖框毛刺点（红叉）",
            padding=4,
        )
        chart_fr.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self.chart = RpmChart(chart_fr)
        self.chart.pack(fill=tk.BOTH, expand=True)
        self.chart.canvas.bind("<ButtonPress-1>", self._on_chart_press)
        self.chart.canvas.bind("<B1-Motion>", self._on_chart_motion)
        self.chart.canvas.bind("<ButtonRelease-1>", self._on_chart_release)
        self.chart.canvas.bind("<Enter>", self._chart_enter)
        self.chart.canvas.bind("<Leave>", self._chart_leave)
        self.chart.canvas.configure(cursor="crosshair", takefocus=True)

        # 时间线轨道
        tl = ttk.LabelFrame(self, text="时间线（点击色块选中段）", padding=4)
        tl.pack(fill=tk.X, padx=8, pady=4)
        self.tl_canvas = tk.Canvas(tl, height=56, background="#1e1e1e", highlightthickness=0)
        self.tl_canvas.pack(fill=tk.X)
        self.tl_canvas.bind("<Button-1>", self._on_timeline_click)

        list_fr = ttk.Frame(self, padding=4)
        list_fr.pack(fill=tk.X, padx=8)
        ttk.Label(list_fr, text="片段列表").pack(side=tk.LEFT)
        self.lst = tk.Listbox(list_fr, height=4, exportselection=False)
        self.lst.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self.lst.bind("<<ListboxSelect>>", self._on_list_sel)
        ttk.Button(list_fr, text="删除选中", command=self._delete_active).pack(side=tk.LEFT)
        ttk.Label(
            self,
            text="ESP32 保持 2kHz 原始；PC：启动门控裁切 + 多方法平滑（含 rate-clamp）；手机仅取数粗看",
            foreground="#555",
            padding=4,
        ).pack(fill=tk.X, padx=8)

    def _show_diff_help(self) -> None:
        """备注：数字信号微分方法说明。"""
        cur = (self.var_diff_method.get() or "ma_central").strip()
        try:
            w = int(float(self.var_smooth.get() or "21"))
        except ValueError:
            w = 21
        try:
            post = int(float(self.var_post_smooth.get() or "5"))
        except ValueError:
            post = 5
        ms = w * 0.5  # 2 kHz
        msg = (
            "【为何要平滑】\n"
            "板内 / 上位机：v40 起 BIN 存 counts；蓝线 RPM 由 PC 多拍差分得到。\n"
            "再对 RPM 求导得加速度 a=d(RPM)/dt，会放大台阶噪声，\n"
            "所以必须先平滑 / 用更稳的差分，而不是裸前向差分。\n"
            "\n"
            "【当前选用】\n"
            f"  方法 = {cur}\n"
            f"  窗   = {w} 点 ≈ {ms:.1f} ms @2kHz\n"
            f"  后再滑 = {post}（对 a 再做一次轻滑动）\n"
            "\n"
            "【方法备注】\n"
            "• ma_central（默认·推荐）\n"
            "    先对 RPM 做滑动均值，再用中心差分求导。\n"
            "    兼顾毛刺抑制与相位，适合 2kHz 数字转速。\n"
            "\n"
            "• savgol\n"
            "    局部二次多项式拟合后再解析求导（类 Savitzky–Golay）。\n"
            "    台阶噪声明显、又想保留拐点时可试。\n"
            "\n"
            "• central\n"
            "    不做预平滑，直接中心差分；(y[i+h]-y[i-h])/(2h·dt)。\n"
            "    比 forward 稳，但仍可能毛刺较多。\n"
            "\n"
            "• forward（旧）\n"
            "    相邻点前向差分。噪声最大，仅作对照。\n"
            "\n"
            "【窗口怎么选 @2kHz】\n"
            "  11 ≈ 5.5 ms　较快响应，毛刺略多\n"
            "  21 ≈ 10 ms　默认，一般够用\n"
            "  41 ≈ 20 ms　曲线更光，尖峰会被抹圆\n"
            "  81 ≈ 40 ms　很光，只看趋势\n"
            "\n"
            "单位：a 为 RPM/s（每秒转速变化量）。"
        )
        messagebox.showinfo("微分方法备注", msg)

    def _show_smooth_help(self) -> None:
        lines = [
            "【电机转速平滑方法】\n"
            "蓝线 rpm_raw：v40 起由 PC 从 counts 多拍差分得到（默认窗 32≈16ms），"
            "详见「转速算法」按钮。\n"
            "橙线：在蓝线（及排除掩码）上再平滑。\n\n"
        ]
        for key, lab in RPM_SMOOTH_METHODS:
            lines.append(f"• {key}\n    {lab}\n")
        lines.append(
            "\n参数：\n"
            "  窗 — MA / 中值 / SG / Hampel后再滑 / 限幅+滑均 的点数（@2kHz：21≈10ms）\n"
            "  Hampel窗 / σ — 仅 hampel_ma\n"
            "  EMA α — 仅 ema（越大越跟手，0.05~0.2 常用）\n"
            "  IIR fcHz — 仅 iir 低通截止频率（@2kHz 采样，试 30~80）\n"
            "  限幅RPM/s — rate_clamp / rate_clamp_ma：最大 |dRPM/dt|\n"
            "    默认 80000 → @2kHz 约 ±40 RPM/点（夹传感器跳变，非朋友文中的 500/点）\n"
            "\n"
            "启动段：检测 |RPM| 门 + 可选 I+1，再「裁掉启动前」；BIN 仍存 2kHz counts。\n"
            "\n"
            "圈毛刺：选「圈毛刺」后点选/拖框/套索；动作=排除(红叉) 或 删除点。\n"
            "排除不改 counts；「固化排除」才改表内数值。Ctrl+Z 撤销。"
        )
        messagebox.showinfo("转速平滑备注", "".join(lines))

    def _show_counts_rpm_help(self) -> None:
        try:
            from abi_monitor import help_text
        except Exception:  # noqa: BLE001
            help_text = None  # type: ignore
        body = (
            help_text("转速计算方法.md")
            if help_text
            else "请从主窗口打开「帮助·转速计算」。"
        )
        win = tk.Toplevel(self)
        win.title("帮助 · 转速计算方法")
        win.geometry("720x560")
        frm = ttk.Frame(win, padding=8)
        frm.pack(fill=tk.BOTH, expand=True)
        txt = tk.Text(frm, wrap=tk.WORD, undo=False)
        sb = ttk.Scrollbar(frm, orient=tk.VERTICAL, command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        txt.insert("1.0", body)
        txt.configure(state=tk.DISABLED)
        ttk.Button(win, text="关闭", command=win.destroy).pack(pady=6)

    def _show_only_raw(self) -> None:
        self.var_show_raw.set(True)
        self.var_show_smooth.set(False)
        self.refresh()

    def _show_only_smooth(self) -> None:
        self.var_show_raw.set(False)
        self.var_show_smooth.set(True)
        self.refresh()

    def _show_both_curves(self) -> None:
        self.var_show_raw.set(True)
        self.var_show_smooth.set(True)
        self.refresh()

    def _reset_chart_view(self) -> None:
        self.chart.reset_view()
        self.var_status.set("视图已复位（显示整体）")

    def _on_close(self) -> None:
        try:
            self.unbind_all("<MouseWheel>")
        except tk.TclError:
            pass
        self.destroy()

    def _chart_enter(self, _evt=None) -> None:
        self.chart.canvas.focus_set()
        self.bind_all("<MouseWheel>", self._on_wheel_forward)

    def _chart_leave(self, _evt=None) -> None:
        self.unbind_all("<MouseWheel>")

    def _on_wheel_forward(self, evt) -> str | None:
        """鼠标在曲线图上时：向前滚放大、向后滚缩小（光标为中心）。"""
        self.chart._on_mousewheel(evt)
        return "break"

    def _schedule_smooth_refresh(self, *_args) -> None:
        """平滑参数变更后防抖重绘（打字时约 200ms 合并一次）。"""
        if self._smooth_refresh_job is not None:
            try:
                self.after_cancel(self._smooth_refresh_job)
            except tk.TclError:
                pass
        self._smooth_refresh_job = self.after(200, self._do_smooth_refresh)

    def _do_smooth_refresh(self) -> None:
        self._smooth_refresh_job = None
        self.refresh()

    def _bind_smooth_auto_refresh(self) -> None:
        """方法/窗/σ/EMA/IIR/限幅等一改就自动重绘，无需点应用。"""
        for var in (
            self.var_rpm_method,
            self.var_rpm_smooth,
            self.var_despike_win,
            self.var_despike_sigma,
            self.var_ema_alpha,
            self.var_iir_fc,
            self.var_max_rpm_per_s,
            self.var_smooth,
            self.var_diff_method,
            self.var_post_smooth,
        ):
            var.trace_add("write", self._schedule_smooth_refresh)
        for w in (
            getattr(self, "cmb_rpm_method", None),
            getattr(self, "cmb_rpm_win", None),
            getattr(self, "ent_despike_win", None),
            getattr(self, "cmb_diff", None),
            getattr(self, "cmb_smooth", None),
        ):
            if w is not None:
                w.bind("<<ComboboxSelected>>", lambda _e: self._schedule_smooth_refresh(), add="+")
        for w in (
            getattr(self, "ent_despike_sigma", None),
            getattr(self, "ent_ema", None),
            getattr(self, "ent_iir", None),
            getattr(self, "ent_rate", None),
        ):
            if w is not None:
                w.bind("<Return>", lambda _e: self._schedule_smooth_refresh(), add="+")
                w.bind("<FocusOut>", lambda _e: self._schedule_smooth_refresh(), add="+")

    def _on_rpm_method_change(self, refresh: bool = True) -> None:
        m = (self.var_rpm_method.get() or "hampel_ma").strip()
        hint = dict(RPM_SMOOTH_METHODS).get(m, m)
        self.lbl_method_hint.configure(text=hint[:22])
        is_h = m == "hampel_ma"
        is_ema = m == "ema"
        is_iir = m == "iir"
        is_rate = m in ("rate_clamp", "rate_clamp_ma")
        state_h = "normal" if is_h else "disabled"
        state_e = "normal" if is_ema else "disabled"
        state_i = "normal" if is_iir else "disabled"
        state_r = "normal" if is_rate else "disabled"
        try:
            self.ent_despike_win.configure(state=state_h)
            self.ent_despike_sigma.configure(state=state_h)
            self.ent_ema.configure(state=state_e)
            self.ent_iir.configure(state=state_i)
            self.ent_rate.configure(state=state_r)
            # 窗对 none/ema/iir/rate_clamp 不关键；rate_clamp_ma 需要窗
            win_on = m not in ("none", "ema", "iir", "rate_clamp")
            self.cmb_rpm_win.configure(state="readonly" if win_on else "disabled")
        except tk.TclError:
            pass
        if refresh:
            self.refresh()

    def _read_rpm_smooth_params(self) -> dict:
        try:
            win = int(float(self.var_rpm_smooth.get().strip() or "21"))
        except ValueError:
            win = 21
        try:
            dwin = int(float(self.var_despike_win.get().strip() or "11"))
        except ValueError:
            dwin = 11
        try:
            dsig = float(self.var_despike_sigma.get().strip() or "3.0")
        except ValueError:
            dsig = 3.0
        try:
            alpha = float(self.var_ema_alpha.get().strip() or "0.08")
        except ValueError:
            alpha = 0.08
        try:
            fc = float(self.var_iir_fc.get().strip() or "50")
        except ValueError:
            fc = 50.0
        try:
            max_d = float(self.var_max_rpm_per_s.get().strip() or "80000")
        except ValueError:
            max_d = 80000.0
        return {
            "method": (self.var_rpm_method.get() or "hampel_ma").strip(),
            "window": max(1, win),
            "n_sigma": dsig,
            "despike_win": dwin,
            "alpha": alpha,
            "fc_hz": fc,
            "fs_hz": 2000.0,
            "max_rpm_per_s": max_d,
        }

    def _startup_gate_params(self) -> tuple[float, bool]:
        try:
            g = float(self.var_startup_rpm.get().strip() or "10")
        except ValueError:
            g = 10.0
        return g, bool(self.var_startup_need_i.get())

    def _detect_startup(self) -> None:
        if not (0 <= self._active < len(self._clips)):
            self.var_status.set("无选中段")
            return
        rows = self._clips[self._active].rows
        gate, need_i = self._startup_gate_params()
        t, msg = find_startup_t_s(rows, rpm_gate=gate, require_index=need_i)
        if t is None:
            self.var_startup_t.set("—")
            self.var_status.set(msg)
            return
        self.var_startup_t.set(f"{t:.4f}")
        # 映射到全局刀口：当前段起点 + t
        _, _, spans = concat_timeline(self._clips)
        t0g = spans[self._active][1] if self._active < len(spans) else 0.0
        self.var_playhead = t0g + t
        self.var_cut_t.set(f"{self.var_playhead:.3f}")
        self.var_status.set(msg)
        self.refresh()

    def _playhead_to_startup(self) -> None:
        try:
            t_local = float(self.var_startup_t.get())
        except ValueError:
            self._detect_startup()
            try:
                t_local = float(self.var_startup_t.get())
            except ValueError:
                return
        _, _, spans = concat_timeline(self._clips)
        t0g = spans[self._active][1] if self._active < len(spans) else 0.0
        self.var_playhead = t0g + t_local
        self.var_cut_t.set(f"{self.var_playhead:.3f}")
        self.refresh()

    def _trim_startup_active(self) -> None:
        if not (0 <= self._active < len(self._clips)):
            return
        rows = self._clips[self._active].rows
        if not rows:
            return
        gate, need_i = self._startup_gate_params()
        try:
            t0 = float(self.var_startup_t.get())
        except ValueError:
            t0, msg = find_startup_t_s(rows, rpm_gate=gate, require_index=need_i)
            if t0 is None:
                self.var_status.set(msg)
                return
            self.var_startup_t.set(f"{t0:.4f}")
        self._push_spike_undo()
        before = len(rows)
        trimmed = trim_before_t(rows, t0)
        if len(trimmed) >= before:
            self._spike_undo.pop()
            self.var_status.set("裁切无效果（启动点已在段首？）")
            return
        if not trimmed:
            self._spike_undo.pop()
            self.var_status.set("裁切后为空，已取消")
            return
        self._clips[self._active].rows = trimmed
        self.var_playhead = 0.0
        self._last_plot_view = None
        self.var_status.set(
            f"已裁掉启动前 {before - len(trimmed)} 点（保留自 t≥{t0:.4f}s），Ctrl+Z 撤销"
        )
        self.refresh()

    def _set_tool(self, name: str) -> None:
        self._tool.set(name)
        self._paint_tool_buttons()
        self._tool_hint()

    def _paint_tool_buttons(self) -> None:
        """高亮当前工具；圈毛刺用醒目红色。"""
        idle = {"bg": "#e8e8e8", "fg": "#222", "relief": tk.RAISED, "activebackground": "#ddd"}
        on = {"bg": "#2d6cdf", "fg": "#fff", "relief": tk.SUNKEN, "activebackground": "#1f5bb8"}
        on_ex = {"bg": "#c0392b", "fg": "#fff", "relief": tk.SUNKEN, "activebackground": "#a93226"}
        tool = self._tool.get()
        if hasattr(self, "btn_tool_cut"):
            self.btn_tool_cut.configure(**(on if tool == "cut" else idle))
            self.btn_tool_sel.configure(**(on if tool == "select" else idle))
            self.btn_tool_excl.configure(**(on_ex if tool == "exclude" else idle))

    def _tool_hint(self) -> None:
        tool = self._tool.get()
        if tool == "cut":
            self.chart.canvas.configure(cursor="crosshair")
            self.var_status.set("Cut 模式：在曲线上点击切开")
        elif tool == "exclude":
            self.chart.canvas.configure(cursor="tcross")
            act = "删除点" if self._exclude_action.get() == "delete" else "排除(红叉)"
            shape = "套索" if self._select_shape.get() == "lasso" else "矩形"
            self.var_status.set(
                f"圈毛刺：动作={act} 选区={shape} · 单击点 / 拖选 · 后台平滑重算"
            )
            try:
                self.chart.canvas.focus_set()
            except tk.TclError:
                pass
        else:
            self.chart.canvas.configure(cursor="arrow")
            self.var_status.set("选择模式：点曲线设刀口，点时间线选段")

    def _on_delete_key(self) -> None:
        """排除模式下 Del=删除已排除点；否则删选中段。"""
        if self._tool.get() == "exclude":
            if self._total_excluded() > 0:
                self._delete_excluded_points()
            else:
                self._clear_exclusions()
        else:
            self._delete_active()

    def _clear_lasso_overlay(self) -> None:
        if self._lasso_item is not None:
            try:
                self.chart.canvas.delete(self._lasso_item)
            except tk.TclError:
                pass
            self._lasso_item = None
        self._lasso_px = []

    def _draw_lasso_overlay(self) -> None:
        c = self.chart.canvas
        if self._lasso_item is not None:
            c.delete(self._lasso_item)
            self._lasso_item = None
        if len(self._lasso_px) < 2:
            return
        flat: list[float] = []
        for x, y in self._lasso_px:
            flat.extend((x, y))
        # 闭合预览
        flat.extend(self._lasso_px[0])
        self._lasso_item = c.create_line(
            *flat, fill="#e74c3c", width=2, dash=(4, 2), tags="lasso"
        )

    def _total_excluded(self) -> int:
        return sum(len(c.excluded) for c in self._clips)

    def _excluded_global_mask(self, rows: list, spans: list[tuple[int, float, float]]) -> list[bool]:
        """与 concat_timeline(rows) 对齐的排除掩码。"""
        mask = [False] * len(rows)
        gi = 0
        for ci, (_i, _t0, _t1) in enumerate(spans):
            clip = self._clips[ci]
            n = len(clip.rows)
            for li in range(n):
                if gi < len(mask) and li in clip.excluded:
                    mask[gi] = True
                gi += 1
        return mask

    def _rpm_for_fit(self, rows: list, mask: list[bool]) -> list[float]:
        """拟合用转速：排除点用邻点线性填充，原始 rows 不动。"""
        rpm = [float(r[1]) for r in rows]
        idx = [i for i, m in enumerate(mask) if m]
        if not idx:
            return rpm
        filled, _ = replace_indices_neighbor_avg(rpm, idx)
        return filled

    def _i0_first(self, refresh: bool = True) -> None:
        rows = self._timeline_rows()
        if rows:
            self.var_i0.set(str(index_baseline(rows)))
        if refresh:
            self.refresh()

    def _timeline_rows(self) -> list:
        rows, _, _ = concat_timeline(self._clips)
        return rows

    def _read_i0_len(self) -> tuple[int, float]:
        try:
            length = float(self.var_len.get().strip() or "1")
        except ValueError:
            length = 1.0
        rows = self._timeline_rows()
        s = (self.var_i0.get() or "auto").strip().lower()
        if s in ("", "auto"):
            i0 = index_baseline(rows) if rows else 0
        else:
            try:
                i0 = int(float(s))
            except ValueError:
                i0 = index_baseline(rows) if rows else 0
        return i0, length

    def _refresh_list(self) -> None:
        self.lst.delete(0, tk.END)
        for i, c in enumerate(self._clips):
            dur = _clip_dur_s(c.rows)
            mark = "● " if i == self._active else "  "
            self.lst.insert(tk.END, f"{mark}{c.name}  {len(c.rows)}点  {dur:.3f}s")
        if 0 <= self._active < len(self._clips):
            self.lst.selection_clear(0, tk.END)
            self.lst.selection_set(self._active)

    def _draw_timeline(self, spans: list[tuple[int, float, float]], total: float) -> None:
        c = self.tl_canvas
        c.delete("all")
        w = max(int(c.winfo_width()), 200)
        h = 56
        pad = 8
        usable = max(w - 2 * pad, 10)
        total = max(total, 0.01)
        colors = ["#3498db", "#2ecc71", "#e67e22", "#9b59b6", "#1abc9c", "#e74c3c"]
        for i, t0, t1 in spans:
            if t1 <= t0:
                continue
            x0 = pad + t0 / total * usable
            x1 = pad + t1 / total * usable
            col = colors[i % len(colors)]
            if i == self._active:
                c.create_rectangle(x0, 10, x1, h - 18, fill=col, outline="#fff", width=2)
            else:
                c.create_rectangle(x0, 14, x1, h - 18, fill=col, outline="#333", width=1)
            if x1 - x0 > 40:
                name = self._clips[i].name if i < len(self._clips) else "?"
                c.create_text(
                    (x0 + x1) / 2, (h - 4) / 2, text=name, fill="#fff", font=("Segoe UI", 8)
                )
        # playhead / cut marker
        ph = self.var_playhead
        xp = pad + ph / total * usable
        c.create_line(xp, 4, xp, h - 4, fill="#f1c40f", width=2)
        c.create_text(xp + 4, 6, anchor="nw", text="✂", fill="#f1c40f", font=("Segoe UI", 9))
        c.create_text(pad, h - 12, anchor="w", text="0s", fill="#aaa", font=("Segoe UI", 7))
        c.create_text(
            pad + usable, h - 12, anchor="e", text=f"{total:.2f}s", fill="#aaa", font=("Segoe UI", 7)
        )

    def _on_list_sel(self, _evt=None) -> None:
        sel = self.lst.curselection()
        if not sel:
            return
        self._active = int(sel[0])
        self.refresh()

    def _on_timeline_click(self, evt) -> None:
        rows, bounds, spans = concat_timeline(self._clips)
        total = bounds[-1] if bounds else 0.01
        w = max(int(self.tl_canvas.winfo_width()), 200)
        pad = 8
        usable = max(w - 2 * pad, 10)
        t = (evt.x - pad) / usable * total
        t = max(0.0, min(total, t))
        self.var_playhead = t
        self.var_cut_t.set(f"{t:.3f}")
        for i, t0, t1 in spans:
            if t0 <= t <= t1 + 1e-9:
                self._active = i
                break
        self.refresh()

    def _on_chart_press(self, evt) -> None:
        if self._tool.get() == "exclude":
            self._drag_start_px = (float(evt.x), float(evt.y))
            if self._select_shape.get() == "lasso":
                self._clear_lasso_overlay()
                self._lasso_px = [(float(evt.x), float(evt.y))]
                self._draw_lasso_overlay()
            else:
                pt = self.chart.data_from_pixel(evt.x, evt.y)
                if pt:
                    t, y = pt
                    self.chart.set_selection_rect((t, t, y, y))
            return
        self._on_chart_click(evt)

    def _on_chart_motion(self, evt) -> None:
        if self._tool.get() != "exclude" or not self._drag_start_px:
            return
        if self._select_shape.get() == "lasso":
            self._lasso_px.append((float(evt.x), float(evt.y)))
            if len(self._lasso_px) % 2 == 0:
                self._draw_lasso_overlay()
            return
        x0, y0 = self._drag_start_px
        p0 = self.chart.data_from_pixel(x0, y0)
        p1 = self.chart.data_from_pixel(evt.x, evt.y)
        if not p0 or not p1:
            return
        self.chart.set_selection_rect((p0[0], p1[0], p0[1], p1[1]))

    def _on_chart_release(self, evt) -> None:
        if self._tool.get() != "exclude" or not self._drag_start_px:
            return
        x0, y0 = self._drag_start_px
        self._drag_start_px = None
        if self._select_shape.get() == "lasso":
            self._lasso_px.append((float(evt.x), float(evt.y)))
            poly = list(self._lasso_px)
            self._clear_lasso_overlay()
            dx = abs(float(evt.x) - x0)
            dy = abs(float(evt.y) - y0)
            if dx < 6 and dy < 6 and len(poly) < 8:
                self._apply_hits_or_click(float(evt.x), float(evt.y), None)
                return
            hits = self._hit_points_in_lasso(poly)
            self._apply_hits_or_click(float(evt.x), float(evt.y), hits)
            return

        p0 = self.chart.data_from_pixel(x0, y0)
        p1 = self.chart.data_from_pixel(evt.x, evt.y)
        self.chart.set_selection_rect(None)
        if not p0 or not p1:
            return
        dx = abs(float(evt.x) - x0)
        dy = abs(float(evt.y) - y0)
        if dx < 6 and dy < 6:
            self._apply_hits_or_click(float(evt.x), float(evt.y), None)
            return
        hits = self._hit_points_in_box(p0[0], p1[0], p0[1], p1[1])
        self._apply_hits_or_click(float(evt.x), float(evt.y), hits)

    def _on_chart_click(self, evt) -> None:
        t = self.chart.x_from_pixel(evt.x)
        if t is None:
            return
        self.var_playhead = t
        self.var_cut_t.set(f"{t:.3f}")
        if self._tool.get() == "cut":
            self._cut_at(t)
        elif self._tool.get() == "select":
            _, _, spans = concat_timeline(self._clips)
            for i, t0, t1 in spans:
                if t0 <= t <= t1 + 1e-9:
                    self._active = i
                    break
            self.refresh()

    def _push_spike_undo(self) -> None:
        snap = [(list(c.rows), set(c.excluded)) for c in self._clips]
        self._spike_undo.append(snap)
        if len(self._spike_undo) > 30:
            self._spike_undo.pop(0)

    def _undo_spike(self) -> None:
        if not self._spike_undo:
            self.var_status.set("没有可撤销的操作")
            return
        snap = self._spike_undo.pop()
        if len(snap) != len(self._clips):
            # 段数变了则尽量按顺序恢复
            n = min(len(snap), len(self._clips))
            for i in range(n):
                self._clips[i].rows = list(snap[i][0])
                self._clips[i].excluded = set(snap[i][1])
        else:
            for c, (rows, excl) in zip(self._clips, snap):
                c.rows = list(rows)
                c.excluded = set(excl)
        self.var_status.set("已撤销上一次排除/固化")
        self.refresh()

    def _y_for_edit(self, r: tuple) -> float:
        """圈毛刺命中用的 Y：counts 视图用累计 counts，否则 |RPM|。"""
        if self.var_view.get() == "counts" and row_has_counts(r):
            return float(r[7])
        return abs(float(r[1]))

    def _clip_has_counts(self, clip: Clip) -> bool:
        return bool(clip.rows) and row_has_counts(clip.rows[0])

    def _recompute_clip_rpm(self, clip: Clip) -> None:
        if self._clip_has_counts(clip):
            clip.rows = recompute_rows_rpm_from_counts(clip.rows)

    def _hit_points_in_box(
        self, t_a: float, t_b: float, y_a: float, y_b: float
    ) -> list[tuple[int, int]]:
        """返回 (clip_i, local_i) 列表。转速视图用 |RPM|；counts 视图用 counts。"""
        t0, t1 = min(t_a, t_b), max(t_a, t_b)
        view = self.var_view.get()
        if view in ("rpm", "all", "counts"):
            y_lo, y_hi = min(y_a, y_b), max(y_a, y_b)
            use_y = True
        else:
            y_lo, y_hi = float("-inf"), float("inf")
            use_y = False
        _, _, spans = concat_timeline(self._clips)
        hits: list[tuple[int, int]] = []
        for ci, (_i, t_start, _t_end) in enumerate(spans):
            clip = self._clips[ci]
            if not clip.rows:
                continue
            base = t_base_ms(clip.rows)
            for li, r in enumerate(clip.rows):
                tg = t_start + (float(r[0]) - base) / 1000.0
                if tg < t0 or tg > t1:
                    continue
                if not use_y:
                    hits.append((ci, li))
                    continue
                yv = self._y_for_edit(r)
                if y_lo <= yv <= y_hi:
                    hits.append((ci, li))
        return hits

    def _exclude_nearest_click(self, px: float, py: float) -> None:
        """单击：切换最近采样点的排除状态（像素距离阈值）。"""
        rows, _, spans = concat_timeline(self._clips)
        if not rows:
            return
        best: tuple[float, int, int] | None = None  # dist2, ci, li
        for ci, (_i, t_start, _t_end) in enumerate(spans):
            clip = self._clips[ci]
            cbase = t_base_ms(clip.rows) if clip.rows else 0.0
            for li, r in enumerate(clip.rows):
                tg = t_start + (float(r[0]) - cbase) / 1000.0
                yv = self._y_for_edit(r)
                if self.chart._xf and self.chart._yf:
                    x0, x1, pl, pw = self.chart._xf
                    y0, y1, pt, ph = self.chart._yf
                    if pw > 0 and ph > 0:
                        sx = pl + (tg - x0) / (x1 - x0) * pw
                        sy = pt + ph - (yv - y0) / (y1 - y0) * ph
                        d2 = (sx - px) ** 2 + (sy - py) ** 2
                        if best is None or d2 < best[0]:
                            best = (d2, ci, li)
        if best is None or best[0] > 14 ** 2:
            self.var_status.set("未点中采样点（请靠近曲线上的点再点，或拖框）")
            return
        _d2, ci, li = best
        self._push_spike_undo()
        excl = self._clips[ci].excluded
        if li in excl:
            excl.discard(li)
            self.var_status.set(f"已恢复点（段{ci} #{li}）· 排除总数={self._total_excluded()}")
        else:
            excl.add(li)
            self.var_status.set(f"已排除点（段{ci} #{li}）· 排除总数={self._total_excluded()}")
        self.refresh()

    def _nearest_point(self, px: float, py: float) -> tuple[int, int] | None:
        _, _, spans = concat_timeline(self._clips)
        best: tuple[float, int, int] | None = None
        if not self.chart._xf or not self.chart._yf:
            return None
        x0, x1, pl, pw = self.chart._xf
        y0, y1, pt, ph = self.chart._yf
        if pw <= 0 or ph <= 0:
            return None
        for ci, (_i, t_start, _t_end) in enumerate(spans):
            clip = self._clips[ci]
            if not clip.rows:
                continue
            cbase = t_base_ms(clip.rows)
            for li, r in enumerate(clip.rows):
                tg = t_start + (float(r[0]) - cbase) / 1000.0
                yv = self._y_for_edit(r)
                sx = pl + (tg - x0) / (x1 - x0) * pw
                sy = pt + ph - (yv - y0) / (y1 - y0) * ph
                d2 = (sx - px) ** 2 + (sy - py) ** 2
                if best is None or d2 < best[0]:
                    best = (d2, ci, li)
        if best is None or best[0] > 14 ** 2:
            return None
        return best[1], best[2]

    def _hit_points_in_lasso(
        self, poly_px: list[tuple[float, float]]
    ) -> list[tuple[int, int]]:
        if len(poly_px) < 3 or not self.chart._xf or not self.chart._yf:
            return []
        _, _, spans = concat_timeline(self._clips)
        x0, x1, pl, pw = self.chart._xf
        y0, y1, pt, ph = self.chart._yf
        if pw <= 0 or ph <= 0:
            return []
        hits: list[tuple[int, int]] = []
        for ci, (_i, t_start, _t_end) in enumerate(spans):
            clip = self._clips[ci]
            if not clip.rows:
                continue
            cbase = t_base_ms(clip.rows)
            for li, r in enumerate(clip.rows):
                tg = t_start + (float(r[0]) - cbase) / 1000.0
                yv = self._y_for_edit(r)
                sx = pl + (tg - x0) / (x1 - x0) * pw
                sy = pt + ph - (yv - y0) / (y1 - y0) * ph
                if _point_in_poly(sx, sy, poly_px):
                    hits.append((ci, li))
        return hits

    def _exclude_hits(self, hits: list[tuple[int, int]]) -> None:
        self._push_spike_undo()
        n_new = 0
        for ci, li in hits:
            if li not in self._clips[ci].excluded:
                self._clips[ci].excluded.add(li)
                n_new += 1
        self.var_status.set(
            f"排除 +{n_new}（命中{len(hits)}）· 总数={self._total_excluded()} · 表未改"
        )
        self.refresh()

    def _delete_point_hits(self, hits: list[tuple[int, int]]) -> None:
        """从序列删除点；若有 counts 则删后重算该段 RPM。"""
        if not hits:
            return
        self._push_spike_undo()
        by_clip: dict[int, set[int]] = {}
        for ci, li in hits:
            by_clip.setdefault(ci, set()).add(li)
        total = 0
        for ci, idxs in by_clip.items():
            clip = self._clips[ci]
            keep = [r for i, r in enumerate(clip.rows) if i not in idxs]
            new_excl: set[int] = set()
            new_i = 0
            for old_i in range(len(clip.rows)):
                if old_i in idxs:
                    continue
                if old_i in clip.excluded:
                    new_excl.add(new_i)
                new_i += 1
            total += len(clip.rows) - len(keep)
            clip.rows = keep
            clip.excluded = new_excl
            self._recompute_clip_rpm(clip)
        self.var_status.set(
            f"已删除 {total} 个点"
            + ("；已按剩余 counts 重算 RPM" if any(self._clip_has_counts(c) for c in self._clips) else "")
        )
        self.refresh()

    def _exclude_box(self, t_a: float, t_b: float, y_a: float, y_b: float) -> None:
        """框选：将框内点加入排除集（不改原始数值）。"""
        hits = self._hit_points_in_box(t_a, t_b, y_a, y_b)
        self._apply_hits_or_click(0.0, 0.0, hits if hits else [])

    def _apply_hits_or_click(
        self, px: float, py: float, hits: list[tuple[int, int]] | None
    ) -> None:
        """hits=None → 单击最近点；否则批量排除或删除。"""
        if hits is None:
            if self._exclude_action.get() == "delete":
                one = self._nearest_point(px, py)
                if one is None:
                    self.var_status.set("未点中采样点")
                    return
                self._delete_point_hits([one])
            else:
                self._exclude_nearest_click(px, py)
            return
        if not hits:
            self.var_status.set("选区内无点命中（建议「仅转速」或「counts-t」视图）")
            return
        if self._exclude_action.get() == "delete":
            self._delete_point_hits(hits)
        else:
            self._exclude_hits(hits)

    def _delete_excluded_points(self) -> None:
        """把当前所有红叉排除点从序列中删掉。"""
        hits: list[tuple[int, int]] = []
        for ci, clip in enumerate(self._clips):
            for li in sorted(clip.excluded):
                hits.append((ci, li))
        if not hits:
            self.var_status.set("没有排除点可删")
            return
        if not messagebox.askyesno(
            "删除已排除点",
            f"将从数据中删除 {len(hits)} 个已排除点；有 counts 时会重算 RPM。确定？",
        ):
            return
        self._delete_point_hits(hits)

    def _clear_exclusions(self) -> None:
        if self._total_excluded() == 0:
            self.var_status.set("当前没有排除点")
            return
        self._push_spike_undo()
        for c in self._clips:
            c.excluded.clear()
        self.var_status.set("已清除全部排除标记")
        self.refresh()

    def _bake_exclusions(self) -> None:
        """把排除点固化为邻点插值写入 rows；有 counts 时先修 counts 再重算 RPM。"""
        if self._total_excluded() == 0:
            messagebox.showinfo("固化排除", "没有排除点可固化")
            return
        if not messagebox.askyesno(
            "固化排除",
            "将把已排除点用邻点线性插值写入数据表。\n"
            "有 counts 时优先插值 counts 并重算 RPM。确定？",
        ):
            return
        self._push_spike_undo()
        total = 0
        for clip in self._clips:
            if not clip.excluded or not clip.rows:
                continue
            idxs = sorted(clip.excluded)
            if self._clip_has_counts(clip):
                cs = [float(r[7]) for r in clip.rows]
                new_c, n_rep = replace_indices_neighbor_avg(cs, idxs)
                new_rows = []
                for i, r in enumerate(clip.rows):
                    lst = list(r)
                    lst[7] = int(round(new_c[i]))
                    new_rows.append(tuple(lst))
                clip.rows = recompute_rows_rpm_from_counts(new_rows)
                total += n_rep
            else:
                rpms = [float(r[1]) for r in clip.rows]
                new_rpms, n_rep = replace_indices_neighbor_avg(rpms, idxs)
                clip.rows = apply_rpm_replacements(clip.rows, new_rpms)
                total += n_rep
            clip.excluded.clear()
        self.var_status.set(f"已固化 {total} 点到数据表（排除标记已清）")
        self.refresh()

    def _cut_at_playhead(self) -> None:
        self._cut_at(float(self.var_playhead))

    def _cut_at(self, t_global: float) -> None:
        """在全局时间轴 t 处用刀片切开所在 Clip。"""
        if not self._clips:
            return
        _, bounds, spans = concat_timeline(self._clips)
        total = bounds[-1] if bounds else 0.0
        if total <= 0:
            return
        t_global = max(0.0, min(total, t_global))
        hit = None
        for i, t0, t1 in spans:
            if t0 < t1 and t0 <= t_global <= t1:
                hit = (i, t0, t1)
                break
        if hit is None:
            messagebox.showinfo("Cut", "刀口不在任何片段上")
            return
        i, t0, _t1 = hit
        local_t = t_global - t0
        rows = self._clips[i].rows
        excl = set(self._clips[i].excluded)
        left, right = split_at_time(rows, local_t)
        # 避免切在端点产生空段
        if len(left) < 2 or len(right) < 2:
            messagebox.showinfo("Cut", "刀口太靠边，无法切开（两端至少各留一点）")
            return
        # 排除索引随切开重映射
        base = t_base_ms(rows)
        left_idx = [j for j, r in enumerate(rows) if (float(r[0]) - base) / 1000.0 <= local_t]
        right_idx = [j for j, r in enumerate(rows) if (float(r[0]) - base) / 1000.0 > local_t]
        left_excl = {ni for ni, oi in enumerate(left_idx) if oi in excl}
        right_excl = {ni for ni, oi in enumerate(right_idx) if oi in excl}
        name = self._clips[i].name
        c1 = Clip(rows=left, name=f"{name}a", uid=CurveStudio._uid, excluded=left_excl)
        CurveStudio._uid += 1
        c2 = Clip(rows=right, name=f"{name}b", uid=CurveStudio._uid, excluded=right_excl)
        CurveStudio._uid += 1
        self._clips[i : i + 1] = [c1, c2]
        self._active = i + 1  # 选中右段，方便继续删
        self.var_playhead = t_global
        self.var_status.set(f"已切开 @ {t_global:.3f}s → {c1.name} + {c2.name}（可删不要的段）")
        self.refresh()

    def _delete_active(self) -> None:
        if not self._clips:
            return
        if len(self._clips) == 1:
            if not messagebox.askyesno("删除", "只剩一段，删除后时间线为空。确定？"):
                return
        name = self._clips[self._active].name
        del self._clips[self._active]
        if not self._clips:
            self._clips = [Clip(rows=[], name="空", uid=CurveStudio._uid)]
            CurveStudio._uid += 1
        self._active = min(self._active, len(self._clips) - 1)
        self.var_status.set(f"已删除 {name}")
        self.refresh()

    def _merge_all(self) -> None:
        rows = self._timeline_rows()
        if not rows:
            return
        # 拼接排除索引
        merged_excl: set[int] = set()
        off = 0
        for c in self._clips:
            for li in c.excluded:
                merged_excl.add(off + li)
            off += len(c.rows)
        self._clips = [
            Clip(rows=rows, name="Merged", uid=CurveStudio._uid, excluded=merged_excl)
        ]
        CurveStudio._uid += 1
        self._active = 0
        self.var_playhead = 0.0
        self.refresh()

    def _filter_rpm_active(self) -> None:
        if not (0 <= self._active < len(self._clips)):
            return
        try:
            thr = float(self.var_rpm_cut.get().strip() or "0")
        except ValueError:
            thr = 0.0
        rows = filter_by_rpm(self._clips[self._active].rows, rpm_min=thr)
        if not rows:
            messagebox.showinfo("过滤", "过滤后为空，未改动")
            return
        self._clips[self._active].rows = rows
        self._clips[self._active].excluded.clear()  # 索引已失效
        self.refresh()

    def _restore(self) -> None:
        self._clips = [
            Clip(rows=list(self._raw), name="Clip1", uid=CurveStudio._uid, excluded=set())
        ]
        CurveStudio._uid += 1
        self._active = 0
        self.var_playhead = 0.0
        self._last_plot_view = None  # 强制复位缩放
        self._spike_undo.clear()
        self.refresh()
        self.chart.reset_view()

    def refresh(self) -> None:
        rows, bounds, spans = concat_timeline(self._clips)
        total = bounds[-1] if bounds else 0.01
        self._refresh_list()
        self.after_idle(lambda: self._draw_timeline(spans, total))

        if not rows:
            self.chart.clear()
            self.var_status.set("时间线为空 — 恢复原片或重新打开")
            return

        i0, length = self._read_i0_len()
        try:
            smooth = int(float(self.var_smooth.get().strip() or "21"))
        except ValueError:
            smooth = 21
        smooth = max(3, smooth)
        try:
            post = int(float(self.var_post_smooth.get().strip() or "5"))
        except ValueError:
            post = 5
        method = (self.var_diff_method.get() or "ma_central").strip()
        sp = self._read_rpm_smooth_params()
        excl_mask = self._excluded_global_mask(rows, spans)
        s_mode = self.var_s_mode.get()
        active = self._active
        a_rows = list(self._clips[active].rows) if 0 <= active < len(self._clips) else []
        a_excl = set(self._clips[active].excluded) if 0 <= active < len(self._clips) else set()
        a_name = self._clips[active].name if 0 <= active < len(self._clips) else "-"
        t0g = spans[active][1] if 0 <= active < len(spans) else 0.0
        n_clips = len(self._clips)
        show_raw = bool(self.var_show_raw.get())
        show_sm = bool(self.var_show_smooth.get())
        view = self.var_view.get()
        ph = self.var_playhead
        preserve_view = self._last_plot_view == view
        self._last_plot_view = view

        self._refresh_seq += 1
        seq = self._refresh_seq
        snap = {
            "seq": seq,
            "rows": list(rows),
            "spans": list(spans),
            "total": total,
            "i0": i0,
            "length": length,
            "smooth": smooth,
            "post": post,
            "method": method,
            "sp": dict(sp),
            "excl_mask": list(excl_mask),
            "s_mode": s_mode,
            "a_rows": a_rows,
            "a_excl": a_excl,
            "a_name": a_name,
            "t0g": t0g,
            "n_clips": n_clips,
            "show_raw": show_raw,
            "show_sm": show_sm,
            "view": view,
            "ph": ph,
            "preserve_view": preserve_view,
        }

        def work() -> None:
            try:
                payload = CurveStudio._compute_plot_payload(snap)
            except Exception as exc:  # noqa: BLE001
                self.after(0, lambda: self.var_status.set(f"重算失败: {exc}"))
                return
            self.after(0, lambda p=payload, s=seq: self._apply_plot_payload(s, p))

        if len(rows) < 600:
            self._apply_plot_payload(seq, CurveStudio._compute_plot_payload(snap))
        else:
            self.var_status.set(f"后台重算平滑 n={len(rows)}…")
            threading.Thread(target=work, daemon=True).start()

    @staticmethod
    def _compute_plot_payload(snap: dict) -> dict:
        """后台线程：平滑 / S / a 重算（不碰 Tk）。"""
        rows = snap["rows"]
        excl_mask = snap["excl_mask"]
        sp = snap["sp"]
        length = snap["length"]
        i0 = snap["i0"]
        base = t_base_ms(rows)
        xs = [(float(r[0]) - base) / 1000.0 for r in rows]
        rpm_signed = [float(r[1]) for r in rows]
        idx = [i for i, m in enumerate(excl_mask) if m]
        if idx:
            rpm_for_fit, _ = replace_indices_neighbor_avg(rpm_signed, idx)
        else:
            rpm_for_fit = list(rpm_signed)
        rpm_sm_signed, n_spike = smooth_rpm_series(rpm_for_fit, **sp)
        rows_sm = apply_rpm_replacements(rows, rpm_sm_signed)
        ys_rpm = [abs(v) for v in rpm_signed]
        ys_rpm_sm = [abs(v) for v in rpm_sm_signed]
        ys_counts = [float(r[7]) if len(r) > 7 and r[7] is not None else 0.0 for r in rows]
        has_counts = bool(rows) and len(rows[0]) > 7 and rows[0][7] is not None
        n_excl = sum(1 for m in excl_mask if m)
        view = snap["view"]
        if view == "counts" and has_counts:
            excl_marks = [(xs[i], ys_counts[i]) for i, m in enumerate(excl_mask) if m]
        else:
            excl_marks = [(xs[i], ys_rpm[i]) for i, m in enumerate(excl_mask) if m]
        sm_tag = sp["method"]
        if snap["s_mode"] == "rpm":
            ser = integrate_rpm_distance(rows_sm, length_per_rev=length)
            ys_s = [p[2] for p in ser]
            s_lab = f"S←∫rpm_smooth×{length}"
        else:
            ser = distance_series(rows, i0=i0, length_per_i=length)
            ys_s = [p[2] for p in ser]
            s_lab = f"S←(I-{i0})×{length}"

        a_rows = snap["a_rows"]
        a_excl = snap["a_excl"]
        tx: list[float] = []
        ay: list[float] = []
        if a_rows:
            abase = t_base_ms(a_rows)
            a_rpm = [float(r[1]) for r in a_rows]
            if a_excl:
                a_rpm, _ = replace_indices_neighbor_avg(a_rpm, sorted(a_excl))
            a_rpm_sm, _ = smooth_rpm_series(a_rpm, **sp)
            a_disp = []
            t0g = snap["t0g"]
            for r, rpm_v in zip(a_rows, a_rpm_sm):
                rel = (float(r[0]) - abase) / 1000.0
                a_disp.append(((t0g + rel) * 1000.0, rpm_v, r[2], r[3], r[4]))
            tx, ay = accel_from_rows(
                a_disp,
                smooth=snap["smooth"],
                method=snap["method"],
                post_smooth=snap["post"],
                use_signed=True,
            )
        return {
            "xs": xs,
            "ys_rpm": ys_rpm,
            "ys_rpm_sm": ys_rpm_sm,
            "ys_counts": ys_counts,
            "has_counts": has_counts,
            "ys_s": ys_s,
            "s_lab": s_lab,
            "tx": tx,
            "ay": ay,
            "n_spike": n_spike,
            "n_excl": n_excl,
            "excl_marks": excl_marks,
            "sm_tag": sm_tag,
            "n_rows": len(rows),
            "total": snap["total"],
            "n_clips": snap["n_clips"],
            "show_raw": snap["show_raw"],
            "show_sm": snap["show_sm"],
            "view": snap["view"],
            "ph": snap["ph"],
            "preserve_view": snap["preserve_view"],
            "smooth": snap["smooth"],
            "post": snap["post"],
            "method": snap["method"],
            "a_name": snap["a_name"],
        }

    def _apply_plot_payload(self, seq: int, p: dict) -> None:
        if seq != self._refresh_seq:
            return
        xs = p["xs"]
        ys_rpm = p["ys_rpm"]
        ys_rpm_sm = p["ys_rpm_sm"]
        ys_s = p["ys_s"]
        tx, ay = p["tx"], p["ay"]
        n_spike, n_excl = p["n_spike"], p["n_excl"]
        sm_tag = p["sm_tag"]
        total = p["total"]
        show_raw, show_sm = p["show_raw"], p["show_sm"]
        if not show_raw and not show_sm:
            show_raw = True
        view = p["view"]
        preserve_view = p["preserve_view"]
        peak = max(ys_rpm) if ys_rpm else 0.0
        peak_sm = max(ys_rpm_sm) if ys_rpm_sm else 0.0
        s_end = ys_s[-1] if ys_s else 0.0
        apeak = max((abs(v) for v in ay), default=0.0)
        ph = p["ph"]
        mark_in = mark_out = ph
        ms_win = p["smooth"] * 0.5
        method = p["method"]
        smooth, post = p["smooth"], p["post"]
        s_lab = p["s_lab"]

        if view == "rpm":
            series = []
            yhi = 50.0
            if show_raw:
                series.append((xs, ys_rpm, "#1a6fb5", f"|RPM|平滑前 peak={peak:.0f}"))
                yhi = max(yhi, peak * 1.15)
            if show_sm:
                extra = f" 刺={n_spike}" if n_spike else ""
                if n_excl:
                    extra += f" 排除={n_excl}"
                series.append(
                    (xs, ys_rpm_sm, "#e67e22", f"|RPM|平滑后[{sm_tag}]{extra} peak={peak_sm:.0f}")
                )
                yhi = max(yhi, peak_sm * 1.15)
            if not series:
                series.append((xs, ys_rpm, "#1a6fb5", "|RPM|"))
            self.chart.set_series(
                series,
                xlim=(0.0, max(total, 0.01)),
                ylim=(0.0, yhi),
                title=(
                    f"时间线 n={p['n_rows']} 段数={p['n_clips']}  黄线=刀口"
                    + (f"  红叉=排除{n_excl}" if n_excl else "")
                ),
                xlabel="t (s)",
                ylabel="|RPM|",
                mark_in=mark_in,
                mark_out=mark_out,
                preserve_view=preserve_view,
            )
        elif view == "counts":
            ys_c = p.get("ys_counts") or []
            if not p.get("has_counts"):
                self.chart.set_series(
                    [([0, total], [0, 0], "#6c3483", "无 counts")],
                    xlim=(0.0, max(total, 0.01)),
                    ylim=(-1, 1),
                    title="本段无 counts（需 v40 BIN / magic 0xAB1C0002）",
                    xlabel="t (s)",
                    ylabel="counts",
                    allow_negative=True,
                    preserve_view=False,
                )
            else:
                clo = min(ys_c) if ys_c else 0.0
                chi = max(ys_c) if ys_c else 1.0
                pad = max(abs(chi - clo) * 0.08, 10.0)
                self.chart.set_series(
                    [
                        (
                            xs,
                            ys_c,
                            "#6c3483",
                            f"counts 原始 [{clo:.0f}~{chi:.0f}]"
                            + (f" 排除={n_excl}" if n_excl else ""),
                        )
                    ],
                    xlim=(0.0, max(total, 0.01)),
                    ylim=(clo - pad, chi + pad),
                    title=(
                        f"counts vs t · n={p['n_rows']} · 圈毛刺可删异常点后自动重算 RPM"
                        + (f" · 红叉={n_excl}" if n_excl else "")
                    ),
                    xlabel="t (s)",
                    ylabel="counts",
                    mark_in=mark_in,
                    mark_out=mark_out,
                    allow_negative=True,
                    preserve_view=preserve_view,
                )
        elif view == "s":
            smin = min(ys_s) if ys_s else 0.0
            smax = max(abs(y) for y in ys_s) if ys_s else 1.0
            self.chart.set_series(
                [(xs, ys_s, "#0b7a4b", f"{s_lab} S末={s_end:.3f}")],
                xlim=(0.0, max(total, 0.01)),
                ylim=(min(0.0, smin), max(smax * 1.1, 1e-3)),
                title="距离 S（∫rpm 模式基于平滑转速）",
                xlabel="t (s)",
                ylabel="S",
                mark_in=mark_in,
                mark_out=mark_out,
                allow_negative=smin < 0,
                preserve_view=preserve_view,
            )
        elif view == "a":
            if tx and ay:
                self.chart.set_series(
                    [(tx, ay, "#c0392b", f"a←rpm_smooth[{sm_tag}] |peak|={apeak:.1f}")],
                    xlim=(min(tx), max(tx) + 1e-3),
                    ylim=(-apeak * 1.2 - 1, apeak * 1.2 + 1),
                    title=(
                        f"加速度←平滑转速 [{method}] 窗{smooth}(~{ms_win:.0f}ms) "
                        f"后再滑{post} · {p['a_name']}"
                    ),
                    xlabel="t (s)",
                    ylabel="a (RPM/s)",
                    allow_negative=True,
                    preserve_view=preserve_view,
                )
            else:
                self.chart.set_series(
                    [([0, total], [0, 0], "#c0392b", "a")],
                    title="加速度（选中段太短，加大段或减小窗）",
                    xlabel="t (s)",
                    ylabel="a",
                    allow_negative=True,
                    preserve_view=False,
                )
        else:
            series = []
            if show_raw:
                nr, r0, r1 = _norm01(ys_rpm)
                series.append((xs, nr, "#1a6fb5", f"RPM平滑前 [{r0:.0f}~{r1:.0f}]"))
            if show_sm:
                nr2, r20, r21 = _norm01(ys_rpm_sm)
                series.append((xs, nr2, "#e67e22", f"RPM平滑后 [{r20:.0f}~{r21:.0f}]"))
            ns, s0, s1 = _norm01(ys_s)
            series.append((xs, ns, "#0b7a4b", f"S(←smooth) [{s0:.3g}~{s1:.3g}]"))
            if tx and ay:
                na, a0, a1 = _norm01(ay, signed=True)
                series.append((tx, na, "#c0392b", f"a←rpm_smooth [{a0:.1f}~{a1:.1f}]"))
            self.chart.set_series(
                series,
                xlim=(0.0, max(total, 0.01)),
                ylim=(-105, 105),
                title="三线同图（S/a 基于平滑转速）",
                xlabel="t (s)",
                ylabel="%",
                mark_in=mark_in,
                mark_out=mark_out,
                allow_negative=True,
                preserve_view=preserve_view,
            )

        self.chart.set_excluded_marks(
            p["excl_marks"] if view in ("rpm", "counts") else []
        )
        layers = []
        if show_raw:
            layers.append("平滑前")
        if show_sm:
            layers.append("平滑后")
        self.var_cut_t.set(f"{ph:.3f}")
        self.var_status.set(
            f"显示={'+'.join(layers) or '-'}  工具={self._tool.get()}  选中={p['a_name']}  "
            f"段数={p['n_clips']}  总长={total:.3f}s  S末={s_end:.3f}  "
            f"|a|peak={apeak:.1f}  RPM刺={n_spike}  排除={n_excl}"
        )

    def _commit(self) -> None:
        rows = self._timeline_rows()
        if self._on_commit:
            self._on_commit(rows)
            messagebox.showinfo("写回", f"已写回主列表 {len(rows)} 点（{len(self._clips)} 段拼接）")
        else:
            messagebox.showinfo("写回", "无主列表回调")

    def _export_csv(self) -> None:
        """导出点集：同一原始 X，两个 Y（原始转速 + 平滑后转速）。"""
        rows, _bounds, spans = concat_timeline(self._clips)
        if not rows:
            messagebox.showinfo("导出", "时间线为空")
            return
        sp = self._read_rpm_smooth_params()
        excl_mask = self._excluded_global_mask(rows, spans)
        n_excl = sum(1 for m in excl_mask if m)
        # Y1：原始（数据表未改）
        rpm_raw = [float(r[1]) for r in rows]
        # Y2：排除毛刺后重算的平滑曲线（与图上橙色线同源）
        rpm_for_fit = self._rpm_for_fit(rows, excl_mask)
        rpm_sm, _ = smooth_rpm_series(rpm_for_fit, **sp)
        base = t_base_ms(rows)
        # X：原始时间轴
        xs = [(float(r[0]) - base) / 1000.0 for r in rows]

        path = filedialog.asksaveasfilename(
            title="导出曲线点集（X=原始时间，Y原始 + Y平滑）",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile=f"curve_raw_smooth_{len(rows)}.csv",
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "t_rel_s",  # X：原始时间 (s)
                    "rpm_raw",  # Y1：原始转速
                    "rpm_smooth",  # Y2：平滑后转速
                    "rpm_raw_abs",
                    "rpm_smooth_abs",
                    "counts",  # v40 原始累计步数（无则空）
                    "excluded",
                    "t_ms",
                    "smooth_method",
                ]
            )
            for i, r in enumerate(rows):
                cval = ""
                if len(r) > 7 and r[7] is not None:
                    cval = str(int(r[7]))
                w.writerow(
                    [
                        f"{xs[i]:.6f}",
                        f"{rpm_raw[i]:.4f}",
                        f"{rpm_sm[i]:.4f}",
                        f"{abs(rpm_raw[i]):.4f}",
                        f"{abs(rpm_sm[i]):.4f}",
                        cval,
                        1 if excl_mask[i] else 0,
                        f"{float(r[0]):.3f}",
                        sp["method"],
                    ]
                )
        self.var_status.set(
            f"已导出 {len(rows)} 点 → {path}（X + Y原始 + Y平滑；排除={n_excl}）"
        )


def open_curve_studio(
    master,
    rows: Sequence[tuple],
    *,
    title: str = "曲线",
    on_commit: Callable[[list], None] | None = None,
) -> CurveStudio | None:
    if not rows:
        messagebox.showinfo("曲线工作室", "没有数据点")
        return None
    win = CurveStudio(master, rows, title=title, on_commit=on_commit)
    win.lift()
    win.focus_force()
    return win


if __name__ == "__main__":
    import sys

    root = tk.Tk()
    root.withdraw()
    from abi_monitor import parse_snap_file

    argv = [a for a in sys.argv[1:] if not a.startswith("-")]
    path = argv[0] if argv else filedialog.askopenfilename(
        filetypes=[
            ("snap/csv", "*.bin;*.csv"),
            ("bin", "*.bin"),
            ("csv", "*.csv"),
            ("all", "*.*"),
        ]
    )
    if not path:
        sys.exit(0)
    p = Path(path)
    if p.suffix.lower() == ".csv":
        rows = load_timeline_csv(p)
    else:
        _n, _hz, rows = parse_snap_file(p.read_bytes())
    open_curve_studio(root, rows, title=p.name)
    root.mainloop()
