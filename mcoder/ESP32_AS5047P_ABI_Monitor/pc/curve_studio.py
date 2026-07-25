#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
曲线工作室（Sony Vegas 风格）：
  - 时间线由若干 Clip 组成
  - Cut 刀片：在曲线上点击切开，分成两段
  - 选中一段 → 删除（剪掉不要的部分）
  - 单图：仅 RPM / 仅 S / 仅 a / 三线归一化
"""

from __future__ import annotations

import csv
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Sequence

from rpm_chart import RpmChart
from snap_edit import (
    accel_from_rows,
    distance_series,
    filter_by_rpm,
    index_baseline,
    integrate_rpm_distance,
    split_at_time,
    t_base_ms,
)


@dataclass
class Clip:
    rows: list
    name: str = ""
    uid: int = 0


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
            Clip(rows=list(rows), name="Clip1", uid=CurveStudio._uid)
        ]
        CurveStudio._uid += 1
        self._active = 0
        self._tool = tk.StringVar(value="cut")  # cut | select

        self.var_i0 = tk.StringVar(value="auto")
        self.var_len = tk.StringVar(value="1.0")
        self.var_s_mode = tk.StringVar(value="i")
        self.var_rpm_cut = tk.StringVar(value="10")
        self.var_smooth = tk.StringVar(value="21")  # @2kHz ≈10ms，数字转速默认
        self.var_diff_method = tk.StringVar(value="ma_central")
        self.var_post_smooth = tk.StringVar(value="5")
        self.var_view = tk.StringVar(value="rpm")
        self.var_status = tk.StringVar(value="")
        self.var_cut_t = tk.StringVar(value="—")
        self.var_playhead = 0.0

        self._build()
        self._i0_first(refresh=False)
        self.refresh()
        self.bind("<KeyPress-c>", lambda e: self._set_tool("cut"))
        self.bind("<KeyPress-C>", lambda e: self._set_tool("cut"))
        self.bind("<KeyPress-v>", lambda e: self._set_tool("select"))
        self.bind("<KeyPress-V>", lambda e: self._set_tool("select"))
        self.bind("<Delete>", lambda e: self._delete_active())
        self.bind("<BackSpace>", lambda e: self._delete_active())
        self.focus_set()
        self.protocol("WM_DELETE_WINDOW", self.destroy)

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text=self._title, font=("Segoe UI", 12, "bold")).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.var_status, foreground="#333").pack(side=tk.LEFT, padx=12)

        tools = ttk.LabelFrame(self, text="工具（Vegas 风格）", padding=6)
        tools.pack(fill=tk.X, padx=8, pady=2)
        ttk.Radiobutton(
            tools, text="✂ Cut 刀片 (C)", variable=self._tool, value="cut", command=self._tool_hint
        ).pack(side=tk.LEFT, padx=6)
        ttk.Radiobutton(
            tools, text="▸ 选择 (V)", variable=self._tool, value="select", command=self._tool_hint
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(tools, text="在刀口切开", command=self._cut_at_playhead).pack(side=tk.LEFT, padx=8)
        ttk.Button(tools, text="删除选中段 Del", command=self._delete_active).pack(side=tk.LEFT, padx=4)
        ttk.Button(tools, text="合并全部段", command=self._merge_all).pack(side=tk.LEFT, padx=4)
        ttk.Label(tools, text="刀口 t=").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(tools, textvariable=self.var_cut_t, width=10).pack(side=tk.LEFT)
        ttk.Label(
            tools, text="  Cut：点曲线切开 → 时间线出两段 → 选中不要的段删掉", foreground="#555"
        ).pack(side=tk.LEFT, padx=8)

        view = ttk.LabelFrame(self, text="显示", padding=4)
        view.pack(fill=tk.X, padx=8, pady=2)
        for text, val in (
            ("仅转速", "rpm"),
            ("仅距离 S", "s"),
            ("仅加速度 a", "a"),
            ("三线同图", "all"),
        ):
            ttk.Radiobutton(
                view, text=text, variable=self.var_view, value=val, command=self.refresh
            ).pack(side=tk.LEFT, padx=6)

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
        ttk.Button(opts, text="导出时间线 CSV", command=self._export_csv).pack(side=tk.LEFT, padx=8)
        ttk.Button(opts, text="写回主列表", command=self._commit).pack(side=tk.LEFT, padx=2)
        ttk.Button(opts, text="恢复原片", command=self._restore).pack(side=tk.LEFT, padx=2)

        # 主曲线
        chart_fr = ttk.LabelFrame(self, text="监视器 · 点击=Cut切开 / 选择模式下点击=移动刀口", padding=4)
        chart_fr.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self.chart = RpmChart(chart_fr)
        self.chart.pack(fill=tk.BOTH, expand=True)
        self.chart.canvas.bind("<Button-1>", self._on_chart_click)
        self.chart.canvas.configure(cursor="crosshair")

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
            text="数字转速已是一次差分，求 a 默认「滑动均值+中心差分」；窗21≈10ms@2kHz，毛刺大可试 savgol / 窗41",
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
            "板内 RPM 已是 ABI 计数的一次差分（数字台阶）。\n"
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

    def _set_tool(self, name: str) -> None:
        self._tool.set(name)
        self._tool_hint()

    def _tool_hint(self) -> None:
        if self._tool.get() == "cut":
            self.chart.canvas.configure(cursor="crosshair")
            self.var_status.set("Cut 模式：在曲线上点击切开")
        else:
            self.chart.canvas.configure(cursor="arrow")
            self.var_status.set("选择模式：点曲线设刀口，点时间线选段")

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

    def _on_chart_click(self, evt) -> None:
        t = self.chart.x_from_pixel(evt.x)
        if t is None:
            return
        self.var_playhead = t
        self.var_cut_t.set(f"{t:.3f}")
        if self._tool.get() == "cut":
            self._cut_at(t)
        else:
            # 选择：只移动刀口并选中所在段
            _, _, spans = concat_timeline(self._clips)
            for i, t0, t1 in spans:
                if t0 <= t <= t1 + 1e-9:
                    self._active = i
                    break
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
        left, right = split_at_time(rows, local_t)
        # 避免切在端点产生空段
        if len(left) < 2 or len(right) < 2:
            messagebox.showinfo("Cut", "刀口太靠边，无法切开（两端至少各留一点）")
            return
        name = self._clips[i].name
        c1 = Clip(rows=left, name=f"{name}a", uid=CurveStudio._uid)
        CurveStudio._uid += 1
        c2 = Clip(rows=right, name=f"{name}b", uid=CurveStudio._uid)
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
        self._clips = [Clip(rows=rows, name="Merged", uid=CurveStudio._uid)]
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
        self.refresh()

    def _restore(self) -> None:
        self._clips = [Clip(rows=list(self._raw), name="Clip1", uid=CurveStudio._uid)]
        CurveStudio._uid += 1
        self._active = 0
        self.var_playhead = 0.0
        self.refresh()

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

        base = t_base_ms(rows)
        xs = [(float(r[0]) - base) / 1000.0 for r in rows]
        ys_rpm = [abs(float(r[1])) for r in rows]
        if self.var_s_mode.get() == "rpm":
            ser = integrate_rpm_distance(rows, length_per_rev=length)
            ys_s = [p[2] for p in ser]
            s_lab = f"S←∫rpm×{length}"
        else:
            ser = distance_series(rows, i0=i0, length_per_i=length)
            ys_s = [p[2] for p in ser]
            s_lab = f"S←(I-{i0})×{length}"

        # 加速度：对当前选中段微分（更贴近「选一段算 a」）
        if 0 <= self._active < len(self._clips) and self._clips[self._active].rows:
            a_rows = self._clips[self._active].rows
            # 映射到全局时间：用 spans
            t0g = spans[self._active][1] if self._active < len(spans) else 0.0
            abase = t_base_ms(a_rows)
            # 构造带全局 t 的临时行
            a_disp = []
            for r in a_rows:
                rel = (float(r[0]) - abase) / 1000.0
                a_disp.append(((t0g + rel) * 1000.0, r[1], r[2], r[3], r[4]))
            tx, ay = accel_from_rows(
                a_disp, smooth=smooth, method=method, post_smooth=post, use_signed=True
            )
        else:
            tx, ay = [], []

        view = self.var_view.get()
        peak = max(ys_rpm) if ys_rpm else 0.0
        s_end = ys_s[-1] if ys_s else 0.0
        apeak = max((abs(v) for v in ay), default=0.0)
        ph = self.var_playhead
        mark_in = ph
        mark_out = ph
        ms_win = smooth * 0.5  # 2kHz → 每点 0.5ms

        if view == "rpm":
            self.chart.set_series(
                [(xs, ys_rpm, "#1a6fb5", f"|RPM| peak={peak:.0f}")],
                xlim=(0.0, max(total, 0.01)),
                ylim=(0.0, max(peak * 1.15, 50.0)),
                title=f"时间线 n={len(rows)} 段数={len(self._clips)}  黄线=刀口",
                xlabel="t (s)",
                ylabel="|RPM|",
                mark_in=mark_in,
                mark_out=mark_out,
            )
        elif view == "s":
            smin = min(ys_s) if ys_s else 0.0
            smax = max(abs(y) for y in ys_s) if ys_s else 1.0
            self.chart.set_series(
                [(xs, ys_s, "#0b7a4b", f"{s_lab} S末={s_end:.3f}")],
                xlim=(0.0, max(total, 0.01)),
                ylim=(min(0.0, smin), max(smax * 1.1, 1e-3)),
                title="距离 S",
                xlabel="t (s)",
                ylabel="S",
                mark_in=mark_in,
                mark_out=mark_out,
                allow_negative=smin < 0,
            )
        elif view == "a":
            if tx and ay:
                self.chart.set_series(
                    [(tx, ay, "#c0392b", f"a |peak|={apeak:.1f} RPM/s")],
                    xlim=(min(tx), max(tx) + 1e-3),
                    ylim=(-apeak * 1.2 - 1, apeak * 1.2 + 1),
                    title=(
                        f"加速度 [{method}] 窗{smooth}(~{ms_win:.0f}ms) 后再滑{post} · "
                        f"{self._clips[self._active].name}"
                    ),
                    xlabel="t (s)",
                    ylabel="a (RPM/s)",
                    allow_negative=True,
                )
            else:
                self.chart.set_series(
                    [([0, total], [0, 0], "#c0392b", "a")],
                    title="加速度（选中段太短，加大段或减小窗）",
                    xlabel="t (s)",
                    ylabel="a",
                    allow_negative=True,
                )
        else:
            nr, r0, r1 = _norm01(ys_rpm)
            ns, s0, s1 = _norm01(ys_s)
            series = [
                (xs, nr, "#1a6fb5", f"RPM [{r0:.0f}~{r1:.0f}]"),
                (xs, ns, "#0b7a4b", f"S [{s0:.3g}~{s1:.3g}]"),
            ]
            if tx and ay:
                na, a0, a1 = _norm01(ay, signed=True)
                series.append((tx, na, "#c0392b", f"a [{a0:.1f}~{a1:.1f}]"))
            self.chart.set_series(
                series,
                xlim=(0.0, max(total, 0.01)),
                ylim=(-105, 105),
                title="三线同图（归一化）",
                xlabel="t (s)",
                ylabel="%",
                mark_in=mark_in,
                mark_out=mark_out,
                allow_negative=True,
            )

        act = self._clips[self._active].name if self._clips else "-"
        self.var_cut_t.set(f"{ph:.3f}")
        self.var_status.set(
            f"工具={self._tool.get()}  选中={act}  段数={len(self._clips)}  "
            f"总长={total:.3f}s  S末={s_end:.3f}  |a|peak={apeak:.1f}"
        )

    def _commit(self) -> None:
        rows = self._timeline_rows()
        if self._on_commit:
            self._on_commit(rows)
            messagebox.showinfo("写回", f"已写回主列表 {len(rows)} 点（{len(self._clips)} 段拼接）")
        else:
            messagebox.showinfo("写回", "无主列表回调")

    def _export_csv(self) -> None:
        rows = self._timeline_rows()
        if not rows:
            messagebox.showinfo("导出", "时间线为空")
            return
        i0, length = self._read_i0_len()
        series = distance_series(rows, i0=i0, length_per_i=length)
        try:
            smooth = int(float(self.var_smooth.get().strip() or "21"))
        except ValueError:
            smooth = 21
        try:
            post = int(float(self.var_post_smooth.get().strip() or "5"))
        except ValueError:
            post = 5
        method = (self.var_diff_method.get() or "ma_central").strip()
        tx, ay = accel_from_rows(
            rows, smooth=max(3, smooth), method=method, post_smooth=post
        )
        a_map = list(zip(tx, ay))

        def a_at(t: float) -> float:
            if not a_map:
                return 0.0
            return min(a_map, key=lambda p: abs(p[0] - t))[1]

        path = filedialog.asksaveasfilename(
            title="导出时间线 CSV",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile=f"timeline_{len(rows)}.csv",
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(
                ["t_rel_s", "rpm", "index_n", "i_rel", "S_I", "a_rpm_per_s", "I0", "length_per_I"]
            )
            for i, r in enumerate(rows):
                tr = series[i][0]
                w.writerow(
                    [
                        f"{tr:.6f}",
                        f"{float(r[1]):.2f}",
                        int(r[4]),
                        series[i][4],
                        f"{series[i][2]:.6f}",
                        f"{a_at(tr):.4f}",
                        i0,
                        length,
                    ]
                )
        self.var_status.set(f"已导出 {path}")


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

    path = filedialog.askopenfilename(filetypes=[("bin", "*.bin"), ("all", "*.*")])
    if not path:
        sys.exit(0)
    _n, _hz, rows = parse_snap_file(Path(path).read_bytes())
    open_curve_studio(root, rows, title=Path(path).name)
    root.mainloop()
