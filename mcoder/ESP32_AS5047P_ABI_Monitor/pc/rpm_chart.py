#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lightweight multi-series chart on Tk Canvas (no matplotlib)."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Sequence


class RpmChart(ttk.Frame):
    """Y vs t(s). One or more series; In/Out marks; click→time; wheel zoom."""

    def __init__(self, master, **kw) -> None:
        super().__init__(master, **kw)
        self._pad_l = 56
        self._pad_r = 12
        self._pad_t = 36
        self._pad_b = 36
        self._title = ""
        self._xlabel = "t (s)"
        self._ylabel = "Y"
        self._series: list[tuple[list[float], list[float], str, str]] = []
        # each: xs, ys, color, label
        self._full_xlim: tuple[float, float] | None = None
        self._full_ylim: tuple[float, float] | None = None
        self._view_xlim: tuple[float, float] | None = None
        self._view_ylim: tuple[float, float] | None = None
        self._mark_in: float | None = None
        self._mark_out: float | None = None
        self._sel_rect: tuple[float, float, float, float] | None = None  # t0,t1,y0,y1
        self._line = "#1a6fb5"
        self._grid = "#d0d7de"
        self._bg = "#f7f9fc"
        self._fg = "#1f2328"
        self._y_fmt = "auto"
        self._allow_negative = False
        self._drawing = False
        self._last_wh = (0, 0)
        self._resize_job: str | None = None
        self._xf: tuple[float, float, float, float] | None = None
        self._yf: tuple[float, float, float, float] | None = None  # y0,y1,pt,ph
        self._on_view_change = None  # optional callback()

        self.canvas = tk.Canvas(self, background=self._bg, highlightthickness=0, height=360)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self._on_resize)
        # 滚轮由上层 CurveStudio bind_all 转发（Windows 需焦点时更稳）
        self.canvas.bind("<Button-4>", lambda e: self._zoom_wheel(e, +1))  # Linux up
        self.canvas.bind("<Button-5>", lambda e: self._zoom_wheel(e, -1))  # Linux down
        self.canvas.bind("<Double-Button-1>", lambda _e: self.reset_view())

    def set_line_color(self, color: str) -> None:
        self._line = color or self._line

    def set_title(self, title: str) -> None:
        self._title = title or ""

    def set_labels(self, xlabel: str = "t (s)", ylabel: str = "Y") -> None:
        self._xlabel = xlabel
        self._ylabel = ylabel

    def set_data(
        self,
        xs: Sequence[float],
        ys: Sequence[float],
        *,
        xlim: tuple[float, float] | None = None,
        ylim: tuple[float, float] | None = None,
        title: str | None = None,
        xlabel: str | None = None,
        ylabel: str | None = None,
        mark_in: float | None = None,
        mark_out: float | None = None,
        y_fmt: str | None = None,
        allow_negative: bool | None = None,
        color: str | None = None,
        label: str | None = None,
        preserve_view: bool = True,
    ) -> None:
        """单曲线（兼容旧调用）。"""
        col = color or self._line
        lab = label or ""
        self.set_series(
            [(list(xs), list(ys), col, lab)],
            xlim=xlim,
            ylim=ylim,
            title=title,
            xlabel=xlabel,
            ylabel=ylabel,
            mark_in=mark_in,
            mark_out=mark_out,
            y_fmt=y_fmt,
            allow_negative=allow_negative,
            preserve_view=preserve_view,
        )

    def set_series(
        self,
        series: Sequence[tuple[Sequence[float], Sequence[float], str, str]],
        *,
        xlim: tuple[float, float] | None = None,
        ylim: tuple[float, float] | None = None,
        title: str | None = None,
        xlabel: str | None = None,
        ylabel: str | None = None,
        mark_in: float | None = None,
        mark_out: float | None = None,
        y_fmt: str | None = None,
        allow_negative: bool | None = None,
        preserve_view: bool = True,
    ) -> None:
        self._series = [(list(xs), list(ys), col, lab) for xs, ys, col, lab in series]
        if title is not None:
            self._title = title
        if xlabel is not None:
            self._xlabel = xlabel
        if ylabel is not None:
            self._ylabel = ylabel
        self._mark_in = mark_in
        self._mark_out = mark_out
        if y_fmt is not None:
            self._y_fmt = y_fmt
        if allow_negative is not None:
            self._allow_negative = allow_negative

        full_x, full_y = self._compute_full_bounds(xlim, ylim)
        self._full_xlim = full_x
        self._full_ylim = full_y

        if (not preserve_view) or self._view_xlim is None or self._view_ylim is None:
            self._view_xlim = full_x
            self._view_ylim = full_y
        else:
            self._view_xlim = self._clamp_view_x(self._view_xlim)
            self._view_ylim = self._clamp_view_y(self._view_ylim)
        self.redraw()

    def reset_view(self) -> None:
        """缩放到整体（全量范围）。"""
        if self._full_xlim and self._full_ylim:
            self._view_xlim = self._full_xlim
            self._view_ylim = self._full_ylim
            self.redraw()
            if self._on_view_change:
                self._on_view_change()

    def is_zoomed(self) -> bool:
        if not self._full_xlim or not self._view_xlim:
            return False
        fx0, fx1 = self._full_xlim
        vx0, vx1 = self._view_xlim
        return (vx1 - vx0) < (fx1 - fx0) * 0.98

    def set_selection_rect(
        self, rect: tuple[float, float, float, float] | None
    ) -> None:
        """数据坐标选区 (t0, t1, y0, y1)；None 清除。"""
        self._sel_rect = rect
        self.redraw()

    def clear(self) -> None:
        self._series.clear()
        self._sel_rect = None
        self._full_xlim = None
        self._full_ylim = None
        self._view_xlim = None
        self._view_ylim = None
        self.redraw()

    def x_from_pixel(self, px: float) -> float | None:
        if not self._xf:
            return None
        x0, x1, pl, pw = self._xf
        if pw <= 0:
            return None
        t = x0 + (px - pl) / pw * (x1 - x0)
        return max(x0, min(x1, t))

    def y_from_pixel(self, py: float) -> float | None:
        if not self._yf:
            return None
        y0, y1, pt, ph = self._yf
        if ph <= 0:
            return None
        y = y0 + (pt + ph - py) / ph * (y1 - y0)
        return max(min(y0, y1), min(max(y0, y1), y))

    def data_from_pixel(self, px: float, py: float) -> tuple[float, float] | None:
        tx = self.x_from_pixel(px)
        ty = self.y_from_pixel(py)
        if tx is None or ty is None:
            return None
        return tx, ty

    def _compute_full_bounds(
        self,
        xlim: tuple[float, float] | None,
        ylim: tuple[float, float] | None,
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        all_x: list[float] = []
        all_y: list[float] = []
        for xs, ys, _, _ in self._series:
            n = min(len(xs), len(ys))
            all_x.extend(xs[:n])
            all_y.extend(ys[:n])
        if xlim:
            fx0, fx1 = float(xlim[0]), float(xlim[1])
        elif all_x:
            fx0, fx1 = min(all_x), max(all_x)
        else:
            fx0, fx1 = 0.0, 1.0
        if fx1 <= fx0:
            fx1 = fx0 + 0.01

        if ylim:
            fy0, fy1 = float(ylim[0]), float(ylim[1])
        elif all_y:
            if self._allow_negative:
                fy0, fy1 = min(all_y), max(all_y)
                pad = max(abs(fy1 - fy0) * 0.1, 1e-6)
                fy0 -= pad
                fy1 += pad
            else:
                fy0 = min(0.0, min(all_y))
                fy1 = max(all_y) * 1.12
                if fy1 <= fy0:
                    fy1 = fy0 + 1.0
        else:
            fy0, fy1 = 0.0, 1.0
        if fy1 <= fy0:
            fy1 = fy0 + 1.0
        return (fx0, fx1), (fy0, fy1)

    def _clamp_view_x(self, lim: tuple[float, float]) -> tuple[float, float]:
        assert self._full_xlim
        fx0, fx1 = self._full_xlim
        a, b = min(lim), max(lim)
        span = b - a
        full = fx1 - fx0
        if span >= full * 0.999:
            return fx0, fx1
        if span < full * 1e-4:
            mid = 0.5 * (a + b)
            span = full * 1e-4
            a, b = mid - span / 2, mid + span / 2
        if a < fx0:
            b += fx0 - a
            a = fx0
        if b > fx1:
            a -= b - fx1
            b = fx1
        a = max(fx0, a)
        b = min(fx1, b)
        if b <= a:
            return fx0, fx1
        return a, b

    def _clamp_view_y(self, lim: tuple[float, float]) -> tuple[float, float]:
        assert self._full_ylim
        fy0, fy1 = self._full_ylim
        a, b = min(lim), max(lim)
        span = b - a
        full = fy1 - fy0
        if span >= full * 0.999:
            return fy0, fy1
        if span < full * 1e-4:
            mid = 0.5 * (a + b)
            span = full * 1e-4
            a, b = mid - span / 2, mid + span / 2
        if a < fy0:
            b += fy0 - a
            a = fy0
        if b > fy1:
            a -= b - fy1
            b = fy1
        a = max(fy0, a)
        b = min(fy1, b)
        if b <= a:
            return fy0, fy1
        return a, b

    def _on_mousewheel(self, evt) -> None:
        # Windows: delta>0 向上/向前 → 放大；delta<0 → 缩小
        direction = 1 if int(evt.delta) > 0 else -1
        self._zoom_wheel(evt, direction)

    def _zoom_wheel(self, evt, direction: int) -> None:
        """direction>0 放大局部；direction<0 缩小看整体。以光标为中心。"""
        if not self._view_xlim or not self._view_ylim or not self._full_xlim:
            return
        # 放大 factor<1；缩小 factor>1
        factor = 0.8 if direction > 0 else 1.25
        cx = self.x_from_pixel(evt.x)
        cy = self.y_from_pixel(evt.y)
        if cx is None or cy is None:
            # 光标在边距外：用视窗中心
            vx0, vx1 = self._view_xlim
            vy0, vy1 = self._view_ylim
            cx = 0.5 * (vx0 + vx1)
            cy = 0.5 * (vy0 + vy1)

        vx0, vx1 = self._view_xlim
        vy0, vy1 = self._view_ylim
        # 缩小到接近全图时直接复位
        if direction < 0:
            fx0, fx1 = self._full_xlim
            if (vx1 - vx0) * factor >= (fx1 - fx0) * 0.98:
                self.reset_view()
                return

        def zoom_1d(a: float, b: float, c: float, f: float) -> tuple[float, float]:
            # 保持 c 在视窗中的相对位置
            span = (b - a) * f
            if b <= a:
                return a, b
            rel = (c - a) / (b - a)
            rel = max(0.0, min(1.0, rel))
            na = c - rel * span
            nb = na + span
            return na, nb

        nx0, nx1 = zoom_1d(vx0, vx1, cx, factor)
        ny0, ny1 = zoom_1d(vy0, vy1, cy, factor)
        self._view_xlim = self._clamp_view_x((nx0, nx1))
        self._view_ylim = self._clamp_view_y((ny0, ny1))
        self.redraw()
        if self._on_view_change:
            self._on_view_change()

    def _on_resize(self, evt=None) -> None:
        if self._drawing:
            return
        if evt is not None and hasattr(evt, "width") and hasattr(evt, "height"):
            wh = (int(evt.width), int(evt.height))
            if wh == self._last_wh:
                return
            self._last_wh = wh
        if self._resize_job is not None:
            try:
                self.after_cancel(self._resize_job)
            except Exception:
                pass
        self._resize_job = self.after(50, self._resize_redraw)

    def _resize_redraw(self) -> None:
        self._resize_job = None
        self.redraw()

    def redraw(self) -> None:
        if self._drawing:
            return
        self._drawing = True
        try:
            self._redraw_impl()
        finally:
            self._drawing = False

    def _fmt_y(self, v: float) -> str:
        av = abs(v)
        if av >= 100:
            return f"{v:.0f}"
        if av >= 10:
            return f"{v:.1f}"
        if av >= 1:
            return f"{v:.2f}"
        return f"{v:.3g}"

    def _redraw_impl(self) -> None:
        c = self.canvas
        w = max(int(c.winfo_width()), 80)
        h = max(int(c.winfo_height()), 80)
        c.delete("all")
        c.create_rectangle(0, 0, w, h, fill=self._bg, outline="")

        pl, pr, pt, pb = self._pad_l, self._pad_r, self._pad_t, self._pad_b
        pw = max(w - pl - pr, 10)
        ph = max(h - pt - pb, 10)

        title = self._title
        if self.is_zoomed() and self._view_xlim:
            title = f"{title}  ·  视窗 {self._view_xlim[0]:.3f}~{self._view_xlim[1]:.3f}s（滚轮缩放/双击复位）"
        if title:
            c.create_text(pl, 8, anchor="w", text=title, fill=self._fg, font=("Segoe UI", 9))

        if not self._series or all(len(s[0]) < 1 for s in self._series):
            c.create_text(
                pl + pw / 2, pt + ph / 2, text="waiting...", fill="#888", font=("Segoe UI", 11)
            )
            self._xf = None
            self._yf = None
            return

        # legend
        lx = pl
        for _, _, col, lab in self._series:
            if not lab:
                continue
            c.create_line(lx, 22, lx + 18, 22, fill=col, width=2)
            c.create_text(lx + 22, 22, anchor="w", text=lab, fill=col, font=("Segoe UI", 8))
            lx += 18 + 7 * max(len(lab), 4) + 16

        if not self._view_xlim or not self._view_ylim:
            self._xf = None
            self._yf = None
            return

        x0, x1 = self._view_xlim
        y0, y1 = self._view_ylim
        if x1 <= x0:
            x1 = x0 + 0.01
        if y1 <= y0:
            y1 = y0 + 1.0

        self._xf = (x0, x1, float(pl), float(pw))
        self._yf = (y0, y1, float(pt), float(ph))

        def sx(x: float) -> float:
            return pl + (x - x0) / (x1 - x0) * pw

        def sy(y: float) -> float:
            return pt + ph - (y - y0) / (y1 - y0) * ph

        if self._mark_in is not None and self._mark_out is not None:
            xa = sx(min(self._mark_in, self._mark_out))
            xb = sx(max(self._mark_in, self._mark_out))
            c.create_rectangle(xa, pt, xb, pt + ph, fill="#fff3cd", outline="")

        if self._sel_rect is not None:
            t0, t1, ya, yb = self._sel_rect
            xa, xb = sx(min(t0, t1)), sx(max(t0, t1))
            ya_p, yb_p = sy(min(ya, yb)), sy(max(ya, yb))
            c.create_rectangle(
                xa,
                min(ya_p, yb_p),
                xb,
                max(ya_p, yb_p),
                fill="#ffcccc",
                outline="#c0392b",
                width=2,
            )

        for i in range(5):
            yy = y0 + (y1 - y0) * i / 4.0
            ypix = sy(yy)
            c.create_line(pl, ypix, pl + pw, ypix, fill=self._grid)
            c.create_text(
                pl - 6, ypix, anchor="e", text=self._fmt_y(yy), fill="#555", font=("Segoe UI", 8)
            )
        for i in range(5):
            xx = x0 + (x1 - x0) * i / 4.0
            xpix = sx(xx)
            c.create_line(xpix, pt, xpix, pt + ph, fill=self._grid)
            c.create_text(
                xpix, pt + ph + 6, anchor="n", text=f"{xx:.3f}", fill="#555", font=("Segoe UI", 8)
            )

        c.create_rectangle(pl, pt, pl + pw, pt + ph, outline="#9aa4af")
        c.create_text(pl + pw / 2, h - 8, text=self._xlabel, fill="#444", font=("Segoe UI", 8))
        c.create_text(
            14, pt + ph / 2, text=self._ylabel, fill="#444", font=("Segoe UI", 8), angle=90
        )

        if self._allow_negative and y0 < 0 < y1:
            c.create_line(pl, sy(0), pl + pw, sy(0), fill="#aaa", dash=(3, 2))

        # 视窗外留一点余量，避免线段截断
        x_pad = (x1 - x0) * 0.02
        for xs, ys, col, _lab in self._series:
            n = min(len(xs), len(ys))
            if n < 2:
                continue
            # 找可见区间索引
            i0 = 0
            i1 = n - 1
            for i in range(n):
                if xs[i] >= x0 - x_pad:
                    i0 = max(0, i - 1)
                    break
            for i in range(n - 1, -1, -1):
                if xs[i] <= x1 + x_pad:
                    i1 = min(n - 1, i + 1)
                    break
            if i1 <= i0:
                continue
            vis_n = i1 - i0 + 1
            step = max(1, vis_n // 2000)
            pts: list[float] = []
            for i in range(i0, i1 + 1, step):
                pts.extend((sx(xs[i]), sy(ys[i])))
            if (i1 - i0) % step != 0:
                pts.extend((sx(xs[i1]), sy(ys[i1])))
            if len(pts) >= 4:
                c.create_line(*pts, fill=col, width=2, smooth=False)

        if self._mark_in is not None:
            xi = sx(self._mark_in)
            c.create_line(xi, pt, xi, pt + ph, fill="#e67e22", width=2)
            c.create_text(
                xi + 3, pt + 4, anchor="nw", text="In", fill="#e67e22", font=("Segoe UI", 8)
            )
        if self._mark_out is not None:
            xo = sx(self._mark_out)
            c.create_line(xo, pt, xo, pt + ph, fill="#8e44ad", width=2)
            c.create_text(
                xo + 3, pt + 4, anchor="nw", text="Out", fill="#8e44ad", font=("Segoe UI", 8)
            )
