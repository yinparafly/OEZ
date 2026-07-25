#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lightweight multi-series chart on Tk Canvas (no matplotlib)."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Sequence


class RpmChart(ttk.Frame):
    """Y vs t(s). One or more series; In/Out marks; click→time."""

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
        self._xlim: tuple[float, float] | None = None
        self._ylim: tuple[float, float] | None = None
        self._mark_in: float | None = None
        self._mark_out: float | None = None
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

        self.canvas = tk.Canvas(self, background=self._bg, highlightthickness=0, height=360)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self._on_resize)

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
    ) -> None:
        self._series = [(list(xs), list(ys), col, lab) for xs, ys, col, lab in series]
        self._xlim = xlim
        self._ylim = ylim
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
        self.redraw()

    def clear(self) -> None:
        self._series.clear()
        self.redraw()

    def x_from_pixel(self, px: float) -> float | None:
        if not self._xf:
            return None
        x0, x1, pl, pw = self._xf
        if pw <= 0:
            return None
        t = x0 + (px - pl) / pw * (x1 - x0)
        return max(x0, min(x1, t))

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

        if self._title:
            c.create_text(pl, 8, anchor="w", text=self._title, fill=self._fg, font=("Segoe UI", 9))

        if not self._series or all(len(s[0]) < 1 for s in self._series):
            c.create_text(pl + pw / 2, pt + ph / 2, text="waiting...", fill="#888", font=("Segoe UI", 11))
            self._xf = None
            return

        # legend
        lx = pl
        for _, _, col, lab in self._series:
            if not lab:
                continue
            c.create_line(lx, 22, lx + 18, 22, fill=col, width=2)
            c.create_text(lx + 22, 22, anchor="w", text=lab, fill=col, font=("Segoe UI", 8))
            lx += 18 + 7 * max(len(lab), 4) + 16

        all_x: list[float] = []
        all_y: list[float] = []
        for xs, ys, _, _ in self._series:
            n = min(len(xs), len(ys))
            all_x.extend(xs[:n])
            all_y.extend(ys[:n])
        if not all_x:
            self._xf = None
            return

        if self._xlim:
            x0, x1 = self._xlim
        else:
            x0, x1 = min(all_x), max(all_x)
            if x1 <= x0:
                x1 = x0 + 0.01

        if self._ylim:
            y0, y1 = self._ylim
        else:
            if self._allow_negative:
                y0 = min(all_y)
                y1 = max(all_y)
                pad = max(abs(y1 - y0) * 0.1, 1e-6)
                y0 -= pad
                y1 += pad
            else:
                y0 = min(0.0, min(all_y))
                y1 = max(all_y) * 1.12
                if y1 <= y0:
                    y1 = y0 + 1.0
        if y1 <= y0:
            y1 = y0 + 1.0

        self._xf = (x0, x1, float(pl), float(pw))

        def sx(x: float) -> float:
            return pl + (x - x0) / (x1 - x0) * pw

        def sy(y: float) -> float:
            return pt + ph - (y - y0) / (y1 - y0) * ph

        if self._mark_in is not None and self._mark_out is not None:
            xa = sx(min(self._mark_in, self._mark_out))
            xb = sx(max(self._mark_in, self._mark_out))
            c.create_rectangle(xa, pt, xb, pt + ph, fill="#fff3cd", outline="")

        for i in range(5):
            yy = y0 + (y1 - y0) * i / 4.0
            ypix = sy(yy)
            c.create_line(pl, ypix, pl + pw, ypix, fill=self._grid)
            c.create_text(pl - 6, ypix, anchor="e", text=self._fmt_y(yy), fill="#555", font=("Segoe UI", 8))
        for i in range(5):
            xx = x0 + (x1 - x0) * i / 4.0
            xpix = sx(xx)
            c.create_line(xpix, pt, xpix, pt + ph, fill=self._grid)
            c.create_text(xpix, pt + ph + 6, anchor="n", text=f"{xx:.2f}", fill="#555", font=("Segoe UI", 8))

        c.create_rectangle(pl, pt, pl + pw, pt + ph, outline="#9aa4af")
        c.create_text(pl + pw / 2, h - 8, text=self._xlabel, fill="#444", font=("Segoe UI", 8))
        c.create_text(14, pt + ph / 2, text=self._ylabel, fill="#444", font=("Segoe UI", 8), angle=90)

        if self._allow_negative and y0 < 0 < y1:
            c.create_line(pl, sy(0), pl + pw, sy(0), fill="#aaa", dash=(3, 2))

        for xs, ys, col, _lab in self._series:
            n = min(len(xs), len(ys))
            if n < 2:
                continue
            step = max(1, n // 1600)
            pts: list[float] = []
            for i in range(0, n, step):
                pts.extend((sx(xs[i]), sy(ys[i])))
            if (n - 1) % step != 0:
                pts.extend((sx(xs[-1]), sy(ys[-1])))
            if len(pts) >= 4:
                c.create_line(*pts, fill=col, width=2, smooth=False)

        if self._mark_in is not None:
            xi = sx(self._mark_in)
            c.create_line(xi, pt, xi, pt + ph, fill="#e67e22", width=2)
            c.create_text(xi + 3, pt + 4, anchor="nw", text="In", fill="#e67e22", font=("Segoe UI", 8))
        if self._mark_out is not None:
            xo = sx(self._mark_out)
            c.create_line(xo, pt, xo, pt + ph, fill="#8e44ad", width=2)
            c.create_text(xo + 3, pt + 4, anchor="nw", text="Out", fill="#8e44ad", font=("Segoe UI", 8))
