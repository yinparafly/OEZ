#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""snap 剪辑：时间裁剪/删除、转速过滤、I 归零、S=ΔI×单圈长度。"""

from __future__ import annotations

from typing import Sequence


Row = tuple  # (t_ms, rpm, dir, seg, index_n, ...)


def t_base_ms(rows: Sequence[Row]) -> float:
    return float(rows[0][0]) if rows else 0.0


def rel_s(row: Row, base_ms: float) -> float:
    return (float(row[0]) - base_ms) / 1000.0


def keep_time_range(rows: Sequence[Row], t0_s: float, t1_s: float) -> list[Row]:
    """只保留 [t0,t1]（相对段起点），类似视频裁剪保留中间。"""
    if not rows:
        return []
    if t1_s < t0_s:
        t0_s, t1_s = t1_s, t0_s
    base = t_base_ms(rows)
    return [r for r in rows if t0_s <= rel_s(r, base) <= t1_s]


def delete_time_range(rows: Sequence[Row], t0_s: float, t1_s: float) -> list[Row]:
    """删除 [t0,t1] 内的点（切开一段）。"""
    if not rows:
        return []
    if t1_s < t0_s:
        t0_s, t1_s = t1_s, t0_s
    base = t_base_ms(rows)
    return [r for r in rows if not (t0_s <= rel_s(r, base) <= t1_s)]


def split_at_time(rows: Sequence[Row], t_cut_s: float) -> tuple[list[Row], list[Row]]:
    """在 t_cut 拆成两段。"""
    if not rows:
        return [], []
    base = t_base_ms(rows)
    left = [r for r in rows if rel_s(r, base) <= t_cut_s]
    right = [r for r in rows if rel_s(r, base) > t_cut_s]
    return left, right


def filter_by_rpm(
    rows: Sequence[Row], rpm_min: float = 0.0, drop_zero: bool = False
) -> list[Row]:
    out: list[Row] = []
    for r in rows:
        rpm = abs(float(r[1]))
        if drop_zero and rpm < 1e-6:
            continue
        if rpm < rpm_min:
            continue
        out.append(r)
    return out


def index_baseline(rows: Sequence[Row]) -> int:
    """默认 I0 = 首点 index_n。"""
    if not rows:
        return 0
    return int(rows[0][4])


def distance_series(
    rows: Sequence[Row],
    *,
    i0: int | None = None,
    length_per_i: float = 1.0,
) -> list[tuple[float, float, float, int, int, float]]:
    """
    由 I 累计生成距离。
    返回: (t_rel_s, rpm, S, index_n, i_rel, t_ms)
    S = (index_n - I0) * length_per_i
    """
    if not rows:
        return []
    if i0 is None:
        i0 = index_baseline(rows)
    base = t_base_ms(rows)
    out: list[tuple[float, float, float, int, int, float]] = []
    for r in rows:
        t_ms = float(r[0])
        rpm = float(r[1])
        idx = int(r[4])
        i_rel = idx - i0
        s = float(i_rel) * float(length_per_i)
        out.append((rel_s(r, base), rpm, s, idx, i_rel, t_ms))
    return out


def integrate_rpm_distance(
    rows: Sequence[Row], *, length_per_rev: float = 1.0
) -> list[tuple[float, float, float]]:
    """
    备用：用 |RPM| 积分得距离（与 I 无关）。
    dS = |rpm|/60 * length_per_rev * dt
    返回 (t_rel_s, rpm, S)
    """
    if not rows:
        return []
    base = t_base_ms(rows)
    s = 0.0
    out: list[tuple[float, float, float]] = []
    prev_t = float(rows[0][0])
    for r in rows:
        t_ms = float(r[0])
        rpm = abs(float(r[1]))
        dt = max(0.0, (t_ms - prev_t) / 1000.0)
        s += (rpm / 60.0) * length_per_rev * dt
        out.append((rel_s(r, base), float(r[1]), s))
        prev_t = t_ms
    return out


def _odd_window(w: int, minimum: int = 3) -> int:
    w = int(w)
    if w < minimum:
        w = minimum
    if w % 2 == 0:
        w += 1
    return w


def moving_average(ys: Sequence[float], window: int) -> list[float]:
    """边沿缩短窗口的滑动均值（适合等间隔数字采样）。"""
    n = len(ys)
    if n == 0:
        return []
    k = _odd_window(window, 1)
    if k <= 1:
        return list(ys)
    half = k // 2
    out: list[float] = []
    for i in range(n):
        a = max(0, i - half)
        b = min(n, i + half + 1)
        out.append(sum(ys[a:b]) / (b - a))
    return out


def _central_diff(
    xs: Sequence[float], ys: Sequence[float], *, half: int = 1
) -> tuple[list[float], list[float]]:
    """中心差分 (y[i+h]-y[i-h])/(x[i+h]-x[i-h])，比前向差分抗台阶噪声。"""
    n = min(len(xs), len(ys))
    h = max(1, int(half))
    if n < 2 * h + 1:
        # 退化前向
        tx: list[float] = []
        dy: list[float] = []
        for i in range(1, n):
            dt = xs[i] - xs[i - 1]
            if dt <= 1e-12:
                continue
            tx.append(0.5 * (xs[i] + xs[i - 1]))
            dy.append((ys[i] - ys[i - 1]) / dt)
        return tx, dy
    tx = []
    dy = []
    for i in range(h, n - h):
        dt = xs[i + h] - xs[i - h]
        if dt <= 1e-12:
            continue
        tx.append(xs[i])
        dy.append((ys[i + h] - ys[i - h]) / dt)
    return tx, dy


def _savgol_deriv(
    xs: Sequence[float], ys: Sequence[float], *, window: int = 11
) -> tuple[list[float], list[float]]:
    """
    局部二次多项式最小二乘 → 解析一阶导（类 Savitzky–Golay）。
    对等间隔数字转速台阶噪声比裸差分稳。
    """
    n = min(len(xs), len(ys))
    k = _odd_window(window, 5)
    half = k // 2
    if n < k:
        return _central_diff(xs, ys, half=1)
    tx: list[float] = []
    dy: list[float] = []
    for i in range(half, n - half):
        # 以 xs[i] 为原点，拟合 y ≈ a0 + a1*t + a2*t^2 → dy/dt = a1
        # 正规方程 3x3
        s0 = s1 = s2 = s3 = s4 = 0.0
        sy = sty = st2y = 0.0
        t0 = xs[i]
        for j in range(i - half, i + half + 1):
            t = xs[j] - t0
            t2 = t * t
            y = ys[j]
            s0 += 1.0
            s1 += t
            s2 += t2
            s3 += t2 * t
            s4 += t2 * t2
            sy += y
            sty += t * y
            st2y += t2 * y
        # Solve [[s0,s1,s2],[s1,s2,s3],[s2,s3,s4]] * [a0,a1,a2] = [sy,sty,st2y]
        # Cramer's / elimination for a1
        det = (
            s0 * (s2 * s4 - s3 * s3)
            - s1 * (s1 * s4 - s3 * s2)
            + s2 * (s1 * s3 - s2 * s2)
        )
        if abs(det) < 1e-18:
            continue
        # a1 = det1 / det where replace col1 with rhs
        det1 = (
            s0 * (sty * s4 - st2y * s3)
            - sy * (s1 * s4 - s3 * s2)
            + s2 * (s1 * st2y - sty * s2)
        )
        a1 = det1 / det
        tx.append(xs[i])
        dy.append(a1)
    return tx, dy


def differentiate_series(
    xs: Sequence[float],
    ys: Sequence[float],
    *,
    smooth: int = 21,
    method: str = "ma_central",
    post_smooth: int = 1,
) -> tuple[list[float], list[float]]:
    """
    数字信号友好的数值微分 dy/dt。

    method:
      - ma_central: 先滑动均值，再中心差分（推荐，2kHz 数字 RPM）
      - savgol: 局部二次拟合求导（更贴合离散噪声）
      - central: 仅中心差分
      - forward: 前向差分（旧行为，噪声大）

    smooth: 奇数窗口点数。@2kHz：11≈5.5ms，21≈10ms，41≈20ms。
    post_smooth: 对加速度再做一次轻滑动（抑制毛刺）。
    """
    n = min(len(xs), len(ys))
    if n < 2:
        return [], []
    x = list(xs[:n])
    y = list(ys[:n])
    m = (method or "ma_central").lower()
    w = _odd_window(smooth, 3)

    if m == "forward":
        if w > 1:
            y = moving_average(y, w)
        tx, dy = [], []
        for i in range(1, n):
            dt = x[i] - x[i - 1]
            if dt <= 1e-12:
                continue
            tx.append(0.5 * (x[i] + x[i - 1]))
            dy.append((y[i] - y[i - 1]) / dt)
    elif m == "central":
        half = max(1, w // 4)  # 窗口大时拉宽差分跨度
        tx, dy = _central_diff(x, y, half=half)
    elif m == "savgol":
        tx, dy = _savgol_deriv(x, y, window=w)
    else:  # ma_central
        y = moving_average(y, w)
        half = max(1, min(3, w // 6))
        tx, dy = _central_diff(x, y, half=half)

    if post_smooth and post_smooth > 1 and dy:
        dy = moving_average(dy, _odd_window(post_smooth, 3))
        # 长度对齐：MA 不改长度
    return tx, dy


def rpm_to_omega(rpm: float) -> float:
    """RPM → rad/s。"""
    return float(rpm) * (3.141592653589793 / 30.0)


def accel_from_rows(
    rows: Sequence[Row],
    *,
    t0_s: float | None = None,
    t1_s: float | None = None,
    smooth: int = 21,
    method: str = "ma_central",
    post_smooth: int = 5,
    use_signed: bool = True,
) -> tuple[list[float], list[float]]:
    """
    对数字转速求角加速度 a = d(RPM)/dt，单位 RPM/s。

    说明：板内 RPM 已是计数差分；再微分会放大台阶噪声，
    故默认「滑动均值 + 中心差分」，窗口约 10ms@2kHz。
    """
    if not rows:
        return [], []
    base = t_base_ms(rows)
    xs: list[float] = []
    ys: list[float] = []
    for r in rows:
        t = rel_s(r, base)
        if t0_s is not None and t < t0_s:
            continue
        if t1_s is not None and t > t1_s:
            continue
        xs.append(t)
        rpm = float(r[1])
        ys.append(rpm if use_signed else abs(rpm))
    return differentiate_series(
        xs, ys, smooth=smooth, method=method, post_smooth=post_smooth
    )


# 2kHz 下常用窗口提示（点数 → 约毫秒）
SMOOTH_PRESETS_2KHZ = (
    (11, "~5.5 ms"),
    (21, "~10 ms"),
    (41, "~20 ms"),
    (81, "~40 ms"),
)
