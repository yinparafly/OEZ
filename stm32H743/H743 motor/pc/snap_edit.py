#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""snap 剪辑：时间裁剪/删除、转速过滤、I 归零、S=ΔI×单圈长度。"""

from __future__ import annotations

from pathlib import Path
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


def median_filter(ys: Sequence[float], window: int = 5) -> list[float]:
    """滑动中位数（去尖刺主力）。"""
    n = len(ys)
    if n == 0:
        return []
    k = _odd_window(window, 3)
    half = k // 2
    out: list[float] = []
    for i in range(n):
        a = max(0, i - half)
        b = min(n, i + half + 1)
        win = sorted(ys[a:b])
        out.append(win[len(win) // 2])
    return out


def hampel_despike(
    ys: Sequence[float],
    *,
    window: int = 11,
    n_sigma: float = 3.0,
) -> tuple[list[float], int]:
    """
    Hampel：局部中位数 + MAD；超阈则用中位数替换。
    返回 (去尖刺后序列, 替换点数)。
    """
    n = len(ys)
    if n == 0:
        return [], 0
    k = _odd_window(window, 5)
    half = k // 2
    out = list(ys)
    n_rep = 0
    for i in range(n):
        a = max(0, i - half)
        b = min(n, i + half + 1)
        win = sorted(ys[a:b])
        med = win[len(win) // 2]
        mad = sorted(abs(v - med) for v in ys[a:b])[len(win) // 2]
        # 1.4826 * MAD ≈ σ；MAD=0 时用小阈值避免误伤
        thr = n_sigma * 1.4826 * mad if mad > 1e-9 else n_sigma * 1.0
        if abs(ys[i] - med) > thr:
            out[i] = med
            n_rep += 1
    return out, n_rep


def despike_and_smooth(
    ys: Sequence[float],
    *,
    despike_win: int = 11,
    n_sigma: float = 3.0,
    smooth_win: int = 21,
) -> tuple[list[float], int]:
    """先 Hampel 去尖刺，再滑动均值平滑。返回 (clean, n_spikes)。"""
    cleaned, n_sp = hampel_despike(ys, window=despike_win, n_sigma=n_sigma)
    if smooth_win and smooth_win > 1:
        cleaned = moving_average(cleaned, smooth_win)
    return cleaned, n_sp


def ema_filter(ys: Sequence[float], alpha: float = 0.1) -> list[float]:
    """一阶 EMA；alpha 越大越跟手（0~1）。电机转速常用 α≈0.05~0.2 @2kHz。"""
    n = len(ys)
    if n == 0:
        return []
    a = min(1.0, max(1e-6, float(alpha)))
    out = [float(ys[0])]
    for i in range(1, n):
        out.append(a * float(ys[i]) + (1.0 - a) * out[-1])
    return out


def iir_lowpass(ys: Sequence[float], *, fc_hz: float = 50.0, fs_hz: float = 2000.0) -> list[float]:
    """
    一阶 IIR 低通（电机转速常用）。
    α = 1 - exp(-2π·fc/fs)；fc 越小越光。
    """
    import math

    n = len(ys)
    if n == 0:
        return []
    fc = max(0.1, float(fc_hz))
    fs = max(fc * 2.1, float(fs_hz))
    alpha = 1.0 - math.exp(-2.0 * math.pi * fc / fs)
    return ema_filter(ys, alpha)


def savgol_smooth(ys: Sequence[float], window: int = 21, poly: int = 2) -> list[float]:
    """
    Savitzky–Golay 平滑（局部多项式拟合取常数项）。
    保留拐点形状，比纯滑动均值更适合转速台阶后的趋势。
    """
    n = len(ys)
    if n == 0:
        return []
    k = _odd_window(window, 5)
    half = k // 2
    degree = 2 if poly >= 2 else 1
    if n < k:
        return moving_average(ys, min(n | 1, 5) if n >= 3 else 1)
    out = list(ys)
    for i in range(n):
        a = max(0, i - half)
        b = min(n, i + half + 1)
        # 以 i 为原点，t = j-i；拟合 y≈a0+a1*t(+a2*t^2)
        s0 = s1 = s2 = s3 = s4 = 0.0
        sy = sty = st2y = 0.0
        for j in range(a, b):
            t = float(j - i)
            t2 = t * t
            y = float(ys[j])
            s0 += 1.0
            s1 += t
            s2 += t2
            s3 += t2 * t
            s4 += t2 * t2
            sy += y
            sty += t * y
            st2y += t2 * y
        if degree < 2 or (b - a) < 5:
            # 一次：a0 = (s2*sy - s1*sty)/(s0*s2-s1^2)
            det = s0 * s2 - s1 * s1
            if abs(det) < 1e-18:
                out[i] = float(ys[i])
            else:
                out[i] = (s2 * sy - s1 * sty) / det
            continue
        det = (
            s0 * (s2 * s4 - s3 * s3)
            - s1 * (s1 * s4 - s3 * s2)
            + s2 * (s1 * s3 - s2 * s2)
        )
        if abs(det) < 1e-18:
            out[i] = float(ys[i])
            continue
        # a0 = det0/det，替换第 0 列
        det0 = (
            sy * (s2 * s4 - s3 * s3)
            - s1 * (sty * s4 - st2y * s3)
            + s2 * (sty * s3 - st2y * s2)
        )
        out[i] = det0 / det
    return out


# 电机转速平滑：UI 可选方法
RPM_SMOOTH_METHODS: tuple[tuple[str, str], ...] = (
    ("hampel_ma", "Hampel+滑均（去尖刺·推荐）"),
    ("ma", "滑动均值 MA"),
    ("median", "中值滤波"),
    ("median_ma", "中值+滑均"),
    ("savgol", "Savitzky–Golay"),
    ("ema", "EMA 指数平滑"),
    ("iir", "一阶低通 IIR"),
    ("rate_clamp", "斜率限幅 rate-clamp"),
    ("rate_clamp_ma", "限幅+滑均"),
    ("none", "不平滑（对照）"),
)


def rate_clamp(
    ys: Sequence[float],
    *,
    max_rpm_per_s: float = 80000.0,
    fs_hz: float = 2000.0,
) -> tuple[list[float], int]:
    """
    相邻点斜率限幅（去传感器跳变，保留真实加减速）。
    max_rpm_per_s：允许的最大 |d(RPM)/dt|；@2kHz 默认 8e4 → 每点约 ±40 RPM。
    返回 (限幅后序列, 被夹点数)。
    """
    n = len(ys)
    if n == 0:
        return [], 0
    fs = max(1.0, float(fs_hz))
    max_step = max(0.0, float(max_rpm_per_s) / fs)
    out = [float(ys[0])]
    n_clip = 0
    for i in range(1, n):
        v = float(ys[i])
        prev = out[-1]
        d = v - prev
        if d > max_step:
            out.append(prev + max_step)
            n_clip += 1
        elif d < -max_step:
            out.append(prev - max_step)
            n_clip += 1
        else:
            out.append(v)
    return out, n_clip


def find_startup_t_s(
    rows: Sequence[Row],
    *,
    rpm_gate: float = 10.0,
    require_index: bool = True,
) -> tuple[float | None, str]:
    """
    检测启动可信起点（相对段起点秒）。
    - 先找 |RPM|>rpm_gate
    - 若 require_index：再等到 index_n 相对首点至少 +1（I 确认真在转）
    返回 (t_s 或 None, 说明文字)。
    """
    if not rows:
        return None, "无数据"
    base = t_base_ms(rows)
    gate = max(0.0, float(rpm_gate))
    t_rpm: float | None = None
    for r in rows:
        if abs(float(r[1])) > gate:
            t_rpm = rel_s(r, base)
            break
    if t_rpm is None:
        return None, f"全程 |RPM|≤{gate}，未检测到启动"

    if not require_index:
        return t_rpm, f"|RPM|>{gate} @ {t_rpm:.4f}s"

    i0 = int(rows[0][4]) if len(rows[0]) > 4 else 0
    t_i: float | None = None
    for r in rows:
        if int(r[4]) >= i0 + 1:
            t_i = rel_s(r, base)
            break
    if t_i is None:
        # 无 I 跳变：退化为转速门控，并提示
        return t_rpm, f"|RPM|>{gate} @ {t_rpm:.4f}s（无 I+1，仅用转速门）"

    t = max(t_rpm, t_i)
    return t, f"启动可信点 t={t:.4f}s（|RPM|>{gate} @ {t_rpm:.4f}s，I+1 @ {t_i:.4f}s）"


def trim_before_t(rows: Sequence[Row], t0_s: float) -> list[Row]:
    """丢掉相对起点 t < t0_s 的点（裁掉启动脏段），保留时刻字段不变。"""
    if not rows:
        return []
    return keep_time_range(rows, t0_s, 1e12)


def smooth_rpm_series(
    ys: Sequence[float],
    *,
    method: str = "hampel_ma",
    window: int = 21,
    n_sigma: float = 3.0,
    despike_win: int | None = None,
    alpha: float = 0.1,
    fc_hz: float = 50.0,
    fs_hz: float = 2000.0,
    max_rpm_per_s: float = 80000.0,
) -> tuple[list[float], int]:
    """
    电机转速常用平滑。返回 (smoothed, n_replaced)。
    n_replaced：Hampel / rate_clamp 类为替换或夹持点数，其它为 0。
    """
    m = (method or "hampel_ma").strip().lower()
    w = max(1, int(window))
    dwin = int(despike_win) if despike_win is not None else max(5, (w // 2) | 1)
    if m in ("none", "raw", "off"):
        return list(ys), 0
    if m == "ma":
        return moving_average(ys, w), 0
    if m == "median":
        return median_filter(ys, w), 0
    if m == "median_ma":
        return moving_average(median_filter(ys, min(w, 11)), w), 0
    if m == "savgol":
        return savgol_smooth(ys, w, poly=2), 0
    if m == "ema":
        return ema_filter(ys, alpha), 0
    if m in ("iir", "lowpass", "lpf"):
        return iir_lowpass(ys, fc_hz=fc_hz, fs_hz=fs_hz), 0
    if m in ("rate_clamp", "clamp", "slew"):
        return rate_clamp(ys, max_rpm_per_s=max_rpm_per_s, fs_hz=fs_hz)
    if m in ("rate_clamp_ma", "clamp_ma"):
        clamped, n_c = rate_clamp(ys, max_rpm_per_s=max_rpm_per_s, fs_hz=fs_hz)
        return moving_average(clamped, w), n_c
    # default: hampel_ma
    return despike_and_smooth(ys, despike_win=dwin, n_sigma=n_sigma, smooth_win=w)


def replace_indices_neighbor_avg(ys: Sequence[float], indices: Sequence[int]) -> tuple[list[float], int]:
    """
    用前后点均值替换指定下标，不删点、不留时间空洞。
    连续毛刺段：用段左右两侧「未选中」点做线性插值（仍保持采样时刻）。
    返回 (新序列, 替换点数)。
    """
    n = len(ys)
    if n == 0 or not indices:
        return list(ys), 0
    src = [float(v) for v in ys]
    out = list(src)
    sel = sorted({int(i) for i in indices if 0 <= int(i) < n})
    if not sel:
        return out, 0

    # 分成连续段
    runs: list[tuple[int, int]] = []
    a = sel[0]
    prev = sel[0]
    for i in sel[1:]:
        if i == prev + 1:
            prev = i
            continue
        runs.append((a, prev))
        a = prev = i
    runs.append((a, prev))

    n_rep = 0
    for lo, hi in runs:
        left_i = lo - 1
        right_i = hi + 1
        if left_i < 0 and right_i >= n:
            continue
        if left_i < 0:
            # 靠左：整段用右侧点
            fill = src[right_i]
            for i in range(lo, hi + 1):
                out[i] = fill
                n_rep += 1
            continue
        if right_i >= n:
            fill = src[left_i]
            for i in range(lo, hi + 1):
                out[i] = fill
                n_rep += 1
            continue
        y_l = src[left_i]
        y_r = src[right_i]
        span = right_i - left_i
        for i in range(lo, hi + 1):
            # 单点且相邻 → 恰为前后平均；多点 → 线性插值
            t = (i - left_i) / span
            out[i] = y_l + (y_r - y_l) * t
            n_rep += 1
    return out, n_rep


def apply_rpm_replacements(rows: Sequence[Row], new_rpms: Sequence[float]) -> list[Row]:
    """按新 rpm 列表改写行（长度须一致）；dir 随符号更新。"""
    if len(rows) != len(new_rpms):
        raise ValueError("rpm 长度与 rows 不一致")
    out: list[Row] = []
    for r, rpm in zip(rows, new_rpms):
        rpm_f = float(rpm)
        direc = 1 if rpm_f > 0.5 else (-1 if rpm_f < -0.5 else 0)
        # (t_ms, rpm, dir, seg, index_n, ...)
        rest = tuple(r[3:]) if len(r) > 3 else ()
        if len(r) >= 3:
            out.append((r[0], rpm_f, direc) + rest)
        else:
            out.append((r[0], rpm_f, direc, 1, 0))
    return out


def load_timeline_csv(path: str | Path) -> list[Row]:
    """
    读取曲线工作室导出的 timeline_*.csv → 内部 Row。
    列：t_rel_s,rpm,index_n,...
    """
    import csv

    p = Path(path)
    rows: list[Row] = []
    with p.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            t_s = float(r.get("t_rel_s") or r.get("t") or 0.0)
            rpm = float(r.get("rpm") or 0.0)
            idx = int(float(r.get("index_n") or 0))
            direc = 1 if rpm > 0.5 else (-1 if rpm < -0.5 else 0)
            t_ms = t_s * 1000.0
            # (t_ms, rpm, dir, seg, index_n, t_rel, unix)
            rows.append((t_ms, rpm, direc, 1, idx, 0, 0))
    return rows


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
