# -*- coding: utf-8 -*-
"""从板端 TUNE?/TEST LOG 摘要或 20Hz 遥测尾列分析前馈/PID 建议。

工作流（与通讯文档对齐）：
  1) 台架：HOST OFF 或走 TEST（禁 HOST_TIMEOUT）；TELEM ON @20Hz
  2) 可选：TUNE REC ON（RAM 环，不依赖保活）
  3) 跑 CLOSED / TEST；结束后 TUNE? 或读 # TEST LOG
  4) 调参仍用串口：LEARN / PID / GAIN（本脚本只给建议，不下发）

用法:
  python tune_analyze.py --from-line "# TUNE n=800 mean_err=365 ..."
  python tune_analyze.py --telem logs/hi_step_7500_*.csv
  python tune_analyze.py --summary   # 打印 7500 静差已知结论提纲
"""
from __future__ import annotations

import argparse
import csv
import re
import statistics as st
from pathlib import Path


def parse_tune_line(line: str) -> dict:
    """解析 # TUNE / # TEST LOG 摘要行。"""
    d: dict = {"raw": line.strip()}
    for k, pat in (
        ("n", r"\bn=(\d+)"),
        ("mean_err", r"mean_err=([-\d.]+)"),
        ("mean_u", r"mean_u=([-\d.]+)"),
        ("mean_ff", r"mean_ff=([-\d.]+)"),
        ("mean_pulse", r"mean_pulse=([-\d.]+)"),
        ("mean_rpm", r"mean_rpm=([-\d.]+)"),
        ("mean_tgt", r"mean_tgt=([-\d.]+)"),
        ("i_sat_n", r"i_sat=(\d+)/"),
        ("i_sat_den", r"i_sat=\d+/(\d+)"),
        ("map_max", r"map_max=([-\d.]+)"),
        ("code", r"code=(\d+)"),
    ):
        m = re.search(pat, line)
        if m:
            d[k] = float(m.group(1)) if "." in m.group(1) or k.startswith("mean") or k == "map_max" else int(m.group(1))
    hm = re.search(r"hint=(\S+)", line)
    if hm:
        d["hint"] = hm.group(1)
    return d


def advise(d: dict) -> list[str]:
    lines = []
    tgt = float(d.get("mean_tgt") or 0)
    err = float(d.get("mean_err") or 0)
    u = float(d.get("mean_u") or 0)
    ff = float(d.get("mean_ff") or 0)
    pulse = float(d.get("mean_pulse") or 0)
    map_max = float(d.get("map_max") or 0)
    n = int(d.get("n") or 0)
    sat_n = int(d.get("i_sat_n") or 0)
    sat_den = int(d.get("i_sat_den") or n or 1)
    sat_frac = sat_n / max(sat_den, 1)

    lines.append(f"样本 n={n}  mean_err={err:.1f}  mean_u={u:.1f}  ff={ff:.0f}  pulse={pulse:.0f}  tgt={tgt:.0f}")
    if map_max > 0:
        lines.append(f"前馈图最高 map_max={map_max:.0f} RPM")

    # A 优先
    if map_max > 0 and tgt > map_max * 1.02:
        lines.append(
            f"[优先 A] 前馈不足：目标 {tgt:.0f} > 图末 {map_max:.0f} -> "
            f"重跑 LEARN（RPMMAX>=8000）扩展前馈；勿先猛拧 PID"
        )
    if sat_frac >= 0.4 and abs(err) / max(tgt, 1) >= 0.02:
        lines.append(
            f"[次 B] 积分触顶 i_sat={sat_frac:.0%} u≈{u:.1f}us -> "
            f"可暂升 i_lim；400Hz 下本固件 i+=err*dt，勿按 400/250 盲目放大 Ki"
        )
    if abs(err) / max(tgt, 1) >= 0.03 and abs(u) < 15:
        lines.append("[B] 静差大但 u 很小 -> Ki 可能偏小或未进闭环")
    if abs(err) / max(tgt, 1) < 0.02:
        lines.append("[OK] 静差已小；保持；扑翼后仍需 flap LEARN")

    lines.append("调参命令：LEARN RAMP | PID <kp> <ki> <kd> | GAIN ON|OFF | ADG ILIM ... | PID SAVE / MEASURE SAVE")
    lines.append("观测：TELEM 20Hz 尾列 err,ff,u（col25-27）或 TUNE? / # TEST LOG；记录在 RAM，不靠保活")
    return lines


def analyze_telem_csv(path: Path, settle_frac: float = 0.4) -> dict:
    """解析板载/PC CSV；若有尾列 err,ff,u 则用；否则用 pulse 与目标粗估。"""
    rows = list(csv.reader(path.open(encoding="utf-8", errors="replace")))
    if not rows:
        return {}
    # 跳过表头
    start = 1 if rows[0] and not re.match(r"^\d", rows[0][0] or "") else 0
    data = []
    for r in rows[start:]:
        if len(r) < 11:
            continue
        try:
            rpm = abs(float(r[4]))
            pulse = float(r[9])
            tgt = float(r[10])
            err = float(r[25]) if len(r) > 25 else (tgt - rpm)
            ff = float(r[26]) if len(r) > 26 else float("nan")
            u = float(r[27]) if len(r) > 27 else float("nan")
        except ValueError:
            continue
        data.append((rpm, pulse, tgt, err, ff, u))
    if not data:
        return {}
    n0 = int(len(data) * settle_frac)
    win = data[n0:] or data
    def mean(i):
        xs = [x[i] for x in win if x[i] == x[i]]
        return st.mean(xs) if xs else float("nan")
    return {
        "n": len(win),
        "mean_rpm": mean(0),
        "mean_pulse": mean(1),
        "mean_tgt": mean(2),
        "mean_err": mean(3),
        "mean_ff": mean(4),
        "mean_u": mean(5),
        "map_max": 0,
    }


def print_known_7500():
    print(
        """
=== 7500 静差已知结论（hi_step_7500_20260722_161754）===
目标 7500，稳态 enc≈7135（-4.9%），pulse≈1508，油门未饱和。
前馈图末点 ≈6259RPM@1448us → 目标超出图 → ff 钳位 1448，u≈60us。
u≈60 与 ADG OFF 时 i_lim=800、ki≈0.062 触顶一致（P≈9 + I≈50）。
主因：前馈不足（A）。次因：积分顶满（B）。非 400Hz 需 x1.6 Ki。
推荐：LEARN 扩到 >=8000 → 再 TUNE?/遥测验收 → 仍差再动 i_lim/Ki。
""".strip()
    )


def main():
    ap = argparse.ArgumentParser(description="TUNE / 遥测 → 前馈/PID 建议")
    ap.add_argument("--from-line", help="整行 # TUNE / # TEST LOG")
    ap.add_argument("--telem", type=Path, help="遥测/阶梯 CSV")
    ap.add_argument("--summary", action="store_true", help="打印 7500 已知结论")
    args = ap.parse_args()
    if args.summary or (not args.from_line and not args.telem):
        print_known_7500()
        if not args.from_line and not args.telem:
            return 0
    d = {}
    if args.from_line:
        d = parse_tune_line(args.from_line)
    elif args.telem:
        d = analyze_telem_csv(args.telem)
        if not d:
            print("无法解析 CSV")
            return 2
    for ln in advise(d):
        print(ln)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
