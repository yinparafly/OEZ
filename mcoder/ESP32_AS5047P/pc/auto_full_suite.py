#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空载全自动测试套件（单进程占 COM，顺序执行）。

依据《分歧实测验证方案》修订版空载优先顺序：
  0) 预检 ctrl_hz / 遥测
  1) T1′ PWM 频率扫频（3s 判转；FAIL→FREQ50，更高频不测；产出冻结表）
  2) S1 升/降斜坡 A1/A2
  3) S2 Ka=0 vs Ka>0 对比
  4) 空载多点闭环（可 --skip-ladder 跳过）

用法:
  python auto_full_suite.py --port COM10
  python auto_full_suite.py --port COM10 --skip-ladder
  python auto_full_suite.py --port COM10 --quick   # 频率只测 50,100,200,400,600；跳过多点
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from com_port_guard import acquire, release

PC = Path(__file__).resolve().parent
LOG_DIR = PC / "logs"
PY = sys.executable


def run_step(name: str, args: list[str], log_lines: list[str]) -> int:
    log_lines.append(f"\n## 步骤: {name}\n")
    log_lines.append(f"命令: `{' '.join(args)}`\n")
    print(f"\n======== {name} ========")
    print(" ".join(args))
    t0 = time.time()
    # 子进程不再 acquire（父进程已占锁）；通过环境变量跳过子锁
    env = dict(**{k: v for k, v in __import__("os").environ.items()})
    env["OEZ_COM_LOCK_HELD"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    p = subprocess.run(
        args,
        cwd=str(PC),
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    dt = time.time() - t0
    out = (p.stdout or "") + (p.stderr or "")
    # 截断过长输出，保留尾部
    if len(out) > 12000:
        out = out[:4000] + "\n...\n" + out[-8000:]
    log_lines.append(f"退出码: {p.returncode}  耗时: {dt:.1f}s\n")
    log_lines.append("```\n" + out.rstrip() + "\n```\n")
    try:
        print(out[-2000:] if len(out) > 2000 else out)
    except UnicodeEncodeError:
        safe = (out[-2000:] if len(out) > 2000 else out).encode(
            "gbk", errors="replace"
        ).decode("gbk", errors="replace")
        print(safe)
    print(f"[{name}] exit={p.returncode}  {dt:.1f}s")
    return int(p.returncode)


def main() -> int:
    ap = argparse.ArgumentParser(description="空载全自动测试套件")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--skip-ladder", action="store_true")
    ap.add_argument("--skip-freq", action="store_true")
    ap.add_argument("--skip-s1", action="store_true")
    ap.add_argument("--skip-s2", action="store_true")
    ap.add_argument(
        "--quick",
        action="store_true",
        help="频率只测 50,100,200,400,600；默认跳过多点阶梯",
    )
    args = ap.parse_args()
    if args.quick:
        args.skip_ladder = True

    # 释放可能残留；本套件独占（子进程经 OEZ_COM_LOCK_HELD=1 跳过再抢锁）
    release()
    try:
        acquire("auto_full_suite")
    except SystemExit as exc:
        print(exc)
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_lines: list[str] = [
        "# 空载全自动测试套件报告",
        "",
        "**工况：空载（noload）**",
        "",
        f"- 时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 端口: {args.port}",
        f"- 模式: {'quick' if args.quick else 'full'}",
        f"- 依据: 分歧实测验证方案修订版 T1′→S1→S2→N6",
        "",
        "## 计划",
        "",
        "1. 预检（ctrl_hz / 遥测）",
        "2. T1′ PWM 频率扫频（产出冻结表）",
        "3. S1 升/降斜坡 A1/A2",
        "4. S2 Ka 对比",
        "5. 空载多点至 6000（可选）",
        "",
    ]
    # 统一用换行拼接，避免标题挤成一行
    body_parts: list[str] = ["\n".join(log_lines)]
    codes: dict[str, int] = {}
    try:
        codes["preflight"] = run_step(
            "0 预检",
            [PY, "auto_s1_accel_decel_test.py", "--port", args.port, "--dry"],
            body_parts,
        )
        if codes["preflight"] != 0:
            body_parts.append("\n**预检失败，中止后续给油测试。**\n")
            return _finish(ts, body_parts, codes, 2)

        if not args.skip_freq:
            freq_args = [
                PY,
                "auto_freq_test.py",
                "--port",
                args.port,
                "--hold",
                "3",
                "--pulse",
                "1100",
            ]
            if args.quick:
                freq_args.extend(["--freqs", "50,100,200,400,600"])
            codes["T1_freq"] = run_step("1 T1′ 频率扫频", freq_args, body_parts)
            if codes["T1_freq"] == 2:
                body_parts.append(
                    "\n**T1′：50Hz 未转 → 中止后续；请查供电/校准/脉宽/编码器。**\n"
                )
                return _finish(ts, body_parts, codes, 2)
        else:
            codes["T1_freq"] = -1
            body_parts.append("\n（跳过 T1′）\n")

        if not args.skip_s1:
            codes["S1"] = run_step(
                "2 S1 升/降斜坡",
                [PY, "auto_s1_accel_decel_test.py", "--port", args.port, "--settle", "6"],
                body_parts,
            )
        else:
            codes["S1"] = -1

        if not args.skip_s2:
            codes["S2"] = run_step(
                "3 S2 Ka 对比",
                [
                    PY,
                    "auto_s2_accel_ff_test.py",
                    "--port",
                    args.port,
                    "--ka",
                    "0.05",
                    "--settle",
                    "6",
                ],
                body_parts,
            )
        else:
            codes["S2"] = -1

        if not args.skip_ladder:
            codes["ladder"] = run_step(
                "4 空载多点至 6000",
                [
                    PY,
                    "auto_noload_rpm_ladder.py",
                    "--port",
                    args.port,
                    "--settle",
                    "6",
                    "--ka",
                    "0.05",
                ],
                body_parts,
            )
        else:
            codes["ladder"] = -1
            body_parts.append("\n（跳过多点阶梯）\n")

        return _finish(ts, body_parts, codes, 0)
    finally:
        release()


def _step_meaning(name: str, code: int) -> str:
    if code == -1:
        return "跳过"
    if name == "T1_freq":
        return {
            0: "PASS（全档通过）",
            2: "FAIL（50Hz 基线未转）",
            3: "OK_LIMIT（某档 FAIL，默认取更低 PASS，更高未测）",
            5: "异常",
        }.get(code, f"退出码 {code}")
    return {
        0: "PASS",
        2: "预检/ctrl 失败",
        3: "ABORT",
        4: "部分 FAIL",
        5: "异常",
        6: "需烧录/固件能力不足",
    }.get(code, f"退出码 {code}")


def _severity(name: str, code: int) -> int:
    """套件严重度：T1′ exit 3（找到频率上限）不算失败。"""
    if code < 0:
        return 0
    if name == "T1_freq" and code == 3:
        return 0
    return code


def _finish(ts: str, log_lines: list[str], codes: dict[str, int], hint: int) -> int:
    parts = list(log_lines)
    parts.append("\n## 总评\n\n")
    parts.append("| 步骤 | 退出码 | 含义 |\n|------|--------|------|\n")
    worst = 0
    for k, c in codes.items():
        parts.append(f"| {k} | {c} | {_step_meaning(k, c)} |\n")
        sev = _severity(k, c)
        if sev > worst:
            worst = sev
    if worst == 0 and hint == 0:
        parts.append("\n**套件总评：PASS（空载全自动）**\n")
    elif worst == 0 and hint != 0:
        parts.append(f"\n**套件总评：中止（hint={hint}）**\n")
        worst = hint
    else:
        parts.append(f"\n**套件总评：CHECK（最严重退出码 {worst}）**\n")

    text = "".join(parts)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    path = LOG_DIR / f"full_suite_{ts}.report.md"
    path.write_text(text, encoding="utf-8")
    copy = PC.parent / f"空载全自动测试报告_{ts}.md"
    copy.write_text(text, encoding="utf-8")
    print(f"\n套件报告: {path}")
    print(f"副本: {copy}")
    return worst if worst else hint


if __name__ == "__main__":
    raise SystemExit(main())
