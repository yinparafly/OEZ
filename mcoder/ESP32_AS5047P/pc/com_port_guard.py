#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COM 口互斥：同一时刻只允许一个 PC 工具占用串口（encoder_monitor 或 auto_*.py）。

避免多个测试窗口 / 脚本抢 COM10。
"""

from __future__ import annotations

import atexit
import os
import sys
from pathlib import Path

LOCK_DIR = Path(__file__).resolve().parent / "logs"
LOCK_PATH = LOCK_DIR / ".com_port.lock"


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    if sys.platform == "win32":
        try:
            import ctypes

            PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
            STILL_ACTIVE = 259
            handle = ctypes.windll.kernel32.OpenProcess(
                PROCESS_QUERY_LIMITED_INFORMATION, False, pid
            )
            if not handle:
                return False
            exit_code = ctypes.c_ulong()
            ok = ctypes.windll.kernel32.GetExitCodeProcess(handle, ctypes.byref(exit_code))
            ctypes.windll.kernel32.CloseHandle(handle)
            return bool(ok) and exit_code.value == STILL_ACTIVE
        except Exception:  # noqa: BLE001
            return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def read_lock() -> tuple[int, str] | None:
    """返回 (pid, owner) 或 None。"""
    if not LOCK_PATH.exists():
        return None
    try:
        text = LOCK_PATH.read_text(encoding="utf-8").strip()
        parts = text.split(None, 1)
        pid = int(parts[0])
        owner = parts[1] if len(parts) > 1 else "?"
        if not _pid_alive(pid):
            try:
                LOCK_PATH.unlink(missing_ok=True)
            except OSError:
                pass
            return None
        return pid, owner
    except (ValueError, OSError):
        return None


def acquire(owner: str, *, force: bool = False) -> None:
    """
    占用串口锁。若已被其它存活进程占用则抛出 SystemExit（除非 force）。
    owner 例: encoder_monitor / auto_s1
    环境变量 OEZ_COM_LOCK_HELD=1 时跳过（父进程套件已占锁）。
    """
    if os.environ.get("OEZ_COM_LOCK_HELD") == "1":
        return
    LOCK_DIR.mkdir(parents=True, exist_ok=True)
    existing = read_lock()
    if existing is not None:
        pid, who = existing
        if pid == os.getpid():
            return
        if not force:
            raise SystemExit(
                f"串口已被占用: {who} (pid={pid})\n"
                f"请先关闭那个窗口/脚本，再重试。\n"
                f"锁文件: {LOCK_PATH}\n"
                f"勿同时开多个 encoder_monitor 或 CLI 自动测试。"
            )
        try:
            if sys.platform == "win32":
                os.system(f'taskkill /PID {pid} /F >nul 2>&1')
            else:
                os.kill(pid, 9)
        except OSError:
            pass
        try:
            LOCK_PATH.unlink(missing_ok=True)
        except OSError:
            pass

    LOCK_PATH.write_text(f"{os.getpid()} {owner}\n", encoding="utf-8")

    def _release() -> None:
        try:
            cur = read_lock()
            if cur and cur[0] == os.getpid():
                LOCK_PATH.unlink(missing_ok=True)
        except OSError:
            pass

    atexit.register(_release)


def release() -> None:
    try:
        cur = read_lock()
        if cur and cur[0] == os.getpid():
            LOCK_PATH.unlink(missing_ok=True)
    except OSError:
        pass


def status_message() -> str:
    cur = read_lock()
    if cur is None:
        return "串口空闲（无其它本工具占用）"
    return f"串口占用中: {cur[1]} (pid={cur[0]})"
