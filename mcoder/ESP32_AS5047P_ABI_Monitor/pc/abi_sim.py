#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
虚拟 ABI 板：在进程内仿真 ESP32 协议，自动注入假转速/弹射数据。
用于验证「记录后数据不会被扔掉」。

用法：
  python abi_monitor.py --sim
  python abi_sim.py          # 无 UI 自测：武装→弹射→停转→DUMP→再 DUMP，断言点数仍在
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field


@dataclass
class Sample:
    t_ms: int
    rpm: float
    seg: int
    index_n: int = 0


@dataclass
class SimBoard:
    """板内状态机精简版：武装 → 转速起来进池 → 确认/停转保留 → 主记录不因 DUMP 清空。"""

    armed: bool = False
    phase: str = "IDLE"  # IDLE | STAGING | RECORD
    seg_count: int = 0
    seg_cur: int = 0
    pool: list[Sample] = field(default_factory=list)
    log: list[Sample] = field(default_factory=list)
    rec_ms: int = 1000
    rpm: float = 0.0
    t_ms: int = 0
    index_n: int = 0
    confirm_counts: int = 500  # ~0.12 rev @4000
    stage_counts0: int = 0
    counts: int = 0
    below_ms: int = 0
    stage_start: int = 0
    rec_start: int = 0
    events: list[str] = field(default_factory=list)

    def tick(self, dt_ms: int = 1, rpm: float | None = None) -> list[str]:
        """推进仿真。返回本拍要发给主机的行。"""
        out: list[str] = []
        if rpm is not None:
            self.rpm = rpm
        self.t_ms += dt_ms
        # 假 ABI：转速 → counts
        steps = int(abs(self.rpm) / 60.0 * 4000.0 * (dt_ms / 1000.0))
        if self.rpm >= 0:
            self.counts += steps
        else:
            self.counts -= steps
        if steps > 0 and (self.t_ms % 60) < dt_ms:
            self.index_n += 1

        if self.armed:
            if self.phase == "IDLE":
                if abs(self.rpm) > 20:
                    self.seg_count += 1
                    self.seg_cur = self.seg_count
                    self.phase = "STAGING"
                    self.stage_start = self.t_ms
                    self.stage_counts0 = self.counts
                    self.pool.clear()
                    self.below_ms = 0
                    self.events.append("STAGING")
            elif self.phase == "STAGING":
                self.pool.append(
                    Sample(self.t_ms, self.rpm, self.seg_cur, self.index_n)
                )
                net = abs(self.counts - self.stage_counts0)
                if net >= self.confirm_counts:
                    self._commit_pool()
                    self.phase = "RECORD"
                    self.rec_start = self.t_ms
                    out.append(
                        f"# CONFIRM real seg={self.seg_cur} pool_committed={len(self.log)} "
                        f"revs≈{net/4000:.2f} → RECORD {self.rec_ms}ms"
                    )
                elif abs(self.rpm) <= 20:
                    self.below_ms += dt_ms
                    if self.below_ms >= 80:
                        if len(self.pool) >= 40 or net >= 40:
                            self._commit_pool()
                            self.phase = "IDLE"
                            self.armed = False
                            self.seg_cur = 0
                            out.append(
                                f"# SALVAGE keep seg={self.seg_count} log_n={len(self.log)} "
                                f"why=rpm_stop (data NOT deleted)"
                            )
                            out.append(f"# RECORD done seg={self.seg_count} log_n={len(self.log)}")
                        else:
                            self.pool.clear()
                            self.phase = "IDLE"
                            self.seg_cur = 0
                            out.append("# NOISE discard seg=0 why=below_gate counts=0")
                else:
                    self.below_ms = 0
                    if self.t_ms - self.stage_start >= 800:
                        if len(self.pool) >= 40:
                            self._commit_pool()
                            self.phase = "IDLE"
                            self.armed = False
                            out.append(
                                f"# SALVAGE keep seg={self.seg_count} log_n={len(self.log)} "
                                f"why=timeout (data NOT deleted)"
                            )
                        else:
                            self.pool.clear()
                            self.phase = "IDLE"
                            out.append("# NOISE discard why=timeout")
            elif self.phase == "RECORD":
                self.log.append(
                    Sample(self.t_ms, self.rpm, self.seg_cur, self.index_n)
                )
                if self.t_ms - self.rec_start >= self.rec_ms:
                    self.phase = "IDLE"
                    self.armed = False
                    seg = self.seg_cur
                    self.seg_cur = 0
                    out.append(
                        f"# RECORD done seg={seg} log_n={len(self.log)} "
                        f"(disarmed, live telem continues)"
                    )
                    out.append(f"# AUTO DUMP BEGIN seg={seg}")
                    out.extend(self._dump_lines(seg))
                    out.append(f"# AUTO DUMP END seg={seg}")

        # 10Hz 遥测
        if self.t_ms % 100 < dt_ms:
            phase_i = {"IDLE": 0, "STAGING": 1, "RECORD": 2}.get(self.phase, 0)
            remain = 0
            if self.phase == "RECORD":
                remain = max(0, self.rec_ms - (self.t_ms - self.rec_start))
            out.append(
                f"L,{self.t_ms},{self.rpm:.2f},{1 if self.rpm>0.5 else (-1 if self.rpm<-0.5 else 0)},"
                f"{1 if self.armed else 0},{len(self.log)},0,2000,{self.counts},"
                f"{self.seg_cur},{phase_i},{len(self.pool)},0.000,{remain},"
                f"{self.seg_count},0.000,0.000,{self.index_n},0,0,0"
            )
        return out

    def _commit_pool(self) -> None:
        # 回溯最多 800
        chunk = self.pool[-800:] if len(self.pool) > 800 else list(self.pool)
        self.log.extend(chunk)
        self.pool.clear()

    def _dump_lines(self, only_seg: int = 0) -> list[str]:
        rows = [s for s in self.log if only_seg == 0 or s.seg == only_seg]
        if not rows:
            return [
                "# LOG empty",
                "# HINT: live RPM != record. Send MONITOR START first.",
                f"# MONITOR armed={1 if self.armed else 0} phase={self.phase} log_n={len(self.log)} (dump empty)",
                "D END 0",
                "# INDEX EVENTS n=0 drop=0 filter_seg=0",
                "I END 0",
            ]
        out = [f"# LOG DUMP seg={only_seg} USB n={len(rows)}"]
        for i, s in enumerate(rows):
            out.append(f"D,{i},{s.t_ms},{s.rpm:.2f},{1 if s.rpm>=0 else -1},{s.seg},{s.index_n},0,0")
        out.append(f"D END {len(rows)}")
        out.append(f"# INDEX EVENTS n=0 drop=0 filter_seg={only_seg}")
        out.append("I END 0")
        # 关键：DUMP 不得清空 log
        out.append(f"# SIM assert log_n_after_dump={len(self.log)} (must keep)")
        return out

    def handle_cmd(self, line: str) -> list[str]:
        line = line.strip()
        if not line:
            return []
        up = line.upper()
        if up.startswith("MONITOR START") or up in ("MONITOR ON", "MONITOR 1"):
            self.armed = True
            return [
                "# MONITOR armed (OK if still). SIM fake ABI ready. "
                f"rec={self.rec_ms}ms log_n={len(self.log)}"
            ]
        if up.startswith("MONITOR STOP"):
            self.armed = False
            self.phase = "IDLE"
            self.pool.clear()
            return ["# MONITOR disarm"]
        if up.startswith("MONITOR"):
            return [
                f"# MONITOR armed={1 if self.armed else 0} phase={self.phase} "
                f"seg={self.seg_cur} pool={len(self.pool)} log_n={len(self.log)} segs={self.seg_count}"
            ]
        if up.startswith("LOG CLEAR"):
            n = len(self.log)
            self.log.clear()
            self.pool.clear()
            self.seg_count = 0
            return [f"# LOG cleared (main+pool) was_n={n}"]
        if up.startswith("LOG DUMP"):
            only = 0
            if "SEG" in up:
                try:
                    only = int(up.split("SEG", 1)[1].strip().split()[0])
                except Exception:
                    only = 0
            return self._dump_lines(only)
        if up.startswith("REC MS"):
            try:
                ms = int(up.split()[-1])
                self.rec_ms = max(500, min(3000, ms))
            except Exception:
                pass
            return [f"# REC MS={self.rec_ms}"]
        if up.startswith("ABI"):
            return [
                f"# ABI hz=2000 meas=2000 rpm={self.rpm:.2f} armed={1 if self.armed else 0} "
                f"phase={self.phase} | revs_abi=0 I_n={self.index_n} (SIM)"
            ]
        if up.startswith("PING"):
            return ["# PONG sim"]
        if up.startswith("TIME"):
            return ["# TIME ok sim"]
        if up.startswith("SIM LAUNCH"):
            # 一键弹射：静止→加速→高速→停转
            return ["# SIM LAUNCH scheduled"]
        return [f"# ERR unknown: {line}"]


class SimLink(threading.Thread):
    """替代 SerialLink / BleLink：进程内假板。"""

    def __init__(self, out_q: queue.Queue, stop_evt: threading.Event, auto_launch: bool = True):
        super().__init__(daemon=True)
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.auto_launch = auto_launch
        self._tx: queue.Queue[str] = queue.Queue()
        self._opened = False
        self.board = SimBoard()
        self._launch_job: str | None = None

    def send(self, cmd: str) -> None:
        line = cmd.strip()
        if line:
            self._tx.put(line)

    @property
    def is_open(self) -> bool:
        return self.is_alive() and self._opened

    def run(self) -> None:
        self._opened = True
        self.out_q.put(("__raw__", "# ESP32 AS5047P ABI Monitor FW=SIM-v1"))
        self.out_q.put(
            (
                "__raw__",
                "# gate=20RPM confirm: SIM → 追800点 pool then record (data kept on stop)",
            )
        )
        self.out_q.put(("__raw__", "# SIM ready — 点①开始监控后将自动弹射假数据"))
        self.out_q.put(("__raw__", "# ABI PCNT + 2kHz OK (simulated)"))

        launch_t0: float | None = None
        armed_seen = False

        try:
            while not self.stop_evt.is_set():
                try:
                    while True:
                        cmd = self._tx.get_nowait()
                        for line in self.board.handle_cmd(cmd):
                            self.out_q.put(("__raw__", line))
                        if cmd.upper().startswith("MONITOR START") and self.auto_launch:
                            launch_t0 = time.monotonic()
                            armed_seen = True
                            self.out_q.put(
                                ("__raw__", "# SIM will auto-launch in 0.5s (fake ABI)")
                            )
                except queue.Empty:
                    pass

                # 自动弹射曲线：0 → 1200rpm → 0
                rpm = 0.0
                if launch_t0 is not None and armed_seen:
                    elapsed = time.monotonic() - launch_t0
                    if elapsed < 0.5:
                        rpm = 0.0
                    elif elapsed < 0.7:
                        rpm = 1200.0 * ((elapsed - 0.5) / 0.2)
                    elif elapsed < 1.5:
                        rpm = 1200.0
                    elif elapsed < 1.8:
                        rpm = 1200.0 * (1.0 - (elapsed - 1.5) / 0.3)
                    else:
                        rpm = 0.0
                        if elapsed > 2.5 and self.board.log and not self.board.armed:
                            # 弹射结束后提示可拉
                            pass

                # 仿真步长 5ms
                for line in self.board.tick(5, rpm):
                    self.out_q.put(("__raw__", line))
                time.sleep(0.005)
        finally:
            self._opened = False


def self_test() -> None:
    """无 UI：完整走一遍并断言 DUMP 后数据仍在。"""
    b = SimBoard(rec_ms=500)
    b.handle_cmd("MONITOR START")
    for _ in range(20):
        b.tick(5, 0.0)
    for _ in range(40):
        b.tick(5, 800.0)
    for _ in range(100):
        b.tick(5, 1200.0)
    lines: list[str] = []
    for _ in range(40):
        lines.extend(b.tick(5, 0.0))
    assert len(b.log) > 0, f"log empty after launch — data was thrown; events={b.events} lines={lines[-5:]}"
    n1 = len(b.log)
    dump1 = b.handle_cmd("LOG DUMP USB")
    assert any(x.startswith("D,") for x in dump1), f"dump has no D lines: {dump1[:6]}"
    assert not any("LOG empty" in x for x in dump1), "dump empty but log had data"
    n2 = len(b.log)
    assert n2 == n1, f"DUMP cleared log! before={n1} after={n2}"
    dump2 = b.handle_cmd("LOG DUMP USB")
    assert any(x.startswith("D,") for x in dump2), "second dump lost data"
    print(f"OK sim self-test: log_n={n1}, dump_lines={len(dump1)}, kept_after_dump={n2}")
    drows = [x for x in dump1 if x.startswith("D,")]
    print("sample:", drows[:2])


if __name__ == "__main__":
    self_test()
