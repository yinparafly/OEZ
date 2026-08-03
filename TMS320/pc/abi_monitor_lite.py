#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AbiMonitor_TJX PC — Complete PC UI (tkinter + matplotlib)
   Adapted from ESP32 abi_monitor.py protocol layer for C2000 firmware.

Usage: python abi_monitor.py [--port COM23]
"""

from __future__ import annotations

import csv, os, queue, struct, threading, time, tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise SystemExit("pip install pyserial")

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

from abi_tjx import (
    SNAP_MAGIC_V2, SNAP_PREAMBLE, SNAP_POINT_SIZE_V2, SNAP_POINT_FMT,
    snap_crc32, parse_snap_bindump, rpm_from_counts_series
)

BAUD = 921600
APP_DIR = Path(__file__).resolve().parent
LOG_DIR = APP_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)

# ── Serial Link ──────────────────────────────────────────────────
class SerialLink(threading.Thread):
    def __init__(self, port="COM23"):
        super().__init__(daemon=True)
        self.port = port
        self.ser = None
        self.running = True
        self.lock = threading.Lock()
        self.q = queue.Queue()
        self.buf = b""
        # live data
        self.rpm = 0
        self.gear = 1
        self.uptime_ms = 0
        self.snap_points = 0
        self.snap_bytes = 0
        self._staging = False
        self._rec = False
        self._alive = False
        self._done = False
        self._armed = False
        self.history = deque(maxlen=500)

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0.5)
            return True
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return False

    def close(self):
        self.running = False
        if self.ser:
            self.ser.close()

    def send(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\r\n").encode())

    def run(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1); continue
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    with self.lock:
                        self.buf += data
                        while b"\n" in self.buf:
                            lb, self.buf = self.buf.split(b"\n", 1)
                            line = lb.decode(errors="replace").strip()
                            self.q.put(line)
                            self._parse(line)
            except Exception:
                time.sleep(0.1)

    def _parse(self, line: str):
        if line.startswith("L,"):
            p = line.split(",")
            try:
                if len(p) >= 9:
                    self.rpm = int(p[1])
                    self.gear = int(p[2])
                    self.uptime_ms = int(p[3])
                    self.snap_points = int(p[4])
                    self._rec = p[6] == "1"
                    self._alive = p[7] == "1"
                    self._armed = p[8] == "1"
                    self.history.append((self.uptime_ms / 1000.0, self.rpm))
            except: pass
        elif line.startswith("# SNAP"):
            for w in line.split():
                if w.startswith("n="): self.snap_points = int(w[2:])
                elif w.startswith("bytes="): self.snap_bytes = int(w[6:])
                elif w.startswith("done="): self._done = bool(int(w[5:]))
        elif line.startswith("# ARMED"):
            self._armed = True
        elif line.startswith("# RPM="):
            p = line.split()
            for w in p:
                if w.startswith("RPM="):
                    self.rpm = int(w[4:])
        elif line.startswith("# PONG"):
            pass

    @property
    def armed(self) -> bool: return self._armed
    @property
    def recording(self) -> bool: return self._rec
    @property
    def alive(self) -> bool: return self._alive
    @property
    def done(self) -> bool: return self._done

    def flush_q(self) -> list[str]:
        lines = []
        while not self.q.empty():
            lines.append(self.q.get())
        return lines


# ── Application ──────────────────────────────────────────────────
class App:
    def __init__(self, port="COM23"):
        self.root = tk.Tk()
        self.root.title("AbiMonitor_TJX — PC v1.0")
        self.root.geometry("900x700")
        self.root.resizable(True, True)
        self.link = SerialLink(port)
        self._last_armed = False
        self._auto_dump_done = False
        self._snap_data = None
        self._segments: list[dict] = []
        self._setup_ui()
        self._poll()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI layout ──────────────────────────────────────────────
    def _setup_ui(self):
        # top toolbar
        tf = ttk.Frame(self.root)
        tf.pack(fill="x", padx=5, pady=3)
        ttk.Label(tf, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value="COM23")
        ttk.Entry(tf, textvariable=self.port_var, width=8).pack(side="left", padx=3)
        self.conn_btn = ttk.Button(tf, text="Connect", command=self._toggle_connect, width=10)
        self.conn_btn.pack(side="left", padx=3)
        self.status_lbl = ttk.Label(tf, text="🔴", font=("", 12))
        self.status_lbl.pack(side="left", padx=5)

        # control strip
        cf = ttk.Frame(self.root)
        cf.pack(fill="x", padx=5, pady=2)
        ttk.Button(cf, text="⏺ ARM", command=lambda: self.link.send("MONITOR START"), width=10).pack(side="left", padx=2)
        ttk.Button(cf, text="⏹ STOP", command=lambda: self.link.send("MONITOR STOP"), width=10).pack(side="left", padx=2)
        ttk.Button(cf, text="🔍 SNAP?", command=lambda: self.link.send("SNAP?"), width=8).pack(side="left", padx=2)
        ttk.Button(cf, text="⏱ SPD?", command=lambda: self.link.send("SPD?"), width=8).pack(side="left", padx=2)
        self.auto_chk = tk.BooleanVar(value=False)
        ttk.Checkbutton(cf, text="Auto-dump", variable=self.auto_chk).pack(side="left", padx=5)
        ttk.Separator(cf, orient="vertical").pack(side="left", fill="y", padx=8)
        self.dump_btn = ttk.Button(cf, text="💾 Save", command=self._save_snap, width=8)
        self.dump_btn.pack(side="left", padx=2)
        ttk.Button(cf, text="� CSV", command=self._export_csv, width=8).pack(side="left", padx=2)
        ttk.Button(cf, text="📊 Plot", command=self._show_plot, width=8).pack(side="left", padx=2)

        # main display panel
        mf = ttk.Frame(self.root)
        mf.pack(fill="both", expand=True, padx=5, pady=3)

        # left: dashboard
        lf = ttk.LabelFrame(mf, text="Dashboard", padding=5)
        lf.pack(side="left", fill="y", padx=(0, 3))
        self.rpm_var = tk.StringVar(value="---")
        ttk.Label(lf, textvariable=self.rpm_var, font=("Consolas", 28, "bold")).pack(pady=5)
        self.state_var = tk.StringVar(value="IDLE")
        ttk.Label(lf, textvariable=self.state_var, font=("", 12)).pack()
        self.pts_var = tk.StringVar(value="SNAP: 0 pts")
        ttk.Label(lf, textvariable=self.pts_var).pack(pady=2)
        self.uptime_var = tk.StringVar(value="")
        ttk.Label(lf, textvariable=self.uptime_var, font=("", 7)).pack()
        # segments list
        seg_lbl = ttk.LabelFrame(lf, text="Captures", padding=2)
        seg_lbl.pack(fill="both", expand=True, pady=5)
        self.seg_list = tk.Listbox(seg_lbl, height=8, width=22, exportselection=False)
        self.seg_list.pack(fill="both", expand=True)
        self.seg_list.bind("<<ListboxSelect>>", self._on_seg_select)

        # right: plot + log
        rf = ttk.Frame(mf)
        rf.pack(side="left", fill="both", expand=True)
        if HAS_MPL:
            self.fig = Figure(figsize=(6, 4), dpi=80)
            self.ax = self.fig.add_subplot(111)
            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("RPM")
            self.ax.grid(True)
            self.line_main, = self.ax.plot([], [], "b-", lw=1)
            self.canvas = FigureCanvasTkAgg(self.fig, rf)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
        else:
            ttk.Label(rf, text="matplotlib not installed", font=("", 14)).pack()

        # log
        logf = ttk.LabelFrame(self.root, text="Log", padding=2)
        logf.pack(fill="both", expand=True, padx=5, pady=3)
        self.log = tk.Text(logf, height=6, wrap="none", font=("Consolas", 8))
        self.log.pack(fill="both", expand=True)

    # ── Actions ────────────────────────────────────────────────
    def _toggle_connect(self):
        if self.link.ser and self.link.ser.is_open:
            self.link.close()
            self.link = SerialLink(self.port_var.get())
            self.conn_btn.configure(text="Connect")
            self.status_lbl.configure(text="🔴")
        else:
            self.link = SerialLink(self.port_var.get())
            if self.link.connect():
                self.link.start()
                self.conn_btn.configure(text="Disconnect")
                self.status_lbl.configure(text="🟢")
            else:
                messagebox.showerror("Error", f"Cannot open {self.port_var.get()}")

    def _save_snap(self):
        """Read data from snap buffer via DUMP BIN protocol (future) or direct memory read"""
        if not self.link.snap_points:
            messagebox.showinfo("Snap", "No data in buffer. ARM first and spin motor.")
            return
        # For now: save raw snap_data from the board (future: DUMP BIN protocol)
        self.link.send("SNAP?")
        fn = LOG_DIR / f"snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.dat"
        messagebox.showinfo("Save", f"Snap status saved to log.\n\nPoints: {self.link.snap_points}\nFile: {fn}")

    def _export_csv(self):
        """Export telemetry history to CSV"""
        if not self.link.history:
            return
        fn = LOG_DIR / f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(fn, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "rpm"])
            for t, rpm in self.link.history:
                w.writerow([f"{t:.3f}", rpm])
        messagebox.showinfo("Export", f"CSV saved to:\n{fn}")

    def _show_plot(self):
        if not self.link.history:
            return
        # pop up dedicated plot window
        top = tk.Toplevel(self.root)
        top.title("RPM History")
        fig = Figure(figsize=(8, 5), dpi=80)
        ax = fig.add_subplot(111)
        ts = [r[0] for r in self.link.history]
        vs = [r[1] for r in self.link.history]
        ax.plot(ts, vs, "b-", lw=1)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("RPM")
        ax.grid(True)
        canvas = FigureCanvasTkAgg(fig, top)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        ttk.Button(top, text="Close", command=top.destroy).pack(pady=3)

    def _on_seg_select(self, ev):
        pass  # future: show selected capture in plot

    # ── Poll loop ──────────────────────────────────────────────
    def _poll(self):
        lines = self.link.flush_q()
        for line in lines:
            if not line.startswith("L,"):
                self.log.insert("end", line + "\n")
                self.log.see("end")

        # update dashboard
        rpm = self.link.rpm
        self.rpm_var.set(f"{rpm} RPM")
        if self.link.done:
            self.state_var.set("✅ DONE")
        elif self.link.recording:
            self.state_var.set("🔴 RECORDING")
        elif self.link.armed:
            self.state_var.set("🔵 ARMED")
        else:
            self.state_var.set("⚪ IDLE")
        self.pts_var.set(f"SNAP: {self.link.snap_points} pts / {self.link.snap_bytes} B")
        self.uptime_var.set(f"uptime: {self.link.uptime_ms / 1000:.0f}s")

        # auto-dump: detect DONE → save
        if self.auto_chk.get() and self.link.done and not self._auto_dump_done:
            self._auto_dump_done = True
            self.link.send("SNAP?")
            # record segment
            seg = {"time": time.time(), "pts": self.link.snap_points, "label": datetime.now().strftime("%H:%M:%S")}
            self._segments.append(seg)
            self.seg_list.insert(0, f"{seg['label']} — {seg['pts']} pts")
            self.log.insert("end", f"[AUTO] Capture saved: {seg['pts']} pts\n")
            self.log.see("end")

        if not self.link.done:
            self._auto_dump_done = False

        # live plot
        if HAS_MPL and self.link.history:
            ts = [r[0] - self.link.history[0][0] for r in self.link.history]
            vs = [r[1] for r in self.link.history]
            self.line_main.set_data(ts, vs)
            if ts:
                self.ax.set_xlim(max(0, ts[-1] - 15), max(ts[-1] + 1, 15))
                if vs:
                    mx = max(vs) or 1
                    self.ax.set_ylim(0, mx * 1.2)
            self.canvas.draw_idle()

        self.root.after(80, self._poll)

    def _on_close(self):
        self.link.close()
        self.root.destroy()


# ── Entry Point ──────────────────────────────────────────────────
def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM23")
    args = ap.parse_args()
    App(args.port).root.mainloop()

if __name__ == "__main__":
    main()
