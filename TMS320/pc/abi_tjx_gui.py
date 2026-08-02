#!/usr/bin/env python3
"""AbiMonitor_TJX — PC GUI (tkinter + serial + matplotlib)

Usage: python abi_tjx_gui.py [--port COM23]
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial, time, threading, struct, os, csv
from datetime import datetime
from collections import deque
from abi_tjx import *

LIVE_WINDOW = 100  # points in live history

class SerialReader(threading.Thread):
    def __init__(self, port="COM23", baud=921600):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        self.lock = threading.Lock()
        self.buf = b""
        self.lines = deque(maxlen=200)
        self.rpm = 0
        self.armed = 0
        self.rec = 0
        self.alive = 0
        self.done = 0
        self.snap_n = 0
        self.snap_bytes = 0
        self.raw = b""
        self.history = deque(maxlen=LIVE_WINDOW)

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            return True
        except Exception as e:
            return False

    def send(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\r\n").encode())

    def run(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    with self.lock:
                        self.buf += data
                        while b"\n" in self.buf:
                            line_bytes, self.buf = self.buf.split(b"\n", 1)
                            line = line_bytes.decode(errors="replace").strip()
                            self.lines.append(line)
                            self._parse(line)
            except:
                time.sleep(0.1)

    def _parse(self, line: str):
        if line.startswith("L,"):
            p = line.split(",")
            try:
                if len(p) >= 7:
                    self.rpm = int(p[1])
                    self.points = int(p[4])
                    self.rec = int(p[6])
                    self.alive = int(p[7])
                    self.armed = int(p[8])
                self.history.append((time.time(), self.rpm))
            except: pass
        elif line.startswith("# SNAP"):
            try:
                parts = line.split()
                for w in parts:
                    if w.startswith("n="):
                        self.snap_n = int(w[2:])
                    elif w.startswith("bytes="):
                        self.snap_bytes = int(w[6:])
                    elif w.startswith("done="):
                        self.done = int(w[5:])
            except: pass
        elif line.startswith("# ARM"):
            self.armed = 1

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("AbiMonitor_TJX PC v0.2")
        self.root.geometry("700x500")
        self.reader = SerialReader()

        f = ttk.Frame(self.root, padding=5)
        f.pack(fill="both", expand=True)

        # Top: connection
        cframe = ttk.LabelFrame(f, text="Connection", padding=3)
        cframe.pack(fill="x", pady=2)
        ttk.Label(cframe, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value="COM23")
        ttk.Entry(cframe, textvariable=self.port_var, width=8).pack(side="left", padx=2)
        self.conn_btn = ttk.Button(cframe, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=5)
        self.status_lbl = ttk.Label(cframe, text="Disconnected", foreground="red")
        self.status_lbl.pack(side="left", padx=10)

        # Status frame
        sframe = ttk.LabelFrame(f, text="Status", padding=3)
        sframe.pack(fill="x", pady=2)
        self.rpm_lbl = ttk.Label(sframe, text="RPM: --", font=("", 14, "bold"))
        self.rpm_lbl.pack(side="left", padx=10)
        self.armed_lbl = ttk.Label(sframe, text="", font=("", 10))
        self.armed_lbl.pack(side="left", padx=5)
        self.snap_lbl = ttk.Label(sframe, text="SNAP: --")
        self.snap_lbl.pack(side="left", padx=10)

        # Control buttons
        btnf = ttk.Frame(f)
        btnf.pack(fill="x", pady=3)
        ttk.Button(btnf, text="ARM", command=lambda: self.reader.send("MONITOR START")).pack(side="left", padx=3)
        ttk.Button(btnf, text="STOP", command=lambda: self.reader.send("MONITOR STOP")).pack(side="left", padx=3)
        ttk.Button(btnf, text="SNAP?", command=lambda: self.reader.send("SNAP?")).pack(side="left", padx=3)
        ttk.Button(btnf, text="SPD?", command=lambda: self.reader.send("SPD?")).pack(side="left", padx=3)
        ttk.Button(btnf, text="ABI?", command=lambda: self.reader.send("ABI?")).pack(side="left", padx=3)
        ttk.Button(btnf, text="Save CSV", command=self.save_csv).pack(side="left", padx=3)

        # Log area
        lframe = ttk.LabelFrame(f, text="Log", padding=3)
        lframe.pack(fill="both", expand=True, pady=2)
        self.log = tk.Text(lframe, height=15, wrap="none")
        self.log.pack(fill="both", expand=True)
        scroll = ttk.Scrollbar(lframe, command=self.log.yview)
        scroll.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=scroll.set)

        self.poll()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def toggle_connect(self):
        if self.reader.ser and self.reader.ser.is_open:
            self.reader.stop()
            self.reader = SerialReader()
            self.conn_btn.configure(text="Connect")
            self.status_lbl.configure(text="Disconnected", foreground="red")
        else:
            self.reader = SerialReader(self.port_var.get())
            if self.reader.connect():
                self.reader.start()
                self.conn_btn.configure(text="Disconnect")
                self.status_lbl.configure(text="Connected", foreground="green")
            else:
                messagebox.showerror("Error", f"Can't open {self.port_var.get()}")

    def poll(self):
        with self.reader.lock:
            lines = list(self.reader.lines)
            self.reader.lines.clear()
        for line in lines:
            if not line.startswith("L,") and line.strip():
                self.log.insert("end", line + "\n")
                self.log.see("end")
        # Update labels
        self.rpm_lbl.configure(text=f"RPM: {self.reader.rpm}")
        armed = "ARMED" if self.reader.armed else ("REC" if self.reader.rec else ("ALIVE" if self.reader.alive else "IDLE"))
        self.armed_lbl.configure(text=armed, foreground="blue" if self.reader.rec else "black")
        self.snap_lbl.configure(text=f"SNAP: {self.reader.snap_n}pts / {self.reader.snap_bytes}B")
        self.root.after(100, self.poll)

    def save_csv(self):
        self.reader.send("SNAP?")
        time.sleep(0.3)
        # Request DUMP BIN here (future)
        fn = f"snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        messagebox.showinfo("Save", f"Data save to {fn} (DUMP BIN not yet implemented)")

    def on_close(self):
        self.reader.stop()
        self.root.destroy()

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM23")
    args = ap.parse_args()
    app = App()
    app.port_var.set(args.port)
    app.root.mainloop()

if __name__ == "__main__":
    main()
