"""
ABI 监控器 PC UI（tkinter，无第三方依赖）
- 串口选择/连接，921600 8N1
- 实时数据显示：rpm / cnt / idx / div 大字状态
- 接收回显日志 + 命令发送区，常用命令快捷按钮
用法:  python monitor_ui.py
"""
import re
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports

BAUD = 921600


class MonitorUI:
    def __init__(self, root):
        self.root = root
        root.title("ABI 监控器 (STM32H743) @ 921600")
        root.geometry("720x560")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.ser = None
        self.rx_q = queue.Queue()

        self._build_ui()
        self._refresh_ports()
        self._poll_queue()
        self._poll_port()

    # ---------------- UI 构建 ----------------

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=4)
        top.pack(fill="x")

        ttk.Label(top, text="串口:").pack(side="left")
        self.cmb_port = ttk.Combobox(top, width=10, state="readonly")
        self.cmb_port.pack(side="left", padx=2)
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side="left")
        self.btn_conn = ttk.Button(top, text="连接", command=self._toggle_conn, width=8)
        self.btn_conn.pack(side="left", padx=8)

        self.lbl_status = ttk.Label(top, text="未连接", foreground="gray")
        self.lbl_status.pack(side="left")

        # 状态大字显示
        st = ttk.Frame(self.root, padding=4)
        st.pack(fill="x")
        self.vars = {}
        for key, text in (("rpm", "RPM"), ("cnt", "COUNTS"), ("idx", "INDEX"), ("div", "DIV")):
            f = ttk.Frame(st)
            f.pack(side="left", expand=True, fill="x")
            ttk.Label(f, text=text, anchor="center").pack(fill="x")
            v = tk.StringVar(value="--")
            self.vars[key] = v
            ttk.Label(f, textvariable=v, font=("Consolas", 20, "bold"),
                      anchor="center", foreground="#0044cc").pack(fill="x")

        # 日志区
        mid = ttk.Frame(self.root, padding=4)
        mid.pack(fill="both", expand=True)
        self.txt = scrolledtext.ScrolledText(mid, font=("Consolas", 10), state="disabled", height=14)
        self.txt.pack(fill="both", expand=True)

        # 命令区
        cmd = ttk.Frame(self.root, padding=4)
        cmd.pack(fill="x")
        self.ent_cmd = ttk.Entry(cmd, font=("Consolas", 11))
        self.ent_cmd.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.ent_cmd.bind("<Return>", lambda e: self._send_cmd())
        ttk.Button(cmd, text="发送", command=self._send_cmd).pack(side="left")

        btns = ttk.Frame(self.root, padding=(4, 0, 4, 6))
        btns.pack(fill="x")
        for label in ("RPM ON", "RPM OFF", "CNT", "IDX", "PWM 0", "PWM 300", "PWM 600", "PWM 1000"):
            ttk.Button(btns, text=label, command=lambda l=label: self._send(l)).pack(side="left", padx=2)

    # ---------------- 串口 ----------------

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        cur = self.cmb_port.get()
        self.cmb_port["values"] = ports
        if cur in ports:
            self.cmb_port.set(cur)
        elif ports:
            self.cmb_port.set(ports[0])

    def _toggle_conn(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.btn_conn.config(text="连接")
            self.lbl_status.config(text="已断开", foreground="gray")
            return
        port = self.cmb_port.get()
        if not port:
            self._log("请先选择串口\r\n")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.1)
        except Exception as e:
            self._log(f"打开失败: {e}\r\n")
            return
        self.btn_conn.config(text="断开")
        self.lbl_status.config(text=f"已连接 {port} @ {BAUD}", foreground="green")
        self._log(f"[连接 {port} @ {BAUD}]\r\n")

    # ---------------- 收发 ----------------

    def _send_cmd(self):
        s = self.ent_cmd.get().strip()
        if s:
            self._send(s)
            self.ent_cmd.delete(0, "end")

    def _send(self, s):
        if not (self.ser and self.ser.is_open):
            self._log("[未连接]\r\n")
            return
        try:
            self.ser.write((s + "\r\n").encode())
        except Exception as e:
            self._log(f"发送失败: {e}\r\n")

    # ---------------- 渲染 ----------------

    def _log(self, s):
        self.txt.config(state="normal")
        self.txt.insert("end", s)
        self.txt.see("end")
        self.txt.config(state="disabled")

    _RX_RE = re.compile(r"rpm = (-?\d+), cnt = (\d+), idx = (\d+), div = (\d+)")

    def _parse(self, line):
        m = self._RX_RE.search(line)
        if m:
            self.vars["rpm"].set(m.group(1))
            self.vars["cnt"].set(m.group(2))
            self.vars["idx"].set(m.group(3))
            self.vars["div"].set(m.group(4))
        elif "rpm = " in line and ", cnt = " not in line:
            m2 = re.search(r"rpm = (-?\d+) rpm, div = (\d+)", line)
            if m2:
                self.vars["rpm"].set(m2.group(1))
                self.vars["div"].set(m2.group(2))

    def _poll_port(self):
        if self.ser and self.ser.is_open:
            try:
                n = self.ser.in_waiting
                if n:
                    self.ser.write_timeout = 1
                    data = self.ser.read(n)
                    text = data.decode("utf-8", "replace")
                    for line in text.splitlines():
                        if line:
                            self._parse(line)
                    self._log(text)
            except Exception:
                pass
        self.root.after(60, self._poll_port)

    def _poll_queue(self):
        try:
            while True:
                self._log(self.rx_q.get_nowait())
        except queue.Empty:
            pass
        self.root.after(60, self._poll_queue)

    def on_close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    MonitorUI(root)
    root.mainloop()