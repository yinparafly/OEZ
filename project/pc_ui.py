import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import time
import os

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import serial
import serial.tools.list_ports


MAX_SAMPLES = 500


class CSVViewerTab:
    def __init__(self, parent):
        self.parent = parent
        self.data = []

        top_frame = ttk.Frame(parent)
        top_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(top_frame, text="Open CSV", command=self.open_file).pack(side=tk.LEFT, padx=2)
        self.file_label = ttk.Label(top_frame, text="No file loaded")
        self.file_label.pack(side=tk.LEFT, padx=10)
        self.sample_label = ttk.Label(top_frame, text="")
        self.sample_label.pack(side=tk.LEFT, padx=10)

        stats_frame = ttk.Frame(parent)
        stats_frame.pack(fill=tk.X, padx=5, pady=2)
        self.stats_label = ttk.Label(stats_frame, text="")
        self.stats_label.pack(side=tk.LEFT, padx=5)

        paned = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_frame = ttk.Frame(paned)
        tree_frame = ttk.Frame(left_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True)

        cols = ("index", "count")
        self.tree = ttk.Treeview(tree_frame, columns=cols, show="headings", height=20)
        self.tree.heading("index", text="Index")
        self.tree.heading("count", text="Count")
        self.tree.column("index", width=100, anchor=tk.E)
        self.tree.column("count", width=100, anchor=tk.E)
        vsb = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        paned.add(left_frame, weight=1)

        plot_frame = ttk.Frame(paned)
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        paned.add(plot_frame, weight=2)

    def open_file(self):
        path = filedialog.askopenfilename(
            title="Open CSV File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            with open(path, "r") as f:
                lines = f.readlines()
            self.data = []
            for line in lines:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(",")
                if len(parts) < 2:
                    continue
                idx = int(parts[0].strip())
                cnt = int(parts[1].strip())
                self.data.append((idx, cnt))
            if not self.data:
                messagebox.showwarning("Warning", "No valid data found in CSV.")
                return
            for row in self.tree.get_children():
                self.tree.delete(row)
            for idx, cnt in self.data:
                self.tree.insert("", tk.END, values=(idx, cnt))
            self.file_label.config(text=os.path.basename(path))
            self.sample_label.config(text=f"Samples: {len(self.data)}")
            self.update_stats()
            self.plot_data()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load CSV:\n{e}")

    def update_stats(self):
        if not self.data:
            return
        counts = [c for _, c in self.data]
        mn = min(counts)
        mx = max(counts)
        avg = sum(counts) / len(counts)
        rpm = avg / 4000.0 * 60.0 * (4.0 / 1.0)
        self.stats_label.config(
            text=f"Min: {mn}  Max: {mx}  Avg: {avg:.1f}  RPM ({avg:.1f} cnt @ 4kHz): {rpm:.2f}"
        )

    def plot_data(self):
        self.ax.clear()
        if not self.data:
            self.canvas.draw()
            return
        indices = [d[0] for d in self.data]
        counts = [d[1] for d in self.data]
        self.ax.plot(indices, counts, marker=".", linestyle="-", markersize=2)
        self.ax.set_xlabel("Index")
        self.ax.set_ylabel("Count")
        self.ax.set_title("Encoder Counts vs Index")
        self.ax.grid(True, linestyle="--", alpha=0.6)
        self.fig.tight_layout()
        self.canvas.draw()


class SerialMonitorTab:
    def __init__(self, parent, serial_manager):
        self.parent = parent
        self.sm = serial_manager
        self.times = []
        self.values = []
        self.running = True

        ctrl_frame = ttk.Frame(parent)
        ctrl_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(ctrl_frame, text="Port:").pack(side=tk.LEFT, padx=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(ctrl_frame, textvariable=self.port_var, state="readonly", width=15)
        self.port_combo.pack(side=tk.LEFT, padx=2)
        self.refresh_ports()

        ttk.Button(ctrl_frame, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT, padx=2)

        self.connect_btn = ttk.Button(ctrl_frame, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=10)

        self.status_indicator = ttk.Label(ctrl_frame, text="Disconnected", foreground="red")
        self.status_indicator.pack(side=tk.LEFT, padx=5)

        ttk.Button(ctrl_frame, text="Clear Plot", command=self.clear_plot).pack(side=tk.RIGHT, padx=2)
        self.auto_scroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctrl_frame, text="Auto-scroll", variable=self.auto_scroll_var).pack(side=tk.RIGHT, padx=5)

        plot_frame = ttk.Frame(parent)
        plot_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.fig = Figure(figsize=(8, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Encoder Count")
        self.ax.set_title("Serial Data - Encoder Counts")
        self.ax.grid(True, linestyle="--", alpha=0.6)
        self.fig.tight_layout()
        self.canvas.draw()

        self.poll_plot()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.sm.connected:
            self.sm.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_indicator.config(text="Disconnected", foreground="red")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showwarning("Warning", "Select a COM port first.")
                return
            try:
                self.sm.connect(port)
                self.connect_btn.config(text="Disconnect")
                self.status_indicator.config(text="Connected", foreground="green")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to connect:\n{e}")

    def clear_plot(self):
        self.times.clear()
        self.values.clear()
        self.ax.clear()
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Encoder Count")
        self.ax.set_title("Serial Data - Encoder Counts")
        self.ax.grid(True, linestyle="--", alpha=0.6)
        self.canvas.draw()

    def add_data_point(self, value):
        if not self.times:
            t0 = 0
        else:
            t0 = self.times[-1] + 1
        self.times.append(t0)
        self.values.append(value)
        if len(self.times) > MAX_SAMPLES:
            self.times.pop(0)
            self.values.pop(0)

    def poll_plot(self):
        if not self.running:
            return
        new_data = self.sm.get_data()
        for val in new_data:
            self.add_data_point(val)
        if self.values:
            self.ax.clear()
            if self.auto_scroll_var.get() and len(self.times) >= MAX_SAMPLES:
                x_data = list(range(len(self.values)))
            else:
                x_data = self.times
            self.ax.plot(x_data, self.values, marker=".", linestyle="-", markersize=2)
            self.ax.set_xlabel("Sample")
            self.ax.set_ylabel("Encoder Count")
            self.ax.set_title("Serial Data - Encoder Counts")
            self.ax.grid(True, linestyle="--", alpha=0.6)
            if self.auto_scroll_var.get() and len(self.values) >= MAX_SAMPLES:
                self.ax.set_xlim(len(self.values) - MAX_SAMPLES, len(self.values))
            self.fig.tight_layout()
            self.canvas.draw()
        self.parent.after(100, self.poll_plot)

    def stop(self):
        self.running = False


class ControlPanelTab:
    def __init__(self, parent, serial_manager):
        self.parent = parent
        self.sm = serial_manager

        led_frame = ttk.LabelFrame(parent, text="LED Test (PB0=Green, PB1=Red)", padding=10)
        led_frame.pack(fill=tk.X, padx=10, pady=5)

        led_btn_frame = ttk.Frame(led_frame)
        led_btn_frame.pack()
        ttk.Button(led_btn_frame, text="Green ON", command=lambda: self.send_cmd(b"1")).pack(side=tk.LEFT, padx=2)
        ttk.Button(led_btn_frame, text="Green OFF", command=lambda: self.send_cmd(b"2")).pack(side=tk.LEFT, padx=2)
        ttk.Button(led_btn_frame, text="Green Toggle", command=lambda: self.send_cmd(b"g")).pack(side=tk.LEFT, padx=2)
        ttk.Separator(led_btn_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        ttk.Button(led_btn_frame, text="Red ON", command=lambda: self.send_cmd(b"3")).pack(side=tk.LEFT, padx=2)
        ttk.Button(led_btn_frame, text="Red OFF", command=lambda: self.send_cmd(b"4")).pack(side=tk.LEFT, padx=2)
        ttk.Button(led_btn_frame, text="Red Toggle", command=lambda: self.send_cmd(b"r")).pack(side=tk.LEFT, padx=2)

        send_frame = ttk.LabelFrame(parent, text="Send Command", padding=10)
        send_frame.pack(fill=tk.X, padx=10, pady=5)

        self.send_g_btn = ttk.Button(send_frame, text="Send 'G' (Start Monitoring)", command=self.send_g)
        self.send_g_btn.pack(pady=5)

        status_frame = ttk.LabelFrame(parent, text="Status", padding=10)
        status_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.status_text = tk.Text(status_frame, height=10, state=tk.DISABLED, wrap=tk.WORD)
        self.status_text.pack(fill=tk.BOTH, expand=True)
        status_scroll = ttk.Scrollbar(self.status_text, orient=tk.VERTICAL, command=self.status_text.yview)
        status_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_text.configure(yscrollcommand=status_scroll.set)

        opts_frame = ttk.LabelFrame(parent, text="Options", padding=10)
        opts_frame.pack(fill=tk.X, padx=10, pady=5)

        self.auto_reconnect_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts_frame, text="Auto-reconnect on disconnect", variable=self.auto_reconnect_var).pack(anchor=tk.W)

        self.running = True
        self.poll_status()

    def send_cmd(self, cmd_bytes):
        if not self.sm.connected:
            messagebox.showwarning("Warning", "Not connected to any serial port.")
            return
        try:
            self.sm.ser.write(cmd_bytes)
            self.append_status(f"Sent: {cmd_bytes!r}\n")
        except Exception as e:
            self.append_status(f"Send error: {e}\n")
            if self.auto_reconnect_var.get():
                self.sm.connected = False
                self.sm.ser = None

    def send_g(self):
        if not self.sm.connected:
            messagebox.showwarning("Warning", "Not connected to any serial port.")
            return
        try:
            self.sm.ser.write(b"G")
            self.append_status("Sent: 'G' (0x47)\n")
        except Exception as e:
            self.append_status(f"Send error: {e}\n")
            if self.auto_reconnect_var.get():
                self.sm.connected = False
                self.sm.ser = None

    def append_status(self, text):
        self.status_text.configure(state=tk.NORMAL)
        self.status_text.insert(tk.END, text)
        self.status_text.see(tk.END)
        self.status_text.configure(state=tk.DISABLED)

    def poll_status(self):
        if not self.running:
            return
        msg = self.sm.get_status_message()
        if msg:
            self.append_status(msg)
        if self.auto_reconnect_var.get() and not self.sm.connected:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            if ports:
                try:
                    self.sm.connect(ports[0])
                    self.append_status(f"Auto-reconnected on {ports[0]}\n")
                except Exception:
                    pass
        self.parent.after(500, self.poll_status)

    def stop(self):
        self.running = False


class SerialManager:
    def __init__(self):
        self.ser = None
        self.connected = False
        self._data_buffer = []
        self._status_buffer = []
        self._lock = threading.Lock()
        self._reader_thread = None
        self._running = False

    def connect(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.connected = True
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def disconnect(self):
        self._running = False
        self.connected = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self._reader_thread = None

    def _reader_loop(self):
        buf = ""
        while self._running:
            if not self.ser or not self.ser.is_open:
                self.connected = False
                break
            try:
                data = self.ser.read(256)
                if data:
                    decoded = data.decode("ascii", errors="replace")
                    buf += decoded
                    lines = buf.split("\n")
                    buf = lines[-1]
                    for line in lines[:-1]:
                        line = line.strip()
                        if not line:
                            continue
                        with self._lock:
                            self._status_buffer.append(line + "\n")
                        try:
                            val = int(line)
                            with self._lock:
                                self._data_buffer.append(val)
                        except ValueError:
                            pass
            except (serial.SerialException, OSError):
                self.connected = False
                break
            except Exception:
                pass
        self.connected = False

    def get_data(self):
        with self._lock:
            data = list(self._data_buffer)
            self._data_buffer.clear()
        return data

    def get_status_message(self):
        with self._lock:
            if self._status_buffer:
                return "".join(self._status_buffer)
            return None


class Application:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ABZ Encoder Data Recorder")
        self.root.geometry("1200x800")
        self.root.minsize(900, 600)

        self.sm = SerialManager()

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        csv_tab = ttk.Frame(self.notebook)
        self.notebook.add(csv_tab, text="CSV Viewer")
        CSVViewerTab(csv_tab)

        serial_tab = ttk.Frame(self.notebook)
        self.notebook.add(serial_tab, text="Serial Monitor")
        self.serial_monitor = SerialMonitorTab(serial_tab, self.sm)

        control_tab = ttk.Frame(self.notebook)
        self.notebook.add(control_tab, text="Control Panel")
        self.control_panel = ControlPanelTab(control_tab, self.sm)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def run(self):
        self.root.mainloop()

    def on_close(self):
        self.serial_monitor.stop()
        self.control_panel.stop()
        self.sm.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    app = Application()
    app.run()
