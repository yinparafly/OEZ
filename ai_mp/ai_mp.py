"""
AI-MP: AI 可操控的全功能地面站
支持 CLI 和 GUI 两种模式

CLI 模式（AI 操作）:
  python ai_mp.py --cli --port COM16 --baud 115200 --command status
  python ai_mp.py --cli --port COM16 --command arm
  python ai_mp.py --cli --port COM16 --command mode --mode AUTO
  python ai_mp.py --cli --port COM16 --command param --get SCR_ENABLE
  python ai_mp.py --cli --port COM16 --command param --set SCR_ENABLE 1
  python ai_mp.py --cli --port COM16 --command monitor

GUI 模式（人类操作）:
  python ai_mp.py --gui --port COM16 --baud 115200
"""

import sys
import os
import time
import json
import argparse
import threading

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink_common
except ImportError:
    print("ERROR: pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)

# ============================================================
# 核心连接模块
# ============================================================
class MAVConnection:
    """MAVLink 连接管理"""
    
    def __init__(self, port, baud=115200, source_system=255):
        self.port = port
        self.baud = baud
        self.source_system = source_system
        self.master = None
        self.connected = False
        self.params = {}
        self.heartbeat = None
        self.position = None
        self.attitude = None
        self.vfr_hud = None
        self.vehicle_type = None
        self.autopilot = None
        self.flightmode = "UNKNOWN"
        
    def connect(self):
        """建立连接"""
        if self.port.startswith('udp') or self.port.startswith('tcp'):
            self.master = mavutil.mavlink_connection(self.port, source_system=self.source_system)
        else:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud, source_system=self.source_system)
        
        print(f"Connecting to {self.port} @ {self.baud}...")
        self.master.wait_heartbeat(timeout=15)
        self.connected = True
        self.vehicle_type = self.master.mav_type
        self.autopilot = getattr(self.master, 'autopilot', 0)
        self.flightmode = self.master.flightmode
        
        print(f"Connected! System={self.master.target_system}, Type={self.vehicle_type}, Autopilot={self.autopilot}")
        return True
    
    def request_data_stream(self, rate=10):
        """请求数据流"""
        if self.master:
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavlink_common.MAV_DATA_STREAM_ALL, rate, 1
            )
    
    def recv_message(self, timeout=1):
        """接收消息"""
        if not self.master:
            return None
        return self.master.recv_match(blocking=True, timeout=timeout)
    
    def get_status(self):
        """获取完整状态"""
        self.request_data_stream(10)
        status = {
            'connected': self.connected,
            'system_id': self.master.target_system if self.master else 0,
            'vehicle_type': self.vehicle_type,
            'autopilot': self.autopilot,
            'flightmode': self.flightmode,
            'timestamp': time.time()
        }
        
        # 收集最新消息
        start = time.time()
        while time.time() - start < 2:
            msg = self.recv_message(timeout=0.5)
            if msg:
                mtype = msg.get_type()
                if mtype == 'VFR_HUD':
                    status['altitude'] = msg.alt
                    status['airspeed'] = msg.airspeed
                    status['groundspeed'] = msg.groundspeed
                    status['heading'] = msg.heading
                    status['throttle'] = msg.throttle
                    status['mode'] = msg.mode
                elif mtype == 'GLOBAL_POSITION_INT':
                    status['lat'] = msg.lat / 1e7
                    status['lon'] = msg.lon / 1e7
                    status['alt_gps'] = msg.alt / 1000
                elif mtype == 'ATTITUDE':
                    import math
                    status['roll'] = math.degrees(msg.roll)
                    status['pitch'] = math.degrees(msg.pitch)
                    status['yaw'] = math.degrees(msg.yaw)
                elif mtype == 'HEARTBEAT':
                    self.flightmode = self.master.flightmode
                    status['flightmode'] = self.flightmode
                    status['system_status'] = msg.system_status
        
        return status
    
    def set_mode(self, mode_name):
        """设置飞行模式"""
        mode_mapping = self.master.mode_mapping()
        if mode_name.upper() in mode_mapping:
            mode_id = mode_mapping[mode_name.upper()]
            self.master.set_mode(mode_id)
            print(f"Mode set to {mode_name} (id={mode_id})")
            return True
        else:
            available = list(mode_mapping.keys())
            print(f"Unknown mode: {mode_name}. Available: {available}")
            return False
    
    def arm(self):
        """解锁"""
        self.master.arducopter_arm()
        print("Arm command sent")
    
    def disarm(self):
        """上锁"""
        self.master.arducopter_disarm()
        print("Disarm command sent")
    
    def set_param(self, name, value):
        """设置参数"""
        if self.master:
            self.master.param_set_send(
                name, float(value), mavlink_common.MAV_PARAM_TYPE_REAL32
            )
            print(f"Parameter {name} = {value}")
            return True
        return False
    
    def get_param(self, name):
        """获取参数"""
        if self.master:
            self.master.param_fetch_one(name.encode())
            time.sleep(0.5)
            msg = self.recv_message(timeout=2)
            if msg and msg.get_type() == 'PARAM_VALUE':
                return msg.param_value
        return None
    
    def fetch_all_params(self):
        """获取所有参数"""
        if self.master:
            self.master.param_fetch_all()
            time.sleep(3)
            while True:
                msg = self.recv_message(timeout=0.5)
                if msg and msg.get_type() == 'PARAM_VALUE':
                    self.params[msg.param_id] = msg.param_value
                else:
                    break
        return self.params
    
    def close(self):
        """关闭连接"""
        if self.master:
            self.master.close()
            self.connected = False
            print("Connection closed")


# ============================================================
# CLI 模式
# ============================================================
def cli_mode(conn, args):
    """命令行模式"""
    cmd = args.command
    
    if cmd == 'status':
        status = conn.get_status()
        print(json.dumps(status, indent=2, default=str))
    
    elif cmd == 'connect':
        print(f"Connected to {args.port}")
    
    elif cmd == 'arm':
        conn.arm()
        time.sleep(1)
        status = conn.get_status()
        print(f"System status: {status.get('system_status', 'unknown')}")
    
    elif cmd == 'disarm':
        conn.disarm()
    
    elif cmd == 'mode':
        if args.mode:
            conn.set_mode(args.mode)
        else:
            print("Available modes:", list(conn.master.mode_mapping().keys()))
    
    elif cmd == 'monitor':
        print("Monitoring (Ctrl+C to stop)...")
        try:
            while True:
                status = conn.get_status()
                alt = status.get('altitude', 0)
                aspd = status.get('airspeed', 0)
                mode = status.get('flightmode', '?')
                lat = status.get('lat', 0)
                lon = status.get('lon', 0)
                print(f"\rMode: {mode:10s} | Alt: {alt:7.1f}m | ASpd: {aspd:5.1f}m/s | Lat: {lat:.6f} | Lon: {lon:.6f}", end='', flush=True)
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nMonitoring stopped")
    
    elif cmd == 'param':
        if args.set:
            name, value = args.set.split('=')
            conn.set_param(name.strip(), value.strip())
        elif args.get:
            val = conn.get_param(args.get)
            print(f"{args.get} = {val}")
        elif args.list:
            params = conn.fetch_all_params()
            for k, v in sorted(params.items()):
                print(f"  {k} = {v}")
    
    elif cmd == 'modes':
        modes = list(conn.master.mode_mapping().keys())
        print("Available flight modes:")
        for m in sorted(modes):
            print(f"  {m}")
    
    elif cmd == 'upload_lua':
        # 上传 Lua 脚本到 SD 卡
        if hasattr(args, 'script') and args.script:
            print(f"Uploading {args.script} to flight controller...")
            # MAVLink FTP upload
            try:
                with open(args.script, 'rb') as f:
                    data = f.read()
                filename = os.path.basename(args.script)
                conn.master.mav.file_transfer_protocol_send(
                    conn.master.target_system, conn.master.target_component,
                    0, 0, len(data), filename.encode(), data
                )
                print(f"Upload command sent: {filename} ({len(data)} bytes)")
            except Exception as e:
                print(f"Upload failed: {e}")
        else:
            print("Usage: --command upload_lua --script path/to/script.lua")
    
    else:
        print(f"Unknown command: {cmd}")
        print("Available: status, arm, disarm, mode, monitor, param, modes, upload_lua")


# ============================================================
# GUI 模式
# ============================================================
def gui_mode(conn):
    """图形界面模式"""
    try:
        import tkinter as tk
        from tkinter import ttk, messagebox
    except ImportError:
        print("ERROR: tkinter not available")
        return
    
    root = tk.Tk()
    root.title("AI-MP Ground Station")
    root.geometry("800x600")
    
    # 连接状态
    status_frame = ttk.LabelFrame(root, text="Connection Status", padding=10)
    status_frame.pack(fill='x', padx=10, pady=5)
    
    ttk.Label(status_frame, text=f"Port: {conn.port}").pack(side='left')
    ttk.Label(status_frame, text=f"Baud: {conn.baud}").pack(side='left', padx=10)
    status_label = ttk.Label(status_frame, text="Connected", foreground="green")
    status_label.pack(side='left', padx=10)
    
    # 飞行数据
    data_frame = ttk.LabelFrame(root, text="Flight Data", padding=10)
    data_frame.pack(fill='both', expand=True, padx=10, pady=5)
    
    labels = {}
    for i, (key, label) in enumerate([
        ('mode', 'Mode'), ('altitude', 'Altitude (m)'),
        ('airspeed', 'Airspeed (m/s)'), ('groundspeed', 'Groundspeed (m/s)'),
        ('lat', 'Latitude'), ('lon', 'Longitude'),
        ('roll', 'Roll (deg)'), ('pitch', 'Pitch (deg)'), ('yaw', 'Yaw (deg)')
    ]):
        row = i // 3
        col = (i % 3) * 2
        ttk.Label(data_frame, text=f"{label}:").grid(row=row, column=col, sticky='e')
        lbl = ttk.Label(data_frame, text="--", font=('Consolas', 12))
        lbl.grid(row=row, column=col+1, sticky='w', padx=5)
        labels[key] = lbl
    
    # 控制按钮
    ctrl_frame = ttk.LabelFrame(root, text="Controls", padding=10)
    ctrl_frame.pack(fill='x', padx=10, pady=5)
    
    def do_arm():
        conn.arm()
    
    def do_disarm():
        conn.disarm()
    
    ttk.Button(ctrl_frame, text="Arm", command=do_arm).pack(side='left', padx=5)
    ttk.Button(ctrl_frame, text="Disarm", command=do_disarm).pack(side='left', padx=5)
    
    ttk.Label(ctrl_frame, text="Mode:").pack(side='left', padx=10)
    mode_var = tk.StringVar(value="FBWA")
    mode_combo = ttk.Combobox(ctrl_frame, textvariable=mode_var, width=10)
    modes = sorted(conn.master.mode_mapping().keys()) if conn.master else []
    mode_combo['values'] = modes
    mode_combo.pack(side='left')
    
    def do_set_mode():
        conn.set_mode(mode_var.get())
    
    ttk.Button(ctrl_frame, text="Set Mode", command=do_set_mode).pack(side='left', padx=5)
    
    # 更新显示
    def update_display():
        try:
            status = conn.get_status()
            for key, lbl in labels.items():
                val = status.get(key, '--')
                if isinstance(val, float):
                    lbl.config(text=f"{val:.2f}")
                else:
                    lbl.config(text=str(val))
        except Exception:
            pass
        root.after(1000, update_display)
    
    root.after(1000, update_display)
    root.mainloop()


# ============================================================
# 主程序
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="AI-MP: AI-controllable Ground Station")
    parser.add_argument('--cli', action='store_true', help='CLI mode')
    parser.add_argument('--gui', action='store_true', help='GUI mode')
    parser.add_argument('--port', default='COM16', help='Serial port or UDP address')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--command', help='CLI command')
    parser.add_argument('--mode', help='Flight mode name')
    parser.add_argument('--set', help='Set param: NAME=VALUE')
    parser.add_argument('--get', help='Get param by name')
    parser.add_argument('--list', action='store_true', help='List all params')
    parser.add_argument('--script', help='Lua script path for upload')
    
    args = parser.parse_args()
    
    if not args.cli and not args.gui:
        args.cli = True  # Default to CLI
    
    # 连接
    conn = MAVConnection(args.port, args.baud)
    try:
        conn.connect()
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit(1)
    
    try:
        if args.cli:
            cli_mode(conn, args)
        elif args.gui:
            gui_mode(conn)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        conn.close()


if __name__ == '__main__':
    main()
