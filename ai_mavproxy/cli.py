"""
AI-MAVProxy: CLI 入口
统一命令行接口
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.connection import Connection
from modules.param_manager import ParamManager
from modules.flight_control import FlightControl
from modules.telemetry import Telemetry
from hardware.esp32_control import ESP32Controller


class AIMAVProxy:
    """AI-MAVProxy 主类"""
    
    def __init__(self, port='COM15', baud=115200, esp32_ip=None):
        self.port = port
        self.baud = baud
        self.conn = None
        self.params = None
        self.flight = None
        self.telemetry = None
        self.esp32 = ESP32Controller(esp32_ip) if esp32_ip else None
    
    def connect(self):
        """连接飞控"""
        self.conn = Connection(self.port, self.baud)
        if self.conn.connect():
            self.params = ParamManager(self.conn)
            self.flight = FlightControl(self.conn)
            self.telemetry = Telemetry(self.conn)
            self.telemetry.request_streams()
            return True
        return False
    
    def disconnect(self):
        """断开连接"""
        if self.conn:
            self.conn.disconnect()
    
    def status(self):
        """获取状态"""
        if self.telemetry:
            return self.telemetry.get_full_status()
        return None
    
    def param_get(self, name):
        """读取参数"""
        if self.params:
            return self.params.get(name)
        return None
    
    def param_set(self, name, value):
        """设置参数"""
        if self.params:
            return self.params.set(name, value)
        return False
    
    def param_search(self, keyword):
        """搜索参数"""
        if self.params:
            return self.params.search(keyword)
        return {}
    
    def param_export(self, filepath):
        """导出参数"""
        if self.params:
            count = self.params.export_mp(filepath)
            print(f"Exported {count} params to {filepath}")
            return count
        return 0
    
    def arm(self):
        """解锁"""
        if self.flight:
            return self.flight.arm()
        return False
    
    def disarm(self):
        """上锁"""
        if self.flight:
            return self.flight.disarm()
        return False
    
    def set_mode(self, mode):
        """设置模式"""
        if self.flight:
            return self.flight.set_mode(mode)
        return False
    
    def takeoff(self, alt=50):
        """起飞"""
        if self.flight:
            return self.flight.takeoff(alt)
        return False
    
    def rtl(self):
        """返航"""
        if self.flight:
            return self.flight.rtl()
        return False
    
    def land(self):
        """降落"""
        if self.flight:
            return self.flight.land()
        return False
    
    def reboot_remote(self):
        """通过 ESP32 远程重启"""
        if self.esp32:
            return self.esp32.reboot()
        return False


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="AI-MAVProxy: AI-driven ArduPilot CLI")
    parser.add_argument('--port', default='COM15', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--esp32', help='ESP32 IP address')
    parser.add_argument('--cmd', required=True,
                       choices=['status', 'param-get', 'param-set', 'param-search',
                               'param-export', 'arm', 'disarm', 'mode', 'takeoff',
                               'rtl', 'land', 'reboot-remote'],
                       help='Command')
    parser.add_argument('args', nargs='*')
    
    parsed = parser.parse_args()
    
    ai = AIMAVProxy(parsed.port, parsed.baud, parsed.esp32)
    
    if not ai.connect():
        print("Connection failed!")
        sys.exit(1)
    
    try:
        if parsed.cmd == 'status':
            status = ai.status()
            print("\n=== Flight Status ===")
            for k, v in sorted(status.items()):
                print(f"  {k:20s}: {v}")
        
        elif parsed.cmd == 'param-get':
            if parsed.args:
                val = ai.param_get(parsed.args[0])
                print(f"{parsed.args[0]} = {val}")
        
        elif parsed.cmd == 'param-set':
            if len(parsed.args) >= 2:
                ai.param_set(parsed.args[0], parsed.args[1])
                print(f"Set {parsed.args[0]} = {parsed.args[1]}")
        
        elif parsed.cmd == 'param-search':
            if parsed.args:
                results = ai.param_search(parsed.args[0])
                print(f"\n=== Parameters matching '{parsed.args[0]}' ===")
                for k, v in sorted(results.items()):
                    print(f"  {k:30s} = {v}")
        
        elif parsed.cmd == 'param-export':
            filepath = parsed.args[0] if parsed.args else 'params.txt'
            ai.param_export(filepath)
        
        elif parsed.cmd == 'arm':
            if ai.arm():
                print("Armed!")
        
        elif parsed.cmd == 'disarm':
            if ai.disarm():
                print("Disarmed!")
        
        elif parsed.cmd == 'mode':
            if parsed.args:
                ai.set_mode(parsed.args[0])
                print(f"Mode set to {parsed.args[0]}")
        
        elif parsed.cmd == 'takeoff':
            alt = int(parsed.args[0]) if parsed.args else 50
            ai.takeoff(alt)
            print(f"Taking off to {alt}m")
        
        elif parsed.cmd == 'rtl':
            ai.rtl()
            print("RTL engaged")
        
        elif parsed.cmd == 'land':
            ai.land()
            print("Landing")
        
        elif parsed.cmd == 'reboot-remote':
            if ai.reboot_remote():
                print("Remote reboot sent!")
    
    finally:
        ai.disconnect()


if __name__ == '__main__':
    main()
