"""
AI-MAVProxy: 连接管理模块
封装 pymavlink 连接，提供统一的连接接口
"""

import sys
import time
from pymavlink import mavutil


class Connection:
    """MAVLink 连接管理"""
    
    def __init__(self, port, baud=115200, source_system=255):
        self.port = port
        self.baud = baud
        self.source_system = source_system
        self.master = None
        self.connected = False
    
    def connect(self, timeout=15):
        """连接飞控"""
        try:
            self.master = mavutil.mavlink_connection(
                self.port, 
                baud=self.baud, 
                source_system=self.source_system
            )
            self.master.wait_heartbeat(timeout=timeout)
            self.connected = True
            
            # 请求数据流
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                0, 4, 1  # ALL streams, 4Hz
            )
            
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.master:
            self.master.close()
            self.connected = False
    
    def get_status(self):
        """获取连接状态"""
        if not self.connected:
            return None
        
        return {
            'port': self.port,
            'baud': self.baud,
            'system_id': self.master.target_system,
            'component_id': self.master.target_component,
            'connected': self.connected
        }
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
