"""
AI-MAVProxy: 遥测监控模块
实时数据流/日志记录/状态获取
"""

import time
from pymavlink import mavutil


class Telemetry:
    """遥测监控"""
    
    def __init__(self, conn):
        self.conn = conn
        self.master = conn.master
    
    def get_full_status(self, timeout=3):
        """获取完整飞行状态"""
        status = {
            'timestamp': time.strftime('%H:%M:%S'),
            'system_id': self.master.target_system,
        }
        
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue
            
            mtype = msg.get_type()
            
            if mtype == 'HEARTBEAT':
                mapping = self.master.mode_mapping()
                mode_name = f"Unknown({msg.custom_mode})"
                for name, mid in mapping.items():
                    if mid == msg.custom_mode:
                        mode_name = name
                        break
                status['flightmode'] = mode_name
                status['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            
            elif mtype == 'VFR_HUD':
                status['altitude'] = round(msg.alt, 1)
                status['airspeed'] = round(msg.airspeed, 1)
                status['groundspeed'] = round(msg.groundspeed, 1)
                status['heading'] = msg.heading
                status['throttle'] = msg.throttle
                status['climb'] = round(msg.climb, 1)
            
            elif mtype == 'GLOBAL_POSITION_INT':
                status['lat'] = round(msg.lat / 1e7, 6)
                status['lon'] = round(msg.lon / 1e7, 6)
                status['relative_alt'] = round(msg.relative_alt / 1000, 1)
            
            elif mtype == 'ATTITUDE':
                import math
                status['roll'] = round(math.degrees(msg.roll), 1)
                status['pitch'] = round(math.degrees(msg.pitch), 1)
                status['yaw'] = round(math.degrees(msg.yaw), 1)
            
            elif mtype == 'SYS_STATUS':
                status['voltage'] = round(msg.voltage_battery / 1000, 2)
                status['current'] = round(msg.current_battery / 100, 2)
                status['cpu_load'] = msg.load / 10
            
            elif mtype == 'GPS_RAW_INT':
                status['satellites'] = msg.satellites_visible
                status['gps_fix'] = msg.fix_type
        
        return status
    
    def request_streams(self, streams=None):
        """请求数据流"""
        if streams is None:
            streams = [0, 1, 2, 3, 6, 10]
        for sid in streams:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                sid, 4, 1
            )
