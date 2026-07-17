"""
AI-MAVProxy: 飞行控制模块
arm/disarm/mode/takeoff/rtl/land
"""

import time
from pymavlink import mavutil


class FlightControl:
    """飞行控制"""
    
    def __init__(self, conn):
        self.conn = conn
        self.master = conn.master
    
    def arm(self):
        """解锁"""
        self.master.arducopter_arm()
        time.sleep(1)
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            return armed
        return False
    
    def disarm(self):
        """上锁"""
        self.master.arducopter_disarm()
        return True
    
    def set_mode(self, mode_name):
        """设置飞行模式"""
        try:
            mapping = self.master.mode_mapping()
            if mode_name.upper() in mapping:
                self.master.set_mode(mapping[mode_name.upper()])
                return True
        except:
            pass
        return False
    
    def get_mode(self):
        """获取当前模式"""
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            mapping = self.master.mode_mapping()
            for name, mid in mapping.items():
                if mid == msg.custom_mode:
                    return name
        return None
    
    def takeoff(self, alt=50):
        """起飞"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )
        return True
    
    def rtl(self):
        """返航"""
        return self.set_mode('RTL')
    
    def land(self):
        """降落"""
        return self.set_mode('LAND')
    
    def guided_go(self, lat, lon, alt):
        """Guided 模式飞往"""
        self.set_mode('GUIDED')
        time.sleep(1)
        self.master.mav.mission_item_send(
            self.master.target_system,
            self.master.target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 0,
            0, 0, 0, 0,
            lat, lon, alt
        )
        return True
    
    def emergency_stop(self):
        """紧急停止"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        return True
