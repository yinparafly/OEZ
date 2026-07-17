"""
AI-MAVProxy: ESP32 硬件控制模块
远程电源控制/释放检测/状态监控
"""

import requests
import time


class ESP32Controller:
    """ESP32 硬件控制"""
    
    def __init__(self, ip, timeout=5):
        self.ip = ip
        self.base_url = f"http://{ip}"
        self.timeout = timeout
    
    def reboot(self):
        """远程重启 Nora"""
        try:
            requests.get(f"{self.base_url}/reboot", timeout=self.timeout)
            return True
        except Exception as e:
            print(f"Reboot failed: {e}")
            return False
    
    def power_on(self):
        """上电"""
        try:
            requests.get(f"{self.base_url}/on", timeout=self.timeout)
            return True
        except:
            return False
    
    def power_off(self):
        """断电"""
        try:
            requests.get(f"{self.base_url}/off", timeout=self.timeout)
            return True
        except:
            return False
    
    def get_status(self):
        """获取 ESP32 状态"""
        try:
            resp = requests.get(f"{self.base_url}/status", timeout=self.timeout)
            return resp.json()
        except:
            return None
    
    def gpio_read(self, pin):
        """读取 GPIO 状态"""
        try:
            resp = requests.get(f"{self.base_url}/gpio/{pin}", timeout=self.timeout)
            return resp.json()
        except:
            return None
    
    def wait_for_boot(self, timeout=30):
        """等待 Nora 重启完成"""
        print("Waiting for Nora to boot...")
        time.sleep(5)  # 等待硬件启动
        
        start = time.time()
        while time.time() - start < timeout:
            try:
                from pymavlink import mavutil
                m = mavutil.mavlink_connection('COM15', baud=115200, source_system=255)
                m.wait_heartbeat(timeout=3)
                m.close()
                print("Nora booted successfully!")
                return True
            except:
                time.sleep(2)
        
        print("Nora boot timeout")
        return False
