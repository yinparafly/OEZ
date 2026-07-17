"""
AI-MAVProxy: 参数管理模块
参数读取/写入/搜索/导出/导入
"""

import time


class ParamManager:
    """参数管理"""
    
    def __init__(self, conn):
        self.conn = conn
        self.master = conn.master
        self.log = conn.log if hasattr(conn, 'log') else None
    
    def get(self, name):
        """读取单个参数"""
        self.master.param_fetch_one(name)
        t0 = time.time()
        while time.time() - t0 < 3:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id == name:
                return msg.param_value
        return None
    
    def set(self, name, value):
        """设置参数"""
        from pymavlink import mavutil
        self.master.param_set_send(
            name, float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.3)
        return True
    
    def fetch_all(self):
        """读取所有参数"""
        self.master.mav.param_request_list_send(
            self.master.target_system,
            self.master.target_component
        )
        
        params = {}
        t0 = time.time()
        while time.time() - t0 < 30:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg:
                params[msg.param_id] = msg.param_value
            elif len(params) > 0:
                break
        
        return params
    
    def search(self, keyword):
        """搜索参数"""
        all_params = self.fetch_all()
        return {k: v for k, v in all_params.items() if keyword.upper() in k.upper()}
    
    def export_mp(self, filepath, params=None):
        """导出为 MP 兼容格式"""
        if params is None:
            params = self.fetch_all()
        
        with open(filepath, 'w', encoding='utf-8') as f:
            for name, val in sorted(params.items()):
                f.write(f"{name}\t{val}\n")
        
        return len(params)
    
    def import_mp(self, filepath):
        """从 MP 格式导入"""
        params = {}
        with open(filepath, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if '\t' in line:
                    k, v = line.split('\t', 1)
                    params[k.strip()] = float(v.strip())
        
        count = 0
        for name, val in params.items():
            self.set(name, val)
            count += 1
        
        return count
