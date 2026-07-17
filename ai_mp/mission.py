"""
AI-MP v2: 航点任务规划模块
支持：读取/创建/编辑/保存/上传 MAVLink 航点任务

使用方法：
  # 读取任务文件
  python ai_mp2.py --port COM16 --cmd mission-load --args mission.txt

  # 显示当前任务
  python ai_mp2.py --port COM16 --cmd mission-show

  # 下载飞控中的任务
  python ai_mp2.py --port COM16 --cmd mission-download

  # 上传任务到飞控
  python ai_mp2.py --port COM16 --cmd mission-upload mission.txt

  # 添加航点
  python ai_mp2.py --port COM16 --cmd mission-add WAYPOINT 37.5 127.0 100 15

  # 清空任务
  python ai_mp2.py --port COM16 --cmd mission-clear
"""

import os
import sys
import time
import math

try:
    from pymavlink import mavutil
    from pymavlink.mavwp import MAVWPLoader
except ImportError:
    print("[ERROR] pymavlink not installed")
    sys.exit(1)


# MAVLink 指令代码
MAV_CMD = {
    'WAYPOINT': 16,         # NAV_WAYPOINT
    'LOITER_TIME': 19,      # NAV_LOITER_TIME
    'LAND': 21,             # NAV_LAND
    'TAKEOFF': 22,          # NAV_TAKEOFF
    'RTL': 20,              # NAV_RETURN_TO_LAUNCH
    'DO_JUMP': 177,         # DO_JUMP
    'DO_CHANGE_SPEED': 178, # DO_CHANGE_SPEED
    'DO_SET_HOME': 179,     # DO_SET_HOME
}

# 指令参数说明
CMD_PARAMS = {
    'WAYPOINT': {
        'p1': 'delay(s)', 'p3': 'acceptance_rad(m)', 'p4': 'yaw(°)',
        'lat': 'lat', 'lon': 'lon', 'alt': 'alt(m)'
    },
    'LOITER_TIME': {
        'p1': 'time(s)', 'p3': 'radius(m)', 'p4': 'yaw(°)',
        'lat': 'lat', 'lon': 'lon', 'alt': 'alt(m)'
    },
    'LAND': {
        'p1': 'abort_alt(m)', 'p4': 'yaw(°)',
        'lat': 'lat', 'lon': 'lon', 'alt': 'alt(m)'
    },
    'TAKEOFF': {
        'p1': 'min_pitch(°)', 'p4': 'yaw(°)',
        'alt': 'alt(m)'
    },
    'RTL': {},
    'DO_JUMP': {
        'p1': 'waypoint_num', 'p2': 'repeat_count'
    },
    'DO_CHANGE_SPEED': {
        'p1': 'speed_type(0=airspeed)', 'p2': 'speed(m/s)'
    },
}


class MissionManager:
    """MAVLink 航点任务管理器"""
    
    def __init__(self, master, log):
        self.master = master
        self.log = log
        self.waypoints = []  # [{cmd, lat, lon, alt, p1, p2, p3, p4, frame}, ...]
    
    def clear(self):
        self.waypoints = []
        self.log.status("任务已清空")
    
    def add_waypoint(self, cmd_name, lat=0, lon=0, alt=100, p1=0, p2=0, p3=0, p4=0, frame=3):
        """添加航点"""
        if cmd_name not in MAV_CMD:
            self.log.error(f"未知指令: {cmd_name}. 可用: {list(MAV_CMD.keys())}")
            return False
        
        wp = {
            'cmd': cmd_name,
            'lat': float(lat),
            'lon': float(lon),
            'alt': float(alt),
            'p1': float(p1),
            'p2': float(p2),
            'p3': float(p3),
            'p4': float(p4),
            'frame': frame,
            'seq': len(self.waypoints) + 1,
        }
        self.waypoints.append(wp)
        self.log.status(f"Added WP#{wp['seq']}: {cmd_name} Lat={lat} Lon={lon} Alt={alt}m")
        return True
    
    def remove_waypoint(self, seq):
        """移除航点（seq 从1开始）"""
        if 0 < seq <= len(self.waypoints):
            removed = self.waypoints.pop(seq - 1)
            # 重新编号
            for i, wp in enumerate(self.waypoints):
                wp['seq'] = i + 1
            self.log.status(f"Removed WP#{seq}: {removed['cmd']}")
            return True
        self.log.error(f"无效航点编号: {seq}")
        return False
    
    def load_from_file(self, filepath):
        """从文件加载任务（支持 MAVLink .waypoints 和简化 .txt 格式）"""
        self.log.action("加载任务文件", filepath)
        
        if not os.path.exists(filepath):
            self.log.error(f"文件不存在: {filepath}")
            return False
        
        self.waypoints = []
        
        with open(filepath, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        # 检测格式
        header = lines[0].strip() if lines else ''
        
        if header.startswith('QGC WPL'):
            return self._load_qgc_format(lines)
        elif header.startswith('ITEM') or lines[0].strip().count('\t') >= 5:
            return self._load_mavlink_format(lines)
        else:
            return self._load_simple_format(lines)
    
    def _load_qgc_format(self, lines):
        """加载 QGC WPL 格式"""
        self.waypoints = []
        for line in lines[1:]:
            line = line.strip()
            if not line:
                continue
            parts = line.split('\t')
            if len(parts) < 12:
                continue
            try:
                wp = {
                    'seq': int(parts[0]),
                    'frame': int(parts[2]),
                    'cmd': self._cmd_name(int(parts[3])),
                    'p1': float(parts[4]),
                    'p2': float(parts[5]),
                    'p3': float(parts[6]),
                    'p4': float(parts[7]),
                    'lat': float(parts[8]),
                    'lon': float(parts[9]),
                    'alt': float(parts[10]),
                }
                self.waypoints.append(wp)
            except (ValueError, IndexError):
                continue
        
        self.log.result(True, f"加载 {len(self.waypoints)} 个航点 (QGC WPL 格式)")
        return True
    
    def _load_mavlink_format(self, lines):
        """加载 MAVLink .waypoints 格式"""
        self.waypoints = []
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split('\t')
            if len(parts) < 8:
                continue
            try:
                wp = {
                    'seq': int(parts[0]),
                    'frame': int(parts[2]),
                    'cmd': self._cmd_name(int(parts[3])),
                    'p1': float(parts[4]),
                    'p2': float(parts[5]),
                    'p3': float(parts[6]),
                    'p4': float(parts[7]),
                    'lat': float(parts[8]) if len(parts) > 8 else 0,
                    'lon': float(parts[9]) if len(parts) > 9 else 0,
                    'alt': float(parts[10]) if len(parts) > 10 else 0,
                }
                self.waypoints.append(wp)
            except (ValueError, IndexError):
                continue
        
        self.log.result(True, f"加载 {len(self.waypoints)} 个航点 (MAVLink 格式)")
        return True
    
    def _load_simple_format(self, lines):
        """加载简化格式：
        # 每行一个航点: CMD LAT LON ALT [P1 P2 P3 P4]
        # 示例:
        WAYPOINT 37.5 127.0 100
        LOITER_TIME 37.6 127.1 80 30
        LAND 37.5 127.0 0
        """
        self.waypoints = []
        seq = 0
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            cmd_name = parts[0].upper()
            if cmd_name not in MAV_CMD:
                self.log.warning(f"跳过未知指令: {cmd_name}")
                continue
            
            try:
                wp = {
                    'seq': seq,
                    'cmd': cmd_name,
                    'frame': 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                    'p1': float(parts[4]) if len(parts) > 4 else 0,
                    'p2': float(parts[5]) if len(parts) > 5 else 0,
                    'p3': float(parts[6]) if len(parts) > 6 else 0,
                    'p4': float(parts[7]) if len(parts) > 7 else 0,
                    'lat': float(parts[1]) if cmd_name not in ('RTL', 'DO_JUMP') else 0,
                    'lon': float(parts[2]) if cmd_name not in ('RTL', 'DO_JUMP') else 0,
                    'alt': float(parts[3]) if len(parts) > 3 and cmd_name not in ('RTL',) else 0,
                }
                self.waypoints.append(wp)
                seq += 1
            except (ValueError, IndexError) as e:
                self.log.warning(f"解析错误: {line} - {e}")
        
        self.log.result(True, f"加载 {len(self.waypoints)} 个航点 (简化格式)")
        return True
    
    def save_to_file(self, filepath):
        """保存任务为 QGC WPL 格式"""
        self.log.action("保存任务文件", filepath)
        
        lines = ["QGC WPL 110\n"]
        for i, wp in enumerate(self.waypoints):
            seq = wp.get('seq', i)
            frame = wp.get('frame', 3)
            cmd_code = self._cmd_code(wp['cmd'])
            p1 = wp.get('p1', 0)
            p2 = wp.get('p2', 0)
            p3 = wp.get('p3', 0)
            p4 = wp.get('p4', 0)
            lat = wp.get('lat', 0)
            lon = wp.get('lon', 0)
            alt = wp.get('alt', 0)
            # QGC WPL format: seq\tcurrent\tframe\tcmd\tp1\tp2\tp3\tp4\tlat\tlon\talt\tautocontinue
            lines.append(f"{seq}\t1\t{frame}\t{cmd_code}\t{p1}\t{p2}\t{p3}\t{p4}\t{lat}\t{lon}\t{alt}\t1\n")
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.writelines(lines)
        
        self.log.result(True, f"已保存 {len(self.waypoints)} 个航点到 {filepath}")
        return True
    
    def save_to_simple(self, filepath):
        """保存任务为简化格式"""
        lines = ["# AI-MP Mission File\n"]
        lines.append("# CMD LAT LON ALT [P1 P2 P3 P4]\n\n")
        
        for wp in self.waypoints:
            cmd = wp['cmd']
            if cmd in ('RTL',):
                lines.append(f"{cmd}\n")
            elif cmd in ('TAKEOFF',):
                lines.append(f"{cmd} {wp['alt']}")
                if wp.get('p1'):
                    lines.append(f" {wp['p1']}")
                lines.append("\n")
            elif cmd == 'DO_JUMP':
                lines.append(f"{cmd} {int(wp['p1'])} {int(wp['p2'])}\n")
            else:
                line = f"{cmd} {wp['lat']} {wp['lon']} {wp['alt']}"
                for p in [wp.get('p1', 0), wp.get('p2', 0), wp.get('p3', 0), wp.get('p4', 0)]:
                    if p:
                        line += f" {p}"
                lines.append(line + "\n")
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.writelines(lines)
        
        self.log.result(True, f"已保存 {len(self.waypoints)} 个航点 (简化格式)")
        return True
    
    def show_mission(self):
        """显示当前任务"""
        self.log.action("当前任务", f"{len(self.waypoints)} 个航点")
        
        if not self.waypoints:
            self.log.status("  (空任务)")
            return
        
        print("\n" + "=" * 70)
        print(f"  {'#':>3}  {'CMD':<16} {'LAT':>10} {'LON':>10} {'ALT':>7}  {'P1':>6} {'P2':>6} {'P3':>6} {'P4':>6}")
        print("=" * 70)
        
        for wp in self.waypoints:
            lat = wp.get('lat', 0)
            lon = wp.get('lon', 0)
            alt = wp.get('alt', 0)
            p1 = wp.get('p1', 0)
            p2 = wp.get('p2', 0)
            p3 = wp.get('p3', 0)
            p4 = wp.get('p4', 0)
            
            print(f"  {wp['seq']:>3}  {wp['cmd']:<16} {lat:>10.6f} {lon:>10.6f} {alt:>7.1f}  {p1:>6.1f} {p2:>6.1f} {p3:>6.1f} {p4:>6.1f}")
        
        print("=" * 70)
        
        # 任务统计
        total_dist = self._calc_total_distance()
        max_alt = max((wp.get('alt', 0) for wp in self.waypoints), default=0)
        print(f"  航点数: {len(self.waypoints)} | 总距离: {total_dist:.1f}m | 最高: {max_alt:.1f}m")
        print()
    
    def upload_to_fc(self):
        """上传任务到飞控 (ArduPlane: 发 MISSION_ITEM 直接，无需等待 REQUEST)"""
        self.log.action("上传任务到飞控", f"{len(self.waypoints)} 个航点")
        
        if not self.waypoints:
            self.log.error("任务为空，无法上传")
            return False
        
        # 先清除缓冲区
        for _ in range(20):
            self.master.recv_match(blocking=False)
        
        # 清除现有任务
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component,
        )
        time.sleep(1)
        
        # 清除缓冲区中的 CLEAR_ALL 响应
        for _ in range(10):
            self.master.recv_match(blocking=False)
        
        # 逐个发送航点 (MISSION_ITEM v1 格式，ArduPlane 直接接受)
        for i, wp in enumerate(self.waypoints):
            cmd_code = self._cmd_code(wp['cmd'])
            
            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                i,                          # seq
                3,                          # frame (MAV_FRAME_GLOBAL_RELATIVE_ALT)
                cmd_code,
                0,                          # current
                1,                          # autocontinue
                wp.get('p1', 0),
                wp.get('p2', 0),
                wp.get('p3', 0),
                wp.get('p4', 0),
                wp.get('lat', 0),
                wp.get('lon', 0),
                wp.get('alt', 0),
            )
            
            # 等待 MISSION_ACK (ArduPlane 每收到一个就 ACK)
            ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
            if ack:
                self.log.status(f"  Uploaded WP#{i+1}: {wp['cmd']} ACK={ack.type}")
            else:
                self.log.status(f"  Uploaded WP#{i+1}: {wp['cmd']} (no ACK)")
        
        self.log.result(True, f"上传完成: {len(self.waypoints)} 个航点")
        return True
    
    def download_from_fc(self):
        """从飞控下载任务"""
        self.log.action("从飞控下载任务", "")
        
        # 先清除输入缓冲区
        for _ in range(10):
            self.master.recv_match(blocking=False)
        
        # 请求任务列表
        self.master.mav.mission_request_list_send(
            self.master.target_system,
            self.master.target_component,
        )
        
        # 接收任务计数
        msg = self.master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if not msg:
            # 重试
            time.sleep(1)
            self.master.mav.mission_request_list_send(
                self.master.target_system,
                self.master.target_component,
            )
            msg = self.master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        
        if not msg:
            self.log.error("未收到任务计数")
            return False
        
        count = msg.count
        self.log.data("飞控任务数", count)
        
        if count == 0:
            self.waypoints = []
            self.log.status("飞控中无任务")
            return True
        
        # 接收每个航点
        self.waypoints = []
        for i in range(count):
            # 发送请求
            self.master.mav.mission_request_int_send(
                self.master.target_system,
                self.master.target_component,
                i,
            )
            
            msg = self.master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
            if not msg:
                self.log.error(f"未收到航点 {i}")
                continue
            
            cmd_name = self._cmd_name(msg.command)
            wp = {
                'seq': msg.seq,
                'frame': msg.frame,
                'cmd': cmd_name,
                'p1': msg.param1,
                'p2': msg.param2,
                'p3': msg.param3,
                'p4': msg.param4,
                'lat': msg.x / 1e7 if msg.x else 0,
                'lon': msg.y / 1e7 if msg.y else 0,
                'alt': msg.z,
            }
            self.waypoints.append(wp)
        
        # 确认
        self.master.mav.mission_ack_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_MISSION_ACCEPTED,
        )
        
        self.log.result(True, f"下载完成: {len(self.waypoints)} 个航点")
        self.show_mission()
        return True
    
    def set_current_waypoint(self, seq):
        """设置当前航点"""
        self.log.action("设置当前航点", f"#{seq}")
        self.master.mav.mission_set_current_send(
            self.master.target_system,
            self.master.target_component,
            seq - 1,
        )
        self.log.result(True, f"当前航点设为 #{seq}")
        return True
    
    def _clear_fc_mission(self):
        """清除飞控中的任务"""
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component,
        )
        time.sleep(0.5)
    
    def _cmd_code(self, cmd_name):
        """指令名 → MAVLink 代码"""
        return MAV_CMD.get(cmd_name, 0)
    
    def _cmd_name(self, cmd_code):
        """MAVLink 代码 → 指令名"""
        for name, code in MAV_CMD.items():
            if code == cmd_code:
                return name
        return f"CMD({cmd_code})"
    
    def _calc_total_distance(self):
        """计算总飞行距离（跳过无坐标的指令）"""
        total = 0
        skip_cmds = {'RTL', 'DO_JUMP', 'DO_CHANGE_SPEED', 'DO_SET_HOME'}
        prev = None
        for wp in self.waypoints:
            if wp['cmd'] in skip_cmds:
                prev = None
                continue
            if prev is not None:
                total += self._haversine(prev['lat'], prev['lon'], wp['lat'], wp['lon'])
            prev = wp
        return total
    
    def _haversine(self, lat1, lon1, lat2, lon2):
        """Haversine 距离计算（米）"""
        R = 6371000
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        return R * 2 * math.asin(math.sqrt(a))


# ============================================================
# Balloon Drop 专用任务模板
# ============================================================
def create_balloon_drop_mission(home_lat, home_lon, drop_lat, drop_lon, 
                                 cruise_alt=100, drop_alt=80):
    """
    创建 Balloon Drop 任务模板
    
    航点序列：
    1. TAKEOFF → 巡航高度
    2. WAYPOINT → 飞往投放点（高空巡航）
    3. WAYPOINT → 到达投放点上方
    4. LOITER_TIME → 在投放点盘旋等待
    5. DO_CHANGE_SPEED → 降到 Balloon Drop 触发速度
    6. WAYPOINT → 接近投放点（下降到投放高度）
    7. WAYPOINT → 触发 Balloon Drop（FBWA 模式手动触发）
    8. RTL → 返回起飞点
    
    注意：实际 Balloon Drop 由 Lua 脚本在 FBWA 模式下触发
    """
    mission = []
    
    # 起飞
    mission.append(('TAKEOFF', home_lat, home_lon, cruise_alt, 10, 0, 0, 0))
    
    # 飞往投放点
    mission.append(('WAYPOINT', drop_lat, drop_lon, cruise_alt, 0, 0, 50, 0))
    
    # 到达投放点上方
    mission.append(('WAYPOINT', drop_lat, drop_lon, cruise_alt, 0, 0, 50, 0))
    
    # 盘旋等待
    mission.append(('LOITER_TIME', drop_lat, drop_lon, cruise_alt, 30, 0, 50, 0))
    
    # 降低速度到 Balloon Drop 触发速度
    mission.append(('DO_CHANGE_SPEED', 0, 0, 0, 0, 15, 0, 0))
    
    # 接近投放点（下降到投放高度）
    mission.append(('WAYPOINT', drop_lat, drop_lon, drop_alt, 0, 0, 30, 0))
    
    # 返回
    mission.append(('RTL', 0, 0, 0, 0, 0, 0, 0))
    
    return mission


if __name__ == '__main__':
    # 独立测试：生成 Balloon Drop 任务模板
    mgr = MissionManager(None, type('Log', (), {
        'action': lambda self, *a: print(f"[ACTION] {a}"),
        'result': lambda self, *a: print(f"[RESULT] {a}"),
        'status': lambda self, *a: print(f"[STATUS] {a}"),
        'data': lambda self, *a: print(f"[DATA] {a}"),
        'warning': lambda self, *a: print(f"[WARN] {a}"),
        'error': lambda self, *a: print(f"[ERROR] {a}"),
    })())
    
    # 生成示例任务
    mission = create_balloon_drop_mission(
        home_lat=37.5, home_lon=127.0,
        drop_lat=37.55, drop_lon=127.05,
        cruise_alt=100, drop_alt=80
    )
    
    for cmd, lat, lon, alt, *params in mission:
        mgr.add_waypoint(cmd, lat, lon, alt, *params)
    
    mgr.show_mission()
    mgr.save_to_simple("D:/oezcon/ai_mp/balloon_drop_template.txt")
    print("Template saved to D:/oezcon/ai_mp/balloon_drop_template.txt")
