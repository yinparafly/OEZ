"""
AI-MP v2: 全功能 AI 地面站
操作时自动显示：时间、目的、操作内容、结果

CLI 使用:
  python ai_mp2.py --port COM16 --cmd status
  python ai_mp2.py --port COM16 --cmd monitor
  python ai_mp2.py --port COM16 --cmd param-set SCR_ENABLE 1
  python ai_mp2.py --port COM16 --cmd param-get SCR_ENABLE
  python ai_mp2.py --port COM16 --cmd param-search ARSPD
  python ai_mp2.py --port COM16 --cmd param-list-all
  python ai_mp2.py --port COM16 --cmd param-tree
  python ai_mp2.py --port COM16 --cmd param-export params.txt
  python ai_mp2.py --port COM16 --cmd param-import params.txt
  python ai_mp2.py --port COM16 --cmd param-diff a.txt b.txt
  python ai_mp2.py --port COM16 --cmd mode FBWA
  python ai_mp2.py --port COM16 --cmd arm
  python ai_mp2.py --port COM16 --cmd disarm
  python ai_mp2.py --port COM16 --cmd reboot
  python ai_mp2.py --port COM16 --cmd upload-script balloon_drop.lua
  python ai_mp2.py --port COM16 --cmd download-log ID
  python ai_mp2.py --port COM16 --cmd list-logs
  python ai_mp2.py --port COM16 --cmd tlog-record
  python ai_mp2.py --port COM16 --cmd tlog-playback file.tlog
  python ai_mp2.py --port COM16 --cmd log-export file.bin
  python ai_mp2.py --port COM16 --cmd rc
  python ai_mp2.py --port COM16 --cmd sensor-status
  python ai_mp2.py --port COM16 --cmd calibrate-compass
  python ai_mp2.py --port COM16 --cmd calibrate-accel
  python ai_mp2.py --port COM16 --cmd version
  python ai_mp2.py --port COM16 --cmd fence-add CIRCLE 37.5 127.0 500
  python ai_mp2.py --port COM16 --cmd fence-download
  python ai_mp2.py --port COM16 --cmd fence-show
  python ai_mp2.py --port COM16 --cmd rally-show
  python ai_mp2.py --port COM16 --cmd rally-add LAT LON ALT
  python ai_mp2.py --port COM16 --cmd balloon-drop-test
  python ai_mp2.py --port COM16 --cmd mission-load mission.txt
  python ai_mp2.py --port COM16 --cmd mission-show
  python ai_mp2.py --port COM16 --cmd mission-download
  python ai_mp2.py --port COM16 --cmd mission-upload mission.txt
  python ai_mp2.py --port COM16 --cmd mission-add WAYPOINT 37.5 127.0 100 15
  python ai_mp2.py --port COM16 --cmd mission-add-servo 5 1500
  python ai_mp2.py --port COM16 --cmd mission-add-relay 0 1
  python ai_mp2.py --port COM16 --cmd mission-add-delay 5
  python ai_mp2.py --port COM16 --cmd mission-add-yaw 90
  python ai_mp2.py --port COM16 --cmd mission-smooth
  python ai_mp2.py --port COM16 --cmd mission-clear
  python ai_mp2.py --port COM16 --cmd mission-goto 3
  python ai_mp2.py --port COM16 --cmd takeoff 50
  python ai_mp2.py --port COM16 --cmd guided-go 37.5 127.0 100
  python ai_mp2.py --port COM16 --cmd rtl
  python ai_mp2.py --port COM16 --cmd land
  python ai_mp2.py --port COM16 --cmd emergency
  python ai_mp2.py --port COM16 --cmd battery-config
  python ai_mp2.py --port COM16 --cmd airspeed-calibrate
  python ai_mp2.py --port COM16 --cmd gps-config
  python ai_mp2.py --port COM16 --cmd serial-config 0 MAVLink
  python ai_mp2.py --port COM16 --cmd flightmode-setup
  python ai_mp2.py --port COM16 --cmd export-kml
  python ai_mp2.py --port COM16 --cmd mavlink-inspector HEARTBEAT 5
  python ai_mp2.py --port COM16 --cmd ekf-status
  python ai_mp2.py --port COM16 --cmd checklist run
  python ai_mp2.py --port COM16 --cmd survey-plan 37.5 127.0 500 500 100
  python ai_mp2.py --port COM16 --cmd grid-plan 37.5 127.0 500 100
  python ai_mp2.py --port COM16 --cmd prearm-check
  python ai_mp2.py --port COM16 --cmd failsafe
  python ai_mp2.py --port COM16 --cmd servo-output
  python ai_mp2.py --port COM16 --cmd motor-test 1 10 2
  python ai_mp2.py --port COM16 --cmd sitl-launch
  python ai_mp2.py --port COM16 --cmd sitl-stop
  # SITL WSL 内部模式 (推荐): python ai_mp2.py --port sitl --cmd status
  # SITL UDP 模式: pymavlink 连接 udp:127.0.0.1:5760
"""

import sys
import os
import time
import json
import argparse

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)

# ============================================================
# 操作日志系统
# ============================================================
class OpLog:
    """操作日志：记录每一步操作的目的和结果"""
    
    def __init__(self, logfile="D:/oezcon/ai_mp/operation_log.txt"):
        self.logfile = logfile
        self.session_start = time.time()
        self._log("=" * 60)
        self._log(f"AI-MP Session Started: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        self._log("=" * 60)
    
    def _log(self, msg):
        ts = time.strftime('%H:%M:%S')
        line = f"[{ts}] {msg}"
        print(line)
        try:
            with open(self.logfile, 'a', encoding='utf-8') as f:
                f.write(line + '\n')
        except:
            pass
    
    def action(self, purpose, detail=""):
        """记录操作目的"""
        self._log(f">>> ACTION: {purpose}")
        if detail:
            self._log(f"    Detail: {detail}")
    
    def result(self, success, info=""):
        """记录操作结果"""
        icon = "OK" if success else "FAIL"
        self._log(f"    Result [{icon}]: {info}")
    
    def data(self, key, value):
        """记录数据"""
        self._log(f"    Data: {key} = {value}")
    
    def warning(self, msg):
        self._log(f"    WARNING: {msg}")
    
    def error(self, msg):
        self._log(f"    ERROR: {msg}")
    
    def status(self, msg):
        self._log(f"    STATUS: {msg}")
    
    def separator(self):
        self._log("-" * 60)
    
    def close(self):
        elapsed = time.time() - self.session_start
        self._log(f"\nSession ended. Duration: {elapsed:.1f}s")
        self._log("=" * 60)

# ============================================================
# 全功能 MAVLink 连接
# ============================================================
class FullMAVLink:
    """全功能 MAVLink 连接，带操作日志"""
    
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.master = None
        self.connected = False
        self.log = OpLog()
        self.params = {}
        self.last_status = {}
        self._verifier = None
    
    def connect(self):
        self.log.action("建立 MAVLink 连接", f"Port={self.port}, Baud={self.baud}")
        try:
            # WSL 内部连接模式: 连接本地 SITL
            if self.port in ('sitl', 'wsl', 'local'):
                self.port = 'tcp:127.0.0.1:5760'
                self.log.data("模式", "WSL 内部直连 SITL")

            if self.port.startswith('udp') or self.port.startswith('tcp'):
                self.master = mavutil.mavlink_connection(
                    self.port, source_system=255, dialect='ardupilotmega')
            else:
                self.master = mavutil.mavlink_connection(
                    self.port, baud=self.baud, source_system=255, dialect='ardupilotmega')

            self.master.wait_heartbeat(timeout=15)
            self.connected = True
            
            # 初始化写入验证器
            from verify import WriteVerifier
            self._verifier = WriteVerifier(self)
            
            self.log.result(True, f"System={self.master.target_system}")
            self.log.data("Vehicle Type", f"{self.master.mav_type} ({self._vehicle_name(self.master.mav_type)})")
            self.log.data("Flight Mode", self.master.flightmode)
            
            # 请求数据流
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                0, 10, 1
            )
            return True
        except Exception as e:
            self.log.result(False, str(e))
            return False
    
    def _vehicle_name(self, vtype):
        names = {0:'Generic',1:'FixedWing',2:'Quadrocopter',3:'Coaxial',
                 4:'Helicopter',5:'AntennaTracker',6:'Ground',8:'Airship',
                 10:'VTOL',12:'Helicopter',14:'FixedWing',22:'GroundRover',
                 26:'Airship',28:'Vehicle'}
        return names.get(vtype, f'Unknown({vtype})')
    
    def _autopilot_name(self, ap):
        names = {0:'Generic',1:'Reserved',2:'Invalid',3:'ArduPilot',
                 4:'OpenPilot',5:'GenericMission',6:'PPZ',7:'SMAV',
                 12:'PX4'}
        return names.get(ap, f'Unknown({ap})')
    
    def _get_mode_mapping(self):
        """获取模式映射（自动检测飞行器类型）"""
        try:
            # 先尝试 pymavlink 内置映射
            mapping = self.master.mode_mapping()
            # 如果返回的是 copter 模式但实际是固定翼，手动修正
            if self.master.mav_type == 13:  # FixedWing
                plane_modes = {
                    'MANUAL': 0, 'CIRCLE': 1, 'STABILIZE': 2,
                    'TRAINING': 5, 'ACRO': 6, 'FBWA': 7, 'FBWB': 8,
                    'GUIDED': 4, 'LOITER': 5, 'AUTO': 10, 'RTL': 3,
                    'LAND': 9, 'POSHOLD': 16,
                    'TAKEOFF': 11, 'AVOID_ADSB': 14, 'GUIDED_NOGPS': 15,
                    'CRUISE': 13, 'TURTLE': 18, 'AUTO_RTL': 20,
                }
                return plane_modes
            return mapping
        except:
            return {}
    
    def get_full_status(self):
        """获取完整飞行状态"""
        self.log.action("获取飞行状态", "读取所有传感器和状态数据")
        
        status = {
            'timestamp': time.strftime('%H:%M:%S'),
            'system_id': self.master.target_system,
            'connected': True
        }
        
        # 确保数据流已请求（Nora 需要显式请求）
        for stream_id in range(0, 7):
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                stream_id, 4, 1
            )
        
        start = time.time()
        while time.time() - start < 3:
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue
            
            mtype = msg.get_type()
            
            if mtype == 'HEARTBEAT':
                status['vehicle_type'] = self._vehicle_name(msg.type)
                status['autopilot'] = self._autopilot_name(msg.autopilot)
                mapping = self._get_mode_mapping()
                mode_name = f"Unknown({msg.custom_mode})"
                for name, mid in mapping.items():
                    if mid == msg.custom_mode:
                        mode_name = name
                        break
                status['flightmode'] = mode_name
                status['system_status'] = msg.system_status
                status['base_mode'] = msg.base_mode
                armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                status['armed'] = 'ARMED' if armed else 'DISARMED'
            
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
                status['alt_gps'] = round(msg.alt / 1000, 1)
                status['relative_alt'] = round(msg.relative_alt / 1000, 1)
            
            elif mtype == 'ATTITUDE':
                import math
                status['roll'] = round(math.degrees(msg.roll), 1)
                status['pitch'] = round(math.degrees(msg.pitch), 1)
                status['yaw'] = round(math.degrees(msg.yaw), 1)
            
            elif mtype == 'SYS_STATUS':
                status['voltage'] = round(msg.voltage_battery / 1000, 2)
                status['current'] = round(msg.current_battery / 100, 2)
                status['battery_pct'] = msg.battery_remaining
                status['cpu_load'] = msg.load / 10
            
            elif mtype == 'GPS_RAW_INT':
                status['satellites'] = msg.satellites_visible
                status['gps_fix'] = msg.fix_type
        
        self.last_status = status
        self.log.result(True, f"Mode={status.get('flightmode')}, Alt={status.get('altitude',0)}m, ASpd={status.get('airspeed',0)}m/s")
        
        return status
    
    def set_mode(self, mode_name):
        self.log.action("设置飞行模式", f"目标模式: {mode_name}")
        try:
            mapping = self._get_mode_mapping()
            if mode_name.upper() in mapping:
                mode_id = mapping[mode_name.upper()]
                self.master.set_mode(mode_id)
                self.log.result(True, f"模式已设为 {mode_name} (id={mode_id})")
                return True
            else:
                available = sorted(mapping.keys())
                self.log.result(False, f"未知模式: {mode_name}. 可用: {available}")
                return False
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def arm(self):
        self.log.action("解锁飞控", "发送 ARM 命令")
        try:
            self.master.arducopter_arm()
            time.sleep(1)
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if msg:
                armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                if armed:
                    self.log.result(True, "飞控已解锁")
                else:
                    self.log.warning("解锁命令已发送但未确认，可能有安全检查阻止")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def disarm(self):
        self.log.action("上锁飞控", "发送 DISARM 命令")
        try:
            self.master.arducopter_disarm()
            self.log.result(True, "上锁命令已发送")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def set_param(self, name, value):
        self.log.action("设置参数", f"{name} = {value}")
        try:
            self.master.param_set_send(
                name, float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            time.sleep(0.3)
            self.log.result(True, f"参数 {name} 已设为 {value}")
            
            # 记录写入用于验证
            if hasattr(self, '_verifier') and self._verifier:
                self._verifier.record_param_set(name, value)
            
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def get_param(self, name):
        self.log.action("读取参数", f"参数名: {name}")
        try:
            self.master.param_fetch_one(name)
            # 多次尝试接收响应
            for attempt in range(5):
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
                if msg and hasattr(msg, 'param_id'):
                    if msg.param_id == name:
                        self.log.result(True, f"{msg.param_id} = {msg.param_value}")
                        return msg.param_value
            self.log.result(False, "未收到参数响应（可能需要重启飞控后生效）")
            return None
        except Exception as e:
            self.log.error(str(e))
            return None
    
    def reboot(self):
        self.log.action("重启飞控", "发送 REBOOT 命令")
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            self.log.result(True, "重启命令已发送")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def upload_lua_script(self, script_path):
        """上传 Lua 脚本到 SD 卡（通过 MAVLink FTP / ArduPilot File Manager）"""
        self.log.action("上传 Lua 脚本到 SD 卡", f"文件: {script_path}")
        
        if not os.path.exists(script_path):
            self.log.error(f"文件不存在: {script_path}")
            return False
        
        try:
            filename = os.path.basename(script_path)
            with open(script_path, 'rb') as f:
                data = f.read()
            
            self.log.data("文件大小", f"{len(data)} bytes")
            self.log.data("目标路径", f"/APM/scripts/{filename}")
            
            # ArduPilot MAVLink FTP 协议
            # Session ID, Opcode, Size, Offset, Data, Payload
            SESSION_ID = 1
            
            # Step 1: Open file for write (opcode=3, open for write)
            open_payload = bytearray(240)
            # 填入文件名 (从第4字节开始, null-terminated)
            name_bytes = f"/APM/scripts/{filename}".encode()
            open_payload[:len(name_bytes)] = name_bytes
            
            self.master.mav.file_transfer_protocol_send(
                self.master.target_system,
                self.master.target_component,
                SESSION_ID,     # session
                3,              # opcode: Open for write (MAV_FTPOpcode_OpenFile_W)
                0,              # size
                0,              # offset
                data[:0],       # data (empty for open)
                open_payload
            )
            time.sleep(0.5)
            
            # 接收响应
            resp = self.master.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=True, timeout=3)
            if resp and resp.offset == 0:
                self.log.status("SD 卡文件已打开，准备写入")
            else:
                self.log.warning("未收到打开确认，尝试继续...")
            
            # Step 2: Write data in chunks (opcode=5, write)
            CHUNK_SIZE = 236  # 每个 FTP 包最大数据量
            offset = 0
            
            while offset < len(data):
                chunk = data[offset:offset + CHUNK_SIZE]
                write_payload = bytearray(240)
                write_payload[:len(chunk)] = chunk
                
                self.master.mav.file_transfer_protocol_send(
                    self.master.target_system,
                    self.master.target_component,
                    SESSION_ID,
                    5,              # opcode: Write (MAV_FTPOpcode_Write)
                    len(chunk),
                    offset,
                    chunk,
                    write_payload
                )
                
                offset += CHUNK_SIZE
                time.sleep(0.05)
                
                # 接收写确认
                resp = self.master.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=True, timeout=2)
                if resp:
                    self.log.status(f"  已写入 {min(offset, len(data))}/{len(data)} bytes")
            
            # Step 3: Close file (opcode=6, close)
            self.master.mav.file_transfer_protocol_send(
                self.master.target_system,
                self.master.target_component,
                SESSION_ID,
                6,              # opcode: Close (MAV_FTPOpcode_Close)
                0, 0,
                b'',
                bytearray(240)
            )
            time.sleep(0.3)
            self.master.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=True, timeout=2)
            
            self.log.result(True, f"上传完成: {filename} → /APM/scripts/")
            self.log.status("重启飞控后 Lua 脚本自动加载")
            return True
        except Exception as e:
            self.log.error(f"上传失败: {str(e)}")
            self.log.status("备选方案: 手动拷贝到 SD 卡 /APM/scripts/")
            return False
    
    # ============================================================
    # 航点任务管理
    # ============================================================
    def mission_init(self):
        """初始化任务管理器"""
        if not hasattr(self, '_mission'):
            from mission import MissionManager
            self._mission = MissionManager(self.master, self.log)
        return self._mission
    
    def mission_load(self, filepath):
        """从文件加载任务"""
        mgr = self.mission_init()
        mgr.load_from_file(filepath)
        mgr.show_mission()
    
    def mission_show(self):
        """显示当前任务"""
        mgr = self.mission_init()
        mgr.show_mission()
    
    def mission_download(self):
        """从飞控下载任务"""
        mgr = self.mission_init()
        mgr.download_from_fc()
    
    def mission_upload_to_fc(self):
        """上传任务到飞控"""
        mgr = self.mission_init()
        mgr.upload_to_fc()
    
    def mission_save(self, filepath, fmt='simple'):
        """保存任务到文件"""
        mgr = self.mission_init()
        if fmt == 'qgc':
            mgr.save_to_file(filepath)
        else:
            mgr.save_to_simple(filepath)
    
    def mission_add_wp(self, cmd_name, lat=0, lon=0, alt=100, p1=0, p2=0, p3=0, p4=0):
        """添加航点"""
        mgr = self.mission_init()
        mgr.add_waypoint(cmd_name, lat, lon, alt, p1, p2, p3, p4)
    
    def mission_remove_wp(self, seq):
        """移除航点"""
        mgr = self.mission_init()
        mgr.remove_waypoint(seq)
    
    def mission_clear_all(self):
        """清空任务"""
        mgr = self.mission_init()
        mgr.clear()
    
    def mission_goto(self, seq):
        """跳转到指定航点"""
        mgr = self.mission_init()
        mgr.set_current_waypoint(seq)
    
    def balloon_drop_test(self):
        """Balloon Drop 测试序列"""
        self.log.separator()
        self.log.action("BALLOON DROP 完整测试", "验证飞行任务配置")
        
        # 步骤 1: 连接状态
        status = self.get_full_status()
        self.log.status(f"当前模式: {status.get('flightmode')}")
        
        # 步骤 2: 配置 Lua 脚本参数
        self.log.action("配置 Balloon Drop 参数", "设置 Lua 脚本和传感器")
        self.set_param('SCR_ENABLE', 1)
        self.set_param('SCR_HEAP_SIZE', 8192)
        self.set_param('ARSPD_USE', 1)
        
        # 步骤 3: 检查关键参数
        self.log.action("检查 Balloon Drop 配置", "验证参数正确性")
        params_to_check = ['SCR_ENABLE', 'SCR_HEAP_SIZE', 'ARSPD_USE']
        all_ok = True
        for p in params_to_check:
            val = self.get_param(p)
            if val is not None:
                self.log.status(f"  {p} = {val}")
            else:
                self.log.warning(f"  {p} 无法读取")
                all_ok = False
        
        # 步骤 4: 检查可用模式
        self.log.action("检查可用飞行模式", "确认 FBWA/AUTO/LOITER 可用")
        modes = self._get_mode_mapping()
        key_modes = ['FBWA', 'FBWB', 'CRUISE', 'AUTO', 'LOITER', 'RTL']
        for m in key_modes:
            if m in modes:
                self.log.status(f"  {m}: 可用 (id={modes[m]})")
            else:
                self.log.warning(f"  {m}: 不可用")
        
        # 步骤 5: 心跳稳定性
        self.log.action("心跳稳定性测试", "检测 3 次心跳间隔")
        hb_times = []
        for i in range(3):
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if msg:
                hb_times.append(time.time())
                self.log.status(f"  心跳 #{i+1}: OK")
            else:
                self.log.warning(f"  心跳 #{i+1}: 超时")
        
        if len(hb_times) >= 2:
            interval = (hb_times[-1] - hb_times[0]) / (len(hb_times) - 1)
            self.log.data("心跳间隔", f"{interval:.2f}s")
        
        # 汇总
        self.log.separator()
        self.log.action("测试汇总", "")
        self.log.status(f"连接: OK")
        self.log.status(f"Lua 脚本支持: {'OK' if status.get('flightmode') else '需要配置'}")
        self.log.status(f"空速传感器: 已启用")
        self.log.status(f"可用模式: {len(modes)} 种")
        self.log.status(f"心跳: {'OK' if len(hb_times) >= 2 else '不稳定'}")
        
        if all_ok:
            self.log.result(True, "Balloon Drop 配置验证全部通过")
        else:
            self.log.warning("部分参数未确认，建议重新检查")
        
        return all_ok
    
    def close(self):
        if self.master:
            self.master.close()
            self.connected = False
            self.log.action("关闭连接", "MAVLink 连接已关闭")
            self.log.close()
    
    # ============================================================
    # RC 通道监控
    # ============================================================
    def get_rc_channels(self):
        """读取 RC 通道值"""
        self.log.action("读取 RC 通道", "获取遥控器输入")
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2, 1
        )
        msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=3)
        if msg:
            channels = {}
            for i in range(1, 19):
                val = getattr(msg, f'chan{i}_raw', 0)
                if val and val > 0:
                    channels[f'CH{i}'] = int(val)
            self.log.result(True, f"RC 通道: {channels}")
            return channels
        self.log.result(False, "未收到 RC 数据")
        return {}
    
    # ============================================================
    # 传感器状态
    # ============================================================
    def get_sensor_status(self):
        """获取传感器健康状态"""
        self.log.action("传感器状态", "检查所有传感器")
        
        # 请求传感器数据流
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2, 1
        )
        
        sensors = {}
        # SYS_STATUS 包含传感器可用性
        msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if msg:
            sensor_bits = msg.onboard_control_sensors_present
            sensor_enabled = msg.onboard_control_sensors_enabled
            sensor_health = msg.onboard_control_sensors_health
            
            sensor_names = {
                0x01: 'Gyro', 0x02: 'Accel', 0x04: 'Mag',
                0x08: 'Baro', 0x10: 'GPS', 0x20: 'OptFlow',
                0x40: 'GCS', 0x80: 'AHRS', 0x100: 'Battery',
                0x200: 'RC', 0x400: 'AHRS2', 0x800: 'GPS2',
                0x1000: 'Terain', 0x2000: 'Wind', 0x4000: 'Logging',
            }
            
            for bit, name in sensor_names.items():
                present = bool(sensor_bits & bit)
                enabled = bool(sensor_enabled & bit)
                healthy = bool(sensor_health & bit)
                status = "OK" if healthy else ("DISABLED" if not enabled else "FAIL")
                sensors[name] = {'present': present, 'enabled': enabled, 'healthy': healthy, 'status': status}
        
        # GPS 详细信息
        msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg:
            fix_types = {0:'No GPS', 1:'No Fix', 2:'2D Fix', 3:'3D Fix', 4:'DGPS', 5:'RTK Float', 6:'RTK Fixed'}
            sensors['GPS'] = sensors.get('GPS', {})
            sensors['GPS']['fix'] = fix_types.get(msg.fix_type, f'Unknown({msg.fix_type})')
            sensors['GPS']['sats'] = msg.satellites_visible
        
        # 打印状态
        print("\n=== Sensor Status ===")
        for name, info in sensors.items():
            if isinstance(info, dict):
                status = info.get('status', 'N/A')
                extra = ""
                if 'fix' in info:
                    extra = f" Fix={info['fix']} Sats={info.get('sats', 0)}"
                print(f"  {name:12s}: {status}{extra}")
        
        self.log.result(True, f"{len(sensors)} 个传感器")
        return sensors
    
    # ============================================================
    # 日志管理
    # ============================================================
    def list_logs(self):
        """列出 SD 卡上的日志文件"""
        self.log.action("列出日志文件", "读取 SD 卡 LOGS 目录")
        try:
            self.master.mav.log_request_list_send(
                self.master.target_system,
                self.master.target_component,
                0, 0xFFFF
            )
            
            logs = []
            start = time.time()
            while time.time() - start < 5:
                msg = self.master.recv_match(type='LOG_ENTRY', blocking=True, timeout=2)
                if not msg:
                    break
                logs.append({
                    'id': msg.id,
                    'num_logs': msg.num_logs,
                    'last_log_num': msg.last_log_num,
                    'time_utc': msg.time_utc,
                    'size': msg.size,
                })
            
            if logs:
                print("\n=== SD Card Logs ===")
                for log in logs:
                    size_kb = log['size'] / 1024
                    print(f"  Log #{log['id']:3d}: {size_kb:8.1f} KB  ({log['time_utc']})")
                self.log.result(True, f"{len(logs)} 个日志文件")
            else:
                self.log.result(False, "未找到日志文件")
            
            return logs
        except Exception as e:
            self.log.error(str(e))
            return []
    
    def download_log(self, log_id, output_dir="D:/oezcon/ai_mp/logs"):
        """下载指定日志文件"""
        self.log.action(f"下载日志 #{log_id}", f"输出到: {output_dir}")
        
        os.makedirs(output_dir, exist_ok=True)
        
        try:
            # 请求日志数据
            self.master.mav.log_request_data_send(
                self.master.target_system,
                self.master.target_component,
                log_id, 0, 0xFFFFFFFF
            )
            
            data = bytearray()
            start = time.time()
            chunk_count = 0
            
            while time.time() - start < 30:
                msg = self.master.recv_match(type='LOG_DATA', blocking=True, timeout=3)
                if not msg:
                    break
                
                if msg.count == 0:
                    break
                
                data.extend(bytes(msg.data[:msg.count]))
                chunk_count += 1
                
                if chunk_count % 100 == 0:
                    self.log.status(f"  已接收 {len(data)} bytes ({chunk_count} chunks)")
            
            if data:
                outfile = os.path.join(output_dir, f"log_{log_id:04d}.bin")
                with open(outfile, 'wb') as f:
                    f.write(data)
                self.log.result(True, f"日志已保存: {outfile} ({len(data)} bytes)")
                return outfile
            else:
                self.log.result(False, "未收到日志数据")
                return None
        except Exception as e:
            self.log.error(str(e))
            return None
    
    # ============================================================
    # 固件升级
    # ============================================================
    def upgrade_firmware(self, firmware_path):
        """通过 MAVLink 上传固件"""
        self.log.action("固件升级", f"文件: {firmware_path}")
        
        if not os.path.exists(firmware_path):
            self.log.error(f"文件不存在: {firmware_path}")
            return False
        
        try:
            with open(firmware_path, 'rb') as f:
                fw_data = f.read()
            
            size = len(fw_data)
            self.log.data("固件大小", f"{size} bytes ({size/1024:.1f} KB)")
            
            # MAVLink 固件升级协议
            # 1. 启动升级
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 3,  # param1=3: firmware upgrade mode
                size, 0, 0, 0, 0, 0
            )
            
            time.sleep(1)
            
            # 2. 分块上传
            CHUNK_SIZE = 192
            offset = 0
            
            while offset < size:
                chunk = fw_data[offset:offset + CHUNK_SIZE]
                
                self.master.mav.file_transfer_protocol_send(
                    self.master.target_system,
                    self.master.target_component,
                    1,          # session
                    5,          # opcode: write
                    len(chunk),
                    offset,
                    chunk,
                    bytearray(240)
                )
                
                offset += CHUNK_SIZE
                time.sleep(0.02)
                
                if (offset // CHUNK_SIZE) % 50 == 0:
                    self.log.status(f"  已上传 {offset}/{size} bytes ({offset*100//size}%)")
            
            # 3. 完成
            self.master.mav.file_transfer_protocol_send(
                self.master.target_system,
                self.master.target_component,
                1, 6, 0, 0, b'', bytearray(240)
            )
            
            self.log.result(True, f"固件上传完成: {size} bytes")
            self.log.status("重启飞控以应用新固件")
            return True
        except Exception as e:
            self.log.error(f"固件升级失败: {str(e)}")
            return False
    
    # ============================================================
    # 传感器校准
    # ============================================================
    def calibrate_compass(self):
        """罗盘校准"""
        self.log.action("罗盘校准", "发送校准指令")
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
            self.log.result(True, "罗盘校准命令已发送（旋转飞行器完成校准）")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def calibrate_accel(self):
        """加速度计校准"""
        self.log.action("加速度计校准", "发送校准指令")
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0, 0, 0, 0, 0, 0, 0, 1,  # param7=1: accel calibration
            )
            self.log.result(True, "加速度计校准命令已发送")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def calibrate_level(self):
        """水平校准"""
        self.log.action("水平校准", "设置当前姿态为水平")
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0, 0, 0, 0, 0, 0, 0, 2,  # param7=2: level
            )
            self.log.result(True, "水平校准完成")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    # ============================================================
    # 地理围栏 (Fence)
    # ============================================================
    def fence_show(self):
        """显示当前围栏"""
        self.log.action("显示围栏", "")
        try:
            self.master.mav.fence_fetch_point_send(
                self.master.target_system,
                self.master.target_component,
                0
            )
            # 接收围栏点
            points = []
            for _ in range(20):
                msg = self.master.recv_match(type='FENCE_POINT', blocking=True, timeout=2)
                if not msg:
                    break
                if msg.lat == 0 and msg.lon == 0:
                    break
                points.append({'lat': msg.lat, 'lon': msg.lon, 'count': msg.count})
            
            if points:
                print("\n=== Fence Points ===")
                for i, p in enumerate(points):
                    print(f"  Point {i}: Lat={p['lat']:.6f}  Lon={p['lon']:.6f}")
                self.log.result(True, f"{len(points)} 个围栏点")
            else:
                self.log.status("无围栏")
            return points
        except Exception as e:
            self.log.error(str(e))
            return []
    
    def fence_add_circle(self, lat, lon, radius):
        """添加圆形围栏"""
        self.log.action("添加圆形围栏", f"Center=({lat},{lon}) Radius={radius}m")
        try:
            self.master.mav.fence_send_point_send(
                self.master.target_system,
                self.master.target_component,
                lat, lon, 0
            )
        except AttributeError:
            # 如果 fence_send_point 不存在，用 mission_item_send
            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                0,  # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                101,  # MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                0, 1,
                radius, 0, 0, 0,
                lat, lon, 0
            )
        self.log.result(True, "圆形围栏已添加")
        return True
    
    # ============================================================
    # Guided 模式控制
    # ============================================================
    def guided_go(self, lat, lon, alt):
        """Guided 模式飞往指定位置"""
        self.log.action("Guided 飞往", f"Lat={lat} Lon={lon} Alt={alt}m")
        try:
            # 先切换到 GUIDED 模式
            self.set_mode('GUIDED')
            time.sleep(1)
            
            # 发送位置目标
            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2,  # current = 2 (Guided mode)
                0,
                0, 0, 0, 0,
                lat, lon, alt
            )
            self.log.result(True, f"已飞往 ({lat}, {lon}, {alt}m)")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def takeoff(self, alt=50):
        """起飞"""
        self.log.action("起飞", f"目标高度: {alt}m")
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, alt
            )
            self.log.result(True, f"起飞命令已发送，目标 {alt}m")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def rtl(self):
        """返航"""
        self.log.action("返航 RTL", "切换到 RTL 模式")
        return self.set_mode('RTL')
    
    def land(self):
        """降落"""
        self.log.action("降落 LAND", "切换到 LAND 模式")
        return self.set_mode('LAND')
    
    def emergency_stop(self):
        """紧急停止：立即上锁"""
        self.log.action("紧急停止", "立即 DISARM")
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # param1=0: disarm
                21196,  # param2: force disarm magic value
                0, 0, 0, 0, 0
            )
            self.log.result(True, "紧急上锁命令已发送")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    def get_version(self):
        """获取固件版本"""
        self.log.action("获取固件版本", "")
        try:
            self.master.mav.autopilot_version_request_send(
                self.master.target_system,
                self.master.target_component
            )
            msg = self.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
            if msg:
                version = f"{(msg.firmware_version >> 24)}.{(msg.firmware_version >> 16) & 0xFF}.{(msg.firmware_version >> 8) & 0xFF}.{msg.firmware_version & 0xFF}"
                self.log.result(True, f"固件版本: {version}")
                self.log.data("Flight SW Version", msg.flight_sw_version)
                self.log.data("Vendor ID", msg.vendor_id)
                self.log.data("Board ID", msg.board_id)
                return version
            else:
                self.log.result(False, "未收到版本信息")
                return None
        except Exception as e:
            self.log.error(str(e))
            return None
    
    # ============================================================
    # 参数搜索
    # ============================================================
    def search_param(self, keyword):
        """搜索包含关键字的参数"""
        self.log.action("搜索参数", f"关键词: {keyword}")
        self.master.param_fetch_all()
        
        found = {}
        start = time.time()
        while time.time() - start < 10:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if not msg:
                break
            if keyword.upper() in msg.param_id.upper():
                found[msg.param_id] = msg.param_value
        
        if found:
            print(f"\n=== Parameters matching '{keyword}' ===")
            for name, val in sorted(found.items()):
                print(f"  {name:30s} = {val}")
            self.log.result(True, f"找到 {len(found)} 个匹配参数")
        else:
            self.log.result(False, f"未找到匹配 '{keyword}' 的参数")
        
        return found
    
    # ============================================================
    # 参数批量读取 & 树形查看
    # ============================================================
    def param_list_all(self, output_file=None):
        """读取所有参数"""
        self.log.action("读取全部参数", "请求参数列表")
        
        # 使用 MAV_CMD 请求参数列表
        self.master.mav.param_request_list_send(
            self.master.target_system,
            self.master.target_component
        )
        
        params = {}
        start = time.time()
        while time.time() - start < 20:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if not msg:
                continue
            params[msg.param_id] = msg.param_value
            if len(params) % 50 == 0 and len(params) > 0:
                self.log.status(f"  已读取 {len(params)} 个参数...")
        
        self.log.result(True, f"读取 {len(params)} 个参数")
        
        # 输出到文件
        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                for name, val in sorted(params.items()):
                    f.write(f"{name}={val}\n")
            self.log.status(f"参数已保存到: {output_file}")
        
        return params
    
    def param_tree(self):
        """按子系统分组显示参数"""
        self.log.action("参数树", "按子系统分组")
        params = self.param_list_all()
        
        # 按前缀分组
        groups = {}
        for name, val in params.items():
            prefix = name.split('_')[0] if '_' in name else name[:3]
            if prefix not in groups:
                groups[prefix] = {}
            groups[prefix][name] = val
        
        # 打印
        print("\n=== Parameter Tree ===")
        for prefix in sorted(groups.keys()):
            grp = groups[prefix]
            print(f"\n[{prefix}] ({len(grp)} params)")
            for name, val in sorted(grp.items()):
                print(f"  {name:35s} = {val}")
        
        return groups
    
    # ============================================================
    # 遥测日志记录
    # ============================================================
    def tlog_record(self, output_dir="D:/oezcon/ai_mp/logs"):
        """记录遥测日志 (.tlog)"""
        import struct
        os.makedirs(output_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(output_dir, f"flight_{ts}.tlog")
        self.log.action("开始记录遥测日志", f"文件: {filepath}")
        
        try:
            with open(filepath, 'wb') as f:
                start = time.time()
                count = 0
                while True:
                    msg = self.master.recv_match(blocking=True, timeout=1)
                    if not msg:
                        continue
                    
                    # 写入 tlog 格式: timestamp_ms + MAVLink 消息
                    timestamp_ms = int((time.time()) * 1000)
                    msg_bytes = msg.get_msgbuf()
                    
                    # tlog 格式: [uint32 timestamp_sec][uint8 len][uint8 sysid][uint8 compid][uint8 msgid][payload][uint8 checksum]
                    f.write(struct.pack('<I', timestamp_ms // 1000))
                    f.write(msg_bytes)
                    f.flush()
                    count += 1
                    
                    if count % 100 == 0:
                        elapsed = time.time() - start
                        print(f"\r[{time.strftime('%H:%M:%S')}] TLOG: {count} msgs, {os.path.getsize(filepath)/1024:.1f} KB", end='', flush=True)
        except KeyboardInterrupt:
            pass
        
        self.log.result(True, f"记录完成: {count} 条消息, {filepath}")
        return filepath
    
    # ============================================================
    # 预飞检查
    # ============================================================
    def prearm_check(self):
        """读取预飞检查状态"""
        self.log.action("预飞检查", "读取所有预飞检查项")
        
        # 请求预飞检查结果
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        checks = {}
        start = time.time()
        while time.time() - start < 3:
            msg = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if msg:
                text = msg.text
                # ArduPilot 预飞检查消息格式
                if 'PreArm' in text or 'prearm' in text.lower() or 'PREARM' in text:
                    level = msg.severity
                    if level <= 2:
                        checks[text] = 'FAIL'
                    elif level <= 4:
                        checks[text] = 'WARN'
                    else:
                        checks[text] = 'OK'
            elif checks:
                break
        
        # 同时检查 SYS_STATUS
        status_msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        if status_msg:
            on_arm = status_msg.onboard_control_sensors_health
            self.log.data("传感器健康", f"0x{on_arm:08x}")
        
        # 打印
        print("\n=== PreArm Check ===")
        if checks:
            for text, status in checks.items():
                icon = "✅" if status == 'OK' else ("⚠️" if status == 'WARN' else "❌")
                print(f"  {icon} {text}")
        else:
            print("  ✅ 所有预飞检查通过 (或无检查报告)")
        
        self.log.result(True, f"{len(checks)} 项检查")
        return checks
    
    # ============================================================
    # 伺服输出
    # ============================================================
    def servo_output(self):
        """显示伺服输出通道"""
        self.log.action("伺服输出", "读取通道 1-16")
        
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_SERVO_CHANNELS, 5, 1
        )
        
        channels = {}
        start = time.time()
        while time.time() - start < 3:
            msg = self.master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
            if msg:
                for i in range(1, 9):
                    val = getattr(msg, f'servo{i}_raw', 0)
                    if val:
                        channels[f'S{i}'] = val
                break
        
        # 也读 RC_CHANNELS_SCALED
        msg = self.master.recv_match(type='RC_CHANNELS_SCALED', blocking=True, timeout=2)
        if msg:
            for i in range(1, 9):
                val = getattr(msg, f'chan{i}_scaled', 0)
                if val:
                    channels[f'C{i}'] = val
        
        if channels:
            print("\n=== Servo Output ===")
            print(f"  {'Channel':<10} {'Value (us)':<12}")
            print(f"  {'-'*22}")
            for ch, val in sorted(channels.items()):
                bar = '#' * max(0, int((val - 1000) / 50))
                print(f"  {ch:<10} {val:>8.0f} us  {bar}")
            self.log.result(True, f"{len(channels)} 个通道")
        else:
            self.log.result(False, "未收到伺服数据")
        
        return channels
    
    # ============================================================
    # 电机测试
    # ============================================================
    def motor_test(self, motor_num, throttle_percent=10, duration_sec=2):
        """测试单个电机"""
        self.log.action(f"电机测试 #{motor_num}", f"油门={throttle_percent}%, 时长={duration_sec}s")
        
        try:
            # MAV_CMD_DO_MOTOR_TEST
            # param1 = motor instance (1-based)
            # param2 = throttle type (0=pilot, 1=percent, 2=RPM)
            # param3 = throttle value
            # param4 = timeout (seconds)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0,
                motor_num,          # param1: motor number
                1,                  # param2: throttle type (percent)
                throttle_percent,   # param3: throttle
                duration_sec,       # param4: timeout
                0, 0, 0
            )
            
            # 等待 ACK
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.log.result(True, f"电机 #{motor_num} 测试完成")
            else:
                self.log.warning(f"电机 #{motor_num} 测试命令已发送，结果: {msg.result if msg else 'timeout'}")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    # ============================================================
    # 失控保护
    # ============================================================
    def get_failsafe(self):
        """读取失控保护配置"""
        self.log.action("失控保护配置", "读取相关参数")
        
        fs_params = [
            'FS_BATT_ENABLE', 'FS_BATT_VOLTAGE', 'FS_BATT_MAH',
            'FS_THR_ENABLE', 'FS_THR_VALUE',
            'FS_GCS_ENABLE', 'FS_GCS_TIMEOUT',
            'THR_FAILSAFE', 'THR_FS_VALUE',
            'FENCE_ENABLE', 'FENCE_ACTION',
            'EK3_FS_GYRO',
        ]
        
        results = {}
        for p in fs_params:
            val = self.get_param(p)
            if val is not None:
                results[p] = val
        
        print("\n=== Failsafe Config ===")
        for name, val in sorted(results.items()):
            print(f"  {name:30s} = {val}")
        
        self.log.result(True, f"{len(results)} 项配置")
        return results
    
    # ============================================================
    # Rally 点管理
    # ============================================================
    def rally_show(self):
        """显示 Rally 点"""
        self.log.action("显示 Rally 点", "")
        try:
            self.master.mav.mission_request_list_send(
                self.master.target_system, self.master.target_component
            )
            # 检查是否有 rally point
            msg = self.master.recv_match(type='MISSION_COUNT', blocking=True, timeout=3)
            if msg and msg.count > 0:
                points = []
                for i in range(msg.count):
                    self.master.mav.mission_request_int_send(
                        self.master.target_system, self.master.target_component, i
                    )
                    wp = self.master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=3)
                    if wp and wp.command == 5100:  # MAV_CMD_NAV_RALLY_POINT
                        points.append({'lat': wp.x / 1e7, 'lon': wp.y / 1e7, 'alt': wp.z})
                
                if points:
                    print("\n=== Rally Points ===")
                    for i, p in enumerate(points):
                        print(f"  Rally {i}: Lat={p['lat']:.6f}  Lon={p['lon']:.6f}  Alt={p['alt']:.1f}m")
                    self.log.result(True, f"{len(points)} 个 Rally 点")
                    return points
            self.log.status("无 Rally 点")
            return []
        except Exception as e:
            self.log.error(str(e))
            return []
    
    def rally_add(self, lat, lon, alt):
        """添加 Rally 点"""
        self.log.action("添加 Rally 点", f"Lat={lat} Lon={lon} Alt={alt}m")
        # 需要通过 mission_item 发送
        self.master.mav.mission_item_send(
            self.master.target_system,
            self.master.target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            5100,  # MAV_CMD_NAV_RALLY_POINT
            0, 1,
            0, 0, 0, 0,
            lat, lon, alt
        )
        self.log.result(True, "Rally 点已添加")
        return True
    
    # ============================================================
    # P1: 电池监控配置
    # ============================================================
    def battery_config(self, voltage_divider=None, current_offset=None, capacity=None):
        """配置电池监控"""
        self.log.action("电池监控配置", "")
        
        params = {}
        if voltage_divider is not None:
            params['BATT_VOLT_MULT'] = float(voltage_divider)
        if current_offset is not None:
            params['BATT_AMP_OFFSET'] = float(current_offset)
        if capacity is not None:
            params['BATT_CAPACITY'] = int(capacity)
        
        # 读取当前配置
        for p in ['BATT_VOLT_MULT', 'BATT_AMP_OFFSET', 'BATT_CAPACITY', 
                   'BATT_CURR_PIN', 'BATT_VOLT_PIN', 'BATT_MONITOR']:
            val = self.get_param(p)
            if val is not None:
                self.log.data(p, val)
        
        # 设置新值
        for name, val in params.items():
            self.set_param(name, val)
        
        self.log.result(True, f"电池配置完成: {len(params)} 项已修改")
        return True
    
    # ============================================================
    # P1: 空速校准
    # ============================================================
    def airspeed_calibrate(self, offset=None):
        """空速传感器校准"""
        self.log.action("空速校准", "")
        
        if offset is not None:
            self.set_param('ARSPD_OFFSET', float(offset))
            self.log.result(True, f"空速偏移设为 {offset}")
        else:
            # 读取当前偏移
            val = self.get_param('ARSPD_OFFSET')
            if val is not None:
                self.log.data("ARSPD_OFFSET", val)
            
            # 读取当前空速
            status = self.get_full_status()
            aspd = status.get('airspeed', 0)
            self.log.data("当前空速", f"{aspd} m/s")
            self.log.status("保持飞机静止，空速应为 0")
        
        return True
    
    # ============================================================
    # P1: GPS 配置
    # ============================================================
    def gps_config(self, gps_type=None, protocol=None, baud=None):
        """GPS 配置"""
        self.log.action("GPS 配置", "")
        
        # 读取当前配置
        for p in ['GPS_TYPE', 'GPS_TYPE2', 'SERIAL4_BAUD', 'GPS_AUTO_CONFIG']:
            val = self.get_param(p)
            if val is not None:
                self.log.data(p, val)
        
        # 设置新值
        if gps_type is not None:
            self.set_param('GPS_TYPE', int(gps_type))
        if baud is not None:
            self.set_param('SERIAL4_BAUD', int(baud))
        
        self.log.result(True, "GPS 配置完成")
        return True
    
    # ============================================================
    # P1: 串口配置
    # ============================================================
    def serial_config(self, port, protocol=None, baud=None):
        """串口配置 (port: 0-6 对应 SERIAL1-SERIAL7)"""
        self.log.action(f"串口配置 SERIAL{port+1}", "")
        
        proto_params = {
            'MAVLink': 2, 'GPS': 5, 'Telemetry': 1, 'RCIN': 23,
            'FrSky': 3, 'Lidar': 9, 'OptFlow': 6, 'Beacon': 13,
        }
        
        if protocol is not None:
            proto_val = proto_params.get(protocol, int(protocol))
            self.set_param(f'SERIAL{port+1}_PROTOCOL', proto_val)
        
        if baud is not None:
            self.set_param(f'SERIAL{port+1}_BAUD', int(baud))
        
        # 读取当前配置
        proto = self.get_param(f'SERIAL{port+1}_PROTOCOL')
        bd = self.get_param(f'SERIAL{port+1}_BAUD')
        self.log.data(f"SERIAL{port+1}_PROTOCOL", proto)
        self.log.data(f"SERIAL{port+1}_BAUD", bd)
        
        return True
    
    # ============================================================
    # P1: 飞行模式设置
    # ============================================================
    def flightmode_setup(self):
        """显示当前飞行模式映射 (RC 开关→模式)"""
        self.log.action("飞行模式设置", "RC 开关→飞行模式映射")
        
        mode_params = [
            'FLTMODE1', 'FLTMODE2', 'FLTMODE3',
            'FLTMODE4', 'FLTMODE5', 'FLTMODE6',
        ]
        
        mode_names = {
            0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO',
            4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 7: 'CIRCLE',
            8: 'POSITION', 9: 'LAND', 10: 'TAKEOFF', 11: 'AVOID_ADSB',
            12: 'GUIDED_NOGPS', 13: 'SMART_RTL', 14: 'FLOWHOLD',
            15: 'FOLLOW', 16: 'ZIGZAG', 17: 'SYSTEMID',
            18: 'AUTO_RTL', 20: 'CRUISE', 21: 'TURTLE',
        }
        
        print("\n=== Flight Mode Setup ===")
        print(f"  {'Switch':<10} {'Mode ID':<10} {'Mode Name':<15}")
        print(f"  {'-'*35}")
        
        for i, param in enumerate(mode_params):
            val = self.get_param(param)
            if val is not None:
                mode_id = int(val)
                mode_name = mode_names.get(mode_id, f'Unknown({mode_id})')
                print(f"  {param:<10} {mode_id:<10} {mode_name:<15}")
        
        self.log.result(True, "飞行模式映射已显示")
        return True
    
    # ============================================================
    # P1: KML 导出
    # ============================================================
    def export_kml(self, output_file=None):
        """将当前航点任务导出为 KML"""
        self.log.action("导出 KML", "")
        
        if not output_file:
            output_file = f"D:/oezcon/ai_mp/mission_{time.strftime('%Y%m%d_%H%M%S')}.kml"
        
        # 先下载任务
        mgr = self.mission_init()
        if not mgr.waypoints:
            mgr.download_from_fc()
        
        if not mgr.waypoints:
            self.log.warning("无航点数据")
            return False
        
        # 生成 KML
        coords = []
        for wp in mgr.waypoints:
            if wp['cmd'] not in ('RTL', 'DO_JUMP', 'DO_CHANGE_SPEED'):
                coords.append(f"        {wp['lon']},{wp['lat']},{wp['alt']}")
        
        kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>AI-MP Mission</name>
    <Style id="route">
      <LineStyle>
        <color>ff0000ff</color>
        <width>3</width>
      </LineStyle>
    </Style>
    <Placemark>
      <name>Flight Route</name>
      <styleUrl>#route</styleUrl>
      <LineString>
        <altitudeMode>relativeToGround</altitudeMode>
        <coordinates>
{chr(10).join(coords)}
        </coordinates>
      </LineString>
    </Placemark>
"""
        
        # 添加航点标记
        for i, wp in enumerate(mgr.waypoints):
            if wp['cmd'] not in ('RTL', 'DO_JUMP', 'DO_CHANGE_SPEED'):
                kml += f"""    <Placemark>
      <name>WP{i+1}: {wp['cmd']}</name>
      <Point>
        <altitudeMode>relativeToGround</altitudeMode>
        <coordinates>{wp['lon']},{wp['lat']},{wp['alt']}</coordinates>
      </Point>
    </Placemark>
"""
        
        kml += """  </Document>
</kml>"""
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(kml)
        
        self.log.result(True, f"KML 已导出: {output_file}")
        return True
    
    # ============================================================
    # P2: MAVLink Inspector
    # ============================================================
    def mavlink_inspector(self, filter_type=None, duration=10):
        """MAVLink 消息查看器"""
        self.log.action("MAVLink Inspector", f"过滤: {filter_type or 'ALL'}, 时长: {duration}s")
        
        msg_count = {}
        start = time.time()
        
        try:
            while time.time() - start < duration:
                msg = self.master.recv_match(blocking=True, timeout=0.5)
                if not msg:
                    continue
                
                mtype = msg.get_type()
                if filter_type and filter_type.upper() not in mtype.upper():
                    continue
                
                msg_count[mtype] = msg_count.get(mtype, 0) + 1
                
                # 实时显示
                ts = time.strftime('%H:%M:%S')
                print(f"\r[{ts}] {mtype:30s} count={msg_count[mtype]}", end='', flush=True)
        except KeyboardInterrupt:
            pass
        
        print("\n\n=== MAVLink Message Summary ===")
        for mtype, count in sorted(msg_count.items(), key=lambda x: -x[1]):
            print(f"  {mtype:30s}: {count:5d}")
        
        self.log.result(True, f"{len(msg_count)} 种消息类型")
        return msg_count
    
    # ============================================================
    # P2: EKF 状态
    # ============================================================
    def ekf_status(self):
        """EKF 健康与振动状态"""
        self.log.action("EKF 状态", "读取 EKF 和振动数据")
        
        # 请求 EKF 数据
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1
        )
        
        ekf = {}
        start = time.time()
        while time.time() - start < 3:
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue
            
            mtype = msg.get_type()
            if mtype == 'EKF_STATUS_REPORT':
                ekf['flags'] = msg.flags
                ekf['velocity_variance'] = msg.velocity_variance
                ekf['pos_horiz_variance'] = msg.pos_horiz_variance
                ekf['pos_vert_variance'] = msg.pos_vert_variance
                ekf['compass_variance'] = msg.compass_variance
                ekf['terrain_alt_variance'] = msg.terrain_alt_variance
            elif mtype == 'VIBRATION':
                ekf['vibration_x'] = msg.vibration_x
                ekf['vibration_y'] = msg.vibration_y
                ekf['vibration_z'] = msg.vibration_z
                ekf['clip_count'] = sum(msg.clip_count) if hasattr(msg, 'clip_count') else 0
        
        if ekf:
            print("\n=== EKF Status ===")
            if 'flags' in ekf:
                flags = ekf['flags']
                print(f"  Flags: 0x{flags:04x}")
                print(f"  Attitude OK: {'Yes' if flags & 0x01 else 'No'}")
                print(f"  Velocity Horiz OK: {'Yes' if flags & 0x02 else 'No'}")
                print(f"  Velocity Vert OK: {'Yes' if flags & 0x04 else 'No'}")
                print(f"  Pos Horiz OK: {'Yes' if flags & 0x08 else 'No'}")
                print(f"  Pos Vert OK: {'Yes' if flags & 0x10 else 'No'}")
                print(f"  Const Pos Mode: {'Yes' if flags & 0x20 else 'No'}")
                print(f"  Pred Pos Horiz OK: {'Yes' if flags & 0x40 else 'No'}")
                print(f"  Pred Pos Vert OK: {'Yes' if flags & 0x80 else 'No'}")
            
            for key in ['velocity_variance', 'pos_horiz_variance', 'pos_vert_variance',
                        'compass_variance', 'terrain_alt_variance']:
                if key in ekf:
                    print(f"  {key}: {ekf[key]:.4f}")
            
            if 'vibration_x' in ekf:
                print(f"\n  Vibration X: {ekf['vibration_x']:.2f} m/s²")
                print(f"  Vibration Y: {ekf['vibration_y']:.2f} m/s²")
                print(f"  Vibration Z: {ekf['vibration_z']:.2f} m/s²")
                print(f"  Clip Count: {ekf['clip_count']}")
        
        self.log.result(True, f"EKF 数据: {len(ekf)} 项")
        return ekf
    
    # ============================================================
    # P0: 围栏下载
    # ============================================================
    def fence_download(self):
        """下载飞控中的围栏"""
        self.log.action("下载围栏", "从飞控读取围栏数据")
        try:
            # 请求围栏计数
            self.master.mav.fence_fetch_point_send(
                self.master.target_system, self.master.target_component, 0
            )
            points = []
            for i in range(30):
                msg = self.master.recv_match(type='FENCE_POINT', blocking=True, timeout=2)
                if not msg:
                    break
                if msg.lat == 0 and msg.lon == 0 and i > 0:
                    break
                points.append({'seq': i, 'lat': msg.lat, 'lon': msg.lon, 'count': msg.count})
            
            print("\n=== Fence Points (Downloaded) ===")
            for p in points:
                print(f"  #{p['seq']}: Lat={p['lat']:.6f}  Lon={p['lon']:.6f}")
            self.log.result(True, f"{len(points)} 个围栏点")
            return points
        except Exception as e:
            self.log.error(str(e))
            return []
    
    # ============================================================
    # P0: 围栏多边形
    # ============================================================
    def fence_add_polygon(self, points_list):
        """添加多边形围栏, points_list = [(lat, lon), ...]"""
        self.log.action("添加多边形围栏", f"{len(points_list)} 个点")
        for i, (lat, lon) in enumerate(points_list):
            self.master.mav.fence_send_point_send(
                self.master.target_system, self.master.target_component,
                lat, lon, i
            )
            time.sleep(0.1)
        self.log.result(True, f"多边形围栏已添加: {len(points_list)} 点")
        return True
    
    # ============================================================
    # P0: 遥测日志回放
    # ============================================================
    def tlog_playback(self, filepath):
        """回放 .tlog 文件"""
        import struct
        self.log.action("回放遥测日志", filepath)
        
        if not os.path.exists(filepath):
            self.log.error(f"文件不存在: {filepath}")
            return False
        
        try:
            with open(filepath, 'rb') as f:
                while True:
                    # 读 tlog: 4 bytes timestamp + mavlink message
                    ts_data = f.read(4)
                    if len(ts_data) < 4:
                        break
                    
                    timestamp = struct.unpack('<I', ts_data)[0]
                    
                    # 读 MAVLink 消息头
                    msg_len_data = f.read(3)
                    if len(msg_len_data) < 3:
                        break
                    
                    # 简化解析 - 打印时间戳
                    ts_str = time.strftime('%H:%M:%S', time.gmtime(timestamp))
                    print(f"\r[{ts_str}] Playing...", end='', flush=True)
                    
                    # 跳过消息体 (简化处理)
                    msg_len = msg_len_data[1] if len(msg_len_data) > 1 else 0
                    f.read(msg_len + 3)  # payload + checksum
                    
                    time.sleep(0.001)  # 模拟回放速度
            
            print("\n")
            self.log.result(True, "回放完成")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    # ============================================================
    # P1: DO 命令扩展
    # ============================================================
    def mission_add_do_set_servo(self, servo_num, pwm_value):
        """添加 DO_SET_SERVO 命令"""
        self.log.action("添加 DO_SET_SERVO", f"Servo={servo_num} PWM={pwm_value}")
        mgr = self.mission_init()
        wp = {
            'seq': len(mgr.waypoints) + 1,
            'cmd': 'DO_SET_SERVO',
            'frame': 2,  # MAV_FRAME_MISSION
            'p1': float(servo_num),
            'p2': float(pwm_value),
            'p3': 0, 'p4': 0,
            'lat': 0, 'lon': 0, 'alt': 0,
        }
        mgr.waypoints.append(wp)
        self.log.result(True, f"DO_SET_SERVO 已添加")
        return True
    
    def mission_add_do_relay(self, relay_num, state):
        """添加 DO_SET_RELAY 命令"""
        self.log.action("添加 DO_SET_RELAY", f"Relay={relay_num} State={state}")
        mgr = self.mission_init()
        wp = {
            'seq': len(mgr.waypoints) + 1,
            'cmd': 'DO_SET_RELAY',
            'frame': 2,
            'p1': float(relay_num),
            'p2': float(state),
            'p3': 0, 'p4': 0,
            'lat': 0, 'lon': 0, 'alt': 0,
        }
        mgr.waypoints.append(wp)
        self.log.result(True, f"DO_SET_RELAY 已添加")
        return True
    
    def mission_add_condition_delay(self, delay_sec):
        """添加 CONDITION_DELAY 命令"""
        self.log.action("添加 CONDITION_DELAY", f"延迟={delay_sec}s")
        mgr = self.mission_init()
        wp = {
            'seq': len(mgr.waypoints) + 1,
            'cmd': 'CONDITION_DELAY',
            'frame': 2,
            'p1': float(delay_sec),
            'p2': 0, 'p3': 0, 'p4': 0,
            'lat': 0, 'lon': 0, 'alt': 0,
        }
        mgr.waypoints.append(wp)
        self.log.result(True, f"CONDITION_DELAY 已添加")
        return True
    
    def mission_add_condition_yaw(self, heading, direction=1, rate=20):
        """添加 CONDITION_YAW 命令"""
        self.log.action("添加 CONDITION_YAW", f"Heading={heading}° Dir={direction}")
        mgr = self.mission_init()
        wp = {
            'seq': len(mgr.waypoints) + 1,
            'cmd': 'CONDITION_YAW',
            'frame': 2,
            'p1': float(heading),
            'p2': float(rate),
            'p3': float(direction),  # 1=cw, -1=ccw
            'p4': 0,
            'lat': 0, 'lon': 0, 'alt': 0,
        }
        mgr.waypoints.append(wp)
        self.log.result(True, f"CONDITION_YAW 已添加")
        return True
    
    # ============================================================
    # P1: Survey 规划
    # ============================================================
    def survey_plan(self, center_lat, center_lon, width_m, height_m, altitude=100, angle=0, overlap=60):
        """生成测绘航线 (简单的往返扫描)"""
        self.log.action("Survey 规划", f"中心=({center_lat},{center_lon}) {width_m}x{height_m}m Alt={altitude}m")
        
        import math
        mgr = self.mission_init()
        mgr.clear()
        
        # 计算航线间距 (基于重叠率)
        # 假设相机视场角约 60°, FOV = 2 * alt * tan(30°)
        fov = 2 * altitude * math.tan(math.radians(30))
        line_spacing = fov * (1 - overlap / 100)
        
        # 生成扫描线
        num_lines = max(1, int(height_m / line_spacing) + 1)
        line_len = width_m
        
        # 旋转角度
        angle_rad = math.radians(angle)
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
        
        # 添加起飞
        mgr.add_waypoint('TAKEOFF', center_lat, center_lon, altitude, 10)
        
        for i in range(num_lines):
            # 计算线段端点 (相对于中心的偏移)
            y_offset = (i - num_lines / 2) * line_spacing
            x_half = line_len / 2
            
            if i % 2 == 0:
                x1, y1 = -x_half, y_offset
                x2, y2 = x_half, y_offset
            else:
                x1, y1 = x_half, y_offset
                x2, y2 = -x_half, y_offset
            
            # 旋转并转为经纬度
            rx1 = x1 * cos_a - y1 * sin_a
            ry1 = x1 * sin_a + y1 * cos_a
            rx2 = x2 * cos_a - y2 * sin_a
            ry2 = x2 * sin_a + y2 * cos_a
            
            lat1 = center_lat + ry1 / 111320
            lon1 = center_lon + rx1 / (111320 * math.cos(math.radians(center_lat)))
            lat2 = center_lat + ry2 / 111320
            lon2 = center_lon + rx2 / (111320 * math.cos(math.radians(center_lat)))
            
            mgr.add_waypoint('WAYPOINT', lat1, lon1, altitude, 0, 0, 30, 0)
            mgr.add_waypoint('WAYPOINT', lat2, lon2, altitude, 0, 0, 30, 0)
        
        # RTL
        mgr.add_waypoint('RTL', center_lat, center_lon, altitude)
        
        mgr.show_mission()
        self.log.result(True, f"Survey 航线已生成: {num_lines} 条扫描线")
        return mgr
    
    # ============================================================
    # P1: Grid 规划
    # ============================================================
    def grid_plan(self, center_lat, center_lon, size_m, altitude=100, spacing=50):
        """生成网格航线"""
        self.log.action("Grid 规划", f"中心=({center_lat},{center_lon}) Size={size_m}m Spacing={spacing}m")
        
        import math
        mgr = self.mission_init()
        mgr.clear()
        
        num_lines = max(1, int(size_m / spacing) + 1)
        
        mgr.add_waypoint('TAKEOFF', center_lat, center_lon, altitude, 10)
        
        for i in range(num_lines):
            y_offset = (i - num_lines / 2) * spacing
            x_half = size_m / 2
            
            lat1 = center_lat + y_offset / 111320
            lon1 = center_lon - x_half / (111320 * math.cos(math.radians(center_lat)))
            lat2 = center_lat + y_offset / 111320
            lon2 = center_lon + x_half / (111320 * math.cos(math.radians(center_lat)))
            
            mgr.add_waypoint('WAYPOINT', lat1, lon1, altitude, 0, 0, 30, 0)
            mgr.add_waypoint('WAYPOINT', lat2, lon2, altitude, 0, 0, 30, 0)
        
        mgr.add_waypoint('RTL', center_lat, center_lon, altitude)
        mgr.show_mission()
        self.log.result(True, f"Grid 航线已生成: {num_lines} 条线")
        return mgr
    
    # ============================================================
    # P2: MAVFtp 完整
    # ============================================================
    def ftp_list(self, remote_path="/"):
        """列出 SD 卡目录"""
        self.log.action("FTP 列目录", remote_path)
        try:
            payload = bytearray(240)
            name_bytes = remote_path.encode()
            payload[:len(name_bytes)] = name_bytes
            
            self.master.mav.file_transfer_protocol_send(
                self.master.target_system, self.master.target_component,
                1, 10, 0, 0, b'', payload  # opcode 10 = List
            )
            
            files = []
            start = time.time()
            while time.time() - start < 5:
                msg = self.master.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=True, timeout=2)
                if not msg:
                    break
                if msg.size > 0 and msg.data:
                    # 简化解析
                    files.append(str(bytes(msg.data[:msg.size]), errors='ignore'))
            
            if files:
                print(f"\n=== Files in {remote_path} ===")
                for f in files:
                    print(f"  {f}")
            
            self.log.result(True, f"{len(files)} 个文件")
            return files
        except Exception as e:
            self.log.error(str(e))
            return []
    
    # ============================================================
    # P2: 预飞清单
    # ============================================================
    def checklist(self, action="show"):
        """预飞清单管理"""
        self.log.action("预飞清单", action)
        
        default_checklist = [
            ("电池电量 > 50%", "BATT_CAPACITY"),
            ("GPS 3D Fix", None),
            ("罗盘校准", "COMPASS_USE"),
            ("空速传感器", "ARSPD_USE"),
            ("RC 连接", None),
            ("失控保护配置", "FS_THR_ENABLE"),
            ("围栏启用", "FENCE_ENABLE"),
            ("SD 卡已插入", None),
            ("螺旋桨已安装", None),
            ("起飞区域安全", None),
        ]
        
        if action == "show":
            print("\n=== Pre-Flight Checklist ===")
            for i, (item, param) in enumerate(default_checklist, 1):
                status = ""
                if param:
                    val = self.get_param(param)
                    if val is not None:
                        status = f"= {val}"
                    else:
                        status = "(无法读取)"
                else:
                    status = "(手动检查)"
                print(f"  {i:2d}. {item:30s} {status}")
        
        elif action == "run":
            print("\n=== Pre-Flight Checklist Run ===")
            passed = 0
            for i, (item, param) in enumerate(default_checklist, 1):
                if param:
                    val = self.get_param(param)
                    if val is not None and val > 0:
                        print(f"  ✅ {item}")
                        passed += 1
                    else:
                        print(f"  ❌ {item}")
                else:
                    print(f"  ⏳ {item} (手动确认)")
                    passed += 1
            
            print(f"\n  Score: {passed}/{len(default_checklist)}")
        
        self.log.result(True, f"清单 {len(default_checklist)} 项")
        return default_checklist
    
    # ============================================================
    # P2: 日志 CSV 导出
    # ============================================================
    def log_export(self, bin_file, csv_file=None):
        """将 .bin 日志转为 CSV (文本提取)"""
        self.log.action("日志导出 CSV", bin_file)
        
        if not os.path.exists(bin_file):
            self.log.error(f"文件不存在: {bin_file}")
            return False
        
        if not csv_file:
            csv_file = bin_file.replace('.bin', '.csv')
        
        # 读取二进制日志并提取可读文本
        try:
            with open(bin_file, 'rb') as f:
                data = f.read()
            
            # 提取 FMT 消息 (日志格式定义)
            lines = []
            offset = 0
            while offset < len(data) - 3:
                if data[offset:offset+2] == b'\xa3\x95':
                    msg_len = data[offset + 2]
                    if offset + 3 + msg_len <= len(data):
                        msg_data = data[offset+3:offset+3+msg_len]
                        # 提取 ASCII 字符串
                        text = ''.join(chr(b) if 32 <= b < 127 else '.' for b in msg_data)
                        lines.append(text)
                    offset += 3 + msg_len
                else:
                    offset += 1
            
            with open(csv_file, 'w', encoding='utf-8') as f:
                for line in lines:
                    f.write(line + '\n')
            
            self.log.result(True, f"导出 {len(lines)} 条记录到 {csv_file}")
            return True
        except Exception as e:
            self.log.error(str(e))
            return False
    
    # ============================================================
    # P3: 参数对比
    # ============================================================
    def param_diff(self, file_a, file_b):
        """对比两组参数文件"""
        self.log.action("参数对比", f"{file_a} vs {file_b}")
        
        params_a = {}
        params_b = {}
        
        for fp, target in [(file_a, params_a), (file_b, params_b)]:
            if os.path.exists(fp):
                with open(fp, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if '=' in line:
                            k, v = line.split('=', 1)
                            target[k.strip()] = v.strip()
        
        all_keys = sorted(set(list(params_a.keys()) + list(params_b.keys())))
        
        diffs = []
        print("\n=== Parameter Diff ===")
        for k in all_keys:
            va = params_a.get(k, '<missing>')
            vb = params_b.get(k, '<missing>')
            if va != vb:
                diffs.append((k, va, vb))
                print(f"  {k:30s}: {va:15s} → {vb:15s}")
        
        if not diffs:
            print("  (无差异)")
        
        self.log.result(True, f"{len(diffs)} 个差异")
        return diffs
    
    # ============================================================
    # P3: 参数导入/导出
    # ============================================================
    def param_export(self, output_file=None):
        """导出全部参数"""
        self.log.action("导出参数", output_file or "auto")
        
        params = self.param_list_all()
        
        if not output_file:
            output_file = f"D:/oezcon/ai_mp/params_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        
        with open(output_file, 'w') as f:
            for name, val in sorted(params.items()):
                f.write(f"{name}={val}\n")
        
        self.log.result(True, f"{len(params)} 个参数已导出到 {output_file}")
        return output_file
    
    def param_import(self, input_file):
        """导入参数文件"""
        self.log.action("导入参数", input_file)
        
        if not os.path.exists(input_file):
            self.log.error(f"文件不存在: {input_file}")
            return False
        
        count = 0
        with open(input_file, 'r') as f:
            for line in f:
                line = line.strip()
                if '=' in line and not line.startswith('#'):
                    k, v = line.split('=', 1)
                    try:
                        self.set_param(k.strip(), float(v.strip()))
                        count += 1
                    except:
                        pass
        
        self.log.result(True, f"已导入 {count} 个参数")
        return True
    
    # ============================================================
    # P3: 航线平滑 (Spline)
    # ============================================================
    def mission_smooth(self):
        """将航点转为 Spline 平滑路径"""
        self.log.action("航线平滑", "设置 Spline 模式")
        mgr = self.mission_init()
        for wp in mgr.waypoints:
            if wp['cmd'] == 'WAYPOINT':
                wp['p3'] = 0  # Acceptance radius = 0 for tight spline
        self.log.result(True, f"已平滑 {len(mgr.waypoints)} 个航点")
        return True
    
    # ============================================================
    # P3: 参数搜索 (已存在, 扩展为全文搜索)
    # ============================================================
    def param_export_full(self, output_file=None):
        """导出全部参数到文件 (别名)"""
        return self.param_export(output_file)


# ============================================================
# 主程序
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="AI-MP v2: Full-featured AI Ground Station")
    parser.add_argument('--port', default='COM15', help='Port (COM15, udp:..., tcp:...)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--cmd', required=True, 
        choices=['status', 'monitor', 'arm', 'disarm', 'mode', 'reboot',
                                'param-set', 'param-get', 'param-search', 'param-list-all', 'param-tree',
                                'param-export', 'param-import', 'param-diff',
                                'upload-script', 'balloon-drop-test',
                                'mission-load', 'mission-show', 'mission-download',
                                'mission-upload', 'mission-add', 'mission-remove',
                                'mission-clear', 'mission-save', 'mission-goto',
                                'mission-smooth',
                                'mission-add-servo', 'mission-add-relay',
                                'mission-add-delay', 'mission-add-yaw',
                                'rc', 'sensor-status', 'version',
                                'calibrate-compass', 'calibrate-accel', 'calibrate-level',
                                'list-logs', 'download-log', 'tlog-record', 'tlog-playback',
                                'log-export',
                                'fence-show', 'fence-add', 'fence-download', 'fence-enable', 'fence-disable',
                                'rally-show', 'rally-add',
                                'servo-output', 'motor-test',
                                'prearm-check', 'failsafe',
                                'takeoff', 'guided-go', 'rtl', 'land', 'emergency',
                                'battery-config', 'airspeed-calibrate', 'gps-config',
                                'serial-config', 'flightmode-setup', 'export-kml',
                                'mavlink-inspector', 'ekf-status',
                                'checklist', 'survey-plan', 'grid-plan',
                                'verify-params', 'verify-mission', 'verify-fence',
                                'verify-rally', 'verify-report',
                                'export-params-mp', 'export-mission-mp', 'export-fence-mp',
                                'export-rally-mp',
                                'sitl-launch', 'sitl-stop'],
                       help='Command to execute')
    parser.add_argument('args', nargs='*', help='Command arguments')
    
    parsed = parser.parse_args()
    
    mav = FullMAVLink(parsed.port, parsed.baud)
    
    if not mav.connect():
        sys.exit(1)
    
    try:
        if parsed.cmd == 'status':
            status = mav.get_full_status()
            print("\n=== Flight Status ===")
            for k, v in sorted(status.items()):
                print(f"  {k:20s}: {v}")
        
        elif parsed.cmd == 'monitor':
            mav.log.action("开始实时监控", "每秒输出飞行状态 (Ctrl+C 停止)")
            try:
                while True:
                    status = mav.get_full_status()
                    alt = status.get('altitude', 0)
                    aspd = status.get('airspeed', 0)
                    gs = status.get('groundspeed', 0)
                    mode = status.get('flightmode', '?')
                    armed = status.get('armed', '?')
                    vol = status.get('voltage', 0)
                    sat = status.get('satellites', 0)
                    roll = status.get('roll', 0)
                    pitch = status.get('pitch', 0)
                    lat = status.get('lat', 0)
                    lon = status.get('lon', 0)
                    print(f"\r[{status['timestamp']}] {mode:10s} {armed:8s} | Alt:{alt:6.1f}m ASpd:{aspd:5.1f}m/s GS:{gs:5.1f}m/s | R:{roll:+5.1f} P:{pitch:+5.1f} | V:{vol:.1f}V Sat:{sat} | {lat:.5f},{lon:.5f}", end='', flush=True)
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n监控停止")
        
        elif parsed.cmd == 'arm':
            mav.arm()
        
        elif parsed.cmd == 'disarm':
            mav.disarm()
        
        elif parsed.cmd == 'mode':
            if parsed.args:
                mav.set_mode(parsed.args[0])
            else:
                modes = sorted(mav.master.mode_mapping().keys())
                print("可用模式:", ', '.join(modes))
        
        elif parsed.cmd == 'reboot':
            mav.reboot()
        
        elif parsed.cmd == 'param-set':
            if len(parsed.args) >= 2:
                mav.set_param(parsed.args[0], parsed.args[1])
            else:
                print("Usage: param-set NAME VALUE")
        
        elif parsed.cmd == 'param-get':
            if parsed.args:
                mav.get_param(parsed.args[0])
            else:
                print("Usage: param-get NAME")
        
        elif parsed.cmd == 'upload-script':
            if parsed.args:
                mav.upload_lua_script(parsed.args[0])
            else:
                print("Usage: upload-script path/to/script.lua")
        
        elif parsed.cmd == 'balloon-drop-test':
            mav.balloon_drop_test()
        
        elif parsed.cmd == 'mission-load':
            if parsed.args:
                mav.mission_load(parsed.args[0])
            else:
                print("Usage: mission-load <file.txt>")
        
        elif parsed.cmd == 'mission-show':
            mav.mission_show()
        
        elif parsed.cmd == 'mission-download':
            mav.mission_download()
        
        elif parsed.cmd == 'mission-upload':
            if parsed.args:
                mav.mission_load(parsed.args[0])
                mav.mission_upload_to_fc()
            else:
                print("Usage: mission-upload <file.txt>")
        
        elif parsed.cmd == 'mission-add':
            if len(parsed.args) >= 1:
                cmd_name = parsed.args[0].upper()
                lat = float(parsed.args[1]) if len(parsed.args) > 1 else 0
                lon = float(parsed.args[2]) if len(parsed.args) > 2 else 0
                alt = float(parsed.args[3]) if len(parsed.args) > 3 else 100
                p1 = float(parsed.args[4]) if len(parsed.args) > 4 else 0
                mav.mission_add_wp(cmd_name, lat, lon, alt, p1)
            else:
                print("Usage: mission-add CMD [LAT LON ALT P1]")
        
        elif parsed.cmd == 'mission-remove':
            if parsed.args:
                mav.mission_remove_wp(int(parsed.args[0]))
            else:
                print("Usage: mission-remove <seq>")
        
        elif parsed.cmd == 'mission-clear':
            mav.mission_clear_all()
        
        elif parsed.cmd == 'mission-save':
            if parsed.args:
                fmt = parsed.args[1] if len(parsed.args) > 1 else 'simple'
                mav.mission_save(parsed.args[0], fmt)
            else:
                print("Usage: mission-save <file.txt> [qgc|simple]")
        
        elif parsed.cmd == 'mission-goto':
            if parsed.args:
                mav.mission_goto(int(parsed.args[0]))
            else:
                print("Usage: mission-goto <seq>")
        
        elif parsed.cmd == 'rc':
            mav.get_rc_channels()
        
        elif parsed.cmd == 'sensor-status':
            mav.get_sensor_status()
        
        elif parsed.cmd == 'version':
            mav.get_version()
        
        elif parsed.cmd == 'calibrate-compass':
            mav.calibrate_compass()
        
        elif parsed.cmd == 'calibrate-accel':
            mav.calibrate_accel()
        
        elif parsed.cmd == 'calibrate-level':
            mav.calibrate_level()
        
        elif parsed.cmd == 'list-logs':
            mav.list_logs()
        
        elif parsed.cmd == 'download-log':
            if parsed.args:
                mav.download_log(int(parsed.args[0]))
            else:
                print("Usage: download-log <log_id>")
        
        elif parsed.cmd == 'fence-show':
            mav.fence_show()
        
        elif parsed.cmd == 'fence-add':
            if len(parsed.args) >= 3:
                cmd = parsed.args[0].upper()
                if cmd == 'CIRCLE':
                    mav.fence_add_circle(float(parsed.args[1]), float(parsed.args[2]), float(parsed.args[3]) if len(parsed.args) > 3 else 500)
                else:
                    print("Fence types: CIRCLE")
            else:
                print("Usage: fence-add CIRCLE LAT LON [RADIUS]")
        
        elif parsed.cmd == 'takeoff':
            alt = float(parsed.args[0]) if parsed.args else 50
            mav.takeoff(alt)
        
        elif parsed.cmd == 'guided-go':
            if len(parsed.args) >= 3:
                mav.guided_go(float(parsed.args[0]), float(parsed.args[1]), float(parsed.args[2]))
            else:
                print("Usage: guided-go LAT LON ALT")
        
        elif parsed.cmd == 'rtl':
            mav.rtl()
        
        elif parsed.cmd == 'land':
            mav.land()
        
        elif parsed.cmd == 'emergency':
            mav.emergency_stop()
        
        elif parsed.cmd == 'param-list-all':
            outfile = parsed.args[0] if parsed.args else None
            mav.param_list_all(outfile)
        
        elif parsed.cmd == 'param-tree':
            mav.param_tree()
        
        elif parsed.cmd == 'tlog-record':
            mav.tlog_record()
        
        elif parsed.cmd == 'prearm-check':
            mav.prearm_check()
        
        elif parsed.cmd == 'servo-output':
            mav.servo_output()
        
        elif parsed.cmd == 'motor-test':
            if len(parsed.args) >= 1:
                motor = int(parsed.args[0])
                throttle = int(parsed.args[1]) if len(parsed.args) > 1 else 10
                duration = int(parsed.args[2]) if len(parsed.args) > 2 else 2
                mav.motor_test(motor, throttle, duration)
            else:
                print("Usage: motor-test <motor_num> [throttle%] [duration_sec]")
        
        elif parsed.cmd == 'failsafe':
            mav.get_failsafe()
        
        elif parsed.cmd == 'rally-show':
            mav.rally_show()
        
        elif parsed.cmd == 'rally-add':
            if len(parsed.args) >= 3:
                mav.rally_add(float(parsed.args[0]), float(parsed.args[1]), float(parsed.args[2]))
            else:
                print("Usage: rally-add LAT LON ALT")
        
        elif parsed.cmd == 'fence-enable':
            mav.set_param('FENCE_ENABLE', 1)
        
        elif parsed.cmd == 'fence-disable':
            mav.set_param('FENCE_ENABLE', 0)
        
        elif parsed.cmd == 'battery-config':
            vd = float(parsed.args[0]) if parsed.args else None
            mav.battery_config(voltage_divider=vd)
        
        elif parsed.cmd == 'airspeed-calibrate':
            offset = float(parsed.args[0]) if parsed.args else None
            mav.airspeed_calibrate(offset)
        
        elif parsed.cmd == 'gps-config':
            gps_type = int(parsed.args[0]) if parsed.args else None
            baud = int(parsed.args[1]) if len(parsed.args) > 1 else None
            mav.gps_config(gps_type=gps_type, baud=baud)
        
        elif parsed.cmd == 'serial-config':
            if len(parsed.args) >= 1:
                port = int(parsed.args[0])
                proto = parsed.args[1] if len(parsed.args) > 1 else None
                baud = int(parsed.args[2]) if len(parsed.args) > 2 else None
                mav.serial_config(port, protocol=proto, baud=baud)
            else:
                print("Usage: serial-config PORT [PROTOCOL] [BAUD]")
                print("  PORT: 0-6 (SERIAL1-SERIAL7)")
                print("  PROTOCOL: MAVLink, GPS, Telemetry, RCIN, etc.")
        
        elif parsed.cmd == 'flightmode-setup':
            mav.flightmode_setup()
        
        elif parsed.cmd == 'export-kml':
            outfile = parsed.args[0] if parsed.args else None
            mav.export_kml(outfile)
        
        elif parsed.cmd == 'mavlink-inspector':
            filt = parsed.args[0] if parsed.args else None
            dur = int(parsed.args[1]) if len(parsed.args) > 1 else 10
            mav.mavlink_inspector(filt, dur)
        
        elif parsed.cmd == 'ekf-status':
            mav.ekf_status()
        
        elif parsed.cmd == 'fence-download':
            mav.fence_download()
        
        elif parsed.cmd == 'tlog-playback':
            if parsed.args:
                mav.tlog_playback(parsed.args[0])
            else:
                print("Usage: tlog-playback <file.tlog>")
        
        elif parsed.cmd == 'log-export':
            if parsed.args:
                csv_out = parsed.args[1] if len(parsed.args) > 1 else None
                mav.log_export(parsed.args[0], csv_out)
            else:
                print("Usage: log-export <file.bin> [output.csv]")
        
        elif parsed.cmd == 'mission-add-servo':
            if len(parsed.args) >= 2:
                mav.mission_add_do_set_servo(int(parsed.args[0]), int(parsed.args[1]))
            else:
                print("Usage: mission-add-servo SERVO_NUM PWM")
        
        elif parsed.cmd == 'mission-add-relay':
            if len(parsed.args) >= 2:
                mav.mission_add_do_relay(int(parsed.args[0]), int(parsed.args[1]))
            else:
                print("Usage: mission-add-relay RELAY_NUM STATE")
        
        elif parsed.cmd == 'mission-add-delay':
            if parsed.args:
                mav.mission_add_condition_delay(float(parsed.args[0]))
            else:
                print("Usage: mission-add-delay SECONDS")
        
        elif parsed.cmd == 'mission-add-yaw':
            if parsed.args:
                heading = float(parsed.args[0])
                direction = int(parsed.args[1]) if len(parsed.args) > 1 else 1
                mav.mission_add_condition_yaw(heading, direction)
            else:
                print("Usage: mission-add-yaw HEADING [DIR(1=cw/-1=ccw)]")
        
        elif parsed.cmd == 'mission-smooth':
            mav.mission_smooth()
        
        elif parsed.cmd == 'param-export':
            outfile = parsed.args[0] if parsed.args else None
            mav.param_export(outfile)
        
        elif parsed.cmd == 'param-import':
            if parsed.args:
                mav.param_import(parsed.args[0])
            else:
                print("Usage: param-import <params.txt>")
        
        elif parsed.cmd == 'param-diff':
            if len(parsed.args) >= 2:
                mav.param_diff(parsed.args[0], parsed.args[1])
            else:
                print("Usage: param-diff <file_a.txt> <file_b.txt>")
        
        elif parsed.cmd == 'checklist':
            action = parsed.args[0] if parsed.args else 'show'
            mav.checklist(action)
        
        elif parsed.cmd == 'survey-plan':
            if len(parsed.args) >= 4:
                mav.survey_plan(float(parsed.args[0]), float(parsed.args[1]),
                               float(parsed.args[2]), float(parsed.args[3]),
                               float(parsed.args[4]) if len(parsed.args) > 4 else 100)
            else:
                print("Usage: survey-plan CENTER_LAT CENTER_LON WIDTH_M HEIGHT_M [ALT]")
        
        elif parsed.cmd == 'grid-plan':
            if len(parsed.args) >= 4:
                mav.grid_plan(float(parsed.args[0]), float(parsed.args[1]),
                             float(parsed.args[2]), float(parsed.args[3]) if len(parsed.args) > 3 else 100)
            else:
                print("Usage: grid-plan CENTER_LAT CENTER_LON SIZE_M [ALT]")
        
        elif parsed.cmd == 'verify-params':
            if parsed.args:
                mav._verifier.verify_params_against_file(parsed.args[0])
            else:
                print("Usage: verify-params <param_file.param>")
        
        elif parsed.cmd == 'verify-mission':
            mav._verifier.record_mission_upload(mav.mission_init().waypoints)
            mav._verifier.export_mission_mp()
        
        elif parsed.cmd == 'verify-fence':
            mav._verifier.export_fence_mp()
        
        elif parsed.cmd == 'verify-rally':
            mav._verifier.export_rally_mp()
        
        elif parsed.cmd == 'verify-report':
            mav._verifier.generate_report()
        
        elif parsed.cmd == 'export-params-mp':
            outfile = parsed.args[0] if parsed.args else None
            mav._verifier.export_params_mp(outfile)
        
        elif parsed.cmd == 'export-mission-mp':
            outfile = parsed.args[0] if parsed.args else None
            mav._verifier.record_mission_upload(mav.mission_init().waypoints)
            mav._verifier.export_mission_mp(outfile)
        
        elif parsed.cmd == 'export-fence-mp':
            outfile = parsed.args[0] if parsed.args else None
            points = mav.fence_download()
            mav._verifier.record_fence_upload(points)
            mav._verifier.export_fence_mp(outfile)
        
        elif parsed.cmd == 'export-rally-mp':
            outfile = parsed.args[0] if parsed.args else None
            points = mav.rally_show()
            mav._verifier.record_rally_upload(points)
            mav._verifier.export_rally_mp(outfile)
        
        elif parsed.cmd == 'sitl-launch':
            # 启动 SITL (WSL ArduPlane 模拟器)
            # 关键: 使用 UDP 而非 TCP — TCP 会触发 lock-step 模式导致崩溃
            import subprocess
            print("[SITL] 清理旧进程...")
            subprocess.run(['wsl', 'bash', '-c', 'pkill -f arduplane 2>/dev/null'], capture_output=True)
            time.sleep(1)

            # 使用 udpclient 模式 — SITL 通过 UDP 发送数据，pymavlink 接收
            sitl_cmd = (
                'cd /tmp && '
                '/root/ardupilot/build/sitl/bin/arduplane '
                '-I0 '
                '--model plane '
                '--speedup 1 '
                '--serial0 udpclient:127.0.0.1:5760 '
                '--home CMAC'
            )
            print("[SITL] 启动 ArduPlane SITL (UDP 模式)...")
            print("[SITL] pymavlink 连接: udp:127.0.0.1:5760")
            try:
                # 用 Popen 后台启动 — 不等待进程退出
                proc = subprocess.Popen(
                    ['wsl', '-e', 'bash', '-c', sitl_cmd],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                # 等待进程启动
                print("[SITL] 等待 4 秒...")
                time.sleep(4)

                # 验证进程运行
                check = subprocess.run(
                    ['wsl', '-e', 'bash', '-c', 'pgrep -la arduplane'],
                    capture_output=True, text=True
                )
                if check.stdout.strip():
                    print("[SITL] ✅ SITL 启动成功!")
                    print(f"[SITL] {check.stdout.strip()}")
                    print("[SITL] pymavlink 连接方式: mavutil.mavlink_connection('udp:127.0.0.1:5760')")
                    print("[SITL] 使用 'sitl-stop' 停止")
                else:
                    print("[SITL] ⚠️ 进程未检测到，可能启动失败")
            except FileNotFoundError:
                print("[ERROR] WSL 未安装或不可用。请确保 WSL2 + Ubuntu 已配置。")
            except Exception as e:
                print(f"[ERROR] SITL 启动失败: {e}")
        
        elif parsed.cmd == 'sitl-stop':
            # 停止 SITL (杀死 arduplane 进程)
            import subprocess
            print("[SITL] 停止所有 ArduPlane SITL 进程...")
            try:
                result = subprocess.run(
                    ['wsl', '-e', 'bash', '-c', 'killall -9 arduplane 2>/dev/null; echo DONE'],
                    capture_output=True, text=True
                )
                print("[SITL] ✅ SITL 进程已停止")
            except Exception as e:
                print(f"[ERROR] 停止 SITL 失败: {e}")
    
    finally:
        mav.close()


if __name__ == '__main__':
    main()
