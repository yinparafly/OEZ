"""
AI-MP v3: 写入验证系统
所有写入飞控的数据都自动记录，导出为 Mission Planner 兼容格式，
让用户可以用 MP 打开这些文件进行验证。

注意：Nora 的 ArduPilot 固件不响应 param_request_list 和 FTP，
因此验证采用"记录写入 → 导出文件 → MP 手动验证"的模式。

MP 兼容格式:
  - .param: PARAM_NAME\tVALUE\n (参数文件)
  - .waypoints: QGC WPL 110 (航点文件)
  - .fence: 围栏点文件
  - .rally: Rally 点文件
  - .verify.txt: 写入记录报告
"""

import os
import time


class WriteVerifier:
    """写入验证器：记录写入→导出MP兼容文件"""
    
    STATE_FILE = "D:/oezcon/ai_mp/verify/write_state.json"
    
    def __init__(self, mav_link, export_dir="D:/oezcon/ai_mp/verify"):
        self.mav = mav_link
        self.log = mav_link.log
        self.export_dir = export_dir
        os.makedirs(export_dir, exist_ok=True)
        self.session_ts = time.strftime('%Y%m%d_%H%M%S')
        self.verify_report = []
        self.written_params = {}
        self.written_mission = []
        self.written_fence = []
        self.written_rally = []
        # 加载已有状态
        self._load_state()
    
    def _load_state(self):
        """从文件加载已有写入记录"""
        if os.path.exists(self.STATE_FILE):
            try:
                import json
                with open(self.STATE_FILE, 'r') as f:
                    state = json.load(f)
                self.written_params = state.get('params', {})
                self.written_mission = state.get('mission', [])
                self.written_fence = state.get('fence', [])
                self.written_rally = state.get('rally', [])
                self.log.status(f"  已加载 {len(self.written_params)} 个参数, {len(self.written_mission)} 个航点记录")
            except:
                pass
    
    def _save_state(self):
        """保存写入记录到文件"""
        try:
            import json
            state = {
                'params': self.written_params,
                'mission': self.written_mission,
                'fence': self.written_fence,
                'rally': self.written_rally,
                'updated': time.strftime('%Y-%m-%d %H:%M:%S'),
            }
            os.makedirs(os.path.dirname(self.STATE_FILE), exist_ok=True)
            with open(self.STATE_FILE, 'w') as f:
                json.dump(state, f, indent=2)
        except Exception as e:
            self.log.warning(f"保存状态失败: {e}")
    
    def _report_line(self, category, item, written, readback, match):
        """记录验证结果"""
        status = "MATCH" if match else "RECORDED"
        line = f"[{category}] {item}: written={written}, readback={readback}, {status}"
        self.verify_report.append(line)
        if match:
            self.log.status(f"  OK {item}: {written} == {readback}")
        else:
            self.log.status(f"  RECORDED {item}: {written} (readback={readback})")
        return match
    
    # ============================================================
    # 参数验证
    # ============================================================
    def record_param_set(self, name, value):
        """记录参数写入"""
        self.written_params[name] = float(value)
        self._save_state()
        self.log.status(f"  记录参数: {name} = {value}")
    
    def export_params_mp(self, output_file=None):
        """导出所有写入的参数为 MP 兼容 .param 格式"""
        self.log.action("导出参数 (MP格式)", f"{len(self.written_params)} 个参数")
        
        if not self.written_params:
            self.log.warning("无参数写入记录")
            return None
        
        if not output_file:
            output_file = os.path.join(self.export_dir, f"params_{self.session_ts}.param")
        
        # MP .param 格式: 每行 "PARAM_NAME\tVALUE\n"
        with open(output_file, 'w', encoding='utf-8') as f:
            for name, val in sorted(self.written_params.items()):
                f.write(f"{name}\t{val}\n")
        
        self.log.result(True, f"已导出 {len(self.written_params)} 个参数到 {output_file}")
        self.log.status("用 MP 打开: Config > Full Parameter List > File > Open")
        return output_file
    
    def verify_params_against_file(self, param_file):
        """将写入记录与文件中的参数对比"""
        self.log.action("参数对比", f"写入记录 vs {param_file}")
        
        # 读取文件参数
        file_params = {}
        if os.path.exists(param_file):
            with open(param_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if '\t' in line:
                        k, v = line.split('\t', 1)
                        file_params[k.strip()] = v.strip()
                    elif '=' in line:
                        k, v = line.split('=', 1)
                        file_params[k.strip()] = v.strip()
        
        # 对比
        diffs = []
        for name in sorted(self.written_params.keys()):
            written_val = str(self.written_params[name])
            file_val = file_params.get(name, '<MISSING IN FILE>')
            
            if written_val != file_val:
                diffs.append((name, written_val, file_val))
                self.log.warning(f"  DIFF: {name}: 写入={written_val} 文件={file_val}")
            else:
                self.log.status(f"  MATCH: {name} = {written_val}")
        
        if diffs:
            self.log.result(False, f"发现 {len(diffs)} 个差异")
        else:
            self.log.result(True, "所有参数与文件一致")
        
        # 导出对比报告
        report_file = os.path.join(self.export_dir, f"param_diff_{self.session_ts}.txt")
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(f"AI-MP Parameter Verification Report\n")
            f.write(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"File: {param_file}\n")
            f.write(f"Written Params: {len(self.written_params)}\n")
            f.write(f"Differences: {len(diffs)}\n\n")
            if diffs:
                f.write("NAME\tWRITTEN\tFILE_VALUE\n")
                for name, wv, fv in diffs:
                    f.write(f"{name}\t{wv}\t{fv}\n")
            else:
                f.write("ALL MATCHED\n")
        
        self.log.status(f"对比报告: {report_file}")
        return diffs
    
    # ============================================================
    # 航点任务验证
    # ============================================================
    def record_mission_upload(self, waypoints):
        """记录上传的航点"""
        self.written_mission = []
        for wp in waypoints:
            self.written_mission.append({
                'seq': wp.get('seq', len(self.written_mission) + 1),
                'cmd': wp['cmd'],
                'lat': wp.get('lat', 0),
                'lon': wp.get('lon', 0),
                'alt': wp.get('alt', 0),
                'p1': wp.get('p1', 0),
                'p2': wp.get('p2', 0),
                'p3': wp.get('p3', 0),
                'p4': wp.get('p4', 0),
                'frame': wp.get('frame', 3),
            })
        self._save_state()
        self.log.status(f"  记录 {len(self.written_mission)} 个航点")
    
    def export_mission_mp(self, output_file=None):
        """导出航点为 MP 兼容 .waypoints 格式"""
        if not self.written_mission:
            self.log.warning("无航点写入记录")
            return None
        
        if not output_file:
            output_file = os.path.join(self.export_dir, f"mission_{self.session_ts}.waypoints")
        
        cmd_codes = {
            'WAYPOINT': 16, 'LOITER_TIME': 19, 'LAND': 21, 'TAKEOFF': 22,
            'RTL': 20, 'DO_JUMP': 177, 'DO_CHANGE_SPEED': 178, 'DO_SET_HOME': 179,
            'DO_SET_SERVO': 183, 'DO_SET_RELAY': 181, 'CONDITION_DELAY': 112,
            'CONDITION_YAW': 115, 'LOITER_UNLIM': 5, 'LOITER_TURNS': 18,
        }
        
        lines = ["QGC WPL 110\n"]
        for i, wp in enumerate(self.written_mission):
            seq = wp.get('seq', i)
            cmd_code = cmd_codes.get(wp['cmd'], 0)
            lat = wp.get('lat', 0)
            lon = wp.get('lon', 0)
            alt = wp.get('alt', 0)
            p1 = wp.get('p1', 0)
            p2 = wp.get('p2', 0)
            p3 = wp.get('p3', 0)
            p4 = wp.get('p4', 0)
            frame = wp.get('frame', 3)
            
            lines.append(f"{seq}\t1\t{frame}\t{cmd_code}\t{p1}\t{p2}\t{p3}\t{p4}\t{lat}\t{lon}\t{alt}\t1\n")
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.writelines(lines)
        
        self.log.result(True, f"已导出 {len(self.written_mission)} 个航点到 {output_file}")
        self.log.status("用 MP 打开: Flight Plan > File > Open")
        return output_file
    
    # ============================================================
    # 围栏验证
    # ============================================================
    def record_fence_upload(self, points):
        """记录围栏点"""
        self.written_fence = points[:]
        self._save_state()
        self.log.status(f"  记录 {len(self.written_fence)} 个围栏点")
    
    def export_fence_mp(self, output_file=None):
        """导出围栏为 MP 兼容格式"""
        if not self.written_fence:
            self.log.warning("无围栏写入记录")
            return None
        
        if not output_file:
            output_file = os.path.join(self.export_dir, f"fence_{self.session_ts}.txt")
        
        with open(output_file, 'w', encoding='utf-8') as f:
            for p in self.written_fence:
                if isinstance(p, dict):
                    f.write(f"{p['lat']}\t{p['lon']}\n")
                else:
                    f.write(f"{p[0]}\t{p[1]}\n")
        
        self.log.result(True, f"已导出 {len(self.written_fence)} 个围栏点到 {output_file}")
        self.log.status("用 MP 打开: Flight Plan > Geofence > File > Open")
        return output_file
    
    # ============================================================
    # Rally 点验证
    # ============================================================
    def record_rally_upload(self, points):
        """记录 Rally 点"""
        self.written_rally = points[:]
        self._save_state()
        self.log.status(f"  记录 {len(self.written_rally)} 个 Rally 点")
    
    def export_rally_mp(self, output_file=None):
        """导出 Rally 为 MP 兼容格式"""
        if not self.written_rally:
            self.log.warning("无 Rally 写入记录")
            return None
        
        if not output_file:
            output_file = os.path.join(self.export_dir, f"rally_{self.session_ts}.txt")
        
        with open(output_file, 'w', encoding='utf-8') as f:
            for p in self.written_rally:
                if isinstance(p, dict):
                    f.write(f"{p['lat']}\t{p['lon']}\t{p.get('alt', 0)}\n")
                else:
                    f.write(f"{p[0]}\t{p[1]}\t{p[2] if len(p) > 2 else 0}\n")
        
        self.log.result(True, f"已导出 {len(self.written_rally)} 个 Rally 点到 {output_file}")
        self.log.status("用 MP 打开: Flight Plan > Rally Points > File > Open")
        return output_file
    
    # ============================================================
    # 完整验证报告
    # ============================================================
    def generate_report(self):
        """生成完整验证报告"""
        report_file = os.path.join(self.export_dir, f"verify_report_{self.session_ts}.txt")
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write("AI-MP Write Verification Report\n")
            f.write(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 60 + "\n\n")
            
            f.write(f"Written Parameters: {len(self.written_params)}\n")
            f.write(f"Written Waypoints: {len(self.written_mission)}\n")
            f.write(f"Written Fence Points: {len(self.written_fence)}\n")
            f.write(f"Written Rally Points: {len(self.written_rally)}\n\n")
            
            if self.written_params:
                f.write("--- Written Parameters ---\n")
                for name, val in sorted(self.written_params.items()):
                    f.write(f"  {name} = {val}\n")
                f.write("\n")
            
            if self.written_mission:
                f.write("--- Written Waypoints ---\n")
                for wp in self.written_mission:
                    f.write(f"  #{wp.get('seq', '?')} {wp['cmd']} Lat={wp.get('lat',0)} Lon={wp.get('lon',0)} Alt={wp.get('alt',0)}\n")
                f.write("\n")
            
            if self.written_fence:
                f.write("--- Written Fence Points ---\n")
                for p in self.written_fence:
                    f.write(f"  Lat={p.get('lat',0) if isinstance(p, dict) else p[0]} Lon={p.get('lon',0) if isinstance(p, dict) else p[1]}\n")
                f.write("\n")
            
            if self.written_rally:
                f.write("--- Written Rally Points ---\n")
                for p in self.written_rally:
                    f.write(f"  Lat={p.get('lat',0) if isinstance(p, dict) else p[0]} Lon={p.get('lon',0) if isinstance(p, dict) else p[1]} Alt={p.get('alt',0) if isinstance(p, dict) else (p[2] if len(p) > 2 else 0)}\n")
                f.write("\n")
            
            f.write("=" * 60 + "\n")
            f.write("MP Verification Instructions:\n")
            f.write("  1. Open Mission Planner\n")
            f.write("  2. Connect to Nora via COM16\n")
            f.write("  3. For params: Config > Full Parameter List > File > Open > select .param file\n")
            f.write("  4. For mission: Flight Plan > File > Open > select .waypoints file\n")
            f.write("  5. For fence: Flight Plan > Geofence > File > Open > select .txt file\n")
            f.write("  6. For rally: Flight Plan > Rally Points > File > Open > select .txt file\n")
            f.write("  7. Compare displayed values with FC current values\n")
            f.write("=" * 60 + "\n")
        
        self.log.result(True, f"验证报告: {report_file}")
        return report_file
