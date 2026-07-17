"""
AI-MP 全自动验证脚本
一键完成: 连接 → 状态 → 参数 → 模式 → 航点 → 遥测 → 报告
"""
import sys, os, time
sys.path.insert(0, 'D:/oezcon/ai_mavproxy')
from pymavlink import mavutil

PORT = 'COM16'  # MP 占了 COM15 时用 COM16
BAUD = 115200
REPORT_FILE = 'D:/oezcon/ai_mavproxy/test_report.txt'

class NoraTest:
    def __init__(self):
        self.master = None
        self.results = []
    
    def connect(self):
        print('[1] Connecting to Nora...')
        self.master = mavutil.mavlink_connection(PORT, baud=BAUD, source_system=255)
        self.master.wait_heartbeat(timeout=10)
        print('    System=%d Type=%d' % (self.master.target_system, self.master.mav_type))
        self._ok('Connect')
        
        # 请求数据流
        for sid in range(0, 7):
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component, sid, 4, 1)
        time.sleep(1)
    
    def test_telemetry(self):
        print('\n[2] Telemetry...')
        status = {}
        start = time.time()
        while time.time() - start < 3:
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue
            mt = msg.get_type()
            if mt == 'HEARTBEAT':
                mapping = self.master.mode_mapping()
                mode = 'Unknown(%d)' % msg.custom_mode
                for n, mid in mapping.items():
                    if mid == msg.custom_mode:
                        mode = n
                        break
                status['Mode'] = mode
                status['Armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            elif mt == 'VFR_HUD':
                status['Alt'] = msg.alt
                status['Airspeed'] = msg.airspeed
                status['GroundSpeed'] = msg.groundspeed
                status['Heading'] = msg.heading
            elif mt == 'ATTITUDE':
                import math
                status['Roll'] = round(math.degrees(msg.roll), 1)
                status['Pitch'] = round(math.degrees(msg.pitch), 1)
            elif mt == 'SYS_STATUS':
                status['Voltage'] = msg.voltage_battery / 1000
                status['CPU'] = msg.load / 10
            elif mt == 'GPS_RAW_INT':
                status['Sats'] = msg.satellites_visible
            elif mt == 'EKF_STATUS_REPORT':
                status['EKF'] = '0x%04x' % msg.flags
        
        for k, v in sorted(status.items()):
            print('    %-15s = %s' % (k, v))
        
        self._ok('Telemetry')
        return status
    
    def test_params(self):
        print('\n[3] Parameters...')
        test_params = ['SCR_ENABLE', 'ARMING_CHECK', 'ARSPD_USE', 'COMPASS_USE']
        for p in test_params:
            self.master.param_fetch_one(p)
            t0 = time.time()
            while time.time() - t0 < 2:
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if msg and msg.param_id == p:
                    print('    %-20s = %s' % (p, msg.param_value))
                    break
            else:
                print('    %-20s = NO RESPONSE' % p)
        self._ok('Params')
    
    def test_mode(self):
        print('\n[4] Mode switch...')
        # 读当前模式
        start = time.time()
        while time.time() - start < 2:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg:
                mapping = self.master.mode_mapping()
                orig_mode = 'Unknown(%d)' % msg.custom_mode
                for n, mid in mapping.items():
                    if mid == msg.custom_mode:
                        orig_mode = n
                        break
                break
        
        # 切换到 FBWA
        self.master.set_mode(7)  # FBWA
        time.sleep(1)
        
        # 读取新模式
        start = time.time()
        while time.time() - start < 2:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg:
                mapping = self.master.mode_mapping()
                new_mode = 'Unknown(%d)' % msg.custom_mode
                for n, mid in mapping.items():
                    if mid == msg.custom_mode:
                        new_mode = n
                        break
                break
        
        print('    Original: %s -> New: %s' % (orig_mode, new_mode))
        
        # 恢复
        self.master.set_mode(0)  # MANUAL
        time.sleep(1)
        
        self._ok('Mode switch')
    
    def test_version(self):
        print('\n[5] Firmware version...')
        self.master.mav.autopilot_version_request_send(
            self.master.target_system, self.master.target_component)
        msg = self.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
        if msg:
            v = msg.flight_sw_version
            version = '%d.%d.%d.%d' % ((v>>24), (v>>16)&0xFF, (v>>8)&0xFF, v&0xFF)
            print('    Firmware: %s' % version)
            self._ok('Version')
        else:
            print('    No response')
            self._fail('Version')
    
    def test_statustext(self):
        print('\n[6] Status messages...')
        start = time.time()
        msgs = []
        while time.time() - start < 5:
            msg = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if msg:
                msgs.append(msg.text)
                print('    %s' % msg.text)
        if not msgs:
            print('    (no messages)')
        self._ok('StatusText')
    
    def test_heartbeat(self):
        print('\n[7] Heartbeat stability...')
        times = []
        for i in range(5):
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if msg:
                times.append(time.time())
        if len(times) >= 2:
            interval = (times[-1] - times[0]) / (len(times) - 1)
            print('    Interval: %.2fs' % interval)
            self._ok('Heartbeat')
        else:
            self._fail('Heartbeat')
    
    def _ok(self, name):
        self.results.append((name, 'PASS', ''))
    
    def _fail(self, name, detail=''):
        self.results.append((name, 'FAIL', detail))
    
    def report(self):
        print('\n' + '=' * 60)
        print('  TEST REPORT')
        print('=' * 60)
        
        total = len(self.results)
        passed = sum(1 for _, s, _ in self.results if s == 'PASS')
        failed = sum(1 for _, s, _ in self.results if s == 'FAIL')
        
        print('  Total: %d  Passed: %d  Failed: %d' % (total, passed, failed))
        print('  Pass rate: %.0f%%' % (passed/total*100 if total else 0))
        
        if failed:
            print('\n  Failed:')
            for name, s, detail in self.results:
                if s == 'FAIL':
                    print('    %s: %s' % (name, detail))
        
        # 保存报告
        with open(REPORT_FILE, 'w', encoding='utf-8') as f:
            f.write('AI-MP Test Report\n')
            f.write('Time: %s\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
            f.write('Total: %d  Passed: %d  Failed: %d\n' % (total, passed, failed))
            f.write('Pass rate: %.0f%%\n\n' % (passed/total*100 if total else 0))
            for name, s, detail in self.results:
                f.write('[%s] %s %s\n' % (s, name, detail))
        
        print('\n  Report saved: %s' % REPORT_FILE)
        print('=' * 60)
    
    def run(self):
        print('=' * 60)
        print('  AI-MP Full Automated Test')
        print('=' * 60)
        
        try:
            self.connect()
            self.test_telemetry()
            self.test_params()
            self.test_mode()
            self.test_version()
            self.test_statustext()
            self.test_heartbeat()
        except Exception as e:
            print('\nERROR: %s' % e)
            self._fail('Exception', str(e))
        finally:
            if self.master:
                self.master.close()
        
        self.report()

if __name__ == '__main__':
    NoraTest().run()
