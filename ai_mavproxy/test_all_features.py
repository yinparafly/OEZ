"""
AI-MP 全功能验证测试 (无 ESP32)
所有 COM15 连接的功能逐项测试
"""

import sys, os, time
sys.path.insert(0, 'D:/oezcon/ai_mavproxy')
sys.path.insert(0, 'D:/oezcon/ai_mp')
from core.connection import Connection
from modules.param_manager import ParamManager
from modules.flight_control import FlightControl
from modules.telemetry import Telemetry
from mission import MissionManager

PORT = 'COM15'
results = []

def test(name, func):
    """执行测试并记录结果"""
    try:
        result = func()
        status = "PASS" if result else "FAIL"
        results.append((name, status, ""))
        print(f"  [{'OK' if result else 'FAIL'}] {name}")
        return result
    except Exception as e:
        results.append((name, "ERROR", str(e)))
        print(f"  [ERR] {name}: {e}")
        return False

def main():
    print("=" * 60)
    print("  AI-MP Full Feature Test (No ESP32)")
    print("=" * 60)
    
    # === Phase 1: 连接 ===
    print("\n--- Phase 1: Connection ---")
    conn = Connection(PORT, 115200)
    test("Connect to Nora", conn.connect)
    
    if not conn.connected:
        print("Cannot connect, aborting")
        return
    
    params = ParamManager(conn)
    fc = FlightControl(conn)
    tel = Telemetry(conn)
    tel.request_streams()
    
    # === Phase 2: 遥测 ===
    print("\n--- Phase 2: Telemetry ---")
    
    def test_telemetry():
        status = tel.get_full_status(timeout=3)
        required = ['flightmode', 'armed', 'altitude', 'airspeed', 'groundspeed',
                    'roll', 'pitch', 'yaw', 'voltage', 'cpu_load']
        missing = [k for k in required if k not in status]
        if missing:
            print(f"    Missing fields: {missing}")
            return False
        print(f"    Mode={status['flightmode']}, Alt={status['altitude']}m, Armed={status['armed']}")
        return True
    
    test("Telemetry fields", test_telemetry)
    
    # === Phase 3: 参数管理 ===
    print("\n--- Phase 3: Parameter Management ---")
    
    def test_param_get():
        val = params.get('SCR_ENABLE')
        ok = val is not None
        if ok:
            print(f"    SCR_ENABLE = {val}")
        return ok
    test("Param get", test_param_get)
    
    def test_param_set():
        original = params.get('SCR_HEAP_SIZE')
        params.set('SCR_HEAP_SIZE', 4096)
        time.sleep(0.3)
        val = params.get('SCR_HEAP_SIZE')
        params.set('SCR_HEAP_SIZE', original)
        return val == 4096
    test("Param set + readback", test_param_set)
    
    def test_param_search():
        results = params.search('ARM')
        return len(results) > 0
    test("Param search", test_param_search)
    
    def test_param_export():
        count = params.export_mp('D:/oezcon/ai_mavproxy/test_export.param')
        return count > 500
    test("Param export (>500 params)", test_param_export)
    
    def test_param_import():
        # 导出再导入验证
        params.export_mp('D:/oezcon/ai_mavproxy/test_roundtrip.param')
        count = params.import_mp('D:/oezcon/ai_mavproxy/test_roundtrip.param')
        return count > 500
    test("Param import roundtrip", test_param_import)
    
    # === Phase 4: 飞行控制 ===
    print("\n--- Phase 4: Flight Control ---")
    
    def test_get_mode():
        mode = fc.get_mode()
        print(f"    Current mode: {mode}")
        return mode is not None
    test("Get mode", test_get_mode)
    
    def test_set_mode():
        original = fc.get_mode()
        fc.set_mode('MANUAL')
        time.sleep(1)
        new_mode = fc.get_mode()
        return new_mode == 'MANUAL'
    test("Set mode MANUAL", test_set_mode)
    
    # === Phase 5: Mission ===
    print("\n--- Phase 5: Mission ---")
    
    def test_mission():
        # 添加航点
        mgr = MissionManager(conn.master, None)
        mgr.add_waypoint('TAKEOFF', 37.5, 127.0, 100, 10)
        mgr.add_waypoint('WAYPOINT', 37.55, 127.05, 100)
        mgr.add_waypoint('RTL')
        
        # 上传
        upload_ok = mgr.upload_to_fc()
        print(f"    Uploaded {len(mgr.waypoints)} waypoints, ACK received")
        return upload_ok
    test("Mission upload", test_mission)
    
    # === Phase 6: 固件信息 ===
    print("\n--- Phase 6: System Info ---")
    
    def test_version():
        conn.master.mav.autopilot_version_request_send(
            conn.master.target_system, conn.master.target_component
        )
        msg = conn.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
        if msg:
            v = msg.flight_sw_version
            version = f"{(v>>24)}.{(v>>16)&0xFF}.{(v>>8)&0xFF}.{v&0xFF}"
            print(f"    Firmware: {version}")
            return True
        return False
    test("Firmware version", test_version)
    
    def test_heartbeat_stable():
        times = []
        for i in range(5):
            msg = conn.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if msg:
                times.append(time.time())
        if len(times) >= 2:
            interval = (times[-1] - times[0]) / (len(times) - 1)
            print(f"    Heartbeat interval: {interval:.2f}s")
            return 0.8 < interval < 1.2
        return False
    test("Heartbeat stability (5 beats)", test_heartbeat_stable)
    
    # === Summary ===
    conn.disconnect()
    
    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    
    total = len(results)
    passed = sum(1 for _, s, _ in results if s == "PASS")
    failed = sum(1 for _, s, _ in results if s == "FAIL")
    errors = sum(1 for _, s, _ in results if s == "ERROR")
    
    print(f"  Total: {total}")
    print(f"  Passed: {passed}")
    print(f"  Failed: {failed}")
    print(f"  Errors: {errors}")
    print(f"  Pass rate: {passed/total*100:.1f}%")
    
    if failed + errors > 0:
        print(f"\n  Failed/Error tests:")
        for name, status, detail in results:
            if status != "PASS":
                print(f"    [{status}] {name}: {detail}")
    
    print("=" * 60)

if __name__ == '__main__':
    main()
