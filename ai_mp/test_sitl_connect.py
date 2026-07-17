"""
SITL 连接测试 - 直接测试 TCP 5760 连接
"""
from pymavlink import mavutil
import time

print("[TEST] 连接 SITL tcp:127.0.0.1:5760 ...")
master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)

print("[TEST] 等待心跳...")
try:
    master.wait_heartbeat(timeout=15)
    print(f"[OK] 心跳收到! System={master.target_system}, Component={master.target_component}")
except Exception as e:
    print(f"[FAIL] 心跳超时: {e}")
    master.close()
    exit(1)

# 读取更多消息
print("[TEST] 读取 5 秒遥测数据...")
start = time.time()
count = 0
while time.time() - start < 5:
    msg = master.recv_match(blocking=True, timeout=2)
    if msg:
        count += 1
        mtype = msg.get_type()
        if mtype == 'HEARTBEAT':
            print(f"  HEARTBEAT: type={msg.type}, autopilot={msg.autopilot}, base_mode={msg.base_mode}")
        elif mtype == 'SYS_STATUS':
            print(f"  SYS_STATUS: voltage={msg.voltage_battery/1000:.1f}V, current={msg.current_battery/100:.1f}A")
        elif mtype == 'GPS_RAW_INT':
            print(f"  GPS: fix={msg.fix_type}, sats={msg.satellites_visible}, lat={msg.lat/1e7:.5f}, lon={msg.lon/1e7:.5f}")
        elif mtype == 'VFR_HUD':
            print(f"  HUD: alt={msg.alt:.1f}m, aspd={msg.airspeed:.1f}m/s, gs={msg.groundspeed:.1f}m/s, heading={msg.heading}")
        elif mtype == 'ATTITUDE':
            print(f"  ATT: roll={msg.roll*57.3:.1f}deg, pitch={msg.pitch*57.3:.1f}deg")

print(f"\n[RESULT] 收到 {count} 条消息，连接稳定!")
master.close()
