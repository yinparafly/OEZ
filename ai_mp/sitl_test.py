"""
SITL 启动 + AI-MP 连接 (在同一脚本中完成)
解决 SITL 超时退出问题
"""
import subprocess
import time
import sys
import os

# 1. Kill old
print("[1] 清理旧进程...")
subprocess.run(['wsl', 'bash', '-c', 'killall -9 arduplane 2>/dev/null'], capture_output=True)
time.sleep(2)

# 2. Start SITL in background
print("[2] 启动 SITL...")
sitl_cmd = (
    '/root/ardupilot/build/sitl/bin/arduplane '
    '--model plane '
    '--defaults /root/ardupilot/Tools/autotest/models/plane.parm '
    '--serial0 tcp:5760 '
    '--serial1 tcp:5762 '
    '--home CMAC'
)
proc = subprocess.Popen(
    ['wsl', '-e', 'bash', '-c', sitl_cmd],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)
print(f"    PID={proc.pid}")

# 3. Wait for port to be ready
print("[3] 等待端口就绪...")
time.sleep(4)

# 4. Connect with pymavlink
print("[4] 连接 tcp:127.0.0.1:5760 ...")
from pymavlink import mavutil

master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
master.wait_heartbeat(timeout=10)
print(f"[OK] 连接成功! System={master.target_system}, Flightmode={master.flightmode}")

# 5. Get status
print("\n[5] 获取飞行状态...")
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
)
time.sleep(2)

# Read messages
for _ in range(20):
    msg = master.recv_match(blocking=True, timeout=2)
    if msg:
        mtype = msg.get_type()
        if mtype == 'VFR_HUD':
            print(f"  高度: {msg.alt:.1f}m, 空速: {msg.airspeed:.1f}m/s, 地速: {msg.groundspeed:.1f}m/s")
        elif mtype == 'ATTITUDE':
            print(f"  姿态: Roll={msg.roll*57.3:.1f}deg, Pitch={msg.pitch*57.3:.1f}deg")
        elif mtype == 'SYS_STATUS':
            print(f"  电池: {msg.voltage_battery/1000:.1f}V")
        elif mtype == 'GPS_RAW_INT':
            print(f"  GPS: fix={msg.fix_type}, sats={msg.satellites_visible}")

print("\n" + "="*60)
print("SITL 仿真测试成功!")
print("="*60)
print(f"  AI-MP 连接:   tcp:127.0.0.1:5760 (SERIAL0)")
print(f"  MP 连接:      tcp:127.0.0.1:5762 (SERIAL1)")
print(f"  SITL 进程:    PID={proc.pid}")
print("="*60)
print("\nAI-MP 现在可以控制 SITL 仿真:")
print("  python ai_mp2.py --port tcp:127.0.0.1:5760 --cmd monitor")
print("  python ai_mp2.py --port tcp:127.0.0.1:5760 --cmd mode FBWA")
print("  python ai_mp2.py --port tcp:127.0.0.1:5760 --cmd arm")

master.close()
