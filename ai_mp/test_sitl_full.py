"""
启动 SITL 并测试连接
"""
import subprocess
import time
import sys
import os

# 1. Kill old processes
print("[1] 清理旧进程...")
subprocess.run(['wsl', 'bash', '-c', 'killall -9 arduplane 2>/dev/null'], capture_output=True)
time.sleep(2)

# 2. Start SITL
print("[2] 启动 SITL (arduplane)...")
sitl_cmd = (
    '/root/ardupilot/build/sitl/bin/arduplane '
    '--model plane '
    '--defaults /root/ardupilot/Tools/autotest/models/plane.parm '
    '--serial0 tcp:5760 '
    '--home CMAC'
)
proc = subprocess.Popen(
    ['wsl', '-e', 'bash', '-c', sitl_cmd],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

# 3. Wait for SITL to bind port
print("[3] 等待 SITL 启动 (5秒)...")
time.sleep(5)

# 4. Test connection with pymavlink
print("[4] 测试 TCP 连接...")
from pymavlink import mavutil

try:
    master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
    print("[4] 连接建立，等待心跳...")
    master.wait_heartbeat(timeout=10)
    print(f"[OK] 心跳收到! System={master.target_system}, Type={master.flightmode}")
except Exception as e:
    print(f"[FAIL] 连接失败: {e}")
    proc.kill()
    sys.exit(1)

# 5. Read telemetry for 5 seconds
print("[5] 读取遥测数据 (5秒)...")
start = time.time()
count = 0
msgs = {}
while time.time() - start < 5:
    msg = master.recv_match(blocking=True, timeout=2)
    if msg:
        count += 1
        mtype = msg.get_type()
        msgs[mtype] = msgs.get(mtype, 0) + 1
        if mtype in ('HEARTBEAT', 'SYS_STATUS', 'VFR_HUD', 'ATTITUDE', 'GPS_RAW_INT'):
            if mtype == 'HEARTBEAT':
                mode_map = {0: 'STABILIZE', 2: 'ACRO', 4: 'AUTO', 5: 'TRAINING', 7: 'FBWA', 10: 'AUTO', 13: 'CRUISE'}
                mode = mode_map.get(msg.custom_mode, str(msg.custom_mode))
                armed = 'ARMED' if (msg.base_mode & 128) else 'DISARMED'
                print(f"  HEARTBEAT: mode={mode}, armed={armed}")
            elif mtype == 'VFR_HUD':
                print(f"  VFR_HUD: alt={msg.alt:.1f}m, aspd={msg.airspeed:.1f}m/s, gs={msg.groundspeed:.1f}m/s")
            elif mtype == 'ATTITUDE':
                print(f"  ATTITUDE: roll={msg.roll*57.3:.1f}deg, pitch={msg.pitch*57.3:.1f}deg")

print(f"\n[RESULT] 收到 {count} 条消息")
print(f"[MSG TYPES] {dict(sorted(msgs.items()))}")
print("[SITL] 运行正常! AI-MP 可以连接 tcp:127.0.0.1:5760")
print("[MP] 也可以连接 tcp:127.0.0.1:5762 (SERIAL1)")

master.close()
# Don't kill SITL - leave it running
print("[DONE] SITL 继续运行中。PID:", proc.pid)
