"""
启动 SITL 双端口版本 - SERIAL0 (5760) + SERIAL1 (5762)
两个端口可以同时连接
"""
import subprocess
import time
import sys

# 1. Kill old
subprocess.run(['wsl', 'bash', '-c', 'killall -9 arduplane 2>/dev/null'], capture_output=True)
time.sleep(3)

# 2. Start SITL with two TCP serial ports
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
print(f"[SITL] 启动 arduplane PID={proc.pid}")
time.sleep(5)

# 3. Check log
result = subprocess.run(['wsl', 'bash', '-c', 'cat /proc/$(pgrep arduplane)/fd/1 2>/dev/null || echo NO_LOG'], 
                       capture_output=True, text=True, timeout=5)
print(f"[LOG] 进程状态: {'运行中' if proc.poll() is None else '已退出'}")

# 4. Verify ports
result = subprocess.run(['wsl', 'bash', '-c', 'ss -tlnp 2>/dev/null | grep -E "5760|5762"'], 
                       capture_output=True, text=True)
print(f"[PORTS]\n{result.stdout}")

# 5. Quick connection test
from pymavlink import mavutil
print("[TEST] 连接 tcp:127.0.0.1:5760 ...")
try:
    m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
    m.wait_heartbeat(timeout=10)
    print(f"[OK] SERIAL0 连接成功! System={m.target_system}")
    m.close()
except Exception as e:
    print(f"[FAIL] {e}")
    proc.kill()
    sys.exit(1)

print(f"\n{'='*60}")
print(f"SITL 运行正常!")
print(f"  SERIAL0 (AI-MP):   tcp:127.0.0.1:5760")
print(f"  SERIAL1 (Mission Planner): tcp:127.0.0.1:5762")
print(f"  两个端口可同时连接")
print(f"{'='*60}")
