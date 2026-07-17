"""
重启 SITL 并用 AI-MP 连接测试
"""
import subprocess
import time

# 1. Kill old processes
subprocess.run(['wsl', 'bash', '-c', 'killall -9 arduplane 2>/dev/null'], capture_output=True)
time.sleep(3)

# 2. Start SITL
proc = subprocess.Popen(
    ['wsl', '-e', 'bash', '-c',
     '/root/ardupilot/build/sitl/bin/arduplane '
     '--model plane '
     '--defaults /root/ardupilot/Tools/autotest/models/plane.parm '
     '--serial0 tcp:5760 '
     '--home CMAC'
    ],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)
print(f"[SITL] 启动中... PID={proc.pid}")
time.sleep(5)

# 3. Verify port is listening
result = subprocess.run(['wsl', 'bash', '-c', 'ss -tlnp | grep 5760'], capture_output=True, text=True)
print(f"[PORT] {result.stdout.strip()}")

# 4. Don't disconnect - leave SITL running for AI-MP
print("[SITL] 运行中，等待 AI-MP 连接...")
print("[SITL] 请在另一个终端运行: python ai_mp2.py --port tcp:127.0.0.1:5760 --cmd status")
