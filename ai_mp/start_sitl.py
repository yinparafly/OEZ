import subprocess
import time

# Kill any existing arduplane
subprocess.run(['wsl', 'bash', '-c', 'killall -9 arduplane 2>/dev/null'], capture_output=True)
time.sleep(1)

# Start arduplane
cmd = [
    'wsl', 'bash', '-c',
    '/root/ardupilot/build/sitl/bin/arduplane '
    '--model plane '
    '--defaults /root/ardupilot/Tools/autotest/models/plane.parm '
    '--serial0 tcp:5760 '
    '--home CMAC '
    '> /tmp/sitl_test.log 2>&1'
]
print(f"Starting: {' '.join(cmd)}")
proc = subprocess.Popen(cmd)

# Wait and check
time.sleep(5)

# Check if running
result = subprocess.run(['wsl', 'bash', '-c', 'pgrep -la arduplane'], capture_output=True, text=True)
print(f"Process check: {result.stdout}")

# Check log
result = subprocess.run(['wsl', 'bash', '-c', 'cat /tmp/sitl_test.log 2>/dev/null'], capture_output=True, text=True)
print(f"Log:\n{result.stdout}")
