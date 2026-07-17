import subprocess
import time
import sys

# Kill any existing
subprocess.run(['wsl', 'bash', '-c', 'killall -9 arduplane sim_vehicle.py 2>/dev/null'], capture_output=True)
time.sleep(2)

# Use sim_vehicle.py with --no-build to skip recompilation
cmd = [
    'wsl', '-e', 'bash', '-c',
    'cd /root/ardupilot/Tools/autotest && '
    'python3 sim_vehicle.py -v ArduPlane --frame plane --no-mavproxy '
    '--out udp:127.0.0.1:14550 '
    '--out udp:127.0.0.1:14552 '
    '-w '
    '2>&1 | head -80'
]
print(f"Running: sim_vehicle.py with dual UDP output")
proc = subprocess.run(cmd, timeout=120, capture_output=True, text=True)
print("STDOUT:")
print(proc.stdout[-3000:] if len(proc.stdout) > 3000 else proc.stdout)
if proc.stderr:
    print("STDERR:")
    print(proc.stderr[-2000:] if len(proc.stderr) > 2000 else proc.stderr)
