import sys
sys.argv = ['ai_mp.py', '--cli', '--port', 'COM16', '--command', 'monitor']
# Import and run just the monitor for a few seconds
import time
sys.path.insert(0, 'D:/oezcon/ai_mp')
from ai_mp import MAVConnection

conn = MAVConnection('COM16', 115200)
conn.connect()
conn.request_data_stream(10)

print("Monitoring for 5 seconds...")
start = time.time()
while time.time() - start < 5:
    status = conn.get_status()
    alt = status.get('altitude', 0)
    aspd = status.get('airspeed', 0)
    mode = status.get('flightmode', '?')
    print(f"Mode: {mode:10s} | Alt: {alt:7.1f}m | ASpd: {aspd:5.1f}m/s")
    time.sleep(1)

conn.close()
print("Monitor test passed!")
