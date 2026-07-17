"""Quick feature verification"""
import sys, time
sys.path.insert(0, 'D:/oezcon/ai_mavproxy')
from core.connection import Connection
from modules.param_manager import ParamManager
from modules.flight_control import FlightControl
from modules.telemetry import Telemetry

conn = Connection('COM15', 115200)
conn.connect()

params = ParamManager(conn)
fc = FlightControl(conn)
tel = Telemetry(conn)
tel.request_streams()

print('1. Telemetry...')
status = tel.get_full_status(timeout=3)
print('   Mode=%s Alt=%.1fm' % (status.get('flightmode'), status.get('altitude', 0)))

print('2. Param get...')
val = params.get('SCR_ENABLE')
print('   SCR_ENABLE = %s' % val)

print('3. Param set+readback...')
original = params.get('SCR_HEAP_SIZE')
params.set('SCR_HEAP_SIZE', 4096)
time.sleep(0.3)
rb = params.get('SCR_HEAP_SIZE')
params.set('SCR_HEAP_SIZE', original)
print('   Write=4096 Read=%s Restore=%s' % (rb, original))

print('4. Get mode...')
mode = fc.get_mode()
print('   Mode = %s' % mode)

print('5. Firmware version...')
conn.master.mav.autopilot_version_request_send(conn.master.target_system, conn.master.target_component)
msg = conn.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
if msg:
    v = msg.flight_sw_version
    print('   Version: %d.%d.%d.%d' % ((v>>24), (v>>16)&0xFF, (v>>8)&0xFF, v&0xFF))

print('6. Param export...')
count = params.export_mp('D:/oezcon/ai_mavproxy/test_final.param')
print('   Exported %d params' % count)

print('7. Heartbeat stability...')
times = []
for i in range(5):
    msg = conn.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if msg:
        times.append(time.time())
if len(times) >= 2:
    interval = (times[-1] - times[0]) / (len(times) - 1)
    print('   Interval: %.2fs' % interval)

conn.disconnect()
print('\nAll tests completed!')
