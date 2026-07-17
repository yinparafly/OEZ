"""Quick test: verify data streams work after fix"""
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('COM16', baud=115200, source_system=255)
master.wait_heartbeat(timeout=10)
print(f'Connected: sys={master.target_system}')

# Request ALL data streams
for stream_id in range(0, 14):
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        stream_id, 4, 1
    )

# Collect messages for 5 seconds
msg_types = {}
start = time.time()
while time.time() - start < 5:
    msg = master.recv_match(blocking=True, timeout=0.5)
    if msg:
        mtype = msg.get_type()
        msg_types[mtype] = msg_types.get(mtype, 0) + 1

print('\nMessage types after requesting streams:')
for mt, count in sorted(msg_types.items(), key=lambda x: -x[1]):
    print(f'  {mt:30s}: {count}')

# Now test param_request_list again
master.mav.param_request_list_send(master.target_system, master.target_component)
time.sleep(3)

params = {}
start = time.time()
while time.time() - start < 5:
    msg = master.recv_match(blocking=True, timeout=1)
    if msg and msg.get_type() == 'PARAM_VALUE':
        params[msg.param_id] = msg.param_value

print(f'\nParams after request_list: {len(params)}')
if params:
    for k in sorted(list(params.keys())[:5]):
        print(f'  {k} = {params[k]}')

master.close()
