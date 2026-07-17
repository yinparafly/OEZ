#!/usr/bin/env python3
import time
from pymavlink import mavutil

mav = mavutil.mavlink_connection('COM15', baud=115200)
mav.wait_heartbeat(timeout=10)
print('Connected')

# Read params
for p in ['CAN_D1_PROTOCOL', 'CAN_D1_SRVOBM', 'CAN_D1_OPTION']:
    while mav.recv_match(type='PARAM_VALUE', blocking=False): pass
    mav.mav.param_request_read_send(mav.target_system, mav.target_component, p.encode(), -1)
    time.sleep(0.5)
    msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:
        name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
        print(f'{name} = {int(msg.param_value)}')
    else:
        print(f'{p} = NOT FOUND')

mav.close()
