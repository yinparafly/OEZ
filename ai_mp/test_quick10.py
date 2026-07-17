"""Quick 10-param test with streams stopped"""
import sys, time, random
sys.path.insert(0, 'D:/oezcon/ai_mp')
from pymavlink import mavutil

master = mavutil.mavlink_connection('COM15', baud=115200, source_system=255)
master.wait_heartbeat(timeout=10)
print('Connected')

# Stop streams
for sid in range(0, 14):
    master.mav.request_data_stream_send(master.target_system, master.target_component, sid, 0, 0)
time.sleep(0.5)
for _ in range(50):
    master.recv_match(blocking=False)

# Read params
master.mav.param_request_list_send(master.target_system, master.target_component)
params = {}
t0 = time.time()
while time.time() - t0 < 30:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if msg:
        params[msg.param_id] = msg.param_value
    elif len(params) > 0:
        break
print(f'Params: {len(params)}')

# Test 10 params
sample = random.sample(list(params.keys()), 10)
t_start = time.time()
pass_c = 0
fail_c = 0

for name in sample:
    orig = params[name]
    test = orig + 1 if orig != 0 else 1.0
    t_param = time.time()
    
    # Write
    master.param_set_send(name, float(test), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.1)
    
    # Read back
    master.param_fetch_one(name)
    rb = None
    t1 = time.time()
    while time.time() - t1 < 1:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg and msg.param_id == name:
            rb = msg.param_value
            break
    w_ok = rb is not None and abs(float(rb) - float(test)) < 0.1
    
    # Restore
    master.param_set_send(name, float(orig), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.1)
    
    # Read restore
    master.param_fetch_one(name)
    rs = None
    t1 = time.time()
    while time.time() - t1 < 1:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg and msg.param_id == name:
            rs = msg.param_value
            break
    r_ok = rs is not None and abs(float(rs) - float(orig)) < 0.1
    
    ok = w_ok and r_ok
    elapsed = time.time() - t_param
    if ok:
        pass_c += 1
    else:
        fail_c += 1
    
    status = 'OK' if ok else 'FAIL'
    print(f'  {status} {name}: orig={orig} test={test} rb={rb} rs={rs} ({elapsed:.1f}s)')

total_time = time.time() - t_start
print(f'\nResults: {pass_c}/{pass_c+fail_c} passed in {total_time:.1f}s')
print(f'Estimated 100 params: {total_time * 10:.0f}s ({total_time * 10 / 60:.1f}min)')
print(f'Estimated 1010 params: {total_time * 101:.0f}s ({total_time * 101 / 60:.1f}min)')

# Restore streams
for sid in range(0, 7):
    master.mav.request_data_stream_send(master.target_system, master.target_component, sid, 4, 1)
master.close()
