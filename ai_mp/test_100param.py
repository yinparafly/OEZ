"""100 param round-trip: stop streams during param ops"""
import sys, os, time, json, random
sys.path.insert(0, 'D:/oezcon/ai_mp')
from pymavlink import mavutil

PORT = 'COM15'
master = mavutil.mavlink_connection(PORT, baud=115200, source_system=255)
master.wait_heartbeat(timeout=10)
print(f'Connected: sys={master.target_system}')

# 停止数据流，减少干扰
for sid in range(0, 14):
    master.mav.request_data_stream_send(master.target_system, master.target_component, sid, 0, 0)
time.sleep(0.5)
# 清空缓冲区
for _ in range(50):
    master.recv_match(blocking=False)
print("Data streams stopped")

# 读全部参数 (无遥测干扰)
print("Reading all params (no stream interference)...")
master.mav.param_request_list_send(master.target_system, master.target_component)
all_params = {}
t0 = time.time()
while time.time() - t0 < 30:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if msg:
        all_params[msg.param_id] = msg.param_value
    elif len(all_params) > 0:
        break
print(f"Total: {len(all_params)} params")

# 抽样100
sample = random.sample(list(all_params.keys()), 100)
print(f"Testing 100 params (no streams)...\n")

results = []
pass_c = 0
fail_c = 0
t_start = time.time()

for i, name in enumerate(sample):
    orig = all_params[name]
    test = orig + 1 if orig != 0 else 1.0
    
    # 写
    master.param_set_send(name, float(test), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.15)
    
    # 回读
    master.param_fetch_one(name)
    rb = None
    t1 = time.time()
    while time.time() - t1 < 1.5:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg and msg.param_id == name:
            rb = msg.param_value
            break
    w_ok = rb is not None and abs(float(rb) - float(test)) < 0.1
    
    # 恢复
    master.param_set_send(name, float(orig), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.15)
    
    # 回读恢复
    master.param_fetch_one(name)
    rs = None
    t1 = time.time()
    while time.time() - t1 < 1.5:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg and msg.param_id == name:
            rs = msg.param_value
            break
    r_ok = rs is not None and abs(float(rs) - float(orig)) < 0.1
    
    ok = w_ok and r_ok
    if ok:
        pass_c += 1
    else:
        fail_c += 1
        results.append({'n': name, 'o': orig, 't': test, 'rb': rb, 'rs': rs})
    
    print(f"  [{i+1:3d}] {'OK' if ok else 'FAIL'} {name:30s} orig={orig:12} test={test:12} rb={str(rb):12} rs={str(rs):12}")

elapsed = time.time() - t_start
total = pass_c + fail_c
print(f"\n{'='*60}")
print(f"  {pass_c}/{total} passed ({pass_c/total*100:.0f}%) in {elapsed:.0f}s ({elapsed/total:.1f}s/param)")
if fail_c > 0:
    print(f"  FAILURES:")
    for r in results:
        print(f"    {r['n']}: orig={r['o']} test={r['t']} rb={r['rb']} rs={r['rs']}")

# 恢复数据流
for sid in range(0, 7):
    master.mav.request_data_stream_send(master.target_system, master.target_component, sid, 4, 1)
print("Data streams restored")

# 保存
d = 'D:/oezcon/ai_mp/verify/roundtrip'
os.makedirs(d, exist_ok=True)
with open(f'{d}/100param.json', 'w') as f:
    json.dump({'total': total, 'passed': pass_c, 'failed': fail_c, 'elapsed': round(elapsed,1), 'fails': results}, f, indent=2)
with open(f'{d}/100param.txt', 'w') as f:
    f.write(f"100 Param Round-Trip: {pass_c}/{total} passed\nTime: {elapsed:.0f}s\n")
    if results:
        f.write("FAILURES:\n")
        for r in results:
            f.write(f"  {r['n']}: orig={r['o']} test={r['t']} rb={r['rb']} rs={r['rs']}\n")
    else:
        f.write("ALL PASSED\n")
print(f"  Saved: {d}/")
master.close()
