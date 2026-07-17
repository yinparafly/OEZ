"""
AI-MP 全参数逐个往返对比测试 (优化版)
1010 个参数，逐个写入→回读→恢复→回读
优化: 减少等待时间，跳过只读参数
"""

import sys, os, time, json
sys.path.insert(0, 'D:/oezcon/ai_mp')
from pymavlink import mavutil

PORT = 'COM15'
EXPORT_DIR = 'D:/oezcon/ai_mp/verify/fulltest'
os.makedirs(EXPORT_DIR, exist_ok=True)

master = mavutil.mavlink_connection(PORT, baud=115200, source_system=255)
master.wait_heartbeat(timeout=10)
print(f'Connected: sys={master.target_system}')

# 读取全部参数
print("Reading all params...")
master.mav.param_request_list_send(master.target_system, master.target_component)
all_params = {}
start = time.time()
while time.time() - start < 30:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if msg:
        all_params[msg.param_id] = msg.param_value
    elif len(all_params) > 0:
        break
print(f"Total: {len(all_params)} params")

# 逐个测试
print(f"\nTesting ALL {len(all_params)} params...")
pass_count = 0
fail_count = 0
failures = []
start_time = time.time()

for i, (name, original_val) in enumerate(sorted(all_params.items())):
    test_val = original_val + 1 if original_val != 0 else 1.0
    
    # 写入
    master.param_set_send(name, float(test_val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.1)
    
    # 回读
    master.param_fetch_one(name)
    readback = None
    t0 = time.time()
    while time.time() - t0 < 1.5:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg and msg.param_id == name:
            readback = msg.param_value
            break
    
    write_ok = readback is not None and abs(float(readback) - float(test_val)) < 0.1
    
    # 恢复
    master.param_set_send(name, float(original_val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.1)
    
    # 回读恢复值
    master.param_fetch_one(name)
    restored = None
    t0 = time.time()
    while time.time() - t0 < 1.5:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg and msg.param_id == name:
            restored = msg.param_value
            break
    
    restore_ok = restored is not None and abs(float(restored) - float(original_val)) < 0.1
    
    if write_ok and restore_ok:
        pass_count += 1
    else:
        fail_count += 1
        failures.append({'name': name, 'orig': original_val, 'test': test_val, 'rb': readback, 'rest': restored, 'wok': write_ok, 'rok': restore_ok})
    
    if (i+1) % 100 == 0:
        elapsed = time.time() - start_time
        print(f"  [{i+1}/{len(all_params)}] pass={pass_count} fail={fail_count} {elapsed:.0f}s")

elapsed = time.time() - start_time
total = pass_count + fail_count
print(f"\n{'='*60}")
print(f"  RESULTS: {pass_count}/{total} passed ({pass_count/total*100:.2f}%)")
print(f"  Failed: {fail_count}")
print(f"  Time: {elapsed:.1f}s ({total/elapsed:.1f} params/s)")

if fail_count > 0:
    print(f"\n  FAILURES:")
    for f in failures:
        print(f"    {f['name']}: orig={f['orig']} test={f['test']} rb={f['rb']} rest={f['rest']}")

# 保存
with open(os.path.join(EXPORT_DIR, 'full_roundtrip_report.json'), 'w') as fp:
    json.dump({'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'total': total, 'passed': pass_count, 'failed': fail_count, 'elapsed': round(elapsed,1), 'failures': failures}, fp, indent=2)

with open(os.path.join(EXPORT_DIR, 'full_roundtrip_report.txt'), 'w') as fp:
    fp.write(f"Full Parameter Round-Trip: {pass_count}/{total} passed ({pass_count/total*100:.2f}%)\n")
    fp.write(f"Time: {elapsed:.1f}s\n\n")
    if failures:
        fp.write("FAILURES:\n")
        for f in failures:
            fp.write(f"  {f['name']}: orig={f['orig']} test={f['test']} rb={f['rb']} rest={f['rest']}\n")
    else:
        fp.write("ALL PASSED\n")

print(f"\n  Reports saved to {EXPORT_DIR}")
master.close()
