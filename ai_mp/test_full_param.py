"""
AI-MP 全参数对比测试
1. 通过 param_request_list 读取全部参数 (基准)
2. 逐个通过 param_fetch_one 读取 (验证)
3. 对比两组数据是否一致
4. 导出为 MP 兼容格式
"""

import sys, os, time
sys.path.insert(0, 'D:/oezcon/ai_mp')
from pymavlink import mavutil

PORT = 'COM15'
EXPORT_DIR = 'D:/oezcon/ai_mp/verify/fulltest'
os.makedirs(EXPORT_DIR, exist_ok=True)

def connect():
    master = mavutil.mavlink_connection(PORT, baud=115200, source_system=255)
    master.wait_heartbeat(timeout=10)
    print(f'Connected: sys={master.target_system} type={master.mav_type}')
    return master

def read_all_params_list(master):
    """通过 param_request_list 读取全部参数"""
    print("\n=== Phase 1: param_request_list ===")
    master.mav.param_request_list_send(master.target_system, master.target_component)
    
    params = {}
    start = time.time()
    while time.time() - start < 30:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg:
            params[msg.param_id] = msg.param_value
            if len(params) % 200 == 0 and len(params) > 0:
                print(f"  ... {len(params)} params so far")
    
    print(f"  Total: {len(params)} params")
    return params

def read_param_one(master, name):
    """通过 param_fetch_one 读取单个参数"""
    master.param_fetch_one(name)
    start = time.time()
    while time.time() - start < 3:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id == name:
            return msg.param_value
    return None

def export_mp_format(params, filepath):
    """导出为 MP 兼容 .param 格式"""
    with open(filepath, 'w', encoding='utf-8') as f:
        for name, val in sorted(params.items()):
            f.write(f"{name}\t{val}\n")
    print(f"  Exported {len(params)} params to {filepath}")

# ============================================================
# Main
# ============================================================
master = connect()

# Phase 1: 读取全部参数 (基准)
params_list = read_all_params_list(master)

# 导出基准参数
export_mp_format(params_list, os.path.join(EXPORT_DIR, 'all_params_baseline.param'))

# Phase 2: 逐个 param_fetch_one 验证 (抽样100个)
print("\n=== Phase 2: param_fetch_one verification (sample 100) ===")
import random
sample_names = random.sample(list(params_list.keys()), min(100, len(params_list)))

fetch_results = {}
for i, name in enumerate(sample_names):
    val = read_param_one(master, name)
    fetch_results[name] = val
    if (i+1) % 20 == 0:
        print(f"  ... verified {i+1}/{len(sample_names)}")

# Phase 3: 对比
print("\n=== Phase 3: Comparison ===")
match = 0
mismatch = 0
fetch_fail = 0

for name in sample_names:
    list_val = params_list.get(name)
    fetch_val = fetch_results.get(name)
    
    if fetch_val is None:
        fetch_fail += 1
        continue
    
    if abs(float(list_val) - float(fetch_val)) < 0.001:
        match += 1
    else:
        mismatch += 1
        print(f"  DIFF: {name}: list={list_val} fetch={fetch_val}")

print(f"\n  Matched: {match}/{len(sample_names)}")
print(f"  Mismatched: {mismatch}")
print(f"  Fetch failed: {fetch_fail}")

# Phase 4: 导出对比报告
report_file = os.path.join(EXPORT_DIR, 'param_comparison_report.txt')
with open(report_file, 'w', encoding='utf-8') as f:
    f.write("AI-MP Parameter Comparison Report\n")
    f.write(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write(f"Total params (list): {len(params_list)}\n")
    f.write(f"Sample tested: {len(sample_names)}\n")
    f.write(f"Matched: {match}\n")
    f.write(f"Mismatched: {mismatch}\n")
    f.write(f"Fetch failed: {fetch_fail}\n\n")
    
    if mismatch > 0:
        f.write("DIFFERENCES:\n")
        for name in sample_names:
            lv = params_list.get(name)
            fv = fetch_results.get(name)
            if fv is not None and abs(float(lv) - float(fv)) >= 0.001:
                f.write(f"  {name}: list={lv} fetch={fv}\n")
    
    f.write("\nALL MATCHED PARAMS:\n")
    for name in sorted(sample_names):
        lv = params_list.get(name)
        fv = fetch_results.get(name)
        if fv is not None and abs(float(lv) - float(fv)) < 0.001:
            f.write(f"  {name} = {lv}\n")

print(f"\n  Report: {report_file}")
print(f"  Baseline: {os.path.join(EXPORT_DIR, 'all_params_baseline.param')}")

# Phase 5: 写入-回读测试 (抽样20个参数)
print("\n=== Phase 5: Write-Read verification (20 params) ===")
test_params = random.sample(list(params_list.keys()), min(20, len(params_list)))
write_read_results = []

for name in test_params:
    original_val = params_list[name]
    
    # 写入一个新值
    if original_val == 0:
        new_val = 1.0
    else:
        new_val = original_val + 1
    
    master.param_set_send(name, float(new_val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)
    
    # 回读
    readback = read_param_one(master, name)
    
    # 恢复原值
    master.param_set_send(name, float(original_val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)
    
    if readback is not None:
        ok = abs(float(readback) - float(new_val)) < 0.1
        write_read_results.append((name, original_val, new_val, readback, ok))
        icon = "OK" if ok else "FAIL"
        print(f"  {icon}: {name} wrote={new_val} read={readback} (original={original_val})")
    else:
        write_read_results.append((name, original_val, new_val, None, False))
        print(f"  FAIL: {name} write={new_val} read=None")

wr_match = sum(1 for _, _, _, _, ok in write_read_results if ok)
wr_total = len(write_read_results)
print(f"\n  Write-Read: {wr_match}/{wr_total} passed")

# Phase 6: MP 兼容格式导出验证
print("\n=== Phase 6: MP format export verification ===")
mp_file = os.path.join(EXPORT_DIR, 'all_params_mp_format.param')
export_mp_format(params_list, mp_file)

# 验证导出文件
with open(mp_file, 'r') as f:
    lines = f.readlines()
    param_count = len([l for l in lines if '\t' in l.strip()])
    print(f"  MP format file: {param_count} params")

# 验证格式正确性 (MP 期望: NAME\tVALUE\n)
format_ok = True
for line in lines[:10]:
    parts = line.strip().split('\t')
    if len(parts) != 2:
        format_ok = False
        print(f"  FORMAT ERROR: {line.strip()}")
print(f"  Format check: {'OK' if format_ok else 'ERROR'}")

# 汇总
print("\n" + "=" * 60)
print("  FINAL SUMMARY")
print("=" * 60)
print(f"  Total params read: {len(params_list)}")
print(f"  param_request_list: OK ({len(params_list)} params)")
print(f"  param_fetch_one sample: {match}/{len(sample_names)} matched, {fetch_fail} failed")
print(f"  Write-Read test: {wr_match}/{wr_total} passed")
print(f"  MP format export: {param_count} params, format={'OK' if format_ok else 'ERROR'}")
print(f"  Reports: {EXPORT_DIR}")
print("=" * 60)

master.close()
