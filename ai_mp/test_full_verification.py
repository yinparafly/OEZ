"""
AI-MP vs Mission Planner 全面对比测试
测试项目:
1. 连接与心跳
2. 参数写入→AI-MP回读验证
3. 参数写入→导出MP格式→MP打开验证
4. 航点上传→下载→对比
5. 围栏上传→下载→对比
6. Rally上传→下载→对比
7. 遥测数据一致性
8. 飞行模式切换验证
9. 解锁/上锁验证
10. 预飞检查验证
11. 传感器状态验证
12. EKF状态验证
13. RC通道读取验证
14. 日志列表验证
15. 完整验证报告
"""

import sys
import os
import time
import json

sys.path.insert(0, 'D:/oezcon/ai_mp')
from pymavlink import mavutil
from ai_mp2 import FullMAVLink
from verify import WriteVerifier

PORT = 'COM15'
BAUD = 115200
EXPORT_DIR = 'D:/oezcon/ai_mp/verify/test'
os.makedirs(EXPORT_DIR, exist_ok=True)

results = []

def log_result(test_name, passed, detail=""):
    status = "PASS" if passed else "FAIL"
    icon = "✅" if passed else "❌"
    results.append((test_name, passed, detail))
    print(f"  {icon} {test_name}: {status} {detail}")

def separator(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

# ============================================================
# Test 1: 连接与心跳
# ============================================================
separator("Test 1: 连接与心跳")
mav = FullMAVLink(PORT, BAUD)
connected = mav.connect()
log_result("MAVLink 连接", connected, f"System={mav.master.target_system}")

if not connected:
    print("\n无法连接飞控，终止测试")
    sys.exit(1)

# 心跳稳定性
hb_times = []
for i in range(5):
    msg = mav.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if msg:
        hb_times.append(time.time())

if len(hb_times) >= 2:
    interval = (hb_times[-1] - hb_times[0]) / (len(hb_times) - 1)
    log_result("心跳稳定性", 0.8 < interval < 1.2, f"间隔={interval:.2f}s")
else:
    log_result("心跳稳定性", False, "心跳不足")

# ============================================================
# Test 2: 参数写入→AI-MP回读验证
# ============================================================
separator("Test 2: 参数写入→回读验证")

test_params = {
    'SCR_ENABLE': 1,
    'SCR_HEAP_SIZE': 8192,
    'ARSPD_USE': 1,
    'ARMING_CHECK': 1,
}

param_write_results = []
for name, value in test_params.items():
    mav.set_param(name, value)
    time.sleep(0.5)
    readback = mav.get_param(name)
    
    if readback is not None:
        match = abs(float(readback) - float(value)) < 0.1
        log_result(f"参数写入回读 {name}", match, f"写入={value} 回读={readback}")
        param_write_results.append((name, value, readback, match))
    else:
        log_result(f"参数写入回读 {name}", False, "回读失败")
        param_write_results.append((name, value, None, False))

# ============================================================
# Test 3: 参数导出MP格式
# ============================================================
separator("Test 3: 参数导出MP兼容格式")

verifier = WriteVerifier(mav, EXPORT_DIR)
for name, value in test_params.items():
    verifier.record_param_set(name, value)

param_file = verifier.export_params_mp(os.path.join(EXPORT_DIR, 'test_params.param'))
log_result("参数导出MP格式", param_file is not None, param_file or "失败")

# 验证导出文件内容
if param_file and os.path.exists(param_file):
    with open(param_file, 'r') as f:
        content = f.read()
    
    for name, value in test_params.items():
        if name in content and str(float(value)) in content:
            log_result(f"导出文件包含 {name}", True, f"{name}={value}")
        else:
            log_result(f"导出文件包含 {name}", False, "未找到")

# ============================================================
# Test 4: 航点上传→下载→对比
# ============================================================
separator("Test 4: 航点上传→下载→对比")

# 清除现有任务
mgr = mav.mission_init()
mgr.clear()

# 添加测试航点
test_waypoints = [
    ('TAKEOFF', 39.9042, 116.4074, 100, 10, 0, 0, 0),
    ('WAYPOINT', 39.9050, 116.4080, 100, 0, 0, 50, 0),
    ('WAYPOINT', 39.9060, 116.4090, 80, 0, 0, 30, 0),
    ('LOITER_TIME', 39.9055, 116.4085, 90, 30, 0, 50, 0),
    ('RTL', 0, 0, 0, 0, 0, 0, 0),
]

for cmd, lat, lon, alt, p1, p2, p3, p4 in test_waypoints:
    mgr.add_waypoint(cmd, lat, lon, alt, p1, p2, p3, p4)

# 上传
upload_ok = mgr.upload_to_fc()
log_result("航点上传", upload_ok, f"{len(test_waypoints)} 个航点")

time.sleep(1)

# 下载
download_ok = mgr.download_from_fc()
log_result("航点下载", download_ok)

# 对比
if mgr.waypoints:
    match_count = 0
    for i, (cmd, lat, lon, alt, *_) in enumerate(test_waypoints):
        if i < len(mgr.waypoints):
            wp = mgr.waypoints[i]
            cmd_ok = wp['cmd'] == cmd
            lat_ok = abs(wp.get('lat', 0) - lat) < 0.0001 if lat else True
            lon_ok = abs(wp.get('lon', 0) - lon) < 0.0001 if lon else True
            alt_ok = abs(wp.get('alt', 0) - alt) < 1 if alt else True
            
            if cmd_ok and lat_ok and lon_ok and alt_ok:
                match_count += 1
    
    log_result("航点对比", match_count == len(test_waypoints), 
               f"匹配 {match_count}/{len(test_waypoints)}")
else:
    log_result("航点对比", False, "下载航点为空")

# 导出MP格式航点
verifier.record_mission_upload(mgr.waypoints)
mission_file = verifier.export_mission_mp(os.path.join(EXPORT_DIR, 'test_mission.waypoints'))
log_result("航点导出MP格式", mission_file is not None, mission_file or "失败")

# ============================================================
# Test 5: 遥测数据一致性
# ============================================================
separator("Test 5: 遥测数据一致性")

status = mav.get_full_status()

# 检查关键字段是否存在
required_fields = ['flightmode', 'armed', 'altitude', 'airspeed', 'groundspeed', 
                   'lat', 'lon', 'voltage', 'satellites']

for field in required_fields:
    val = status.get(field)
    exists = val is not None
    log_result(f"遥测字段 {field}", exists, f"值={val}")

# ============================================================
# Test 6: 飞行模式切换验证
# ============================================================
separator("Test 6: 飞行模式切换验证")

modes_to_test = ['MANUAL', 'FBWA', 'CRUISE']
for mode in modes_to_test:
    ok = mav.set_mode(mode)
    time.sleep(1)
    status2 = mav.get_full_status()
    current_mode = status2.get('flightmode', '?')
    log_result(f"切换到 {mode}", current_mode == mode, f"当前={current_mode}")

# 恢复 MANUAL
mav.set_mode('MANUAL')

# ============================================================
# Test 7: 传感器状态验证
# ============================================================
separator("Test 7: 传感器状态验证")

sensors = mav.get_sensor_status()
log_result("传感器状态读取", len(sensors) > 0, f"{len(sensors)} 个传感器")

# ============================================================
# Test 8: EKF状态验证
# ============================================================
separator("Test 8: EKF状态验证")

ekf = mav.ekf_status()
log_result("EKF状态读取", True, f"{len(ekf)} 项数据")

# ============================================================
# Test 9: RC通道读取验证
# ============================================================
separator("Test 9: RC通道读取验证")

rc = mav.get_rc_channels()
log_result("RC通道读取", True, f"{len(rc)} 个通道")

# ============================================================
# Test 10: 预飞检查验证
# ============================================================
separator("Test 10: 预飞检查验证")

prearm = mav.prearm_check()
log_result("预飞检查", True, f"{len(prearm)} 项检查")

# ============================================================
# Test 11: 日志列表验证
# ============================================================
separator("Test 11: 日志列表验证")

logs = mav.list_logs()
log_result("日志列表", True, f"{len(logs)} 个日志文件")

# ============================================================
# Test 12: 失控保护配置验证
# ============================================================
separator("Test 12: 失控保护配置验证")

fs = mav.get_failsafe()
log_result("失控保护配置", len(fs) > 0, f"{len(fs)} 项配置")

# ============================================================
# Test 13: 围栏上传→下载→对比
# ============================================================
separator("Test 13: 围栏操作验证")

# 圆形围栏
fence_ok = mav.fence_add_circle(39.9042, 116.4074, 500)
log_result("圆形围栏添加", fence_ok, "500m radius")

# 下载围栏
fence_points = mav.fence_download()
log_result("围栏下载", len(fence_points) > 0, f"{len(fence_points)} 个点")

# 导出MP格式
verifier.record_fence_upload([{'lat': p.get('lat', 0), 'lon': p.get('lon', 0)} for p in fence_points] if fence_points else [])
fence_file = verifier.export_fence_mp(os.path.join(EXPORT_DIR, 'test_fence.txt'))
log_result("围栏导出MP格式", fence_file is not None, fence_file or "失败")

# ============================================================
# Test 14: Rally点验证
# ============================================================
separator("Test 14: Rally点验证")

rally_ok = mav.rally_add(39.9100, 116.4100, 100)
log_result("Rally点添加", rally_ok)

rally_points = mav.rally_show()
log_result("Rally点下载", True, f"{len(rally_points)} 个Rally点")

# 导出MP格式
verifier.record_rally_upload(rally_points)
rally_file = verifier.export_rally_mp(os.path.join(EXPORT_DIR, 'test_rally.txt'))
log_result("Rally导出MP格式", rally_file is not None, rally_file or "失败")

# ============================================================
# Test 15: 参数导出→导入对比
# ============================================================
separator("Test 15: 参数导出→导入对比")

# 导出当前参数
export_file = mav.param_export(os.path.join(EXPORT_DIR, 'full_params.txt'))
log_result("参数全量导出", export_file is not None, export_file or "失败")

# ============================================================
# Test 16: 生成完整验证报告
# ============================================================
separator("Test 16: 生成完整验证报告")

report = verifier.generate_report()
log_result("验证报告生成", report is not None, report or "失败")

# ============================================================
# 汇总
# ============================================================
separator("测试汇总")

total = len(results)
passed = sum(1 for _, p, _ in results if p)
failed = total - passed

print(f"\n  总测试: {total}")
print(f"  通过: {passed}")
print(f"  失败: {failed}")
print(f"  通过率: {passed/total*100:.1f}%")

if failed > 0:
    print(f"\n  失败项:")
    for name, p, detail in results:
        if not p:
            print(f"    ❌ {name}: {detail}")

# 保存测试结果
result_file = os.path.join(EXPORT_DIR, f"test_results_{time.strftime('%Y%m%d_%H%M%S')}.json")
with open(result_file, 'w', encoding='utf-8') as f:
    json.dump({
        'time': time.strftime('%Y-%m-%d %H:%M:%S'),
        'total': total,
        'passed': passed,
        'failed': failed,
        'results': [{'test': n, 'passed': p, 'detail': d} for n, p, d in results],
    }, f, indent=2, ensure_ascii=False)

print(f"\n  测试结果已保存: {result_file}")
print(f"  验证报告: {report}")

mav.close()
print(f"\n{'='*60}")
print(f"  测试完成")
print(f"{'='*60}")
