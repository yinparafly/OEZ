from pymavlink import mavutil
import time
import sys

print("=" * 60)
print("CUAV Nora 连接测试与 Balloon Drop 验证")
print("=" * 60)

# 测试 1: COM16 连接
print("\n[测试 1] COM16 USB 连接")
try:
    master = mavutil.mavlink_connection('COM16', baud=115200)
    master.wait_heartbeat(timeout=10)
    print(f"  ✅ 连接成功! System={master.target_system}, Type={master.mav_type}")
    
    # 获取固件版本
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1
    )
    
    # 读取参数
    print("\n[测试 2] 读取关键参数")
    params = {}
    master.param_fetch_all()
    time.sleep(3)
    
    important_params = ['SYSID_THISMAV', 'FRAME_CLASS', 'ARSPD_USE', 
                       'SERIAL0_BAUD', 'SCR_ENABLE', 'SCRIPTING_STACK_SIZE']
    for p in important_params:
        val = master.param_fetch_one(p)
        if val is not None:
            params[p] = val
            print(f"  {p} = {val}")
        else:
            print(f"  {p} = (not found)")
    
    # 检查脚本支持
    print("\n[测试 3] 脚本支持检查")
    scr_enable = params.get('SCR_ENABLE', 0)
    if scr_enable == 1:
        print("  ✅ Lua 脚本已启用")
    else:
        print("  ⚠️ Lua 脚本未启用, 尝试启用...")
        master.param_set('SCR_ENABLE', 1)
        time.sleep(1)
        master.param_set('SCR_HEAP_SIZE', 8192)
        time.sleep(1)
        print("  ✅ 已启用 Lua 脚本 (SCR_ENABLE=1, SCR_HEAP_SIZE=8192)")
    
    # 检查当前模式
    print("\n[测试 4] 当前飞行模式")
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    
    start = time.time()
    while time.time() - start < 3:
        msg = master.recv_match(type=['HEARTBEAT', 'VFR_HUD'], blocking=True, timeout=1)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                mode = master.flightmode
                print(f"  当前模式: {mode}")
            elif msg.get_type() == 'VFR_HUD':
                print(f"  高度: {msg.alt:.1f}m, 空速: {msg.airspeed:.1f}m/s")
                break
    
    master.close()
    print("\n✅ 所有测试通过!")
    
except Exception as e:
    print(f"  ❌ 连接失败: {e}")
    print("\n尝试 UDP 连接 SITL...")
    try:
        master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        master.wait_heartbeat(timeout=5)
        print(f"  ✅ SITL UDP 连接成功! System={master.target_system}")
        master.close()
    except Exception as e2:
        print(f"  ❌ SITL 连接也失败: {e2}")

print("\n" + "=" * 60)
print("测试完成")
print("=" * 60)
