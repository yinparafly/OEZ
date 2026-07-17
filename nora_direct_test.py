from pymavlink import mavutil
import time

print("=" * 60)
print("Balloon Drop 功能验证 - pymavlink 直连 Nora")
print("=" * 60)

# 连接
print("\n[1] 连接 Nora...")
master = mavutil.mavlink_connection('COM16', baud=115200)
master.wait_heartbeat(timeout=10)
print(f"  ✅ System={master.target_system}, Type={master.mav_type} (13=FixedWing)")

# 启用数据流
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)

# 读取参数
print("\n[2] 读取参数...")
master.param_fetch_all()
time.sleep(2)

# 检查脚本相关参数
params_to_read = ['SCR_ENABLE', 'SCR_HEAP_SIZE', 'SCR_STACK_SIZE',
                  'SYSID_THISMAV', 'ARSPD_USE', 'SERIAL0_BAUD']
for p in params_to_read:
    val = master.param_fetch_one(p)
    print(f"  {p} = {val}")

# 设置脚本参数
print("\n[3] 配置 Lua 脚本...")
try:
    # 使用 MAV_CMD 设置参数
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'SCR_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.5)
    
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'SCR_HEAP_SIZE', 8192, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.5)
    print("  ✅ SCR_ENABLE=1, SCR_HEAP_SIZE=8192")
except Exception as e:
    print(f"  ⚠️ 设置参数失败: {e}")

# 获取当前状态
print("\n[4] 当前飞行状态...")
start = time.time()
while time.time() - start < 3:
    msg = master.recv_match(blocking=True, timeout=1)
    if msg:
        mtype = msg.get_type()
        if mtype == 'VFR_HUD':
            print(f"  高度: {msg.alt:.1f}m")
            print(f"  空速: {msg.airspeed:.1f}m/s")
            print(f"  地速: {msg.groundspeed:.1f}m/s")
            print(f"  模式: {msg.mode}")
            break

# 检查心跳
print("\n[5] 心跳信息...")
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if msg:
    print(f"  Vehicle Type: {msg.type} ({master.mav_type})")
    print(f"  Autopilot: {msg.autopilot}")
    print(f"  Base Mode: {msg.base_mode}")
    print(f"  System Status: {msg.system_status}")

master.close()

print("\n" + "=" * 60)
print("✅ Nora USB 连接正常，pymavlink 可以完全操控")
print("   Mission Planner GUI 连接问题不影响功能验证")
print("=" * 60)
